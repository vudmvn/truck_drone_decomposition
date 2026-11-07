#include "bbcp/master/MasterCallback.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>

#include "bbcp/master/MasterCuts.h"

namespace bbcp
{

    MasterCallback::MasterCallback(
        const Instance &inst,
        const MasterOptions &options,
        const MasterModelData &data,
        const ISlaveSolver &slave,
        const FullSolution &initial_incumbent) : inst_(inst),
                                                 options_(options),
                                                 data_(data),
                                                 slave_(slave),
                                                 best_solution_(initial_incumbent),
                                                 best_valid_ub_(initial_incumbent.feasible ? initial_incumbent.objective : GRB_INFINITY),
                                                 last_dynamic_ub_workload_cut_ub_(initial_incumbent.feasible ? initial_incumbent.objective : GRB_INFINITY),
                                                 pending_dynamic_ub_workload_cut_ub_(GRB_INFINITY),
                                                 has_pending_dynamic_ub_workload_cut_(false),
                                                 last_dynamic_ub_energy_workload_cut_ub_(initial_incumbent.feasible ? initial_incumbent.objective : GRB_INFINITY),
                                                 pending_dynamic_ub_energy_workload_cut_ub_(GRB_INFINITY),
                                                 has_pending_dynamic_ub_energy_workload_cut_(false)
    {
        initial_valid_ub_ = best_valid_ub_;
        time_to_best_solution_sec_ = std::isfinite(best_valid_ub_) ? 0.0 : -1.0;
    }

    const FullSolution &MasterCallback::BestSolution() const
    {
        return best_solution_;
    }

    double MasterCallback::BestValidUB() const
    {
        return best_valid_ub_;
    }

    void MasterCallback::RegisterDynamicUBWorkloadUserCut(double new_ub)
    {
        if (!options_.enable_dynamic_ub_workload_usercut)
        {
            return;
        }

        if (!std::isfinite(new_ub))
        {
            return;
        }

        const double min_improvement =
            std::max(0.0, options_.dynamic_ub_workload_usercut_min_improvement);

        if (std::isfinite(last_dynamic_ub_workload_cut_ub_) &&
            last_dynamic_ub_workload_cut_ub_ - new_ub + 1e-9 < min_improvement)
        {
            return;
        }

        // The actual addCut() is done at MIPNODE. Keep only the tightest pending
        // UB; if several incumbents arrive before the next MIPNODE, the latest
        // one dominates the previous pending cut.
        pending_dynamic_ub_workload_cut_ub_ = new_ub;
        has_pending_dynamic_ub_workload_cut_ = true;

        if (options_.cut_verbose)
        {
            std::cout << "[PENDING-UB-WORKLOAD-USERCUT] UB=" << new_ub
                      << ", rhs=K*(UB-1)="
                      << static_cast<double>(inst_.n_drones) * (new_ub - 1.0)
                      << "\n";
        }

        if (options_.enable_dynamic_ub_energy_workload_usercut)
        {
            if (!std::isfinite(last_dynamic_ub_energy_workload_cut_ub_) ||
                last_dynamic_ub_energy_workload_cut_ub_ - new_ub + 1e-9 >= min_improvement)
            {
                pending_dynamic_ub_energy_workload_cut_ub_ = new_ub;
                has_pending_dynamic_ub_energy_workload_cut_ = true;

                if (options_.cut_verbose)
                {
                    const double rhs =
                        static_cast<double>(inst_.battery_capacity) *
                        static_cast<double>(inst_.n_drones) *
                        (new_ub - 1.0 + static_cast<double>(inst_.swap_time));

                    std::cout << "[PENDING-UB-ENERGY-WORKLOAD-USERCUT] UB=" << new_ub
                              << ", rhs=B*K*(UB-1+tau)=" << rhs
                              << "\n";
                }
            }
        }
    }

    bool MasterCallback::TryAddDynamicUBWorkloadUserCut(
        int &cuts_added,
        int max_cuts)
    {
        if (!options_.enable_dynamic_ub_workload_usercut)
        {
            return false;
        }

        if (!has_pending_dynamic_ub_workload_cut_ || cuts_added >= max_cuts)
        {
            return false;
        }

        if (!std::isfinite(pending_dynamic_ub_workload_cut_ub_))
        {
            return false;
        }

        if (inst_.n_drones <= 0)
        {
            return false;
        }

        const double ub_for_cut = pending_dynamic_ub_workload_cut_ub_;
        const double rhs_val =
            static_cast<double>(inst_.n_drones) * (ub_for_cut - 1.0);

        if (rhs_val < -1e-6)
        {
            return false;
        }

        GRBLinExpr lhs = 0.0;
        double lhs_val = 0.0;

        for (Node i = 1; i <= inst_.n; ++i)
        {
            if (!inst_.IsDroneFeasible(i))
            {
                continue;
            }

            const double coeff = static_cast<double>(inst_.DroneRoundTripTime(i));
            lhs += coeff * data_.y[i];
            lhs_val += coeff * getNodeRel(data_.y[i]);
        }

        if (options_.assignment_mode == MasterAssignmentMode::Explicit)
        {
            for (int k = 0; k < inst_.n_drones; ++k)
            {
                lhs += static_cast<double>(inst_.swap_time) * data_.q[k];
                lhs_val += static_cast<double>(inst_.swap_time) * getNodeRel(data_.q[k]);
            }
        }
        else
        {
            lhs += static_cast<double>(inst_.swap_time) * data_.Q;
            lhs_val += static_cast<double>(inst_.swap_time) * getNodeRel(data_.Q);
        }

        // addCut() is a user cut and should cut off the current LP relaxation.
        // If the current node does not violate the pending cut, keep it pending
        // so a later MIPNODE can still add it if violated.
        if (lhs_val <= rhs_val + 1e-6)
        {
            return false;
        }

        addCut(lhs <= rhs_val);
        ++cuts_added;

        last_dynamic_ub_workload_cut_ub_ = ub_for_cut;
        has_pending_dynamic_ub_workload_cut_ = false;

        if (options_.cut_verbose)
        {
            std::cout << "[USERCUT-MIPNODE-UB-WORKLOAD] "
                      << "lhs=" << lhs_val
                      << ", rhs=" << rhs_val
                      << ", UB=" << ub_for_cut
                      << "\n";
        }

        return true;
    }

    bool MasterCallback::TryAddDynamicUBEnergyWorkloadUserCut(
        int &cuts_added,
        int max_cuts)
    {
        if (!options_.enable_dynamic_ub_energy_workload_usercut)
        {
            return false;
        }

        if (!has_pending_dynamic_ub_energy_workload_cut_ || cuts_added >= max_cuts)
        {
            return false;
        }

        if (!std::isfinite(pending_dynamic_ub_energy_workload_cut_ub_))
        {
            return false;
        }

        if (inst_.n_drones <= 0 || inst_.battery_capacity <= 0)
        {
            return false;
        }

        const double ub_for_cut = pending_dynamic_ub_energy_workload_cut_ub_;
        const double tau = static_cast<double>(inst_.swap_time);
        const double B = static_cast<double>(inst_.battery_capacity);
        const double K = static_cast<double>(inst_.n_drones);
        const double U = ub_for_cut - 1.0;

        // Derived dynamic projection cut:
        //   tau * sum_i e_i y_i + B * sum_i r_i y_i
        //       <= B * K * (U + tau), where U = UB - 1.
        // It combines the dynamic workload cutoff
        //   sum_i r_i y_i + tau Q <= K U
        // and the energy lower bound
        //   B (K + Q) >= sum_i e_i y_i,
        // eliminating Q from the relaxation.
        const double rhs_val = B * K * (U + tau);

        if (rhs_val < -1e-6)
        {
            return false;
        }

        GRBLinExpr lhs = 0.0;
        double lhs_val = 0.0;

        for (Node i = 1; i <= inst_.n; ++i)
        {
            if (!inst_.IsDroneFeasible(i))
            {
                continue;
            }

            const double r = static_cast<double>(inst_.DroneRoundTripTime(i));
            const double e = static_cast<double>(inst_.energy[i]);
            const double coeff = tau * e + B * r;

            lhs += coeff * data_.y[i];
            lhs_val += coeff * getNodeRel(data_.y[i]);
        }

        if (lhs_val <= rhs_val + 1e-6)
        {
            return false;
        }

        addCut(lhs <= rhs_val);
        ++cuts_added;

        last_dynamic_ub_energy_workload_cut_ub_ = ub_for_cut;
        has_pending_dynamic_ub_energy_workload_cut_ = false;

        if (options_.cut_verbose)
        {
            std::cout << "[USERCUT-MIPNODE-UB-ENERGY-WORKLOAD] "
                      << "lhs=" << lhs_val
                      << ", rhs=" << rhs_val
                      << ", UB=" << ub_for_cut
                      << "\n";
        }

        return true;
    }

    void MasterCallback::callback()
    {
        try
        {
            if (where == GRB_CB_MIPSOL)
            {
                HandleIntegerSolution();
                try
                {
                    const double master_lb = getDoubleInfo(GRB_CB_MIPSOL_OBJBND);
                    UpdateGapTelemetry(master_lb);
                }
                catch (...)
                {
                    // Ignore if unavailable in this callback context.
                }

                return;
            }

            if (where == GRB_CB_MIPNODE)
            {
                HandleMIPNode();
                return;
            }

            if (where == GRB_CB_MIP) // Called periodically during the MIP search to report global progress.
            {
                const double master_lb = getDoubleInfo(GRB_CB_MIP_OBJBND);
                UpdateGapTelemetry(master_lb);
                return;
            }
        }
        catch (const GRBException &e)
        {
            std::cerr << "[CALLBACK-GUROBI-ERROR] " << e.getMessage() << "\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "[CALLBACK-ERROR] " << e.what() << "\n";
        }
    }

    double MasterCallback::ComputeValidGap(
        double valid_ub,
        double lower_bound) const
    {
        if (!std::isfinite(valid_ub) || valid_ub <= 1e-9)
        {
            return -1.0;
        }

        if (!std::isfinite(lower_bound))
        {
            return -1.0;
        }

        const double denom = std::max(1.0, std::abs(valid_ub));
        return std::max(0.0, valid_ub - lower_bound) / denom;
    }

    double MasterCallback::CallbackRuntime()
    {
        try
        {
            return getDoubleInfo(GRB_CB_RUNTIME);
        }
        catch (...)
        {
            return -1.0;
        }
    }

    void MasterCallback::UpdateGapTelemetry(double master_lb)
    {
        if (!std::isfinite(master_lb))
        {
            return;
        }

        // For minimization, the global lower bound should increase over time.
        best_master_lb_seen_ = std::max(best_master_lb_seen_, master_lb);

        if (!std::isfinite(best_valid_ub_) || best_valid_ub_ <= 1e-9)
        {
            return;
        }

        const double gap = ComputeValidGap(best_valid_ub_, best_master_lb_seen_);

        if (gap < 0.0)
        {
            return;
        }

        // Record the first time the valid original-problem gap reaches 1%.
        if (gap <= 0.01 + 1e-12 && time_to_1pct_gap_sec_ < 0.0)
        {
            time_to_1pct_gap_sec_ = CallbackRuntime();
            first_1pct_gap_value_ = gap;
            first_1pct_gap_bound_ = best_master_lb_seen_;

            if (options_.verbose)
            {
                std::cout << "[GAP-1PCT] "
                          << "time=" << time_to_1pct_gap_sec_
                          << ", valid_ub=" << best_valid_ub_
                          << ", master_lb=" << best_master_lb_seen_
                          << ", gap=" << 100.0 * gap << "%"
                          << "\n";
            }
        }
    }

    std::vector<Node> MasterCallback::DecodeTruckRoute(
        const std::vector<std::vector<int>> &xbar) const
    {
        const int N = inst_.NumNodes();

        std::vector<Node> route;
        std::vector<int> seen(N, 0);

        Node cur = inst_.start_depot;
        route.push_back(cur);
        seen[cur] = 1;

        while (cur != inst_.end_depot)
        {
            Node nxt = -1;

            for (int j = 0; j < N; ++j)
            {
                if (xbar[cur][j] == 1)
                {
                    nxt = j;
                    break;
                }
            }

            if (nxt == -1)
            {
                return {};
            }

            if (nxt != inst_.end_depot && seen[nxt])
            {
                return {};
            }

            route.push_back(nxt);
            seen[nxt] = 1;
            cur = nxt;
        }

        return route;
    }

    double MasterCallback::ComputeTruckValue(
        const std::vector<std::vector<int>> &xbar) const
    {
        const int N = inst_.NumNodes();
        double value = 0.0;

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (xbar[i][j] == 1)
                {
                    value += static_cast<double>(inst_.truck_time[i][j]);
                }
            }
        }

        return value;
    }

    void MasterCallback::AddDroneSubsetLowerBoundCut(
        int drone_id,
        const std::vector<Node> &S,
        double rhs)
    {
        if (S.empty() || rhs <= 0.0)
        {
            return;
        }

        const double local_m = MasterCuts::LocalBigM(rhs);

        GRBLinExpr missing = 0.0;
        for (Node i : S)
        {
            missing += (1.0 - data_.a[i][drone_id]);
        }

        GRBLinExpr lhs = data_.zK[drone_id] + local_m * missing;
        addLazy(lhs >= rhs);

        if (options_.cut_verbose)
        {
            std::cout << "[CUT] subset-LB drone=" << drone_id
                      << ", size=" << S.size()
                      << ", rhs=" << rhs
                      << ", M=" << local_m << "\n";
        }
    }

    void MasterCallback::AddBroadcastDroneSubsetLowerBoundCut(
        const std::vector<Node> &S,
        double rhs)
    {
        for (int ell = 0; ell < inst_.n_drones; ++ell)
        {
            AddDroneSubsetLowerBoundCut(ell, S, rhs);
        }
    }

    void MasterCallback::AddGlobalDroneSubsetLowerBoundCut(
        const std::vector<Node> &S,
        double rhs)
    {
        if (S.empty() || rhs <= 0.0)
        {
            return;
        }

        const double local_m = MasterCuts::LocalBigM(rhs);

        GRBLinExpr missing = 0.0;
        for (Node i : S)
        {
            missing += (1.0 - data_.y[i]);
        }

        GRBLinExpr lhs = data_.zD + local_m * missing;
        addLazy(lhs >= rhs);

        if (options_.cut_verbose || options_.verbose)
        {
            std::cout << "[CUT] global drone subset-LB"
                      << ", |S|=" << S.size()
                      << ", rhs=" << rhs
                      << ", S=" << NodeListString(S)
                      << "\n";
        }
    }

    void MasterCallback::AddDroneNoGoodCut(
        int drone_id,
        const std::vector<Node> &S)
    {
        if (S.empty())
        {
            return;
        }

        GRBLinExpr lhs = 0.0;
        for (Node i : S)
        {
            lhs += data_.a[i][drone_id];
        }

        addLazy(lhs <= static_cast<int>(S.size()) - 1);

        if (options_.cut_verbose)
        {
            std::cout << "[CUT] drone no-good drone=" << drone_id
                      << ", size=" << S.size() << "\n";
        }
    }

    void MasterCallback::AddBroadcastDroneNoGoodCut(
        const std::vector<Node> &S)
    {
        for (int ell = 0; ell < inst_.n_drones; ++ell)
        {
            AddDroneNoGoodCut(ell, S);
        }
    }

    void MasterCallback::AddGlobalDroneNoGoodCut(
        const std::vector<Node> &S)
    {
        if (S.empty())
        {
            return;
        }

        GRBLinExpr lhs = 0.0;
        for (Node i : S)
        {
            lhs += data_.y[i];
        }

        addLazy(lhs <= static_cast<int>(S.size()) - 1);

        if (options_.cut_verbose || options_.verbose)
        {
            std::cout << "[CUT] global drone no-good"
                      << ", |S|=" << S.size()
                      << ", S=" << NodeListString(S)
                      << "\n";
        }
    }

    std::string MasterCallback::NodeListString(const std::vector<Node> &nodes) const
    {
        std::ostringstream oss;
        oss << "{";
        for (size_t p = 0; p < nodes.size(); ++p)
        {
            if (p > 0)
                oss << ",";
            oss << nodes[p];
        }
        oss << "}";
        return oss.str();
    }

    void MasterCallback::PrintSlaveCall(
        int drone_id,
        const std::vector<Node> &S,
        int64_t cutoff) const
    {
        if (!options_.verbose)
        {
            return;
        }

        std::cout << "  [SLAVE-CALL] drone=" << drone_id
                  << ", |S|=" << S.size()
                  << ", cutoff=";

        if (cutoff >= 0)
        {
            std::cout << cutoff;
        }
        else
        {
            std::cout << "none";
        }

        std::cout << ", S=" << NodeListString(S) << "\n";
    }

    void MasterCallback::PrintSlaveResult(
        int drone_id,
        const std::vector<Node> &S,
        int64_t cutoff,
        const SlaveResult &result) const
    {
        if (!options_.verbose)
        {
            return;
        }

        std::cout << "  [SLAVE-RESULT] drone=" << drone_id
                  << ", |S|=" << S.size()
                  << ", cutoff=";

        if (cutoff >= 0)
        {
            std::cout << cutoff;
        }
        else
        {
            std::cout << "none";
        }

        std::cout << ", feasible=" << (result.feasible ? 1 : 0)
                  << ", optimal=" << (result.optimal ? 1 : 0)
                  << ", proven_infeasible=" << (result.proven_infeasible ? 1 : 0)
                  << ", unknown=" << (result.unknown ? 1 : 0);

        if (result.feasible)
        {
            std::cout << ", obj=" << result.objective;
        }
        else
        {
            std::cout << ", obj=N/A";
        }

        if (result.lower_bound_valid)
        {
            std::cout << ", lb=" << result.lower_bound;
        }
        else
        {
            std::cout << ", lb=N/A";
        }

        std::cout << ", wall_time=" << result.wall_time
                  << ", S=" << NodeListString(S)
                  << "\n";

        if (result.feasible && !result.chains.empty())
        {
            int total_assigned = 0;

            std::cout << "                 chains_size: "
                      << result.chains.size() << "\n";

            for (size_t k = 0; k < result.chains.size(); ++k)
            {
                const std::vector<Node> &chain = result.chains[k];

                const std::vector<int> *swaps = nullptr;
                if (k < result.swap_before.size())
                {
                    swaps = &result.swap_before[k];
                }

                total_assigned += static_cast<int>(chain.size());

                std::cout << "                 drone " << k << " chain: ";

                if (chain.empty())
                {
                    std::cout << "(empty)";
                }

                for (size_t p = 0; p < chain.size(); ++p)
                {
                    if (p > 0)
                    {
                        std::cout << " -> ";
                    }

                    const bool has_swap =
                        (swaps != nullptr &&
                         p < swaps->size() &&
                         (*swaps)[p] != 0);

                    if (has_swap)
                    {
                        std::cout << "[SWAP] ";
                    }

                    std::cout << chain[p];
                }

                std::cout << "\n";

                std::cout << "                 drone " << k << " swaps_before: ";

                bool any = false;

                if (swaps != nullptr)
                {
                    for (size_t p = 0; p < chain.size() && p < swaps->size(); ++p)
                    {
                        if ((*swaps)[p] != 0)
                        {
                            if (any)
                            {
                                std::cout << ",";
                            }

                            std::cout << chain[p];
                            any = true;
                        }
                    }
                }

                if (!any)
                {
                    std::cout << "none";
                }

                std::cout << "\n";
            }

            std::cout << "                 assigned_count: "
                      << total_assigned
                      << ", expected=" << S.size()
                      << "\n";
        }
    }

    void MasterCallback::SubmitRepairedIncumbent(
        const std::vector<int> &tbar,
        const std::vector<std::vector<int>> &abar,
        const std::vector<std::vector<int>> &xbar,
        const std::vector<double> &drone_values,
        const std::vector<int> &swap_counts,
        double truck_value,
        double drone_value,
        double submitted_z)
    {
        std::vector<GRBVar> vars;
        std::vector<double> vals;

        const int N = inst_.NumNodes();

        std::vector<int> ybar(N, 0);

        for (Node i = 1; i <= inst_.n; ++i)
        {
            for (int k = 0; k < inst_.n_drones; ++k)
            {
                if (abar[i][k] == 1)
                {
                    ybar[i] = 1;
                }
            }
        }

        for (Node i = 1; i <= inst_.n; ++i)
        {
            vars.push_back(data_.t[i]);
            vals.push_back(static_cast<double>(tbar[i]));

            vars.push_back(data_.y[i]);
            vals.push_back(static_cast<double>(ybar[i]));

            if (options_.assignment_mode == MasterAssignmentMode::Explicit)
            {
                for (int k = 0; k < inst_.n_drones; ++k)
                {
                    vars.push_back(data_.a[i][k]);
                    vals.push_back(static_cast<double>(abar[i][k]));
                }
            }
        }

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (i == j)
                    continue;
                if (i == inst_.end_depot)
                    continue;
                if (j == inst_.start_depot)
                    continue;

                vars.push_back(data_.x[i][j]);
                vals.push_back(static_cast<double>(xbar[i][j]));
            }
        }

        if (options_.assignment_mode == MasterAssignmentMode::Explicit)
        {
            for (int k = 0; k < inst_.n_drones; ++k)
            {
                vars.push_back(data_.zK[k]);
                vals.push_back(k < static_cast<int>(drone_values.size()) ? drone_values[k] : 0.0);

                vars.push_back(data_.q[k]);
                vals.push_back(k < static_cast<int>(swap_counts.size()) ? swap_counts[k] : 0.0);
            }
        }
        else
        {
            int total_swaps = 0;
            for (int v : swap_counts)
            {
                total_swaps += v;
            }
            vars.push_back(data_.Q);
            vals.push_back(static_cast<double>(total_swaps));
        }

        vars.push_back(data_.zT);
        vals.push_back(truck_value);

        vars.push_back(data_.zD);
        vals.push_back(drone_value);

        vars.push_back(data_.z);
        vals.push_back(submitted_z);

        setSolution(vars.data(), vals.data(), static_cast<int>(vars.size()));

        try
        {
            useSolution();
        }
        catch (const GRBException &e)
        {
            if (options_.verbose)
            {
                std::cout << "[SUBMIT] useSolution failed: "
                          << e.getMessage() << "\n";
            }
        }
    }

    bool MasterCallback::TryAccept(
        const std::vector<int> &tbar,
        const std::vector<std::vector<int>> &abar,
        const std::vector<std::vector<int>> &xbar,
        const std::vector<std::vector<Node>> &per_drone_customers,
        const DroneSchedule &drone_schedule,
        const std::vector<double> &drone_values,
        const std::vector<int> &swap_counts,
        double truck_value,
        double drone_value,
        double accepted_z,
        double submitted_z)
    {
        if (accepted_z + 1e-6 >= best_valid_ub_)
        {
            return false;
        }

        // Update the valid original-problem UB by the true checked value.
        // Submit to Gurobi using submitted_z = max(master_z, accepted_z).
        best_valid_ub_ = accepted_z;

        valid_incumbent_updates_++;

        const double rt = CallbackRuntime();
        if (rt >= 0.0)
        {
            time_to_best_solution_sec_ = rt;
        }

        // Schedule the dynamic UB workload user cut if the valid UB has
        // decreased sufficiently since the last such cut was scheduled/added.
        RegisterDynamicUBWorkloadUserCut(accepted_z);

        FullSolution sol;
        sol.feasible = true;
        sol.objective = accepted_z;
        sol.truck_value = truck_value;
        sol.drone_value = drone_value;
        sol.truck_route = DecodeTruckRoute(xbar);
        sol.per_drone_customers = per_drone_customers;
        sol.drone_schedule = drone_schedule;

        for (Node i = 1; i <= inst_.n; ++i)
        {
            if (tbar[i] == 1)
            {
                sol.truck_customers.push_back(i);
            }
            else
            {
                sol.drone_customers.push_back(i);
            }
        }

        best_solution_ = sol;

        SubmitRepairedIncumbent(
            tbar,
            abar,
            xbar,
            drone_values,
            swap_counts,
            truck_value,
            drone_value,
            submitted_z);

        if (options_.verbose)
        {
            std::cout << "\n[NEW BEST] accepted_z=" << accepted_z
                      << ", submitted_master_z=" << submitted_z
                      << ", truck=" << truck_value
                      << ", drone=" << drone_value
                      << "\n";
        }

        return true;
    }

    bool MasterCallback::IsTruckArcAllowed(Node i, Node j) const
    {
        if (i == j)
            return false;
        if (i == inst_.end_depot)
            return false;
        if (j == inst_.start_depot)
            return false;
        return true;
    }

    std::vector<std::vector<Node>> MasterCallback::FindTruckSubtours(
        const std::vector<int> &tbar,
        const std::vector<std::vector<int>> &xbar) const
    {
        const int N = inst_.NumNodes();

        // First follow the unique truck path starting at the start depot.
        // Any selected truck customer not reached by this path belongs to a
        // disconnected truck component, which must be cut in SEC mode.
        std::vector<int> reached(N, 0);
        Node cur = inst_.start_depot;

        for (int step = 0; step <= N + 1; ++step)
        {
            if (cur < 0 || cur >= N)
            {
                break;
            }

            if (reached[cur])
            {
                break;
            }

            reached[cur] = 1;

            if (cur == inst_.end_depot)
            {
                break;
            }

            Node nxt = -1;
            for (int j = 0; j < N; ++j)
            {
                if (xbar[cur][j] == 1)
                {
                    nxt = j;
                    break;
                }
            }

            if (nxt == -1)
            {
                break;
            }

            cur = nxt;
        }

        std::vector<std::vector<Node>> subtours;
        std::vector<int> visited(N, 0);

        for (Node root = 1; root <= inst_.n; ++root)
        {
            if (tbar[root] == 0 || reached[root] || visited[root])
            {
                continue;
            }

            std::vector<Node> stack = {root};
            std::vector<Node> comp;
            visited[root] = 1;

            while (!stack.empty())
            {
                Node v = stack.back();
                stack.pop_back();
                comp.push_back(v);

                for (Node u = 1; u <= inst_.n; ++u)
                {
                    if (tbar[u] == 0 || reached[u] || visited[u])
                    {
                        continue;
                    }

                    // Use the undirected support of the integer truck arcs to
                    // collect the entire disconnected component.
                    if (xbar[v][u] == 1 || xbar[u][v] == 1)
                    {
                        visited[u] = 1;
                        stack.push_back(u);
                    }
                }
            }

            if (comp.size() <= 1)
            {
                continue;
            }

            int internal_arcs = 0;
            for (Node i : comp)
            {
                for (Node j : comp)
                {
                    if (i == j)
                    {
                        continue;
                    }
                    if (xbar[i][j] == 1)
                    {
                        ++internal_arcs;
                    }
                }
            }

            // For a pure subtour, the number of internal directed arcs is |S|.
            // The SEC cut sum_{i,j in S} x_ij <= |S|-1 removes it.
            if (internal_arcs >= static_cast<int>(comp.size()))
            {
                std::sort(comp.begin(), comp.end());
                subtours.push_back(comp);
            }
        }

        return subtours;
    }

    void MasterCallback::AddTruckSubtourEliminationCut(
        const std::vector<Node> &S)
    {
        if (S.size() <= 1)
        {
            return;
        }

        GRBLinExpr lhs = 0.0;

        for (Node i : S)
        {
            for (Node j : S)
            {
                if (i == j)
                {
                    continue;
                }
                if (!IsTruckArcAllowed(i, j))
                {
                    continue;
                }
                lhs += data_.x[i][j];
            }
        }

        addLazy(lhs <= static_cast<int>(S.size()) - 1);

        if (options_.cut_verbose)
        {
            std::cout << "[LAZY-SEC] truck subtour cut"
                      << ", |S|=" << S.size()
                      << ", S=" << NodeListString(S)
                      << "\n";
        }
    }

    bool MasterCallback::SeparateIntegerTruckSubtourCuts(
        const std::vector<int> &tbar,
        const std::vector<std::vector<int>> &xbar)
    {
        if (options_.subtour_mode != MasterSubtourMode::SEC)
        {
            return false;
        }

        const std::vector<std::vector<Node>> subtours = FindTruckSubtours(tbar, xbar);

        for (const std::vector<Node> &S : subtours)
        {
            AddTruckSubtourEliminationCut(S);
        }

        return !subtours.empty();
    }

    void MasterCallback::HandleMIPNode()
    {
        const int node_status = getIntInfo(GRB_CB_MIPNODE_STATUS);

        if (node_status != GRB_OPTIMAL)
        {
            return;
        }

        const int N = inst_.NumNodes();
        const int K = inst_.n_drones;

        if (options_.assignment_mode == MasterAssignmentMode::SlaveAssign)
        {
            std::vector<double> trel(N, 0.0);
            std::vector<std::vector<double>> xrel(N, std::vector<double>(N, 0.0));

            for (Node i = 1; i <= inst_.n; ++i)
            {
                trel[i] = getNodeRel(data_.t[i]);
            }

            for (int i = 0; i < N; ++i)
            {
                for (int j = 0; j < N; ++j)
                {
                    if (!IsTruckArcAllowed(i, j))
                        continue;
                    xrel[i][j] = getNodeRel(data_.x[i][j]);
                }
            }

            int cuts_added = 0;
            const int max_cuts = 80;

            TryAddDynamicUBWorkloadUserCut(cuts_added, max_cuts);

            if (cuts_added < max_cuts)
            {
                TryAddDynamicUBEnergyWorkloadUserCut(cuts_added, max_cuts);
            }

            if (cuts_added < max_cuts)
            {
                SeparateFractionalTruckConnectivityCuts(trel, xrel, cuts_added, max_cuts);
            }

            if (options_.cut_verbose && cuts_added > 0)
            {
                std::cout << "[USERCUT-MIPNODE] cuts_added=" << cuts_added << "\n";
            }
            return;
        }

        std::vector<double> trel(N, 0.0);
        std::vector<std::vector<double>> xrel(N, std::vector<double>(N, 0.0));
        std::vector<std::vector<double>> arel(N, std::vector<double>(K, 0.0));
        std::vector<double> qrel(K, 0.0);
        std::vector<double> zKrel(K, 0.0);

        for (Node i = 1; i <= inst_.n; ++i)
        {
            trel[i] = getNodeRel(data_.t[i]);

            for (int k = 0; k < K; ++k)
            {
                arel[i][k] = getNodeRel(data_.a[i][k]);
            }
        }

        for (int k = 0; k < K; ++k)
        {
            qrel[k] = getNodeRel(data_.q[k]);
            zKrel[k] = getNodeRel(data_.zK[k]);
        }

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (!IsTruckArcAllowed(i, j))
                    continue;
                xrel[i][j] = getNodeRel(data_.x[i][j]);
            }
        }

        int cuts_added = 0;
        const int max_cuts = 80;

        TryAddDynamicUBWorkloadUserCut(cuts_added, max_cuts);

        if (cuts_added < max_cuts)
        {
            TryAddDynamicUBEnergyWorkloadUserCut(cuts_added, max_cuts);
        }

        if (cuts_added < max_cuts)
        {
            SeparateFractionalTruckConnectivityCuts(trel, xrel, cuts_added, max_cuts);
        }

        if (cuts_added < max_cuts)
        {
            SeparateFractionalDroneCoverCuts(arel, qrel, zKrel, cuts_added, max_cuts);
        }

        if (options_.cut_verbose && cuts_added > 0)
        {
            std::cout << "[USERCUT-MIPNODE] cuts_added=" << cuts_added << "\n";
        }
    }

    void MasterCallback::SeparateFractionalTruckConnectivityCuts(
        const std::vector<double> &trel,
        const std::vector<std::vector<double>> &xrel,
        int &cuts_added,
        int max_cuts)
    {
        const int N = inst_.NumNodes();
        const double eps = 1e-6;

        // Build an undirected support graph from positive truck arcs.
        std::vector<std::vector<int>> adj(N);

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (!IsTruckArcAllowed(i, j))
                    continue;

                if (xrel[i][j] > eps)
                {
                    adj[i].push_back(j);
                    adj[j].push_back(i);
                }
            }
        }

        std::vector<int> visited(N, 0);

        for (int root = 0; root < N; ++root)
        {
            if (visited[root])
                continue;

            std::vector<int> stack = {root};
            std::vector<int> comp;
            visited[root] = 1;

            bool contains_start = false;
            bool contains_end = false;

            while (!stack.empty())
            {
                int v = stack.back();
                stack.pop_back();

                comp.push_back(v);

                if (v == inst_.start_depot)
                    contains_start = true;
                if (v == inst_.end_depot)
                    contains_end = true;

                for (int nxt : adj[v])
                {
                    if (visited[nxt])
                        continue;
                    visited[nxt] = 1;
                    stack.push_back(nxt);
                }
            }

            // Components containing depot nodes are allowed. Components without
            // depot nodes and containing truck-selected customers must connect to
            // the rest of the truck path.
            if (contains_start || contains_end)
            {
                continue;
            }

            std::vector<Node> S;
            for (int v : comp)
            {
                if (v >= 1 && v <= inst_.n && trel[v] > eps)
                {
                    S.push_back(v);
                }
            }

            if (S.empty())
            {
                continue;
            }

            TryAddTruckConnectivityCutsForSet(S, trel, xrel, cuts_added, max_cuts);

            if (cuts_added >= max_cuts)
            {
                return;
            }
        }
    }

    bool MasterCallback::TryAddTruckConnectivityCutsForSet(
        const std::vector<Node> &S,
        const std::vector<double> &trel,
        const std::vector<std::vector<double>> &xrel,
        int &cuts_added,
        int max_cuts)
    {
        if (S.empty() || cuts_added >= max_cuts)
        {
            return false;
        }

        const int N = inst_.NumNodes();
        const double eps = 1e-6;

        std::vector<int> inS(N, 0);
        for (Node i : S)
        {
            inS[i] = 1;
        }

        // Use the most truck-selected customer in S as the RHS representative.
        Node h = S.front();
        for (Node i : S)
        {
            if (trel[i] > trel[h])
            {
                h = i;
            }
        }

        if (trel[h] <= eps)
        {
            return false;
        }

        double out_value = 0.0;
        double in_value = 0.0;

        for (Node i : S)
        {
            for (int j = 0; j < N; ++j)
            {
                if (inS[j])
                    continue;
                if (!IsTruckArcAllowed(i, j))
                    continue;
                out_value += xrel[i][j];
            }
        }

        for (int i = 0; i < N; ++i)
        {
            if (inS[i])
                continue;

            for (Node j : S)
            {
                if (!IsTruckArcAllowed(i, j))
                    continue;
                in_value += xrel[i][j];
            }
        }

        bool added = false;

        if (out_value + eps < trel[h] && cuts_added < max_cuts)
        {
            GRBLinExpr out_expr = 0.0;

            for (Node i : S)
            {
                for (int j = 0; j < N; ++j)
                {
                    if (inS[j])
                        continue;
                    if (!IsTruckArcAllowed(i, j))
                        continue;
                    out_expr += data_.x[i][j];
                }
            }

            addCut(out_expr >= data_.t[h]);
            cuts_added++;
            added = true;

            if (options_.cut_verbose)
            {
                std::cout << "[USERCUT] truck outgoing connectivity"
                          << ", |S|=" << S.size()
                          << ", lhs=" << out_value
                          << ", rhs=t_" << h << "=" << trel[h]
                          << "\n";
            }
        }

        if (in_value + eps < trel[h] && cuts_added < max_cuts)
        {
            GRBLinExpr in_expr = 0.0;

            for (int i = 0; i < N; ++i)
            {
                if (inS[i])
                    continue;

                for (Node j : S)
                {
                    if (!IsTruckArcAllowed(i, j))
                        continue;
                    in_expr += data_.x[i][j];
                }
            }

            addCut(in_expr >= data_.t[h]);
            cuts_added++;
            added = true;

            if (options_.cut_verbose)
            {
                std::cout << "[USERCUT] truck incoming connectivity"
                          << ", |S|=" << S.size()
                          << ", lhs=" << in_value
                          << ", rhs=t_" << h << "=" << trel[h]
                          << "\n";
            }
        }

        return added;
    }

    void MasterCallback::SeparateFractionalDroneCoverCuts(
        const std::vector<std::vector<double>> &arel,
        const std::vector<double> &qrel,
        const std::vector<double> &zKrel,
        int &cuts_added,
        int max_cuts)
    {
        const int K = inst_.n_drones;
        const double eps = 1e-6;

        for (int k = 0; k < K; ++k)
        {
            std::vector<Node> order;

            for (Node i = 1; i <= inst_.n; ++i)
            {
                if (!inst_.IsDroneFeasible(i))
                    continue;
                if (arel[i][k] <= eps)
                    continue;
                order.push_back(i);
            }

            if (order.size() < 2)
            {
                continue;
            }

            // Customers with high fractional assignment and high energy are more
            // likely to form violated conditional energy-cover cuts.
            std::sort(order.begin(), order.end(), [&](Node a, Node b)
                      {
            const double score_a =
                arel[a][k] * (1.0 + static_cast<double>(inst_.energy[a]));
            const double score_b =
                arel[b][k] * (1.0 + static_cast<double>(inst_.energy[b]));

            if (std::abs(score_a - score_b) > 1e-9) {
                return score_a > score_b;
            }
            return a < b; });

            std::vector<Node> H;
            H.reserve(order.size());

            for (Node i : order)
            {
                H.push_back(i);

                if (H.size() < 2)
                {
                    continue;
                }

                TryAddDroneCoverCutsForSet(k, H, arel, qrel[k], zKrel[k],
                                           cuts_added, max_cuts);

                if (cuts_added >= max_cuts)
                {
                    return;
                }
            }
        }
    }

    bool MasterCallback::TryAddDroneCoverCutsForSet(
        int drone_id,
        const std::vector<Node> &H,
        const std::vector<std::vector<double>> &arel,
        double qrel,
        double zKrel,
        int &cuts_added,
        int max_cuts)
    {
        if (H.empty() || cuts_added >= max_cuts)
        {
            return false;
        }

        if (inst_.battery_capacity <= 0)
        {
            return false;
        }

        const double eps = 1e-6;

        int64_t total_energy = 0;
        double total_roundtrip = 0.0;
        double sum_a = 0.0;

        for (Node i : H)
        {
            total_energy += inst_.energy[i];
            total_roundtrip += static_cast<double>(inst_.DroneRoundTripTime(i));
            sum_a += arel[i][drone_id];
        }

        const int64_t cycles =
            (total_energy + inst_.battery_capacity - 1) / inst_.battery_capacity;

        const int64_t required_swaps = std::max<int64_t>(0, cycles - 1);

        bool added = false;

        // Conditional energy-cover cut:
        //
        //   q_k + R(|H| - sum_{i in H} a_ik) >= R,
        //
        // where R = ceil(sum e_i / E) - 1.
        //
        // This is valid for all integer assignments. If all H are assigned to
        // drone k, then q_k >= R. If at least one member is missing, the big-M term
        // relaxes the cut.
        if (required_swaps > 0 && cuts_added < max_cuts)
        {
            const double R = static_cast<double>(required_swaps);
            const double lhs_value = qrel + R * (static_cast<double>(H.size()) - sum_a);

            if (lhs_value + 1e-6 < R)
            {
                GRBLinExpr lhs = data_.q[drone_id];

                for (Node i : H)
                {
                    lhs += R * (1.0 - data_.a[i][drone_id]);
                }

                addCut(lhs >= R);
                cuts_added++;
                added = true;

                if (options_.cut_verbose)
                {
                    std::cout << "[USERCUT] drone energy cover"
                              << ", drone=" << drone_id
                              << ", |H|=" << H.size()
                              << ", lhs=" << lhs_value
                              << ", rhs=" << R
                              << "\n";
                }
            }
        }

        if (cuts_added >= max_cuts)
        {
            return added;
        }

        // Conditional drone workload lower-bound cut:
        //
        //   z_k + M(|H| - sum_{i in H} a_ik)
        //       >= sum r_i + tau_swap * R.
        //
        // This strengthens fractional nodes for subsets that are almost fully
        // assigned to a drone.
        const double rhs =
            total_roundtrip +
            static_cast<double>(inst_.swap_time) * static_cast<double>(required_swaps);

        if (rhs > eps)
        {
            const double M = std::max(1.0, rhs);
            const double lhs_value =
                zKrel + M * (static_cast<double>(H.size()) - sum_a);

            if (lhs_value + 1e-6 < rhs && cuts_added < max_cuts)
            {
                GRBLinExpr lhs = data_.zK[drone_id];

                for (Node i : H)
                {
                    lhs += M * (1.0 - data_.a[i][drone_id]);
                }

                addCut(lhs >= rhs);
                cuts_added++;
                added = true;

                if (options_.cut_verbose)
                {
                    std::cout << "[USERCUT] drone subset workload"
                              << ", drone=" << drone_id
                              << ", |H|=" << H.size()
                              << ", lhs=" << lhs_value
                              << ", rhs=" << rhs
                              << "\n";
                }
            }
        }

        return added;
    }

    void MasterCallback::HandleIntegerSolution()
    {
        if (options_.assignment_mode == MasterAssignmentMode::SlaveAssign)
        {
            HandleIntegerSolutionSlaveAssign();
        }
        else
        {
            HandleIntegerSolutionExplicit();
        }
    }

    void MasterCallback::HandleIntegerSolutionExplicit()
    {
        const int N = inst_.NumNodes();

        std::vector<int> tbar(N, 0);
        std::vector<std::vector<int>> abar(
            N,
            std::vector<int>(inst_.n_drones, 0));
        std::vector<std::vector<int>> xbar(N, std::vector<int>(N, 0));

        for (Node i = 1; i <= inst_.n; ++i)
        {
            tbar[i] = getSolution(data_.t[i]) > 0.5 ? 1 : 0;

            for (int k = 0; k < inst_.n_drones; ++k)
            {
                abar[i][k] = getSolution(data_.a[i][k]) > 0.5 ? 1 : 0;
            }
        }

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (i == j)
                    continue;
                if (i == inst_.end_depot)
                    continue;
                if (j == inst_.start_depot)
                    continue;

                xbar[i][j] = getSolution(data_.x[i][j]) > 0.5 ? 1 : 0;
            }
        }

        if (SeparateIntegerTruckSubtourCuts(tbar, xbar))
        {
            // In SEC mode, reject this integer solution before invoking any
            // drone subproblem. This keeps the slave from being called on a
            // master solution whose truck route is disconnected.
            return;
        }

        const double master_z = getSolution(data_.z);
        const double truck_value = ComputeTruckValue(xbar);

        if (std::isfinite(best_valid_ub_) &&
            truck_value >= best_valid_ub_ - 1e-6)
        {
            if (options_.verbose)
            {
                std::cout << "\n[REJECT] truck_value >= valid UB"
                          << ", master_z=" << master_z
                          << ", truck=" << truck_value
                          << ", UB=" << best_valid_ub_
                          << "\n";
            }
            return;
        }

        std::vector<std::vector<Node>> per_drone_customers(inst_.n_drones);

        for (Node i = 1; i <= inst_.n; ++i)
        {
            for (int k = 0; k < inst_.n_drones; ++k)
            {
                if (abar[i][k] == 1)
                {
                    per_drone_customers[k].push_back(i);
                }
            }
        }

        if (options_.verbose)
        {
            std::cout << "\n[MIPSOL] master_z=" << master_z
                      << ", truck=" << truck_value
                      << ", UB=" << best_valid_ub_
                      << "\n";
        }

        DroneSchedule schedule;
        schedule.chains.assign(inst_.n_drones, {});
        schedule.swap_before.assign(inst_.n_drones, {});
        schedule.makespan = 0.0;
        schedule.swap_count = 0;

        std::vector<double> drone_values(inst_.n_drones, 0.0);
        std::vector<int> swap_counts(inst_.n_drones, 0);

        bool all_slaves_feasible = true;
        bool unresolved_invalid_candidate = false;
        int unresolved_drone = -1;
        double unresolved_slave_obj = -1.0;
        double unresolved_slave_lb = -1.0;
        double unresolved_master_zk = -1.0;

        for (int k = 0; k < inst_.n_drones; ++k)
        {
            const std::vector<Node> &S = per_drone_customers[k];

            SlaveOptions slave_opts = options_.slave_options;

            // Important: every slave is solved only in the improving region.
            // Therefore the slave objective is constrained by:
            //
            //     z_slave <= current_valid_UB - 1.
            //
            // This is passed to the slave through makespan_cutoff and is also
            // printed in the log for debugging.
            int64_t slave_cutoff = -1;

            if (std::isfinite(best_valid_ub_))
            {
                slave_cutoff =
                    static_cast<int64_t>(std::floor(best_valid_ub_ - 1.0 + 1e-9));

                if (slave_opts.makespan_cutoff >= 0)
                {
                    slave_opts.makespan_cutoff =
                        std::min<int64_t>(slave_opts.makespan_cutoff, slave_cutoff);
                }
                else
                {
                    slave_opts.makespan_cutoff = slave_cutoff;
                }
            }

            PrintSlaveCall(k, S, slave_opts.makespan_cutoff);

            SlaveResult r = slave_.Solve(S, slave_opts);

            PrintSlaveResult(k, S, slave_opts.makespan_cutoff, r);

            if (!r.feasible)
            {
                all_slaves_feasible = false;

                if (r.proven_infeasible && std::isfinite(best_valid_ub_))
                {
                    // The set S cannot be served by one drone under UB-1.
                    // Since drones are identical, forbid this set on every drone.
                    AddBroadcastDroneNoGoodCut(S);
                    AddBroadcastDroneSubsetLowerBoundCut(S, best_valid_ub_);
                }
                else if (r.lower_bound_valid)
                {
                    AddBroadcastDroneSubsetLowerBoundCut(S, static_cast<double>(r.lower_bound));
                }
                else
                {
                    unresolved_invalid_candidate = true;
                    unresolved_drone = k;
                    unresolved_slave_obj = -1.0;
                    unresolved_slave_lb = r.lower_bound_valid ? static_cast<double>(r.lower_bound) : -1.0;
                    unresolved_master_zk = getSolution(data_.zK[k]);
                }

                continue;
            }

            drone_values[k] = static_cast<double>(r.objective);
            schedule.makespan = std::max(schedule.makespan, drone_values[k]);

            if (!r.chains.empty())
            {
                schedule.chains[k] = r.chains.front();
                schedule.swap_before[k] = r.swap_before.front();

                for (int v : schedule.swap_before[k])
                {
                    if (v != 0)
                    {
                        schedule.swap_count++;
                        swap_counts[k]++;
                    }
                }
            }

            const double zK_master = getSolution(data_.zK[k]);

            if (r.optimal)
            {
                // If the slave proves optimality, its objective is a valid lower
                // bound for the same customer subset. Broadcast this cut to all
                // identical drones.
                if (zK_master + 1e-6 < static_cast<double>(r.objective))
                {
                    AddBroadcastDroneSubsetLowerBoundCut(S, static_cast<double>(r.objective));
                }
            }
            else if (r.lower_bound_valid)
            {
                // If the slave is feasible but not optimal, only its lower bound is
                // valid for cutting. Its feasible objective is still a valid
                // original-problem completion and may be used to submit/update UB.
                if (zK_master + 1e-6 < static_cast<double>(r.lower_bound))
                {
                    AddBroadcastDroneSubsetLowerBoundCut(S, static_cast<double>(r.lower_bound));
                }

                if (options_.verbose &&
                    static_cast<double>(r.objective) > zK_master + 1e-6)
                {
                    std::cout << "  [SLAVE-NOTE] feasible but not optimal; "
                              << "do not cut with obj. "
                              << "drone=" << k
                              << ", slave_obj=" << r.objective
                              << ", slave_lb=" << r.lower_bound
                              << ", master_zk=" << zK_master
                              << "\n";
                }
            }
            else
            {
                // Feasible slave without a valid lower bound:
                // - do NOT abort;
                // - do NOT add a z_k >= objective cut;
                // - use the feasible objective only to construct a valid incumbent.
                if (options_.verbose &&
                    static_cast<double>(r.objective) > zK_master + 1e-6)
                {
                    std::cout << "  [SLAVE-NOTE] feasible without proof; "
                              << "use obj only for incumbent submission. "
                              << "drone=" << k
                              << ", slave_obj=" << r.objective
                              << ", master_zk=" << zK_master
                              << "\n";
                }
            }
        }

        if (unresolved_invalid_candidate)
        {
            std::cerr << "\n[ERROR] A slave call did not provide enough proof "
                      << "to verify or cut the master incumbent. Aborting.\n"
                      << "        unresolved_drone=" << unresolved_drone
                      << ", slave_obj=" << unresolved_slave_obj
                      << ", slave_lb=" << unresolved_slave_lb
                      << ", master_zk=" << unresolved_master_zk
                      << ", current_UB=" << best_valid_ub_
                      << "\n"
                      << "        Increase slave_time, use exact CP-SAT, or use MILP slave. "
                      << "For SWAP TIME = 0, v9 returns an exact slave value directly.\n";
            abort();
            return;
        }

        if (!all_slaves_feasible)
        {
            if (options_.verbose)
            {
                std::cout << "[REJECT] at least one slave infeasible/unknown\n";
            }
            return;
        }

        const double accepted_z = std::max(truck_value, schedule.makespan);

        // This is the value submitted to Gurobi's master model.
        //
        // If a feasible slave completion has z_slave > master_z, then accepted_z
        // raises the submitted master objective accordingly. If master_z is already
        // larger, submitted_z keeps the incumbent consistent with the current
        // master objective value.
        //
        // The valid original-problem UB remains accepted_z.
        const double submitted_z = std::max(master_z, accepted_z);

        if (options_.verbose)
        {
            std::cout << "  [SLAVE-SUMMARY] truck=" << truck_value
                      << ", drone_makespan=" << schedule.makespan
                      << ", accepted_z=" << accepted_z
                      << ", submitted_master_z=" << submitted_z
                      << ", master_z=" << master_z
                      << ", current_UB=" << best_valid_ub_
                      << "\n";
        }

        TryAccept(
            tbar,
            abar,
            xbar,
            per_drone_customers,
            schedule,
            drone_values,
            swap_counts,
            truck_value,
            schedule.makespan,
            accepted_z,
            submitted_z);
    }

    void MasterCallback::HandleIntegerSolutionSlaveAssign()
    {
        const int N = inst_.NumNodes();
        const int K = inst_.n_drones;

        std::vector<int> tbar(N, 0);
        std::vector<int> ybar(N, 0);
        std::vector<std::vector<int>> abar(N, std::vector<int>(K, 0));
        std::vector<std::vector<int>> xbar(N, std::vector<int>(N, 0));

        for (Node i = 1; i <= inst_.n; ++i)
        {
            tbar[i] = getSolution(data_.t[i]) > 0.5 ? 1 : 0;
            ybar[i] = getSolution(data_.y[i]) > 0.5 ? 1 : 0;
        }

        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (i == j)
                    continue;
                if (i == inst_.end_depot)
                    continue;
                if (j == inst_.start_depot)
                    continue;

                xbar[i][j] = getSolution(data_.x[i][j]) > 0.5 ? 1 : 0;
            }
        }

        if (SeparateIntegerTruckSubtourCuts(tbar, xbar))
        {
            return;
        }

        const double master_z = getSolution(data_.z);
        const double zD_master = getSolution(data_.zD);
        const double truck_value = ComputeTruckValue(xbar);

        if (std::isfinite(best_valid_ub_) &&
            truck_value >= best_valid_ub_ - 1e-6)
        {
            if (options_.verbose)
            {
                std::cout << "\n[REJECT] truck_value >= valid UB"
                          << ", master_z=" << master_z
                          << ", truck=" << truck_value
                          << ", UB=" << best_valid_ub_
                          << "\n";
            }
            return;
        }

        std::vector<Node> drone_customers;
        for (Node i = 1; i <= inst_.n; ++i)
        {
            if (ybar[i] == 1)
            {
                drone_customers.push_back(i);
            }
        }

        if (options_.verbose)
        {
            std::cout << "\n[MIPSOL] slave-assign master_z=" << master_z
                      << ", truck=" << truck_value
                      << ", zD_master=" << zD_master
                      << ", |S|=" << drone_customers.size()
                      << ", UB=" << best_valid_ub_
                      << "\n";
        }

        DroneSchedule schedule;
        schedule.chains.assign(K, {});
        schedule.swap_before.assign(K, {});
        schedule.makespan = 0.0;
        schedule.swap_count = 0;

        std::vector<double> drone_values(K, 0.0);
        std::vector<int> swap_counts(K, 0);
        std::vector<std::vector<Node>> per_drone_customers(K);

        SlaveResult r;

        if (static_cast<int>(drone_customers.size()) < K)
        {
            // User-requested guard for the K-route slave: if fewer than K drone
            // customers are selected, do not build the slave. Return a zero
            // drone makespan. In the intended benchmark setting, |S| >= K.
            r.feasible = true;
            r.optimal = true;
            r.objective = 0;
            r.lower_bound = 0;
            r.lower_bound_valid = true;

            if (options_.verbose)
            {
                std::cout << "  [SLAVE-SKIP] |S| < K; return drone objective 0"
                          << ", |S|=" << drone_customers.size()
                          << ", K=" << K << "\n";
            }
        }
        else
        {
            SlaveOptions slave_opts = options_.slave_options;

            int64_t slave_cutoff = -1;
            if (std::isfinite(best_valid_ub_))
            {
                slave_cutoff =
                    static_cast<int64_t>(std::floor(best_valid_ub_ - 1.0 + 1e-9));

                if (slave_opts.makespan_cutoff >= 0)
                {
                    slave_opts.makespan_cutoff =
                        std::min<int64_t>(slave_opts.makespan_cutoff, slave_cutoff);
                }
                else
                {
                    slave_opts.makespan_cutoff = slave_cutoff;
                }
            }

            PrintSlaveCall(-1, drone_customers, slave_opts.makespan_cutoff);
            r = slave_.Solve(drone_customers, slave_opts);
            PrintSlaveResult(-1, drone_customers, slave_opts.makespan_cutoff, r);
        }

        if (!r.feasible)
        {
            if (r.proven_infeasible)
            {
                AddGlobalDroneNoGoodCut(drone_customers);
            }
            else if (r.lower_bound_valid)
            {
                AddGlobalDroneSubsetLowerBoundCut(drone_customers, static_cast<double>(r.lower_bound));
            }
            else
            {
                std::cerr << "\n[ERROR] slave-assign slave returned UNKNOWN without a valid bound.\n"
                          << "        Increase slave_time or use an exact slave.\n";
                abort();
            }
            return;
        }

        schedule.makespan = static_cast<double>(r.objective);

        const int chains_to_copy = std::min<int>(K, static_cast<int>(r.chains.size()));
        for (int k = 0; k < chains_to_copy; ++k)
        {
            schedule.chains[k] = r.chains[k];
            per_drone_customers[k] = r.chains[k];

            if (k < static_cast<int>(r.swap_before.size()))
            {
                schedule.swap_before[k] = r.swap_before[k];
            }

            for (Node i : per_drone_customers[k])
            {
                if (i >= 1 && i < N)
                {
                    abar[i][k] = 1;
                }
            }

            for (int v : schedule.swap_before[k])
            {
                if (v != 0)
                {
                    schedule.swap_count++;
                    swap_counts[k]++;
                }
            }
        }

        if (r.optimal)
        {
            if (zD_master + 1e-6 < static_cast<double>(r.objective))
            {
                AddGlobalDroneSubsetLowerBoundCut(drone_customers, static_cast<double>(r.objective));
            }
        }
        else if (r.lower_bound_valid)
        {
            if (zD_master + 1e-6 < static_cast<double>(r.lower_bound))
            {
                AddGlobalDroneSubsetLowerBoundCut(drone_customers, static_cast<double>(r.lower_bound));
            }
        }

        const double accepted_z = std::max(truck_value, schedule.makespan);
        const double submitted_z = std::max(master_z, accepted_z);

        if (options_.verbose)
        {
            std::cout << "  [SLAVE-SUMMARY] mode=slave-assign"
                      << ", truck=" << truck_value
                      << ", drone_makespan=" << schedule.makespan
                      << ", accepted_z=" << accepted_z
                      << ", submitted_master_z=" << submitted_z
                      << ", master_z=" << master_z
                      << ", current_UB=" << best_valid_ub_
                      << "\n";
        }

        TryAccept(
            tbar,
            abar,
            xbar,
            per_drone_customers,
            schedule,
            drone_values,
            swap_counts,
            truck_value,
            schedule.makespan,
            accepted_z,
            submitted_z);
    }

    double MasterCallback::InitialValidUB() const
    {
        return initial_valid_ub_;
    }

    double MasterCallback::BestMasterLBSeen() const
    {
        return best_master_lb_seen_;
    }

    double MasterCallback::TimeToBestSolutionSec() const
    {
        return time_to_best_solution_sec_;
    }

    double MasterCallback::TimeToOnePercentGapSec() const
    {
        return time_to_1pct_gap_sec_;
    }

    double MasterCallback::FirstOnePercentGapValue() const
    {
        return first_1pct_gap_value_;
    }

    double MasterCallback::FirstOnePercentGapBound() const
    {
        return first_1pct_gap_bound_;
    }

    int MasterCallback::ValidIncumbentUpdates() const
    {
        return valid_incumbent_updates_;
    }

} // namespace bbcp
