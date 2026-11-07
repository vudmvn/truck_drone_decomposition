"""Replace uniform-M conditional drone cuts by certified per-customer coefficients.

This module runs AFTER the existing correctness patch. It only transforms C++
code at uniquely verified anchors, and is intentionally strict on mismatches.
"""

from __future__ import annotations


def replace_once(s: str, old: str, new: str, label: str) -> str:
    n = s.count(old)
    if n != 1:
        raise ValueError(f"{label}: expected exactly one anchor; found {n}")
    return s.replace(old, new, 1)


def patch_callback(s: str) -> str:
    s = replace_once(
        s,
        '#include "bbcp/master/MasterCuts.h"',
        '#include "bbcp/master/MasterCuts.h"\n#include "bbcp/master/LipschitzCuts.h"',
        'include Lipschitz helper in callback',
    )

    # Per-drone Benders cut, broadcast over identical drones.
    s = replace_once(s,
        '''        const double local_m = MasterCuts::LocalBigM(rhs);

        GRBLinExpr missing = 0.0;
        for (Node i : S)
        {
            missing += (1.0 - data_.a[i][drone_id]);
        }

        GRBLinExpr lhs = data_.zK[drone_id] + local_m * missing;
        addLazy(lhs >= rhs);''',
        '''        // phi(S) <= phi(S\\{i}) + (r_i + tau). For every certified
        // L(S)<=phi(S), the per-item reverse-Lipschitz coefficient is valid.
        GRBLinExpr lhs = data_.zK[drone_id];
        for (Node i : S)
        {
            const double delta = LipschitzCuts::MakespanDrop(inst_, i, rhs);
            lhs += delta * (1.0 - data_.a[i][drone_id]);
        }
        addLazy(lhs >= rhs);''',
        'per-drone certified LBBD Lipschitz')
    s = replace_once(s,
        '''                      << ", rhs=" << rhs
                      << ", M=" << local_m << "\\n";''',
        '''                      << ", rhs=" << rhs
                      << ", coefficients=per-customer-lipschitz\\n";''',
        'per-drone LBBD logging')

    # Global K-drone LBBD cut.
    s = replace_once(s,
        '''        const double local_m = MasterCuts::LocalBigM(rhs);

        GRBLinExpr missing = 0.0;
        for (Node i : S)
        {
            missing += (1.0 - data_.y[i]);
        }

        GRBLinExpr lhs = data_.zD + local_m * missing;
        addLazy(lhs >= rhs);''',
        '''        // Inserting a customer into an existing drone schedule requires
        // at most its round-trip time and one additional battery swap.
        GRBLinExpr lhs = data_.zD;
        for (Node i : S)
        {
            const double delta = LipschitzCuts::MakespanDrop(inst_, i, rhs);
            lhs += delta * (1.0 - data_.y[i]);
        }
        addLazy(lhs >= rhs);''',
        'global K-drone certified LBBD Lipschitz')
    s = replace_once(s,
        '''                      << ", rhs=" << rhs
                      << ", S=" << NodeListString(S)''',
        '''                      << ", rhs=" << rhs
                      << ", coefficients=per-customer-lipschitz"
                      << ", S=" << NodeListString(S)''',
        'global LBBD logging')

    # Aggregate energy-cover: removing one task reduces a required battery
    # cycle count by at most one (because each e_i <= capacity). Big-R can
    # therefore be replaced by coefficient 1 for every missing customer.
    s = replace_once(s,
        '''            const double lhs_rel =
                Qrel + static_cast<double>(R) *
                           (static_cast<double>(H.size()) - sum_y);''',
        '''            const double lhs_rel =
                Qrel + static_cast<double>(H.size()) - sum_y;''',
        'global energy-cover unit removal coefficients')
    s = replace_once(s,
        '''                GRBLinExpr lhs = data_.Q +
                                 static_cast<double>(R) * static_cast<double>(H.size());
                for (Node i : H)
                {
                    lhs -= static_cast<double>(R) * data_.y[i];
                }''',
        '''                GRBLinExpr lhs = data_.Q;
                for (Node i : H)
                {
                    lhs += 1.0 - data_.y[i];
                }''',
        'global energy-cover formulation')

    # The bound F(H)=sum r_i+tau max{0,ceil(sum e_i/B)-K} drops by at most
    # r_i+tau upon removing i. The K*zD scale is kept untouched.
    s = replace_once(s,
        '''            const double rhs = sum_roundtrip + tau * static_cast<double>(R);
            const double M = std::max(1.0, rhs);
            const double lhs_rel =
                K * zDrel + M * (static_cast<double>(H.size()) - sum_y);''',
        '''            const double rhs = sum_roundtrip + tau * static_cast<double>(R);
            double lhs_rel = K * zDrel;
            for (Node i : H)
            {
                lhs_rel += LipschitzCuts::MakespanDrop(inst_, i, rhs) *
                           (1.0 - yrel[i]);
            }''',
        'global fractional workload Lipschitz violation')
    s = replace_once(s,
        '''                GRBLinExpr lhs = K * data_.zD + M * static_cast<double>(H.size());
                for (Node i : H)
                {
                    lhs -= M * data_.y[i];
                }''',
        '''                GRBLinExpr lhs = K * data_.zD;
                for (Node i : H)
                {
                    lhs += LipschitzCuts::MakespanDrop(inst_, i, rhs) *
                           (1.0 - data_.y[i]);
                }''',
        'global fractional workload Lipschitz formulation')

    # Explicit per-drone energy-cover, also exact coefficient 1 per missing task.
    s = replace_once(s,
        '''            const double lhs_value = qrel + R * (static_cast<double>(H.size()) - sum_a);''',
        '''            const double lhs_value = qrel + static_cast<double>(H.size()) - sum_a;''',
        'explicit energy-cover violation')
    s = replace_once(s,
        '''                    lhs += R * (1.0 - data_.a[i][drone_id]);''',
        '''                    lhs += 1.0 - data_.a[i][drone_id];''',
        'explicit energy-cover formulation')

    # Per-drone aggregate workload inequality and separation at fractional nodes.
    s = replace_once(s,
        '''            const double M = std::max(1.0, rhs);
            const double lhs_value =
                zKrel + M * (static_cast<double>(H.size()) - sum_a);''',
        '''            double lhs_value = zKrel;
            for (Node i : H)
            {
                lhs_value += LipschitzCuts::MakespanDrop(inst_, i, rhs) *
                             (1.0 - arel[i][drone_id]);
            }''',
        'explicit fractional workload Lipschitz violation')
    s = replace_once(s,
        '''                    lhs += M * (1.0 - data_.a[i][drone_id]);''',
        '''                    lhs += LipschitzCuts::MakespanDrop(inst_, i, rhs) *
                           (1.0 - data_.a[i][drone_id]);''',
        'explicit fractional workload Lipschitz formulation')

    # Update stale Big-M descriptions from the old code. These strings are
    # comments only; mathematical inequalities now all use per-customer terms.
    s = s.replace('''        //     K zD + M (|H| - sum_i y_i) >= sum_i r_i + tau R.''',
                  '''        //     K zD + sum_i delta_i (1-y_i) >= sum_i r_i + tau R.''')
    s = s.replace('''        //   q_k + R(|H| - sum_{i in H} a_ik) >= R,''',
                  '''        //   q_k + sum_{i in H}(1-a_ik) >= R,''')
    s = s.replace('''        //   z_k + M(|H| - sum_{i in H} a_ik)''',
                  '''        //   z_k + sum_{i in H} delta_i(1-a_ik)''')
    s = s.replace('''        // relaxes the cut.''',
                  '''        // relaxes the cut by at most one battery cycle per missing task.''')
    return s


def patch_builder(s: str) -> str:
    s = replace_once(s,
        '#include "bbcp/master/MasterModelBuilder.h"',
        '#include "bbcp/master/MasterModelBuilder.h"\n#include "bbcp/master/LipschitzCuts.h"',
        'include Lipschitz helper in master builder')

    s = replace_once(s,
        '''                    const double M = std::max(1.0, rhs);

                    model.addConstr(
                        data.zK[k] + M * (2.0 - data.a[i][k] - data.a[j][k]) >= rhs,''',
        '''                    // A pair needs at most one swap. Removing one member
                    // eliminates that member's flight time and, if needed,
                    // the single swap between the pair.
                    const double delta_i = std::min(rhs,
                        static_cast<double>(inst_.DroneRoundTripTime(i)) +
                        static_cast<double>(inst_.swap_time) * needs_swap);
                    const double delta_j = std::min(rhs,
                        static_cast<double>(inst_.DroneRoundTripTime(j)) +
                        static_cast<double>(inst_.swap_time) * needs_swap);

                    model.addConstr(
                        data.zK[k] + delta_i * (1.0 - data.a[i][k]) +
                        delta_j * (1.0 - data.a[j][k]) >= rhs,''',
        'static pair lower-bound Lipschitz')
    return s


def patch_master_cuts_header(s: str) -> str:
    s = replace_once(s,
        '        static double LocalBigM(double rhs);\n\n',
        '',
        'remove unused uniform Big-M helper declaration')
    return s


def patch_master_cuts_impl(s: str) -> str:
    s = replace_once(s,
        '''    double MasterCuts::LocalBigM(double rhs)
    {
        return std::max(1.0, rhs);
    }

''',
        '',
        'remove unused uniform Big-M helper definition')
    s = replace_once(s,
        '''        oss << "z_" << cut.drone_id
            << " + M(" << cut.customers.size()
            << " - sum_{i in S} a_i_" << cut.drone_id
            << ") >= " << cut.rhs''',
        '''        oss << "z_" << cut.drone_id
            << " + sum_{i in S} delta_i*(1-a_i_" << cut.drone_id
            << ") >= " << cut.rhs''',
        'update MasterCuts describe text')
    return s
