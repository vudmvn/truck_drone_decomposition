#include "bbcp/core/Options.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace bbcp
{

    namespace
    {
        std::string Lower(std::string s)
        {
            std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c)
                           { return static_cast<char>(std::tolower(c)); });
            return s;
        }
    } // namespace

    SlaveSolverType ParseSlaveSolverType(const std::string &raw)
    {
        const std::string s = Lower(raw);
        if (s == "cp" || s == "cpsat" || s == "cp-sat" || s == "ortools")
        {
            return SlaveSolverType::CP;
        }
        if (s == "milp" || s == "mip" || s == "gurobi")
        {
            return SlaveSolverType::MILP;
        }
        if (s == "single" || s == "single-drone" || s == "single-drone-ordering" || s == "ordering")
        {
            return SlaveSolverType::SingleDroneOrdering;
        }
        throw std::runtime_error("Unknown slave solver type: " + raw);
    }

    std::string SlaveSolverTypeName(SlaveSolverType type)
    {
        switch (type)
        {
        case SlaveSolverType::CP:
            return "cp";
        case SlaveSolverType::MILP:
            return "milp";
        case SlaveSolverType::SingleDroneOrdering:
            return "single-drone-ordering";
        }
        return "unknown";
    }

    MasterSubtourMode ParseMasterSubtourMode(const std::string &raw)
    {
        const std::string s = Lower(raw);

        if (s == "mtz")
        {
            return MasterSubtourMode::MTZ;
        }

        if (s == "sec" || s == "subtour" || s == "subtour-elimination" ||
            s == "cut" || s == "lazy" || s == "lazycut")
        {
            return MasterSubtourMode::SEC;
        }

        throw std::runtime_error("Unknown master subtour mode: " + raw);
    }

    std::string MasterSubtourModeName(MasterSubtourMode mode)
    {
        switch (mode)
        {
        case MasterSubtourMode::MTZ:
            return "mtz";
        case MasterSubtourMode::SEC:
            return "sec";
        }
        return "unknown";
    }

    MasterAssignmentMode ParseMasterAssignmentMode(const std::string &raw)
    {
        const std::string s = Lower(raw);

        if (s == "explicit" || s == "master-assign" || s == "master_assign" ||
            s == "aik" || s == "a_i_k")
        {
            return MasterAssignmentMode::Explicit;
        }

        if (s == "slave-assign" || s == "slave_assign" || s == "slaveassign" ||
            s == "compact" || s == "y-only" || s == "yonly")
        {
            return MasterAssignmentMode::SlaveAssign;
        }

        throw std::runtime_error("Unknown master assignment mode: " + raw);
    }

    std::string MasterAssignmentModeName(MasterAssignmentMode mode)
    {
        switch (mode)
        {
        case MasterAssignmentMode::Explicit:
            return "explicit";
        case MasterAssignmentMode::SlaveAssign:
            return "slave-assign";
        }
        return "unknown";
    }

    BranchingStrategy ParseBranchingStrategy(const std::string &raw)
    {
        const std::string s = Lower(raw);
        if (s == "default" || s == "none" || s == "off")
            return BranchingStrategy::Default;
        if (s == "assignment" || s == "assignment-first")
            return BranchingStrategy::AssignmentFirst;
        if (s == "cp" || s == "cp-aware" || s == "cpaware")
            return BranchingStrategy::CPAware;
        if (s == "ambiguity" || s == "ambiguity-first")
            return BranchingStrategy::AmbiguityFirst;
        if (s == "benefit" || s == "drone-benefit")
            return BranchingStrategy::DroneBenefit;
        if (s == "hybrid" || s == "mix")
            return BranchingStrategy::Hybrid;
        throw std::runtime_error("Unknown branching strategy: " + raw);
    }

    std::string BranchingStrategyName(BranchingStrategy strategy)
    {
        switch (strategy)
        {
        case BranchingStrategy::Default:
            return "default";
        case BranchingStrategy::AssignmentFirst:
            return "assignment-first";
        case BranchingStrategy::CPAware:
            return "cp-aware";
        case BranchingStrategy::AmbiguityFirst:
            return "ambiguity-first";
        case BranchingStrategy::DroneBenefit:
            return "drone-benefit";
        case BranchingStrategy::Hybrid:
            return "hybrid";
        }
        return "unknown";
    }

} // namespace bbcp
