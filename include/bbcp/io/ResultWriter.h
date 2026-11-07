#pragma once

#include <string>

#include "bbcp/core/Instance.h"
#include "bbcp/core/Options.h"
#include "bbcp/core/Solution.h"

namespace bbcp
{

    class ResultWriter
    {
    public:
        static void WriteCSV(
            const std::string &path,
            const Instance &inst,
            const MasterOptions &options,
            const FullSolution &solution);

        static void WriteSQLite(
            const std::string &db_path,
            const Instance &inst,
            const MasterOptions &options,
            const FullSolution &solution);

        static void WriteSolutionText(
            const std::string &path,
            const FullSolution &solution);

        static void WriteSolutionText(
            const std::string &path,
            const Instance &inst,
            const FullSolution &solution);

        static void PrintSolutionText(
            const FullSolution &solution);
    };

} // namespace bbcp
