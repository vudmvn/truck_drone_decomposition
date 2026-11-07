cmake_minimum_required(VERSION 3.16)
project(truck_drone_decomposition CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BBCP_WITH_GUROBI "Build exact Gurobi master and MILP single-drone slave" ON)
option(BBCP_WITH_ORTOOLS "Build exact OR-Tools CP-SAT single-drone slave" ON)

if(NOT BBCP_WITH_GUROBI)
    message(FATAL_ERROR
        "BBCP_WITH_GUROBI must be ON. "
        "This project version always runs the Gurobi decomposition master; "
        "the heuristic baseline has been removed.")
endif()

set(BBCP_SOURCES
    src/main.cpp

    src/core/Instance.cpp
    src/core/Options.cpp
    src/core/Solution.cpp

    src/io/DataReader.cpp
    src/io/ResultWriter.cpp
    src/io/Logger.cpp

    src/master/MasterProblem.cpp
    src/master/MasterModelBuilder.cpp
    src/master/MasterCallback.cpp
    src/master/MasterCuts.cpp
    src/master/MasterPreprocess.cpp
    src/master/BranchingStrategy.cpp

    src/slave/CPSubproblem.cpp
    src/slave/MILPSubproblem.cpp
    src/slave/SingleDroneOrderingSlave.cpp
    src/slave/SingleDroneOrderingCPSatSlave.cpp
    src/slave/SingleDroneOrderingMILPSlave.cpp
    src/slave/SlaveSolverFactory.cpp

    src/heuristics/InitialSolution.cpp
    src/heuristics/LocalSearch.cpp
    src/heuristics/TruckTwoOpt.cpp
    src/heuristics/DroneGreedyInsert.cpp

    src/utils/RouteUtils.cpp
    src/utils/MathUtils.cpp
    src/utils/Timer.cpp
)

add_executable(truck_drone_solver ${BBCP_SOURCES})

target_include_directories(truck_drone_solver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(truck_drone_solver PRIVATE -Wall -Wextra -O2)

# ---------------------------------------------------------------------
# Gurobi
# ---------------------------------------------------------------------
target_compile_definitions(truck_drone_solver PRIVATE BBCP_WITH_GUROBI)

set(GUROBI_HOME "" CACHE PATH "Path to Gurobi installation")

if(GUROBI_HOME STREQUAL "")
    if(DEFINED ENV{GUROBI_HOME})
        set(GUROBI_HOME "$ENV{GUROBI_HOME}")
    else()
        message(FATAL_ERROR
            "Please set GUROBI_HOME, e.g. "
            "-DGUROBI_HOME=/home/orlab/gurobi1301/linux64"
        )
    endif()
endif()

set(GUROBI_INCLUDE_DIR "${GUROBI_HOME}/include")
set(GUROBI_LIB_DIR "${GUROBI_HOME}/lib")

message(STATUS "Using GUROBI_HOME=${GUROBI_HOME}")
message(STATUS "GUROBI_LIB_DIR=${GUROBI_LIB_DIR}")

target_include_directories(truck_drone_solver PRIVATE
    ${GUROBI_INCLUDE_DIR}
)

unset(GUROBI_CXX_LIB CACHE)
unset(GUROBI_MAIN_LIB CACHE)

find_library(GUROBI_CXX_LIB
    NAMES gurobi_c++ gurobi_g++8.5
    PATHS ${GUROBI_LIB_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

find_library(GUROBI_MAIN_LIB
    NAMES gurobi130 gurobi120 gurobi110 gurobi1103 gurobi100
    PATHS ${GUROBI_LIB_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

message(STATUS "GUROBI_CXX_LIB=${GUROBI_CXX_LIB}")
message(STATUS "GUROBI_MAIN_LIB=${GUROBI_MAIN_LIB}")

target_link_libraries(truck_drone_solver PRIVATE
    ${GUROBI_CXX_LIB}
    ${GUROBI_MAIN_LIB}
)

# ---------------------------------------------------------------------
# OR-Tools / CP-SAT
# ---------------------------------------------------------------------
if(BBCP_WITH_ORTOOLS)
    target_compile_definitions(truck_drone_solver PRIVATE BBCP_WITH_ORTOOLS)

    set(ORTOOLS_HOME "" CACHE PATH "Path to OR-Tools C++ installation")

    if(ORTOOLS_HOME STREQUAL "")
        if(DEFINED ENV{ORTOOLS_HOME})
            set(ORTOOLS_HOME "$ENV{ORTOOLS_HOME}")
        else()
            set(ORTOOLS_HOME "/opt/or-tools_x86_64_Debian-11_cpp_v9.9.3963")
        endif()
    endif()

    message(STATUS "Using ORTOOLS_HOME=${ORTOOLS_HOME}")

    list(APPEND CMAKE_PREFIX_PATH
        "${ORTOOLS_HOME}"
        "${ORTOOLS_HOME}/lib/cmake"
        "${ORTOOLS_HOME}/share"
    )

    if(EXISTS "${ORTOOLS_HOME}/lib/cmake/absl/abslConfig.cmake")
        set(absl_DIR "${ORTOOLS_HOME}/lib/cmake/absl" CACHE PATH "absl config directory")
    endif()

    if(EXISTS "${ORTOOLS_HOME}/lib/cmake/protobuf/protobuf-config.cmake")
        set(protobuf_DIR "${ORTOOLS_HOME}/lib/cmake/protobuf" CACHE PATH "protobuf config directory")
        set(Protobuf_DIR "${ORTOOLS_HOME}/lib/cmake/protobuf" CACHE PATH "Protobuf config directory")
    endif()

    if(EXISTS "${ORTOOLS_HOME}/lib/cmake/re2/re2Config.cmake")
        set(re2_DIR "${ORTOOLS_HOME}/lib/cmake/re2" CACHE PATH "re2 config directory")
    endif()

    if(EXISTS "${ORTOOLS_HOME}/lib/cmake/utf8_range/utf8_rangeConfig.cmake")
        set(utf8_range_DIR "${ORTOOLS_HOME}/lib/cmake/utf8_range" CACHE PATH "utf8_range config directory")
    endif()

    find_package(ortools CONFIG REQUIRED)

    message(STATUS "Found OR-Tools through CMake package")
    target_link_libraries(truck_drone_solver PRIVATE ortools::ortools)
endif()

# ---------------------------------------------------------------------
# Runtime library paths
# ---------------------------------------------------------------------
set_target_properties(truck_drone_solver PROPERTIES
    BUILD_RPATH "${GUROBI_LIB_DIR};${ORTOOLS_HOME}/lib"
    INSTALL_RPATH "${GUROBI_LIB_DIR};${ORTOOLS_HOME}/lib"
)