cmake_minimum_required(VERSION 3.30)
cmake_policy(SET CMP0168 NEW)
project(UnrealVoxelSim.Navigation.Voxel VERSION 0.1.0 LANGUAGES CXX)
option(UNREALVOXELSIM_ENABLE_PROFILING "Enable backend-neutral profiling instrumentation." OFF)
include(FetchContent)
set(UNREALVOXELSIM_BUILD_CMAKE_SOURCE "" CACHE PATH "Optional local source override for Build.CMake.")

# A sibling checkout is the normal development layout. Discover it automatically so
# IDE preset environments do not need machine-specific workspace variables.
get_filename_component(unrealVoxelSimWorkspaceRoot "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(unrealVoxelSimSiblingBuildCMake "${unrealVoxelSimWorkspaceRoot}/Build.CMake")
if(
    NOT UNREALVOXELSIM_BUILD_CMAKE_SOURCE
    AND EXISTS "${unrealVoxelSimSiblingBuildCMake}/UnrealVoxelSim.Build.CMake.cmake"
)
    set(
        UNREALVOXELSIM_BUILD_CMAKE_SOURCE
        "${unrealVoxelSimSiblingBuildCMake}"
        CACHE PATH
        "Optional local source override for Build.CMake."
        FORCE
    )
endif()
if(
    NOT UNREALVOXELSIM_INTERNAL_SOURCE_ROOT
    AND EXISTS "${unrealVoxelSimWorkspaceRoot}/Build.CMake/CMakeLists.txt"
)
    set(
        UNREALVOXELSIM_INTERNAL_SOURCE_ROOT
        "${unrealVoxelSimWorkspaceRoot}"
        CACHE PATH
        "Optional directory containing local UnrealVoxelSim repository checkouts."
        FORCE
    )
endif()
if(UNREALVOXELSIM_BUILD_CMAKE_SOURCE)
    FetchContent_Declare(uvsbuild SOURCE_DIR "${UNREALVOXELSIM_BUILD_CMAKE_SOURCE}")
else()
    FetchContent_Declare(uvsbuild GIT_REPOSITORY https://github.com/UnrealVoxelSim/Build.CMake.git GIT_TAG main GIT_SHALLOW TRUE GIT_PROGRESS TRUE)
endif()
FetchContent_MakeAvailable(uvsbuild)
include("${uvsbuild_SOURCE_DIR}/UnrealVoxelSim.Build.CMake.cmake")
UnrealVoxelSimAddInternalDependency(GITHUB_REPOSITORY UnrealVoxelSim/Ecs.Api)
UnrealVoxelSimAddInternalDependency(GITHUB_REPOSITORY UnrealVoxelSim/Events.Api)
UnrealVoxelSimAddInternalDependency(GITHUB_REPOSITORY UnrealVoxelSim/Movement.Api)
UnrealVoxelSimAddInternalDependency(GITHUB_REPOSITORY UnrealVoxelSim/Navigation.Api)
UnrealVoxelSimAddInternalDependency(GITHUB_REPOSITORY UnrealVoxelSim/Profiling.Api)
UnrealVoxelSimAddInternalDependency(GITHUB_REPOSITORY UnrealVoxelSim/Simulation.Api)
UnrealVoxelSimAddInternalDependency(GITHUB_REPOSITORY UnrealVoxelSim/Voxel.Api)
UnrealVoxelSimAddInternalDependency(GITHUB_REPOSITORY UnrealVoxelSim/Voxel.Solid.Api)
if(UNREALVOXELSIM_BUILD_TESTS AND BUILD_TESTING AND CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    UnrealVoxelSimAddInternalDependency(GITHUB_REPOSITORY UnrealVoxelSim/Ecs.EnTT)
endif()
UnrealVoxelSimConfigureModule()
