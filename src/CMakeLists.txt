add_library(
    ${PROJECT_NAME}
    UnrealVoxelSim/Navigation/Voxel/Planner.cpp
    UnrealVoxelSim/Navigation/Voxel/Follower.cpp
    UnrealVoxelSim/Navigation/Voxel/SolidEnvironment.cpp
    UnrealVoxelSim/Navigation/Voxel/SolidInvalidation.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/public>" "$<INSTALL_INTERFACE:include>")
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
        UnrealVoxelSim.Ecs.Api
        UnrealVoxelSim.Events.Api
        UnrealVoxelSim.Movement.Api
        UnrealVoxelSim.Navigation.Api
        UnrealVoxelSim.Profiling.Api
        UnrealVoxelSim.Simulation.Api
        UnrealVoxelSim.Voxel.Api
        UnrealVoxelSim.Voxel.Solid.Api
)
if(UNREALVOXELSIM_ENABLE_PROFILING)
    target_compile_definitions(${PROJECT_NAME} PRIVATE UNREALVOXELSIM_PROFILING_ENABLED=1)
endif()
UnrealVoxelSimConfigureTarget(${PROJECT_NAME})
