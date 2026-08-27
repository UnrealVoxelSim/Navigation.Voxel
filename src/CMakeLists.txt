add_library(${PROJECT_NAME} UnrealVoxelSim/Navigation/Voxel/Planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/public>" "$<INSTALL_INTERFACE:include>")
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC UnrealVoxelSim.Navigation.Api UnrealVoxelSim.Navigation.Voxel.Api UnrealVoxelSim.Profiling.Api
)
if(UNREALVOXELSIM_ENABLE_PROFILING)
    target_compile_definitions(${PROJECT_NAME} PRIVATE UNREALVOXELSIM_PROFILING_ENABLED=1)
endif()
UnrealVoxelSimConfigureTarget(${PROJECT_NAME})
