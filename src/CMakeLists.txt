add_library(${PROJECT_NAME} UnrealVoxelSim/Navigation/Voxel/Planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/public>" "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} PUBLIC UnrealVoxelSim.Navigation.Api UnrealVoxelSim.Navigation.Voxel.Api)
UnrealVoxelSimConfigureTarget(${PROJECT_NAME})
