#pragma once

#include "UnrealVoxelSim/Navigation/Api/Goal.h"
#include "UnrealVoxelSim/Navigation/Api/Path.h"
#include "UnrealVoxelSim/Navigation/Api/PlanRequestId.h"

#include <cstddef>
#include <memory>

namespace UnrealVoxelSim::Navigation::Voxel
{
	// Implementation state owned structurally by the Following system.
	struct FollowingStateComponent final
	{
		Api::Goal Goal;
		Api::PlanRequestId Request;
		std::shared_ptr<const Api::Path> Path;
		std::size_t Waypoint{};
	};
} // namespace UnrealVoxelSim::GetNavigation::Following
