#pragma once

#include <cstdint>

namespace UnrealVoxelSim::Navigation::Voxel
{
	struct Cell final
	{
		bool BlocksOccupancy{};
		bool SupportsGroundedBody{};
		bool AllowsSwimming{};
		std::uint16_t TraversalCost{1000};
	};
}
