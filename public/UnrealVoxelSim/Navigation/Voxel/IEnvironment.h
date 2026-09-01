#pragma once

#include "UnrealVoxelSim/Navigation/Voxel/Cell.h"
#include "UnrealVoxelSim/Voxel/Api/ReadError.h"
#include "UnrealVoxelSim/Voxel/Api/Region.h"

#include <expected>
#include <span>

namespace UnrealVoxelSim::Navigation::Voxel
{
	class IEnvironment
	{
	public:
		virtual ~IEnvironment() = default;
		[[nodiscard]] virtual UnrealVoxelSim::Voxel::Api::Region GetBounds() const noexcept = 0;
		[[nodiscard]] virtual std::expected<void, UnrealVoxelSim::Voxel::Api::ReadError>
		ReadRegion(UnrealVoxelSim::Voxel::Api::Region region, std::span<Cell> output) const = 0;
	};
}
