#pragma once

#include "UnrealVoxelSim/Navigation/Voxel/IEnvironment.h"
#include "UnrealVoxelSim/Voxel/Api/IBounds.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/IRegionReader.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/MaterialTraversal.h"

#include <span>
#include <thread>
#include <vector>

namespace UnrealVoxelSim::Navigation::Voxel
{
	class SolidEnvironment final : public IEnvironment
	{
	public:
		SolidEnvironment(const UnrealVoxelSim::Voxel::Api::IBounds& bounds,
						 const UnrealVoxelSim::Voxel::Solid::Api::IRegionReader& reader,
						 std::span<const UnrealVoxelSim::Voxel::Solid::Api::MaterialTraversal> traversal = {});

		[[nodiscard]] UnrealVoxelSim::Voxel::Api::Region GetBounds() const noexcept override;
		[[nodiscard]] std::expected<void, UnrealVoxelSim::Voxel::Api::ReadError>
		ReadRegion(UnrealVoxelSim::Voxel::Api::Region region, std::span<Cell> output) const override;

	private:
		const UnrealVoxelSim::Voxel::Api::IBounds& m_Bounds;
		const UnrealVoxelSim::Voxel::Solid::Api::IRegionReader& m_Reader;
		std::vector<UnrealVoxelSim::Voxel::Solid::Api::MaterialTraversal> m_Traversal;
		mutable std::vector<UnrealVoxelSim::Voxel::Solid::Api::Cell> m_Scratch;
		std::thread::id m_OwnerThread{std::this_thread::get_id()};
	};
}
