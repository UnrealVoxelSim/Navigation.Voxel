#include "UnrealVoxelSim/Navigation/Voxel/SolidEnvironment.h"

#include <algorithm>
#include <cassert>

namespace UnrealVoxelSim::Navigation::Voxel
{
	SolidEnvironment::SolidEnvironment(const UnrealVoxelSim::Voxel::Api::IBounds& bounds,
									   const UnrealVoxelSim::Voxel::Solid::Api::IRegionReader& reader) noexcept :
		m_Bounds(bounds), m_Reader(reader)
	{
	}

	UnrealVoxelSim::Voxel::Api::Region SolidEnvironment::GetBounds() const noexcept
	{
		assert(std::this_thread::get_id() == m_OwnerThread);
		return m_Bounds.GetBounds();
	}

	std::expected<void, UnrealVoxelSim::Voxel::Api::ReadError>
	SolidEnvironment::ReadRegion(const UnrealVoxelSim::Voxel::Api::Region region, const std::span<Cell> output) const
	{
		assert(std::this_thread::get_id() == m_OwnerThread);
		m_Scratch.resize(output.size());
		const auto result = m_Reader.ReadRegion(region, m_Scratch);
		if (!result)
			return std::unexpected{result.error()};
		std::ranges::transform(m_Scratch,
							   output.begin(),
							   [](const UnrealVoxelSim::Voxel::Solid::Api::Cell cell)
							   {
								   const auto occupied = !cell.IsEmpty();
								   return Cell{occupied, occupied, 1000};
							   });
		return {};
	}
}
