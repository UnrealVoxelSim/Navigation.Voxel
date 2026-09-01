#include "UnrealVoxelSim/Navigation/Voxel/SolidEnvironment.h"

#include <algorithm>
#include <cassert>
#include <stdexcept>

namespace UnrealVoxelSim::Navigation::Voxel
{
	SolidEnvironment::SolidEnvironment(
		const UnrealVoxelSim::Voxel::Api::IBounds& bounds,
		const UnrealVoxelSim::Voxel::Solid::Api::IRegionReader& reader,
		const std::span<const UnrealVoxelSim::Voxel::Solid::Api::MaterialTraversal> traversal) :
		m_Bounds(bounds), m_Reader(reader), m_Traversal(traversal.begin(), traversal.end())
	{
		if (std::ranges::any_of(m_Traversal, [](const auto& value) { return !value.IsValid(); }))
		{
			throw std::invalid_argument{"Navigation material traversal definitions must be valid."};
		}
		std::ranges::sort(m_Traversal, {}, &UnrealVoxelSim::Voxel::Solid::Api::MaterialTraversal::Material);
		if (std::ranges::adjacent_find(
				m_Traversal, {}, &UnrealVoxelSim::Voxel::Solid::Api::MaterialTraversal::Material) != m_Traversal.end())
		{
			throw std::invalid_argument{"Navigation material traversal definitions must be unique."};
		}
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
		std::ranges::transform(
			m_Scratch,
			output.begin(),
			[this](const UnrealVoxelSim::Voxel::Solid::Api::Cell cell)
			{
				if (cell.IsEmpty())
				{
					return Cell{};
				}
				const auto iterator = std::ranges::lower_bound(
					m_Traversal, cell.Material(), {}, &UnrealVoxelSim::Voxel::Solid::Api::MaterialTraversal::Material);
				if (iterator == m_Traversal.end() || iterator->Material != cell.Material())
				{
					return Cell{true, true, false, 1000};
				}
				return Cell{iterator->BlocksOccupancy,
							iterator->SupportsGroundedBody,
							iterator->AllowsSwimming,
							iterator->TraversalCost};
			});
		return {};
	}
}
