#include "UnrealVoxelSim/Navigation/Voxel/SolidInvalidation.h"

#include "UnrealVoxelSim/Navigation/Voxel/Planner.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/Changed.h"

namespace UnrealVoxelSim::Navigation::Voxel
{
	SolidInvalidation::SolidInvalidation(UnrealVoxelSim::Voxel::Solid::Api::IChangeSource& changes, Planner& planner)
	{
		m_Subscription =
			changes.Subscribe([&planner](const UnrealVoxelSim::Voxel::Solid::Api::Changed& changed) noexcept
							  { planner.Invalidate(changed.Regions); });
	}
}
