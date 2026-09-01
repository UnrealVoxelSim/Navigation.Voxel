#pragma once

#include "UnrealVoxelSim/Events/Api/Subscription.h"
#include "UnrealVoxelSim/Voxel/Solid/Api/IChangeSource.h"

namespace UnrealVoxelSim::Navigation::Voxel
{
	class Planner;

	class SolidInvalidation final
	{
	public:
		SolidInvalidation(UnrealVoxelSim::Voxel::Solid::Api::IChangeSource& changes, Planner& planner);

	private:
		Events::Api::Subscription m_Subscription;
	};
}
