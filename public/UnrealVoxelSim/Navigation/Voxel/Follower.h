#pragma once

#include "UnrealVoxelSim/Ecs/Api/Access.h"
#include "UnrealVoxelSim/Movement/Api/GroundedProfile.h"
#include "UnrealVoxelSim/Movement/Api/IIntentReceiver.h"
#include "UnrealVoxelSim/Movement/Api/ProfileComponent.h"
#include "UnrealVoxelSim/Navigation/Api/INavigation.h"
#include "UnrealVoxelSim/Navigation/Api/IPlanner.h"
#include "UnrealVoxelSim/Navigation/Api/ExecutionStateComponent.h"
#include "UnrealVoxelSim/Navigation/Voxel/FollowingStateComponent.h"
#include "UnrealVoxelSim/Simulation/Api/IStepParticipant.h"
#include "UnrealVoxelSim/Spatial/Api/PositionComponent.h"

#include <cstdint>
#include <span>
#include <thread>
#include <vector>

namespace UnrealVoxelSim::Navigation::Voxel
{
	class Follower final : public Api::INavigation, public Simulation::Api::IStepParticipant
	{
		using Query =
		Ecs::Api::Query<Ecs::Api::Read<Spatial::Api::PositionComponent, Movement::Api::ProfileComponent>,
		                Ecs::Api::Write<Api::ExecutionStateComponent,
		                                FollowingStateComponent>>;
		using Permissions = Ecs::Api::Permissions<
			Ecs::Api::Read<Spatial::Api::PositionComponent, Movement::Api::ProfileComponent>,
			Ecs::Api::Write<>,
			Ecs::Api::Structural<Api::ExecutionStateComponent, FollowingStateComponent>>;

	public:
		using Access = Ecs::Api::Access<Permissions, Query>;

		Follower(Access access,
		           Api::IPlanner& planner,
		           Movement::Api::IIntentReceiver& movementIntent,
		           std::span<const Movement::Api::GroundedProfile> profiles);

		[[nodiscard]] std::expected<void, Api::NavigationError> BeginNavigateToGoal(Ecs::Api::EntityId entity, Api::Goal goal) override;
		void CancelNavigateToGoal(Ecs::Api::EntityId entity) noexcept override;
		void Step(Simulation::Api::StepContext context) override;

	private:
		void AssertOwnerThread() const noexcept;
		[[nodiscard]] static Math::Api::FixedPointScalar DiagonalComponent(Math::Api::FixedPointScalar speed) noexcept;
		[[nodiscard]] static bool Within(Spatial::Api::Position left,
		                                 Spatial::Api::Position right,
		                                 Math::Api::FixedPointScalar tolerance) noexcept;
		[[nodiscard]] static Math::Api::FixedPointScalar VelocityToward(
			std::int64_t delta,
			Math::Api::FixedPointScalar maximum,
			Simulation::Api::StepDuration duration) noexcept;
		[[nodiscard]] const Movement::Api::GroundedProfile* Profile(Movement::Api::ProfileId id) const noexcept;
		void Begin(FollowingStateComponent& following,
		           Api::ExecutionStateComponent& execution,
		           const Spatial::Api::Position& position,
		           Movement::Api::ProfileId profile,
		           Api::ExecutionState pendingState = Api::ExecutionState::Planning);

		Access m_Access;
		Api::IPlanner& m_Planner;
		Movement::Api::IIntentReceiver& m_MovementIntent;
		std::vector<Movement::Api::GroundedProfile> m_Profiles;
		std::thread::id m_OwnerThread{std::this_thread::get_id()};
	};
} // namespace UnrealVoxelSim::Navigation::Voxel
