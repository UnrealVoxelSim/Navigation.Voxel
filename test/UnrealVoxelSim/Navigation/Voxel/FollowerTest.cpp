#include "UnrealVoxelSim/Navigation/Voxel/Follower.h"

#include "UnrealVoxelSim/Ecs/Api/RegistryScopeId.h"
#include "UnrealVoxelSim/Ecs/EnTT/Registry.h"
#include "UnrealVoxelSim/Movement/Api/InputComponent.h"

#include <gtest/gtest.h>

#include <array>
#include <map>

namespace UnrealVoxelSim::Navigation::Voxel
{
	namespace
	{
		using Scalar = Math::Api::FixedPointScalar;

		[[nodiscard]] constexpr Spatial::Api::Position Location(const int x, const int z = 1)
		{
			return {Scalar::FromRaw(static_cast<std::int64_t>(x) * Scalar::OneRaw + Scalar::OneRaw / 2),
					Scalar::FromRaw(Scalar::OneRaw / 2),
					Scalar::FromWhole(z)};
		}

		class PlannerStub final : public Api::IPlanner
		{
		public:
			std::expected<Api::PlanRequestId, Api::PlanError> BeginPathPlanning(const Api::PlanRequest request) override
			{
				const Api::PlanRequestId id{m_Next++};
				Requests[id] = request;
				return id;
			}

			void CancelPathPlanning(const Api::PlanRequestId request) noexcept override
			{
				if (request.IsValid())
					States[request] = Api::PlanState::Cancelled;
			}

			std::uint64_t GetCurrentEnvironmentRevision() const noexcept override { return 1; }
			bool IsPathCurrent(const Api::Path&) const noexcept override { return true; }

			Api::PlanState GetPlanState(const Api::PlanRequestId request) const noexcept override
			{
				const auto iterator = States.find(request);
				return iterator == States.end() ? Api::PlanState::Complete : iterator->second;
			}

			std::shared_ptr<const Api::Path> GetPath(const Api::PlanRequestId request) const noexcept override
			{
				const auto iterator = Requests.find(request);
				if (iterator == Requests.end())
					return nullptr;
				auto path = std::make_shared<Api::Path>();
				path->EnvironmentRevision = 1;
				path->Waypoints = {{iterator->second.Start, Api::StandardPrimitives::Traverse},
								   {iterator->second.Goal, Primitive}};
				return path;
			}

			std::map<Api::PlanRequestId, Api::PlanRequest> Requests;
			std::map<Api::PlanRequestId, Api::PlanState> States;
			Api::PrimitiveId Primitive{Api::StandardPrimitives::Traverse};

		private:
			std::uint64_t m_Next{1};
		};

		class IntentReceiverStub final : public Movement::Api::IIntentReceiver
		{
		public:
			std::expected<void, Movement::Api::IntentError> SetIntent(const Ecs::Api::EntityId entity,
																	  const Simulation::Api::TickIndex tick,
																	  const Movement::Api::Intent intent) override
			{
				Inputs[entity] = {tick, intent.DesiredVelocity, intent.JumpRequested};
				return {};
			}

			std::map<Ecs::Api::EntityId, Movement::Api::InputComponent> Inputs;
		};

		TEST(FollowerTest, SubmitsIntentWithoutMovementComponentWriteAuthority)
		{
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Ecs::EnTT::Registry registry{Ecs::Api::RegistryScopeId{1}};
			const auto entity = registry.Create();
			ASSERT_TRUE(registry.Assign<Spatial::Api::PositionComponent>(entity, Location(0)));
			ASSERT_TRUE(registry.Assign<Movement::Api::ProfileComponent>(entity, profiles[0].Id));
			PlannerStub planner;
			IntentReceiverStub movementIntent;
			Follower follower{Follower::Access{registry}, planner, movementIntent, profiles};

			ASSERT_TRUE(follower.BeginNavigateToGoal(entity, {Location(3), Scalar::FromRaw(Scalar::OneRaw / 4)}));
			follower.Step({Simulation::Api::TickIndex{0}, Simulation::Api::StandardStepDuration});

			ASSERT_TRUE(movementIntent.Inputs.contains(entity));
			EXPECT_GT(movementIntent.Inputs.at(entity).DesiredVelocity.X, Scalar{});
			EXPECT_FALSE(registry.Contains<Movement::Api::InputComponent>(entity));
		}

		TEST(FollowerTest, CancellationDoesNotRequireMovementComponentAccess)
		{
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Ecs::EnTT::Registry registry{Ecs::Api::RegistryScopeId{1}};
			const auto entity = registry.Create();
			ASSERT_TRUE(registry.Assign<Spatial::Api::PositionComponent>(entity, Location(0)));
			ASSERT_TRUE(registry.Assign<Movement::Api::ProfileComponent>(entity, profiles[0].Id));
			PlannerStub planner;
			IntentReceiverStub movementIntent;
			Follower follower{Follower::Access{registry}, planner, movementIntent, profiles};
			ASSERT_TRUE(follower.BeginNavigateToGoal(entity, {Location(3), Scalar::FromRaw(Scalar::OneRaw / 4)}));

			follower.CancelNavigateToGoal(entity);

			EXPECT_EQ(registry.Get<Api::ExecutionStateComponent>(entity)->get().State, Api::ExecutionState::Cancelled);
		}

		TEST(FollowerTest, FollowsSwimmingWaypointWithVerticalVelocity)
		{
			auto profile = Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}};
			profile.SwimmingSpeed = Scalar::FromWhole(2);
			const std::array profiles{profile};
			Ecs::EnTT::Registry registry{Ecs::Api::RegistryScopeId{1}};
			const auto entity = registry.Create();
			ASSERT_TRUE(registry.Assign<Spatial::Api::PositionComponent>(entity, Location(0, 1)));
			ASSERT_TRUE(registry.Assign<Movement::Api::ProfileComponent>(entity, profiles[0].Id));
			PlannerStub planner;
			planner.Primitive = Api::StandardPrimitives::Swim;
			IntentReceiverStub movementIntent;
			Follower follower{Follower::Access{registry}, planner, movementIntent, profiles};

			ASSERT_TRUE(follower.BeginNavigateToGoal(entity, {Location(0, 3), Scalar::FromRaw(Scalar::OneRaw / 4)}));
			follower.Step({Simulation::Api::TickIndex{0}, Simulation::Api::StandardStepDuration});

			ASSERT_TRUE(movementIntent.Inputs.contains(entity));
			EXPECT_GT(movementIntent.Inputs.at(entity).DesiredVelocity.Z, Scalar{});
			EXPECT_FALSE(movementIntent.Inputs.at(entity).JumpRequested);
		}
	} // namespace
} // namespace UnrealVoxelSim::Navigation::Voxel
