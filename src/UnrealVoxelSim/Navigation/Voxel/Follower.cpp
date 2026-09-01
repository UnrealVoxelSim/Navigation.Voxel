#include "UnrealVoxelSim/Navigation/Voxel/Follower.h"

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <utility>

namespace UnrealVoxelSim::Navigation::Voxel
{
	Follower::Follower(Access access,
	                   Api::IPlanner& planner,
	                   Movement::Api::IIntentReceiver& movementIntent,
	                   const std::span<const Movement::Api::GroundedProfile> profiles) :
		m_Access(access),
		m_Planner(planner),
		m_MovementIntent(movementIntent),
		m_Profiles(profiles.begin(), profiles.end())
	{
		if (m_Profiles.empty() ||
			std::ranges::any_of(m_Profiles, [](const auto& profile) { return !profile.IsValid(); }))
		{
			throw std::invalid_argument{"Following requires at least one valid movement profile."};
		}
		std::ranges::sort(m_Profiles, {}, &Movement::Api::GroundedProfile::Id);
	}

	void Follower::AssertOwnerThread() const noexcept { assert(std::this_thread::get_id() == m_OwnerThread); }

	Math::Api::FixedPointScalar Follower::DiagonalComponent(const Math::Api::FixedPointScalar speed) noexcept
	{
		const auto quotient = speed.Raw() / 1414;
		const auto remainder = speed.Raw() % 1414;
		return Math::Api::FixedPointScalar::FromRaw(quotient * 1000 + remainder * 1000 / 1414);
	}

	bool Follower::Within(const Spatial::Api::Position left,
	                      const Spatial::Api::Position right,
	                      const Math::Api::FixedPointScalar tolerance) noexcept
	{
		const auto absolute = [](const std::int64_t value) { return value < 0 ? -value : value; };
		return absolute(left.X.Raw() - right.X.Raw()) <= tolerance.Raw() &&
			absolute(left.Y.Raw() - right.Y.Raw()) <= tolerance.Raw() &&
			absolute(left.Z.Raw() - right.Z.Raw()) <= tolerance.Raw();
	}

	Math::Api::FixedPointScalar Follower::VelocityToward(const std::int64_t delta,
	                                                     const Math::Api::FixedPointScalar maximum,
	                                                     const Simulation::Api::StepDuration duration) noexcept
	{
		constexpr std::int64_t NanosecondsPerSecond = 1'000'000'000;
		const auto nanoseconds = duration.Value().count();
		if (delta == 0 || nanoseconds <= 0)
		{
			return {};
		}
		const auto magnitude = static_cast<std::uint64_t>(delta < 0 ? -delta : delta);
		const auto maximumRaw = static_cast<std::uint64_t>(maximum.Raw());
		const auto maximumStep = maximumRaw / NanosecondsPerSecond * static_cast<std::uint64_t>(nanoseconds) +
			maximumRaw % NanosecondsPerSecond * static_cast<std::uint64_t>(nanoseconds) / NanosecondsPerSecond;
		if (magnitude >= maximumStep)
		{
			return Math::Api::FixedPointScalar::FromRaw(delta < 0 ? -maximum.Raw() : maximum.Raw());
		}
		const auto numerator = magnitude * NanosecondsPerSecond + static_cast<std::uint64_t>(nanoseconds) - 1;
		const auto required = std::min(maximumRaw, numerator / static_cast<std::uint64_t>(nanoseconds));
		return Math::Api::FixedPointScalar::FromRaw(delta < 0
			                                            ? -static_cast<std::int64_t>(required)
			                                            : static_cast<std::int64_t>(required));
	}

	const Movement::Api::GroundedProfile* Follower::Profile(const Movement::Api::ProfileId id) const noexcept
	{
		const auto iterator = std::ranges::lower_bound(m_Profiles, id, {}, &Movement::Api::GroundedProfile::Id);
		return iterator != m_Profiles.end() && iterator->Id == id ? &*iterator : nullptr;
	}

	void Follower::Begin(FollowingStateComponent& following,
	                     Api::ExecutionStateComponent& execution,
	                     const Spatial::Api::Position& position,
	                     const Movement::Api::ProfileId profile,
	                     const Api::ExecutionState pendingState)
	{
		m_Planner.CancelPathPlanning(following.Request);
		following.Request = {};
		following.Path.reset();
		following.Waypoint = 0;
		const auto result = m_Planner.BeginPathPlanning({profile, position, following.Goal.Location});
		if (result)
		{
			following.Request = *result;
			execution.State = pendingState;
		}
		else
		{
			execution.State = Api::ExecutionState::Unreachable;
		}
	}

	std::expected<void, Api::NavigationError> Follower::BeginNavigateToGoal(const Ecs::Api::EntityId entity, const Api::Goal goal)
	{
		AssertOwnerThread();
		if (!entity.IsValid() || !m_Access.IsAlive(entity))
		{
			return std::unexpected{Api::NavigationError::InvalidEntity};
		}
		if (goal.ArrivalRadius.Raw() <= 0)
		{
			return std::unexpected{Api::NavigationError::InvalidGoal};
		}

		if (!m_Access.Contains<Api::ExecutionStateComponent>(entity))
		{
			if (!m_Access.Assign(entity, Api::ExecutionStateComponent{}) ||
				!m_Access.Assign(entity, FollowingStateComponent{goal}))
			{
				throw std::logic_error{"Navigation execution components could not be attached."};
			}
		}
		else if (!m_Access.Contains<FollowingStateComponent>(entity))
		{
			throw std::logic_error{"Navigation execution state is incomplete."};
		}

		auto& execution = m_Access.Get<Api::ExecutionStateComponent>(entity)->get();
		auto& following = m_Access.Get<FollowingStateComponent>(entity)->get();
		m_Planner.CancelPathPlanning(following.Request);
		following.Goal = goal;
		const auto position = m_Access.Get<Spatial::Api::PositionComponent>(entity);
		const auto profile = m_Access.Get<Movement::Api::ProfileComponent>(entity);
		if (position && profile)
		{
			Begin(following, execution, position->get().Value, profile->get().Profile);
		}
		else
		{
			execution.State = Api::ExecutionState::Unreachable;
		}
		return {};
	}

	void Follower::CancelNavigateToGoal(const Ecs::Api::EntityId entity) noexcept
	{
		AssertOwnerThread();
		if (!m_Access.IsAlive(entity))
		{
			return;
		}
		const auto execution = m_Access.Get<Api::ExecutionStateComponent>(entity);
		const auto following = m_Access.Get<FollowingStateComponent>(entity);
		if (!execution || !following)
		{
			return;
		}
		m_Planner.CancelPathPlanning(following->get().Request);
		execution->get().State = Api::ExecutionState::Cancelled;
		following->get().Path.reset();
	}

	void Follower::Step(const Simulation::Api::StepContext context)
	{
		AssertOwnerThread();
		m_Access.ForEach(
			Query{},
			[this, context](const Ecs::Api::EntityId entity,
			                const Spatial::Api::PositionComponent& positionComponent,
			                const Movement::Api::ProfileComponent& profileComponent,
			                Api::ExecutionStateComponent& execution,
			                FollowingStateComponent& following)
			{
				if (execution.State == Api::ExecutionState::Planning ||
					execution.State == Api::ExecutionState::Replanning)
				{
					const auto state = m_Planner.GetPlanState(following.Request);
					if (state == Api::PlanState::Complete)
					{
						following.Path = m_Planner.GetPath(following.Request);
						following.Waypoint = following.Path && following.Path->Waypoints.size() > 1 ? 1 : 0;
						execution.State =
							following.Path ? Api::ExecutionState::Following : Api::ExecutionState::Unreachable;
					}
					else if (state == Api::PlanState::Unreachable || state == Api::PlanState::Cancelled)
					{
						execution.State = Api::ExecutionState::Unreachable;
					}
				}
				if (execution.State != Api::ExecutionState::Following || !following.Path)
				{
					return;
				}
				const auto* profile = Profile(profileComponent.Profile);
				if (!profile)
				{
					execution.State = Api::ExecutionState::Unreachable;
					return;
				}
				const auto tolerance =
					Math::Api::FixedPointScalar::FromRaw(std::max<std::int64_t>(1, profile->CollisionSkin.Raw() / 2));
				if (!m_Planner.IsPathCurrent(*following.Path))
				{
					Begin(following,
					      execution,
					      positionComponent.Value,
					      profileComponent.Profile,
					      Api::ExecutionState::Replanning);
					return;
				}
				while (
					following.Waypoint < following.Path->Waypoints.size() &&
					Within(positionComponent.Value, following.Path->Waypoints[following.Waypoint].Location, tolerance))
				{
					++following.Waypoint;
				}
				if (following.Waypoint >= following.Path->Waypoints.size())
				{
					execution.State = Api::ExecutionState::Arrived;
					if (!m_MovementIntent.SetIntent(entity, context.Tick, {}))
					{
						throw std::logic_error{"Following could not clear movement intent."};
					}
					return;
				}
				const auto& waypoint = following.Path->Waypoints[following.Waypoint];
				const auto xDelta = waypoint.Location.X.Raw() - positionComponent.Value.X.Raw();
				const auto yDelta = waypoint.Location.Y.Raw() - positionComponent.Value.Y.Raw();
				const auto sign = [](const std::int64_t value) { return value < 0 ? -1 : value > 0 ? 1 : 0; };
				const auto component = sign(xDelta) != 0 && sign(yDelta) != 0
					                       ? DiagonalComponent(profile->MaximumSpeed)
					                       : profile->MaximumSpeed;
				const auto jump = waypoint.Primitive == Api::StandardPrimitives::Rise &&
					positionComponent.Value.Z.Raw() + tolerance.Raw() < waypoint.Location.Z.Raw();
				const Movement::Api::Intent intent{
					{
						VelocityToward(xDelta, component, context.Duration),
						VelocityToward(yDelta, component, context.Duration),
						{}
					},
					jump,
				};
				if (!m_MovementIntent.SetIntent(entity, context.Tick, intent))
				{
					throw std::logic_error{"Following produced invalid movement intent."};
				}
			});
	}
} // namespace UnrealVoxelSim::Navigation::Voxel
