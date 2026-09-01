#pragma once
#include "UnrealVoxelSim/Movement/Api/GroundedProfile.h"
#include "UnrealVoxelSim/Navigation/Api/IPlanner.h"
#include "UnrealVoxelSim/Navigation/Api/IReachability.h"
#include "UnrealVoxelSim/Navigation/Voxel/IEnvironment.h"
#include "UnrealVoxelSim/Profiling/Api/IRecorder.h"
#include "UnrealVoxelSim/Simulation/Api/IStepParticipant.h"
#include <cstddef>
#include <memory>
#include <span>

namespace UnrealVoxelSim::Navigation::Voxel
{
	class PlannerState;

	class Planner final : public UnrealVoxelSim::Navigation::Api::IPlanner,
	                      public UnrealVoxelSim::Navigation::Api::IReachability,
	                      public Simulation::Api::IStepParticipant
	{
	public:
		static constexpr std::size_t DefaultExpansionsPerTick = 768;
		static constexpr std::size_t DefaultMaximumExpansionsPerRequest = 65'536;
		static constexpr std::size_t DefaultReachabilityComponentExpansionsPerTick = 32;
		static constexpr std::size_t DefaultTileBuildsPerTopologyUpdate = 16;
		static constexpr std::size_t DefaultComponentCellsPerTick = 256;

		Planner(const IEnvironment& environment,
		        std::span<const Movement::Api::GroundedProfile> profiles,
		        std::size_t expansionsPerTick = DefaultExpansionsPerTick,
		        std::size_t maximumExpansionsPerRequest = DefaultMaximumExpansionsPerRequest,
		        std::size_t reachabilityComponentExpansionsPerTick = DefaultReachabilityComponentExpansionsPerTick,
		        std::size_t tileBuildsPerTopologyUpdate = DefaultTileBuildsPerTopologyUpdate,
		        std::size_t componentCellsPerTick = DefaultComponentCellsPerTick);
		Planner(const IEnvironment& environment,
		        std::span<const Movement::Api::GroundedProfile> profiles,
		        UnrealVoxelSim::Profiling::Api::IRecorder& profiling,
		        std::size_t expansionsPerTick = DefaultExpansionsPerTick,
		        std::size_t maximumExpansionsPerRequest = DefaultMaximumExpansionsPerRequest,
		        std::size_t reachabilityComponentExpansionsPerTick = DefaultReachabilityComponentExpansionsPerTick,
		        std::size_t tileBuildsPerTopologyUpdate = DefaultTileBuildsPerTopologyUpdate,
		        std::size_t componentCellsPerTick = DefaultComponentCellsPerTick);
		~Planner() override;
		Planner(const Planner&) = delete;
		Planner& operator=(const Planner&) = delete;

		[[nodiscard]] std::expected<UnrealVoxelSim::Navigation::Api::PlanRequestId,
		                            UnrealVoxelSim::Navigation::Api::PlanError> BeginPathPlanning(
			UnrealVoxelSim::Navigation::Api::PlanRequest request) override;
		void CancelPathPlanning(UnrealVoxelSim::Navigation::Api::PlanRequestId request) noexcept override;
		void Step(Simulation::Api::StepContext context) override;
		[[nodiscard]] std::uint64_t GetCurrentEnvironmentRevision() const noexcept override;
		[[nodiscard]] bool IsPathCurrent(
			const UnrealVoxelSim::Navigation::Api::Path& path) const noexcept override;
		[[nodiscard]] UnrealVoxelSim::Navigation::Api::PlanState GetPlanState(
			UnrealVoxelSim::Navigation::Api::PlanRequestId request) const noexcept override;
		[[nodiscard]] std::shared_ptr<const UnrealVoxelSim::Navigation::Api::Path> GetPath(
			UnrealVoxelSim::Navigation::Api::PlanRequestId request) const noexcept override;
		[[nodiscard]] std::expected<void, UnrealVoxelSim::Navigation::Api::ReachabilityError> BeginReachabilityQuery(
			UnrealVoxelSim::Navigation::Api::ReachabilityQuery query) override;
		void CancelReachabilityQuery(UnrealVoxelSim::Navigation::Api::ReachabilityRequestId request) noexcept override;
		[[nodiscard]] std::shared_ptr<const UnrealVoxelSim::Navigation::Api::ReachabilityResult> GetReachabilityQueryResult(
			UnrealVoxelSim::Navigation::Api::ReachabilityRequestId request) const noexcept override;
		void Invalidate(std::span<const UnrealVoxelSim::Voxel::Api::Region> regions);
		void Prepare(std::span<const UnrealVoxelSim::Voxel::Api::Region> regions);

	private:
		std::unique_ptr<PlannerState> m_Impl;
	};
}
