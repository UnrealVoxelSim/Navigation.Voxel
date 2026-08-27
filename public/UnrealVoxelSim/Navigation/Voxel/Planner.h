#pragma once
#include "UnrealVoxelSim/Movement/Api/GroundedProfile.h"
#include "UnrealVoxelSim/Navigation/Api/IPlanner.h"
#include "UnrealVoxelSim/Navigation/Api/IReachability.h"
#include "UnrealVoxelSim/Navigation/Voxel/Api/IEnvironment.h"
#include "UnrealVoxelSim/Navigation/Voxel/Api/IInvalidationSink.h"
#include "UnrealVoxelSim/Navigation/Voxel/Api/IPreparationSink.h"
#include "UnrealVoxelSim/Navigation/Voxel/Api/ITopologyUpdater.h"
#include "UnrealVoxelSim/Profiling/Api/IRecorder.h"
#include <cstddef>
#include <memory>
#include <span>
namespace UnrealVoxelSim::Navigation::Voxel
{
class Planner final : public UnrealVoxelSim::Navigation::Api::IPlanner,
                      public UnrealVoxelSim::Navigation::Api::IReachability,
                      public UnrealVoxelSim::Navigation::Voxel::Api::IInvalidationSink,
                      public UnrealVoxelSim::Navigation::Voxel::Api::IPreparationSink,
                      public UnrealVoxelSim::Navigation::Voxel::Api::ITopologyUpdater
{
  public:
    static constexpr std::size_t DefaultExpansionsPerTick = 768;
    static constexpr std::size_t DefaultMaximumExpansionsPerRequest = 65'536;
    static constexpr std::size_t DefaultReachabilityComponentExpansionsPerTick = 32;
    static constexpr std::size_t DefaultTileBuildsPerTopologyUpdate = 16;
    static constexpr std::size_t DefaultComponentCellsPerTick = 256;

    Planner(const UnrealVoxelSim::Navigation::Voxel::Api::IEnvironment &environment,
            std::span<const Movement::Api::GroundedProfile> profiles,
            std::size_t expansionsPerTick = DefaultExpansionsPerTick,
            std::size_t maximumExpansionsPerRequest = DefaultMaximumExpansionsPerRequest,
            std::size_t reachabilityComponentExpansionsPerTick = DefaultReachabilityComponentExpansionsPerTick,
            std::size_t tileBuildsPerTopologyUpdate = DefaultTileBuildsPerTopologyUpdate,
            std::size_t componentCellsPerTick = DefaultComponentCellsPerTick);
    Planner(const UnrealVoxelSim::Navigation::Voxel::Api::IEnvironment &environment,
            std::span<const Movement::Api::GroundedProfile> profiles,
            UnrealVoxelSim::Profiling::Api::IRecorder &profiling,
            std::size_t expansionsPerTick = DefaultExpansionsPerTick,
            std::size_t maximumExpansionsPerRequest = DefaultMaximumExpansionsPerRequest,
            std::size_t reachabilityComponentExpansionsPerTick = DefaultReachabilityComponentExpansionsPerTick,
            std::size_t tileBuildsPerTopologyUpdate = DefaultTileBuildsPerTopologyUpdate,
            std::size_t componentCellsPerTick = DefaultComponentCellsPerTick);
    ~Planner() override;
    Planner(const Planner &) = delete;
    Planner &operator=(const Planner &) = delete;

    [[nodiscard]] std::expected<void, UnrealVoxelSim::Navigation::Api::PlanError> Begin(
        UnrealVoxelSim::Navigation::Api::PlanRequest request) override;
    void Cancel(UnrealVoxelSim::Navigation::Api::RequestId request) noexcept override;
    void Advance(Simulation::Api::StepContext context) override;
    [[nodiscard]] std::uint64_t CurrentEnvironmentRevision() const noexcept override;
    [[nodiscard]] bool IsPathCurrent(
        const UnrealVoxelSim::Navigation::Api::Path &path) const noexcept override;
    [[nodiscard]] UnrealVoxelSim::Navigation::Api::PlanState State(
        UnrealVoxelSim::Navigation::Api::RequestId request) const noexcept override;
    [[nodiscard]] std::shared_ptr<const UnrealVoxelSim::Navigation::Api::Path> ReadPath(
        UnrealVoxelSim::Navigation::Api::RequestId request) const noexcept override;
    [[nodiscard]] std::expected<void, UnrealVoxelSim::Navigation::Api::ReachabilityError> BeginReachability(
        UnrealVoxelSim::Navigation::Api::ReachabilityQuery query) override;
    void CancelReachability(UnrealVoxelSim::Navigation::Api::ReachabilityRequestId request) noexcept override;
    [[nodiscard]] std::shared_ptr<const UnrealVoxelSim::Navigation::Api::ReachabilityResult> ReadReachability(
        UnrealVoxelSim::Navigation::Api::ReachabilityRequestId request) const noexcept override;
    void Invalidate(std::span<const UnrealVoxelSim::Voxel::Api::Region> regions) override;
    void Prepare(std::span<const UnrealVoxelSim::Voxel::Api::Region> regions) override;
    void UpdateTopology(Simulation::Api::StepContext context) override;

  private:
    class Impl;
    std::unique_ptr<Impl> m_Impl;
};
} // namespace UnrealVoxelSim::Navigation::Voxel
