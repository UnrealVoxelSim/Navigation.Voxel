#include "UnrealVoxelSim/Navigation/Voxel/Planner.h"

#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <vector>

namespace UnrealVoxelSim::Navigation::Voxel
{
namespace
{

class Terrain final : public Api::IEnvironment
{
  public:
    explicit Terrain(const bool complex = false, const bool sealed = false, const bool pillar = false) noexcept
        : m_Complex(complex), m_Sealed(sealed), m_Pillar(pillar) {}

    [[nodiscard]] UnrealVoxelSim::Voxel::Api::Region Bounds() const noexcept override
    {
        return {{-512, -128, -16}, {513, 128, 32}};
    }

    [[nodiscard]] std::expected<void, UnrealVoxelSim::Voxel::Api::ReadError> ReadRegion(
        const UnrealVoxelSim::Voxel::Api::Region region, const std::span<Api::Cell> output) const override
    {
        const auto count = region.CellCount();
        if (!count || *count != output.size())
            return std::unexpected{UnrealVoxelSim::Voxel::Api::ReadError::OutputSizeMismatch};
        std::size_t index{};
        for (auto z = region.Min.Z; z < region.Max.Z; ++z)
            for (auto y = region.Min.Y; y < region.Max.Y; ++y)
                for (auto x = region.Min.X; x < region.Max.X; ++x)
                {
                    static_cast<void>(x);
                    static_cast<void>(y);
                    const bool wallLine = x >= -160 && x <= 160 && (x + 160) % 64 == 0;
                    const auto wallIndex = (x + 160) / 64;
                    const auto gapCenter = (wallIndex % 3 - 1) * 64;
                    const bool wall = (m_Complex || m_Sealed) && wallLine &&
                                      (m_Sealed || std::abs(y - gapCenter) > 4) && z <= 3;
                    const bool pillar = m_Pillar && x == 200 && y == 0 && z <= 16;
                    const bool occupied = z <= 0 || wall || pillar;
                    output[index++] = {occupied, occupied, 1000};
                }
        return {};
    }

  private:
    bool m_Complex{};
    bool m_Sealed{};
    bool m_Pillar{};
};

[[nodiscard]] constexpr Movement::Api::Position Location(const std::int32_t x, const std::int32_t y,
                                                         const std::int32_t z = 1)
{
    constexpr auto half = Movement::Api::Scalar::OneRaw / 2;
    return {Movement::Api::Scalar::FromRaw(static_cast<std::int64_t>(x) * Movement::Api::Scalar::OneRaw + half),
            Movement::Api::Scalar::FromRaw(static_cast<std::int64_t>(y) * Movement::Api::Scalar::OneRaw + half),
            Movement::Api::Scalar::FromWhole(z)};
}

void Advance(Planner &planner, const std::uint64_t tick)
{
    const Simulation::Api::StepContext context{Simulation::Api::TickIndex{tick},
                                                Simulation::Api::StandardStepDuration};
    planner.UpdateTopology(context);
    planner.Advance(context);
}

void RunLongRoutes(benchmark::State &state, const Terrain &terrain)
{
    const auto requestCount = static_cast<std::size_t>(state.range(0));
    const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};

    for (auto _ : state)
    {
        static_cast<void>(_);
        const auto setupStart = std::chrono::steady_clock::now();
        auto planner = std::make_unique<Planner>(terrain, profiles, Planner::DefaultExpansionsPerTick);
        for (std::size_t index = 0; index < requestCount; ++index)
        {
            const auto y = static_cast<std::int32_t>(index % 64) - 32;
            const auto begun = planner->Begin({Navigation::Api::RequestId{index + 1}, profiles[0].Id,
                                               Location(-200, y), Location(200, y)});
            if (!begun) state.SkipWithError("Path request setup failed");
        }
        const auto setupDuration = std::chrono::steady_clock::now() - setupStart;
        state.counters["setup_ms"] = std::chrono::duration<double, std::milli>(setupDuration).count();
        std::uint64_t tick{};
        std::uint64_t maximumTickIndex{};
        double maximumTickMilliseconds{};
        bool pending = true;
        while (pending)
        {
            const auto tickStart = std::chrono::steady_clock::now();
            Advance(*planner, tick++);
            const auto tickDuration = std::chrono::steady_clock::now() - tickStart;
            const auto tickMilliseconds = std::chrono::duration<double, std::milli>(tickDuration).count();
            if (tickMilliseconds > maximumTickMilliseconds)
            {
                maximumTickMilliseconds = tickMilliseconds;
                maximumTickIndex = tick - 1;
            }
            pending = false;
            for (std::size_t index = 0; index < requestCount; ++index)
                pending = pending || planner->State(Navigation::Api::RequestId{index + 1}) == Navigation::Api::PlanState::Pending;
        }
        state.counters["planner_ticks"] = static_cast<double>(tick);
        state.counters["max_tick_ms"] = maximumTickMilliseconds;
        state.counters["max_tick_index"] = static_cast<double>(maximumTickIndex);
        const auto finalPath = planner->ReadPath(Navigation::Api::RequestId{requestCount});
        if (!finalPath) state.SkipWithError("A benchmark path was not completed");
        const auto *finalPathAddress = finalPath.get();
        benchmark::DoNotOptimize(finalPathAddress);
        state.counters["waypoints"] = static_cast<double>(finalPath->Waypoints.size());
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(requestCount));
}

void FlatLongRoutes(benchmark::State &state)
{
    const Terrain terrain;
    RunLongRoutes(state, terrain);
}

void ComplexLongRoutes(benchmark::State &state)
{
    const Terrain terrain{true};
    RunLongRoutes(state, terrain);
}

void ImpossibleLongRoute(benchmark::State &state)
{
    const Terrain terrain{false, true};
    const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
    for (auto _ : state)
    {
        static_cast<void>(_);
        Planner planner{terrain, profiles};
        const auto setupStart = std::chrono::steady_clock::now();
        if (!planner.Begin({Navigation::Api::RequestId{1}, profiles[0].Id, Location(-200, 0), Location(200, 0)}))
            state.SkipWithError("Impossible path request setup failed");
        state.counters["setup_ms"] = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - setupStart).count();
        std::uint64_t tick{};
        double maximumTickMilliseconds{};
        while (planner.State(Navigation::Api::RequestId{1}) == Navigation::Api::PlanState::Pending)
        {
            const auto tickStart = std::chrono::steady_clock::now();
            Advance(planner, tick++);
            maximumTickMilliseconds = std::max(
                maximumTickMilliseconds,
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tickStart).count());
        }
        if (planner.State(Navigation::Api::RequestId{1}) != Navigation::Api::PlanState::Unreachable)
            state.SkipWithError("Impossible path did not fail");
        state.counters["planner_ticks"] = static_cast<double>(tick);
        state.counters["max_tick_ms"] = maximumTickMilliseconds;
    }
}

void BatchedReachabilityFiveHundredDestinations(benchmark::State &state)
{
    constexpr std::size_t destinationCount = 500;
    const Terrain terrain{false, true};
    const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
    std::vector<Movement::Api::Position> destinations;
    destinations.reserve(destinationCount);
    for (std::size_t index = 0; index < destinationCount; ++index)
        destinations.push_back(Location(index % 2 == 0 ? -220 + static_cast<std::int32_t>(index % 32)
                                                        : 200 + static_cast<std::int32_t>(index % 32),
                                        static_cast<std::int32_t>(index % 64) - 32));

    for (auto _ : state)
    {
        static_cast<void>(_);
        Planner planner{terrain, profiles};
        if (!planner.BeginReachability({Navigation::Api::ReachabilityRequestId{1}, profiles[0].Id,
                                        Location(-200, 0), destinations}))
            state.SkipWithError("Reachability batch setup failed");
        std::uint64_t tick{};
        double maximumTickMilliseconds{};
        while (true)
        {
            const auto tickStart = std::chrono::steady_clock::now();
            Advance(planner, tick++);
            maximumTickMilliseconds = std::max(
                maximumTickMilliseconds,
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tickStart).count());
            const auto result = planner.ReadReachability(Navigation::Api::ReachabilityRequestId{1});
            if (result && result->IsComplete())
            {
                benchmark::DoNotOptimize(result->Destinations.data());
                break;
            }
        }
        state.counters["planner_ticks"] = static_cast<double>(tick);
        state.counters["max_tick_ms"] = maximumTickMilliseconds;
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(destinationCount));
}

void ColdIsolatedPillar(benchmark::State &state)
{
    const Terrain terrain{false, false, true};
    const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
    for (auto _ : state)
    {
        static_cast<void>(_);
        Planner planner{terrain, profiles};
        if (!planner.Begin({Navigation::Api::RequestId{1}, profiles[0].Id,
                            Location(-200, 0), Location(200, 0, 17)}))
            state.SkipWithError("Pillar path request setup failed");
        std::uint64_t tick{};
        double maximumTickMilliseconds{};
        while (planner.State(Navigation::Api::RequestId{1}) == Navigation::Api::PlanState::Pending && tick < 1'000)
        {
            const auto tickStart = std::chrono::steady_clock::now();
            Advance(planner, tick++);
            maximumTickMilliseconds = std::max(
                maximumTickMilliseconds,
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tickStart).count());
        }
        if (planner.State(Navigation::Api::RequestId{1}) != Navigation::Api::PlanState::Unreachable)
            state.SkipWithError("Isolated pillar did not fail");
        state.counters["planner_ticks"] = static_cast<double>(tick);
        state.counters["max_tick_ms"] = maximumTickMilliseconds;
    }
}

BENCHMARK(FlatLongRoutes)->Arg(1)->Arg(500)->Iterations(3);
BENCHMARK(ComplexLongRoutes)->Arg(1)->Arg(100)->Arg(500)->Iterations(1);
BENCHMARK(ImpossibleLongRoute)->Iterations(3);
BENCHMARK(BatchedReachabilityFiveHundredDestinations)->Iterations(3);
BENCHMARK(ColdIsolatedPillar)->Iterations(3);

}
}
