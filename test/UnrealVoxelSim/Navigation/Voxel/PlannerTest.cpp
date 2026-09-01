#include "UnrealVoxelSim/Navigation/Voxel/Planner.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <gtest/gtest.h>

namespace UnrealVoxelSim::Navigation::Voxel
{
	namespace
	{
		class Terrain final : public IEnvironment
		{
		public:
			[[nodiscard]] UnrealVoxelSim::Voxel::Api::Region GetBounds() const noexcept override
			{
				++BoundsReads;
				return {{-64, -64, -16}, {64, 64, 32}};
			}

			[[nodiscard]] std::expected<void, UnrealVoxelSim::Voxel::Api::ReadError>
			ReadRegion(const UnrealVoxelSim::Voxel::Api::Region region, const std::span<Cell> output) const override
			{
				++RegionReads;
				const auto count = region.CellCount();
				if (!count || *count != output.size())
					return std::unexpected{UnrealVoxelSim::Voxel::Api::ReadError::OutputSizeMismatch};
				std::size_t index{};
				for (auto z = region.Min.Z; z < region.Max.Z; ++z)
					for (auto y = region.Min.Y; y < region.Max.Y; ++y)
						for (auto x = region.Min.X; x < region.Max.X; ++x)
						{
							const auto height = VerticalTileCliff ? (y == 8 ? (x < 4 ? 15 : 11) : 0)
								: DropCliff						  ? (x < 0 ? 3 : 0)
								: Ledge && x >= 2				  ? 1
																  : 0;
							const auto barrier = (SealedBarrier || Barrier) && x == 16 &&
								(SealedBarrier || std::abs(y - 20) > 2) && z <= 3;
							const auto passage = HeadBlockedPassage &&
								(x < 0 || x > 3 || z < 0 || z > 4 || z == 0 || z == 4 || (z == 3 && x <= 1) ||
								 (z == 1 && x >= 2));
							const auto water = Water && x >= 0 && x < 2 && y >= 0 && y < 2 && z >= 1 && z < 24;
							const auto occupied = !water && (HeadBlockedPassage ? passage : z <= height || barrier);
							output[index++] = {occupied, occupied, water, 1000};
						}
				return {};
			}

			bool Ledge{};
			bool Barrier{};
			bool SealedBarrier{};
			bool DropCliff{};
			bool VerticalTileCliff{};
			bool HeadBlockedPassage{};
			bool Water{};
			mutable std::size_t BoundsReads{};
			mutable std::size_t RegionReads{};
		};

		[[nodiscard]] constexpr Spatial::Api::Position
		Location(const std::int32_t x, const std::int32_t y, const std::int32_t z)
		{
			constexpr auto Half = Math::Api::FixedPointScalar::OneRaw / 2;
			return {Math::Api::FixedPointScalar::FromRaw(
						static_cast<std::int64_t>(x) * Math::Api::FixedPointScalar::OneRaw + Half),
					Math::Api::FixedPointScalar::FromRaw(
						static_cast<std::int64_t>(y) * Math::Api::FixedPointScalar::OneRaw + Half),
					Math::Api::FixedPointScalar::FromWhole(z)};
		}

		void Advance(Planner& planner, const std::uint64_t tick)
		{
			const Simulation::Api::StepContext context{Simulation::Api::TickIndex{tick},
													   Simulation::Api::StandardStepDuration};
			planner.Step(context);
		}

		TEST(PlannerTest, BoundsPendingEndpointPollingPerTick)
		{
			Terrain terrain;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles, 1};
			for (std::uint64_t index = 1; index <= 10'000; ++index)
				ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(48, 48, 1)}));

			terrain.BoundsReads = 0;
			planner.Step({Simulation::Api::TickIndex{0}, Simulation::Api::StandardStepDuration});

			EXPECT_GT(terrain.BoundsReads, 0U);
			EXPECT_LT(terrain.BoundsReads, 5'000U);
		}

		TEST(PlannerTest, FindsDeterministicEightWayPathAcrossFlatTerrain)
		{
			Terrain terrain;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles, 4096};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(12, 12, 1)}));

			for (std::uint64_t tick = 0; tick < 20 &&
				 planner.GetPlanState(Navigation::Api::PlanRequestId{1}) == Navigation::Api::PlanState::Pending;
				 ++tick)
				Advance(planner, tick);

			EXPECT_EQ(planner.GetPlanState(Navigation::Api::PlanRequestId{1}), Navigation::Api::PlanState::Complete);
			const auto path = planner.GetPath(Navigation::Api::PlanRequestId{1});
			ASSERT_NE(path, nullptr);
			ASSERT_FALSE(path->Waypoints.empty());
			EXPECT_EQ(path->Waypoints.front().Location, Location(0, 0, 1));
			EXPECT_EQ(path->Waypoints.back().Location, Location(12, 12, 1));
			EXPECT_EQ(path->EnvironmentRevision, 1U);
			EXPECT_NE(path->ValidationToken, 0U);
			EXPECT_TRUE(planner.IsPathCurrent(*path));
		}

		TEST(PlannerTest, PlansVerticalPathThroughSwimmableVoxelsForSwimmingProfile)
		{
			Terrain terrain;
			terrain.Water = true;
			auto profile = Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}};
			profile.SwimmingSpeed = Math::Api::FixedPointScalar::FromWhole(2);
			const std::array profiles{profile};
			Planner planner{terrain, profiles, 4096};
			const auto request = planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(0, 0, 20)});
			ASSERT_TRUE(request);
			for (std::uint64_t tick = 0; tick < 1'000 && planner.GetPlanState(*request) == Api::PlanState::Pending;
				 ++tick)
			{
				Advance(planner, tick);
			}
			ASSERT_EQ(planner.GetPlanState(*request), Api::PlanState::Complete);
			const auto path = planner.GetPath(*request);
			ASSERT_TRUE(path);
			EXPECT_TRUE(std::ranges::any_of(path->Waypoints,
											[](const auto& waypoint)
											{ return waypoint.Primitive == Api::StandardPrimitives::Swim; }));
			EXPECT_EQ(path->Waypoints.back().Location, Location(0, 0, 20));
		}

		TEST(PlannerTest, PathPlanningDoesNotFloodUnrelatedReachabilityTopology)
		{
			Terrain terrain;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain,
							profiles,
							4096,
							Planner::DefaultMaximumExpansionsPerRequest,
							Planner::DefaultReachabilityComponentExpansionsPerTick,
							64};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(-48, 0, 1), Location(48, 0, 1)}));

			for (std::uint64_t tick = 0; tick < 200 &&
				 planner.GetPlanState(Navigation::Api::PlanRequestId{1}) == Navigation::Api::PlanState::Pending;
				 ++tick)
				Advance(planner, tick);

			EXPECT_EQ(planner.GetPlanState(Navigation::Api::PlanRequestId{1}), Navigation::Api::PlanState::Complete);
			EXPECT_LT(terrain.RegionReads, 128U);
		}

		TEST(PlannerTest, InvalidatesOnlyPathsNearChangedVoxels)
		{
			Terrain terrain;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain,
							profiles,
							4096,
							Planner::DefaultMaximumExpansionsPerRequest,
							Planner::DefaultReachabilityComponentExpansionsPerTick,
							64};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(8, 0, 1)}));
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 32, 1), Location(8, 32, 1)}));
			for (std::uint64_t tick = 0; tick < 200; ++tick)
			{
				Advance(planner, tick);
				if (planner.GetPlanState(Navigation::Api::PlanRequestId{1}) == Navigation::Api::PlanState::Complete &&
					planner.GetPlanState(Navigation::Api::PlanRequestId{2}) == Navigation::Api::PlanState::Complete)
					break;
			}
			const auto nearPath = planner.GetPath(Navigation::Api::PlanRequestId{1});
			const auto distantPath = planner.GetPath(Navigation::Api::PlanRequestId{2});
			ASSERT_NE(nearPath, nullptr);
			ASSERT_NE(distantPath, nullptr);
			ASSERT_TRUE(planner.IsPathCurrent(*nearPath));
			ASSERT_TRUE(planner.IsPathCurrent(*distantPath));

			const std::array regions{UnrealVoxelSim::Voxel::Api::Region{{4, 0, 0}, {5, 1, 3}}};
			planner.Invalidate(regions);

			EXPECT_FALSE(planner.IsPathCurrent(*nearPath));
			EXPECT_TRUE(planner.IsPathCurrent(*distantPath));
		}

		TEST(PlannerTest, UsesConfiguredAdjacentRise)
		{
			Terrain terrain;
			terrain.Ledge = true;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles, 4096};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(4, 0, 2)}));

			for (std::uint64_t tick = 0; tick < 20 &&
				 planner.GetPlanState(Navigation::Api::PlanRequestId{1}) == Navigation::Api::PlanState::Pending;
				 ++tick)
				Advance(planner, tick);

			const auto path = planner.GetPath(Navigation::Api::PlanRequestId{1});
			ASSERT_NE(path, nullptr);
			EXPECT_TRUE(std::ranges::any_of(
				path->Waypoints,
				[](const auto& waypoint) { return waypoint.Primitive == Navigation::Api::StandardPrimitives::Rise; }));
		}

		TEST(PlannerTest, RejectsRiseWithoutLaunchHeadroom)
		{
			Terrain terrain;
			terrain.HeadBlockedPassage = true;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles, 4096};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(3, 0, 2)}));

			for (std::uint64_t tick = 0; tick < 200 &&
				 planner.GetPlanState(Navigation::Api::PlanRequestId{1}) == Navigation::Api::PlanState::Pending;
				 ++tick)
				Advance(planner, tick);

			EXPECT_EQ(planner.GetPlanState(Navigation::Api::PlanRequestId{1}), Navigation::Api::PlanState::Unreachable);
		}

		TEST(PlannerTest, HonorsDeterministicExpansionBudget)
		{
			Terrain terrain;
			terrain.Ledge = true;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles, 1};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(20, 0, 1)}));

			Advance(planner, 0);
			EXPECT_EQ(planner.GetPlanState(Navigation::Api::PlanRequestId{1}), Navigation::Api::PlanState::Pending);
			for (std::uint64_t tick = 1; tick < 100 &&
				 planner.GetPlanState(Navigation::Api::PlanRequestId{1}) == Navigation::Api::PlanState::Pending;
				 ++tick)
				Advance(planner, tick);
			EXPECT_EQ(planner.GetPlanState(Navigation::Api::PlanRequestId{1}), Navigation::Api::PlanState::Complete);
		}

		TEST(PlannerTest, UsesHierarchicalPortalThroughLongBarrier)
		{
			Terrain terrain;
			terrain.Barrier = true;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles, 4096};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(32, 0, 1)}));

			for (std::uint64_t tick = 0; tick < 200 &&
				 planner.GetPlanState(Navigation::Api::PlanRequestId{1}) == Navigation::Api::PlanState::Pending;
				 ++tick)
				Advance(planner, tick);

			const auto path = planner.GetPath(Navigation::Api::PlanRequestId{1});
			ASSERT_NE(path, nullptr);
			EXPECT_TRUE(std::ranges::any_of(
				path->Waypoints, [](const auto& waypoint) { return waypoint.Location.Y.ToDouble() >= 17.0; }));
		}

		TEST(PlannerTest, BoundedImpossibleSearchFailsDeterministically)
		{
			Terrain terrain;
			terrain.SealedBarrier = true;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles, 128, 1, Planner::DefaultReachabilityComponentExpansionsPerTick, 64};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(32, 0, 1)}));

			std::uint64_t tick{};
			while (tick < 200 &&
				   planner.GetPlanState(Navigation::Api::PlanRequestId{1}) == Navigation::Api::PlanState::Pending)
			{
				Advance(planner, tick);
				++tick;
			}

			EXPECT_EQ(planner.GetPlanState(Navigation::Api::PlanRequestId{1}), Navigation::Api::PlanState::Unreachable);
			EXPECT_LE(tick, 200U);
		}

		TEST(ReachabilityTest, BatchesReachableAndUnreachableDestinations)
		{
			Terrain terrain;
			terrain.SealedBarrier = true;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain,
							profiles,
							Planner::DefaultExpansionsPerTick,
							Planner::DefaultMaximumExpansionsPerRequest,
							Planner::DefaultReachabilityComponentExpansionsPerTick,
							64};
			ASSERT_TRUE(planner.BeginReachabilityQuery({Navigation::Api::ReachabilityRequestId{1},
														profiles[0].Id,
														Location(0, 0, 1),
														{Location(8, 0, 1), Location(32, 0, 1)}}));

			std::shared_ptr<const Navigation::Api::ReachabilityResult> result;
			for (std::uint64_t tick = 0; tick < 200; ++tick)
			{
				Advance(planner, tick);
				result = planner.GetReachabilityQueryResult(Navigation::Api::ReachabilityRequestId{1});
				if (result && result->IsComplete())
					break;
			}

			ASSERT_NE(result, nullptr);
			ASSERT_EQ(result->Destinations.size(), 2U);
			EXPECT_EQ(result->Destinations[0], Navigation::Api::ReachabilityState::Reachable);
			EXPECT_EQ(result->Destinations[1], Navigation::Api::ReachabilityState::Unreachable);
		}

		TEST(ReachabilityTest, PreservesDirectedDropSemantics)
		{
			Terrain terrain;
			terrain.DropCliff = true;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain,
							profiles,
							Planner::DefaultExpansionsPerTick,
							Planner::DefaultMaximumExpansionsPerRequest,
							Planner::DefaultReachabilityComponentExpansionsPerTick,
							64};
			ASSERT_TRUE(planner.BeginReachabilityQuery(
				{Navigation::Api::ReachabilityRequestId{1}, profiles[0].Id, Location(-2, 0, 4), {Location(2, 0, 1)}}));
			ASSERT_TRUE(planner.BeginReachabilityQuery(
				{Navigation::Api::ReachabilityRequestId{2}, profiles[0].Id, Location(2, 0, 1), {Location(-2, 0, 4)}}));

			for (std::uint64_t tick = 0; tick < 200; ++tick)
				Advance(planner, tick);

			const auto downward = planner.GetReachabilityQueryResult(Navigation::Api::ReachabilityRequestId{1});
			const auto upward = planner.GetReachabilityQueryResult(Navigation::Api::ReachabilityRequestId{2});
			ASSERT_NE(downward, nullptr);
			ASSERT_NE(upward, nullptr);
			EXPECT_EQ(downward->Destinations[0], Navigation::Api::ReachabilityState::Reachable);
			EXPECT_EQ(upward->Destinations[0], Navigation::Api::ReachabilityState::Unreachable);
		}

		TEST(ReachabilityTest, FindsDirectedEdgesAcrossVerticalTileBoundaries)
		{
			Terrain terrain;
			terrain.VerticalTileCliff = true;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain,
							profiles,
							Planner::DefaultExpansionsPerTick,
							Planner::DefaultMaximumExpansionsPerRequest,
							Planner::DefaultReachabilityComponentExpansionsPerTick,
							64};
			ASSERT_TRUE(planner.BeginReachabilityQuery(
				{Navigation::Api::ReachabilityRequestId{1}, profiles[0].Id, Location(2, 8, 16), {Location(6, 8, 12)}}));
			ASSERT_TRUE(planner.BeginReachabilityQuery(
				{Navigation::Api::ReachabilityRequestId{2}, profiles[0].Id, Location(6, 8, 12), {Location(2, 8, 16)}}));

			for (std::uint64_t tick = 0; tick < 500; ++tick)
				Advance(planner, tick);

			const auto downward = planner.GetReachabilityQueryResult(Navigation::Api::ReachabilityRequestId{1});
			const auto upward = planner.GetReachabilityQueryResult(Navigation::Api::ReachabilityRequestId{2});
			ASSERT_NE(downward, nullptr);
			ASSERT_NE(upward, nullptr);
			EXPECT_EQ(downward->Destinations[0], Navigation::Api::ReachabilityState::Reachable);
			EXPECT_EQ(upward->Destinations[0], Navigation::Api::ReachabilityState::Unreachable);
		}

		TEST(ReachabilityTest, AppliesInitializationBudgetDeterministically)
		{
			Terrain terrain;
			terrain.SealedBarrier = true;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles, 128, 1024, 1, 64};
			ASSERT_TRUE(planner.BeginReachabilityQuery({Navigation::Api::ReachabilityRequestId{1},
														profiles[0].Id,
														Location(0, 0, 1),
														{Location(8, 0, 1), Location(32, 0, 1)}}));

			Advance(planner, 0);
			auto result = planner.GetReachabilityQueryResult(Navigation::Api::ReachabilityRequestId{1});
			ASSERT_NE(result, nullptr);
			EXPECT_FALSE(result->IsComplete());

			for (std::uint64_t tick = 1; tick < 200 && !result->IsComplete(); ++tick)
				Advance(planner, tick);

			EXPECT_TRUE(result->IsComplete());
			EXPECT_EQ(result->Destinations[0], Navigation::Api::ReachabilityState::Reachable);
			EXPECT_EQ(result->Destinations[1], Navigation::Api::ReachabilityState::Unreachable);
		}

		TEST(ReachabilityTest, RecomputesAfterEnvironmentInvalidation)
		{
			Terrain terrain;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles};
			ASSERT_TRUE(planner.BeginReachabilityQuery(
				{Navigation::Api::ReachabilityRequestId{1}, profiles[0].Id, Location(0, 0, 1), {Location(8, 0, 1)}}));
			for (std::uint64_t tick = 0; tick < 20; ++tick)
				Advance(planner, tick);
			const auto result = planner.GetReachabilityQueryResult(Navigation::Api::ReachabilityRequestId{1});
			ASSERT_NE(result, nullptr);
			ASSERT_TRUE(result->IsComplete());
			ASSERT_EQ(result->EnvironmentRevision, 1U);

			const std::array regions{UnrealVoxelSim::Voxel::Api::Region{{0, 0, 0}, {1, 1, 1}}};
			planner.Invalidate(regions);

			EXPECT_EQ(result->EnvironmentRevision, 2U);
			EXPECT_EQ(result->Destinations[0], Navigation::Api::ReachabilityState::Pending);
			for (std::uint64_t tick = 20; tick < 40 && !result->IsComplete(); ++tick)
				Advance(planner, tick);
			EXPECT_EQ(result->Destinations[0], Navigation::Api::ReachabilityState::Reachable);
		}

		TEST(ReachabilityTest, RetainsResultAfterDistantEnvironmentInvalidation)
		{
			Terrain terrain;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain,
							profiles,
							Planner::DefaultExpansionsPerTick,
							Planner::DefaultMaximumExpansionsPerRequest,
							Planner::DefaultReachabilityComponentExpansionsPerTick,
							64};
			ASSERT_TRUE(planner.BeginReachabilityQuery(
				{Navigation::Api::ReachabilityRequestId{1}, profiles[0].Id, Location(0, 32, 1), {Location(8, 32, 1)}}));
			for (std::uint64_t tick = 0; tick < 200; ++tick)
			{
				Advance(planner, tick);
				if (planner.GetReachabilityQueryResult(Navigation::Api::ReachabilityRequestId{1})->IsComplete())
					break;
			}
			const auto result = planner.GetReachabilityQueryResult(Navigation::Api::ReachabilityRequestId{1});
			ASSERT_NE(result, nullptr);
			ASSERT_TRUE(result->IsComplete());

			const std::array regions{UnrealVoxelSim::Voxel::Api::Region{{4, 0, 0}, {5, 1, 3}}};
			planner.Invalidate(regions);

			EXPECT_EQ(result->EnvironmentRevision, 2U);
			EXPECT_TRUE(result->IsComplete());
			EXPECT_EQ(result->Destinations[0], Navigation::Api::ReachabilityState::Reachable);
		}

		TEST(PlannerTest, AdvancesEnvironmentRevisionOnInvalidation)
		{
			Terrain terrain;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles};
			const std::array regions{UnrealVoxelSim::Voxel::Api::Region{{0, 0, 0}, {1, 1, 1}}};

			planner.Invalidate(regions);

			EXPECT_EQ(planner.GetCurrentEnvironmentRevision(), 2U);
		}

		TEST(TopologyTest, StepBuildsColdTopologyWithinItsDeterministicBudget)
		{
			Terrain terrain;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(8, 0, 1)}));

			planner.Step({Simulation::Api::TickIndex{0}, Simulation::Api::StandardStepDuration});

			EXPECT_GT(terrain.RegionReads, 0U);
			EXPECT_LE(terrain.RegionReads, Planner::DefaultTileBuildsPerTopologyUpdate);
		}

		TEST(TopologyTest, VoxelInvalidationProactivelyQueuesRebuild)
		{
			Terrain terrain;
			const std::array profiles{Movement::Api::GroundedProfile{Movement::Api::ProfileId{1}}};
			Planner planner{terrain, profiles};
			ASSERT_TRUE(planner.BeginPathPlanning({profiles[0].Id, Location(0, 0, 1), Location(8, 0, 1)}));
			Advance(planner, 0);
			for (std::uint64_t tick = 1; tick < 10; ++tick)
				planner.Step({Simulation::Api::TickIndex{tick}, Simulation::Api::StandardStepDuration});
			const auto readsBeforeInvalidation = terrain.RegionReads;
			const std::array regions{UnrealVoxelSim::Voxel::Api::Region{{0, 0, 0}, {1, 1, 2}}};

			planner.Invalidate(regions);
			planner.Step({Simulation::Api::TickIndex{10}, Simulation::Api::StandardStepDuration});

			EXPECT_GT(terrain.RegionReads, readsBeforeInvalidation);
		}
	}
}
