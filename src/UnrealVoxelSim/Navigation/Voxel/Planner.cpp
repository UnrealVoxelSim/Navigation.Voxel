#include "UnrealVoxelSim/Navigation/Voxel/Planner.h"

#include "UnrealVoxelSim/Math/Api/FixedPointScalar.h"
#include "UnrealVoxelSim/Navigation/Api/MovementPrimitiveId.h"
#include "UnrealVoxelSim/Profiling/Api/Macros.h"
#include "UnrealVoxelSim/Profiling/Api/NullRecorder.h"
#include "UnrealVoxelSim/Voxel/Api/Position.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace UnrealVoxelSim::Navigation::Voxel
{
	namespace
	{
		namespace NavigationApi = UnrealVoxelSim::Navigation::Api;
		namespace EnvironmentApi = UnrealVoxelSim::Navigation::Voxel::Api;
		namespace ProfilingApi = UnrealVoxelSim::Profiling::Api;
		namespace VoxelApi = UnrealVoxelSim::Voxel::Api;

		constexpr std::int32_t TileEdge = 16;
		constexpr std::size_t TileCellCount = static_cast<std::size_t>(TileEdge) * TileEdge * TileEdge;
		constexpr std::uint64_t CardinalCost = 1000;
		constexpr std::uint64_t DiagonalCost = 1414;
		constexpr std::size_t MinimumCoarseExpansions = 64;
		constexpr std::size_t MaximumCoarseExpansions = 512;
		constexpr std::size_t CoarseExpansionDistanceFactor = 32;
		constexpr std::size_t PlanPromotionsPerTick = 256;
		constexpr std::size_t PlanEndpointChecksPerTick = 1024;
		constexpr std::size_t PlanEndpointProjectionsPerTick = 2048;
		constexpr std::size_t TopologyStandablePositionsPerUpdate = 1536;
		constexpr std::size_t ColdCorridorBuildsPerTick = 1;
		constexpr std::size_t ColdIncomingComponentBuildsPerTick = 1;
		constexpr std::size_t FineExpansionsAfterColdCorridor = 128;
		constexpr std::uint16_t NoComponent = std::numeric_limits<std::uint16_t>::max();

		struct TileKey final
		{
			std::int32_t X{};
			std::int32_t Y{};
			std::int32_t Z{};
			auto operator<=>(const TileKey&) const = default;
		};

		struct ProfileTileKey final
		{
			Movement::Api::ProfileId Profile;
			TileKey Tile;
			auto operator<=>(const ProfileTileKey&) const = default;
		};

		struct ComponentKey final
		{
			Movement::Api::ProfileId Profile;
			TileKey Tile;
			std::uint16_t Component{};
			auto operator<=>(const ComponentKey&) const = default;
		};

		struct CorridorKey final
		{
			Movement::Api::ProfileId Profile;
			TileKey Start;
			TileKey Goal;
			auto operator<=>(const CorridorKey&) const = default;
		};

		struct Tile final
		{
			VoxelApi::Region Dependency;
			std::array<std::uint8_t, TileCellCount> Standable{};
			std::array<std::uint16_t, TileCellCount> Clearance{};
			std::array<std::uint16_t, TileCellCount> ComponentByCell{};
			std::vector<std::vector<VoxelApi::Position>> ComponentCells;
			std::vector<std::vector<std::uint16_t>> LocalEdges;
			std::size_t StandablePositions{};
		};

		[[nodiscard]] constexpr std::int32_t FloorDiv(const std::int32_t value) noexcept
		{
			if (value >= 0)
				return value / TileEdge;
			return static_cast<std::int32_t>((static_cast<std::int64_t>(value) - (TileEdge - 1)) / TileEdge);
		}

		[[nodiscard]] constexpr TileKey ToTile(const VoxelApi::Position position) noexcept
		{
			return {FloorDiv(position.X), FloorDiv(position.Y), FloorDiv(position.Z)};
		}

		[[nodiscard]] constexpr std::int32_t Local(const std::int32_t value, const std::int32_t tile) noexcept
		{
			return static_cast<std::int32_t>(static_cast<std::int64_t>(value) -
											 static_cast<std::int64_t>(tile) * TileEdge);
		}

		[[nodiscard]] constexpr std::size_t LocalIndex(const VoxelApi::Position position, const TileKey tile) noexcept
		{
			const auto x = static_cast<std::size_t>(Local(position.X, tile.X));
			const auto y = static_cast<std::size_t>(Local(position.Y, tile.Y));
			const auto z = static_cast<std::size_t>(Local(position.Z, tile.Z));
			return x + TileEdge * (y + TileEdge * z);
		}

		[[nodiscard]] constexpr VoxelApi::Region TileRegion(const TileKey key) noexcept
		{
			const auto x = static_cast<std::int32_t>(static_cast<std::int64_t>(key.X) * TileEdge);
			const auto y = static_cast<std::int32_t>(static_cast<std::int64_t>(key.Y) * TileEdge);
			const auto z = static_cast<std::int32_t>(static_cast<std::int64_t>(key.Z) * TileEdge);
			return {{x, y, z}, {x + TileEdge, y + TileEdge, z + TileEdge}};
		}

		[[nodiscard]] constexpr bool Intersects(const VoxelApi::Region left, const VoxelApi::Region right) noexcept
		{
			return left.Min.X < right.Max.X && left.Max.X > right.Min.X && left.Min.Y < right.Max.Y &&
				left.Max.Y > right.Min.Y && left.Min.Z < right.Max.Z && left.Max.Z > right.Min.Z;
		}

		[[nodiscard]] constexpr std::int32_t FloorCoordinate(const Math::Api::FixedPointScalar value) noexcept
		{
			const auto raw = value.Raw();
			auto result = raw / Math::Api::FixedPointScalar::OneRaw;
			if (raw % Math::Api::FixedPointScalar::OneRaw < 0)
				--result;
			return static_cast<std::int32_t>(result);
		}

		[[nodiscard]] constexpr VoxelApi::Position ToVoxel(const Spatial::Api::Position position) noexcept
		{
			return {FloorCoordinate(position.X), FloorCoordinate(position.Y), FloorCoordinate(position.Z)};
		}

		[[nodiscard]] constexpr Spatial::Api::Position ToContinuous(const VoxelApi::Position position) noexcept
		{
			constexpr auto Half = Math::Api::FixedPointScalar::OneRaw / 2;
			return {Math::Api::FixedPointScalar::FromRaw(
						static_cast<std::int64_t>(position.X) * Math::Api::FixedPointScalar::OneRaw + Half),
					Math::Api::FixedPointScalar::FromRaw(
						static_cast<std::int64_t>(position.Y) * Math::Api::FixedPointScalar::OneRaw + Half),
					Math::Api::FixedPointScalar::FromRaw(static_cast<std::int64_t>(position.Z) *
														 Math::Api::FixedPointScalar::OneRaw)};
		}

		struct PositionHash final
		{
			[[nodiscard]] std::size_t operator()(const VoxelApi::Position value) const noexcept
			{
				auto hash = static_cast<std::uint64_t>(static_cast<std::uint32_t>(value.X)) * 0x9E3779B185EBCA87ULL;
				hash ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(value.Y)) * 0xC2B2AE3D27D4EB4FULL;
				hash ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(value.Z)) * 0x165667B19E3779F9ULL;
				return static_cast<std::size_t>(hash ^ (hash >> 32U));
			}
		};

		[[nodiscard]] std::uint64_t Heuristic(const VoxelApi::Position from, const VoxelApi::Position to) noexcept
		{
			const auto dx = static_cast<std::uint64_t>(std::abs(static_cast<std::int64_t>(from.X) - to.X));
			const auto dy = static_cast<std::uint64_t>(std::abs(static_cast<std::int64_t>(from.Y) - to.Y));
			const auto diagonal = std::min(dx, dy);
			return diagonal * DiagonalCost + (std::max(dx, dy) - diagonal) * CardinalCost;
		}

		struct Record final
		{
			std::uint64_t Cost{std::numeric_limits<std::uint64_t>::max()};
			VoxelApi::Position Parent{};
			bool HasParent{};
			bool Closed{};
		};

		struct OpenEntry final
		{
			std::uint64_t Score{};
			std::uint64_t Heuristic{};
			VoxelApi::Position Position;
		};

		struct OpenWorse final
		{
			[[nodiscard]] bool operator()(const OpenEntry& left, const OpenEntry& right) const noexcept
			{
				if (left.Score != right.Score)
					return left.Score > right.Score;
				if (left.Heuristic != right.Heuristic)
					return left.Heuristic > right.Heuristic;
				return left.Position > right.Position;
			}
		};

		struct TileGuidance final
		{
			VoxelApi::Position Target;
			std::uint32_t RemainingTiles{};
		};

		struct Search final
		{
			Movement::Api::ProfileId Profile;
			VoxelApi::Position Start;
			VoxelApi::Position Goal;
			std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenWorse> Open;
			std::unordered_map<VoxelApi::Position, Record, PositionHash> Records;
			std::set<TileKey> Corridor;
			std::map<TileKey, TileGuidance> Guidance;
			bool FallbackUsed{};
			std::size_t ExpandedNodes{};
		};

		struct CoarseCorridor final
		{
			std::vector<TileKey> Allowed;
			std::map<TileKey, TileGuidance> Guidance;
			std::set<TileKey> Dependencies;
		};

		struct CoarseRecord final
		{
			std::uint32_t Cost{std::numeric_limits<std::uint32_t>::max()};
			TileKey Parent{};
			bool HasParent{};
			bool Closed{};
		};

		struct CoarseOpenEntry final
		{
			std::uint32_t Score{};
			std::uint32_t Heuristic{};
			TileKey Tile;
		};

		struct CoarseOpenWorse final
		{
			[[nodiscard]] bool operator()(const CoarseOpenEntry& left, const CoarseOpenEntry& right) const noexcept
			{
				if (left.Score != right.Score)
					return left.Score > right.Score;
				if (left.Heuristic != right.Heuristic)
					return left.Heuristic > right.Heuristic;
				return left.Tile > right.Tile;
			}
		};

		[[nodiscard]] std::uint32_t CoarseHeuristic(const TileKey from, const TileKey to) noexcept
		{
			return static_cast<std::uint32_t>(std::abs(static_cast<std::int64_t>(from.X) - to.X) +
											  std::abs(static_cast<std::int64_t>(from.Y) - to.Y) +
											  std::abs(static_cast<std::int64_t>(from.Z) - to.Z));
		}

		struct Request final
		{
			NavigationApi::RequestId Id;
			NavigationApi::PlanRequest Plan;
			NavigationApi::PlanState State{NavigationApi::PlanState::Pending};
			std::optional<Search> SearchState;
			std::shared_ptr<const NavigationApi::Path> Path;
			std::optional<NavigationApi::PlanRequest> PendingPlan;
			std::optional<VoxelApi::Position> ProjectedStart;
			std::optional<VoxelApi::Position> ProjectedGoal;
			std::optional<ComponentKey> ReachabilitySource;
			std::optional<ComponentKey> ReachabilityGoal;
		};

		struct ComponentSearch final
		{
			ComponentKey Source;
			std::deque<ComponentKey> Open;
			std::set<ComponentKey> Visited;
			bool Complete{};
		};

		struct ComponentEdgeBuild final
		{
			std::size_t NextCell{};
			std::vector<ComponentKey> Edges;
		};

		struct ReachabilityRequest final
		{
			NavigationApi::ReachabilityQuery Query;
			std::shared_ptr<NavigationApi::ReachabilityResult> Result;
			std::optional<ComponentKey> Source;
			std::vector<std::optional<ComponentKey>> Goals;
			std::size_t NextDestination{};
			bool SourceResolved{};
			bool Initialized{};
		};

		struct PathValidation final
		{
			Movement::Api::ProfileId Profile;
			std::set<TileKey> Dependencies;
			bool Current{true};
		};

		enum class DirectPathState
		{
			Pending,
			Blocked,
			Complete,
		};
	}

	class Planner::Impl final
	{
	public:
		Impl(const EnvironmentApi::IEnvironment& environment,
			 std::span<const Movement::Api::GroundedProfile> profiles,
			 ProfilingApi::IRecorder* profiling,
			 const std::size_t expansionsPerTick,
			 const std::size_t maximumExpansionsPerRequest,
			 const std::size_t reachabilityComponentExpansionsPerTick,
			 const std::size_t tileBuildsPerTopologyUpdate,
			 const std::size_t componentCellsPerTick) :
			Environment(environment), Profiling(profiling ? *profiling : NullProfiling),
			Profiles(profiles.begin(), profiles.end()), ExpansionsPerTick(expansionsPerTick),
			MaximumExpansionsPerRequest(maximumExpansionsPerRequest),
			ReachabilityComponentExpansionsPerTick(reachabilityComponentExpansionsPerTick),
			TileBuildsPerTopologyUpdate(tileBuildsPerTopologyUpdate), ComponentCellsPerTick(componentCellsPerTick)
		{
			if (Profiles.empty() || ExpansionsPerTick == 0 || MaximumExpansionsPerRequest == 0 ||
				ReachabilityComponentExpansionsPerTick == 0 || TileBuildsPerTopologyUpdate == 0 ||
				ComponentCellsPerTick == 0 ||
				std::ranges::any_of(Profiles, [](const auto& profile) { return !profile.IsValid(); }))
				throw std::invalid_argument{"Planner requires valid profiles and a non-zero expansion budget."};
			std::ranges::sort(Profiles, {}, &Movement::Api::GroundedProfile::Id);
			if (std::ranges::adjacent_find(Profiles, {}, &Movement::Api::GroundedProfile::Id) != Profiles.end())
				throw std::invalid_argument{"Planner profile identifiers must be unique."};
		}

		void AssertOwnerThread() const noexcept { assert(std::this_thread::get_id() == OwnerThread); }

		[[nodiscard]] const Movement::Api::GroundedProfile* Profile(const Movement::Api::ProfileId id) const noexcept
		{
			const auto iterator = std::ranges::lower_bound(Profiles, id, {}, &Movement::Api::GroundedProfile::Id);
			return iterator != Profiles.end() && iterator->Id == id ? &*iterator : nullptr;
		}

		[[nodiscard]] bool GraphTileAffected(const Movement::Api::ProfileId profileId,
											 const TileKey tile,
											 const std::set<ProfileTileKey>& affected) const noexcept
		{
			const auto* profile = Profile(profileId);
			if (!profile)
				return true;
			const auto verticalDistance =
				1 + static_cast<std::int64_t>(std::max(profile->MaximumRise, profile->MaximumDrop)) / TileEdge;
			return std::ranges::any_of(affected,
									   [&](const ProfileTileKey candidate)
									   {
										   return candidate.Profile == profileId &&
											   std::abs(static_cast<std::int64_t>(candidate.Tile.X) - tile.X) <= 1 &&
											   std::abs(static_cast<std::int64_t>(candidate.Tile.Y) - tile.Y) <= 1 &&
											   std::abs(static_cast<std::int64_t>(candidate.Tile.Z) - tile.Z) <=
											   verticalDistance;
									   });
		}

		[[nodiscard]] bool ComponentAffected(const ComponentKey component,
											 const std::set<ProfileTileKey>& affected) const noexcept
		{
			return GraphTileAffected(component.Profile, component.Tile, affected);
		}

		[[nodiscard]] auto FindRequest(const NavigationApi::RequestId id) noexcept
		{
			return std::ranges::lower_bound(Requests, id, {}, &Request::Id);
		}

		[[nodiscard]] auto FindRequest(const NavigationApi::RequestId id) const noexcept
		{
			return std::ranges::lower_bound(Requests, id, {}, &Request::Id);
		}

		void AddActive(const NavigationApi::RequestId id) { ActiveRequests.push_back(id); }

		void RemoveActive(const NavigationApi::RequestId id) noexcept
		{
			const auto iterator = std::ranges::find(ActiveRequests, id);
			if (iterator == ActiveRequests.end())
				return;
			const auto index = static_cast<std::size_t>(std::distance(ActiveRequests.begin(), iterator));
			if (index < ActiveCursor)
				--ActiveCursor;
			ActiveRequests.erase(iterator);
			if (ActiveCursor >= ActiveRequests.size())
				ActiveCursor = 0;
		}

		[[nodiscard]] Tile BuildTile(const TileKey key, const Movement::Api::GroundedProfile& profile)
		{
			UNREALVOXELSIM_PROFILE_ZONE(Profiling, "Build navigation tile");
			const auto tileRegion = TileRegion(key);
			const auto bounds = Environment.Bounds();
			const auto leftX = static_cast<std::int32_t>(profile.Width / 2);
			const auto rightX = static_cast<std::int32_t>((profile.Width - 1) / 2);
			const auto leftY = static_cast<std::int32_t>(profile.Length / 2);
			const auto rightY = static_cast<std::int32_t>((profile.Length - 1) / 2);
			VoxelApi::Region dependency{
				{std::max(bounds.Min.X, tileRegion.Min.X - leftX),
				 std::max(bounds.Min.Y, tileRegion.Min.Y - leftY),
				 std::max(bounds.Min.Z, tileRegion.Min.Z - 1)},
				{std::min(bounds.Max.X, tileRegion.Max.X + rightX),
				 std::min(bounds.Max.Y, tileRegion.Max.Y + rightY),
				 std::min(bounds.Max.Z,
						  tileRegion.Max.Z + static_cast<std::int32_t>(profile.Height) +
							  static_cast<std::int32_t>(std::max(profile.MaximumRise, profile.MaximumDrop)) - 1)}};
			Tile tile{dependency};
			if (!dependency.IsValid())
				return tile;
			const auto count = dependency.CellCount();
			if (!count)
				return tile;
			std::vector<EnvironmentApi::Cell> cells(*count);
			if (!Environment.ReadRegion(dependency, cells))
				return tile;
			tile.ComponentByCell.fill(NoComponent);
			if (std::ranges::none_of(cells, &EnvironmentApi::Cell::SupportsGroundedBody) ||
				std::ranges::all_of(cells, &EnvironmentApi::Cell::BlocksOccupancy))
				return tile;

			const auto sample = [&](const VoxelApi::Position position)
			{
				if (!dependency.Contains(position))
					return EnvironmentApi::Cell{true, false, 1000};
				const auto x = static_cast<std::size_t>(position.X - dependency.Min.X);
				const auto y = static_cast<std::size_t>(position.Y - dependency.Min.Y);
				const auto z = static_cast<std::size_t>(position.Z - dependency.Min.Z);
				const auto width = static_cast<std::size_t>(dependency.Max.X - dependency.Min.X);
				const auto length = static_cast<std::size_t>(dependency.Max.Y - dependency.Min.Y);
				return cells[x + width * (y + length * z)];
			};

			for (auto z = tileRegion.Min.Z; z < tileRegion.Max.Z; ++z)
				for (auto y = tileRegion.Min.Y; y < tileRegion.Max.Y; ++y)
					for (auto x = tileRegion.Min.X; x < tileRegion.Max.X; ++x)
					{
						bool supported = true;
						for (auto offsetY = -leftY; supported && offsetY <= rightY; ++offsetY)
							for (auto offsetX = -leftX; offsetX <= rightX; ++offsetX)
								if (!sample({x + offsetX, y + offsetY, z - 1}).SupportsGroundedBody)
								{
									supported = false;
									break;
								}
						const auto index = LocalIndex({x, y, z}, key);
						if (!supported)
						{
							tile.Standable[index] = 0;
							tile.Clearance[index] = 0;
							continue;
						}
						const auto maximumClearance = static_cast<std::int32_t>(profile.Height) +
							static_cast<std::int32_t>(std::max(profile.MaximumRise, profile.MaximumDrop));
						std::uint16_t clearance{};
						for (std::int32_t bodyZ = 0; bodyZ < maximumClearance; ++bodyZ)
						{
							bool clear = true;
							for (auto offsetY = -leftY; clear && offsetY <= rightY; ++offsetY)
								for (auto offsetX = -leftX; offsetX <= rightX; ++offsetX)
									if (sample({x + offsetX, y + offsetY, z + bodyZ}).BlocksOccupancy)
									{
										clear = false;
										break;
									}
							if (!clear)
								break;
							++clearance;
						}
						tile.Clearance[index] = clearance;
						tile.Standable[index] = supported && clearance >= profile.Height ? 1 : 0;
						if (tile.Standable[index] != 0)
							++tile.StandablePositions;
					}

			const auto localNeighbor =
				[&](const VoxelApi::Position current, const int dx, const int dy) -> std::optional<VoxelApi::Position>
			{
				const auto standable = [&](const VoxelApi::Position position)
				{ return ToTile(position) == key && tile.Standable[LocalIndex(position, key)] != 0; };
				const auto test = [&](const std::int32_t z)
				{ return VoxelApi::Position{current.X + dx, current.Y + dy, z}; };
				auto candidate = test(current.Z);
				if (!standable(candidate))
				{
					bool found{};
					for (std::int32_t rise = 1; rise <= profile.MaximumRise; ++rise)
					{
						candidate = test(current.Z + rise);
						if (standable(candidate))
						{
							found = true;
							break;
						}
					}
					if (!found)
						for (std::int32_t drop = 1; drop <= profile.MaximumDrop; ++drop)
						{
							candidate = test(current.Z - drop);
							if (standable(candidate))
							{
								found = true;
								break;
							}
						}
					if (!found)
						return std::nullopt;
				}
				if (dx != 0 && dy != 0 &&
					(!standable({current.X + dx, current.Y, candidate.Z}) ||
					 !standable({current.X, current.Y + dy, candidate.Z})))
					return std::nullopt;
				const auto elevation = candidate.Z - current.Z;
				if (elevation > 0 && tile.Clearance[LocalIndex(current, key)] < profile.Height + elevation)
					return std::nullopt;
				if (elevation < 0 && tile.Clearance[LocalIndex(candidate, key)] < profile.Height - elevation)
					return std::nullopt;
				return candidate;
			};

			for (auto z = tileRegion.Min.Z; z < tileRegion.Max.Z; ++z)
				for (auto y = tileRegion.Min.Y; y < tileRegion.Max.Y; ++y)
					for (auto x = tileRegion.Min.X; x < tileRegion.Max.X; ++x)
					{
						const VoxelApi::Position seed{x, y, z};
						const auto seedIndex = LocalIndex(seed, key);
						if (tile.Standable[seedIndex] == 0 || tile.ComponentByCell[seedIndex] != NoComponent)
							continue;
						const auto component = static_cast<std::uint16_t>(tile.ComponentCells.size());
						tile.ComponentCells.emplace_back();
						std::deque<VoxelApi::Position> open{seed};
						tile.ComponentByCell[seedIndex] = component;
						while (!open.empty())
						{
							const auto current = open.front();
							open.pop_front();
							tile.ComponentCells.back().push_back(current);
							for (int dy = -1; dy <= 1; ++dy)
								for (int dx = -1; dx <= 1; ++dx)
								{
									if (dx == 0 && dy == 0)
										continue;
									const auto neighbor = localNeighbor(current, dx, dy);
									if (!neighbor)
										continue;
									const auto reverse = localNeighbor(*neighbor, -dx, -dy);
									if (!reverse || *reverse != current)
										continue;
									const auto neighborIndex = LocalIndex(*neighbor, key);
									if (tile.ComponentByCell[neighborIndex] != NoComponent)
										continue;
									tile.ComponentByCell[neighborIndex] = component;
									open.push_back(*neighbor);
								}
						}
					}

			tile.LocalEdges.resize(tile.ComponentCells.size());
			for (std::uint16_t component = 0; component < tile.ComponentCells.size(); ++component)
				for (const auto current : tile.ComponentCells[component])
					for (int dy = -1; dy <= 1; ++dy)
						for (int dx = -1; dx <= 1; ++dx)
						{
							if (dx == 0 && dy == 0)
								continue;
							const auto neighbor = localNeighbor(current, dx, dy);
							if (!neighbor)
								continue;
							const auto target = tile.ComponentByCell[LocalIndex(*neighbor, key)];
							if (target != component)
								tile.LocalEdges[component].push_back(target);
						}
			for (auto& edges : tile.LocalEdges)
			{
				std::ranges::sort(edges);
				edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
			}
			return tile;
		}

		void QueueTile(const ProfileTileKey key, const bool urgent = true)
		{
			if (Tiles.contains(key))
				return;
			if (!Intersects(TileRegion(key.Tile), Environment.Bounds()))
				return;
			if (PendingTileBuilds.insert(key).second)
			{
				if (urgent)
				{
					UrgentTileBuilds.insert(key);
					UrgentTileBuildOrder.push_back(key);
				}
				else
					BackgroundTileBuildOrder.push_back(key);
				++TopologyDemandGeneration;
			}
			else if (urgent && UrgentTileBuilds.insert(key).second)
				UrgentTileBuildOrder.push_back(key);
		}

		[[nodiscard]] bool IsTileReady(const ProfileTileKey key) const noexcept
		{
			return Tiles.contains(key) || !Intersects(TileRegion(key.Tile), Environment.Bounds());
		}

		void QueueProjection(const Spatial::Api::Position position, const Movement::Api::GroundedProfile& profile)
		{
			const auto origin = ToVoxel(position);
			QueueTile({profile.Id, ToTile(origin)});
			const auto verticalLimit = std::max<std::int32_t>(profile.MaximumDrop, profile.MaximumRise + 2);
			for (std::int32_t distance = 1; distance <= verticalLimit; ++distance)
			{
				QueueTile({profile.Id, ToTile({origin.X, origin.Y, origin.Z + distance})});
				QueueTile({profile.Id, ToTile({origin.X, origin.Y, origin.Z - distance})});
			}
		}

		template <typename Visitor>
		void VisitRouteHint(const Spatial::Api::Position start,
							const Spatial::Api::Position goal,
							const Movement::Api::GroundedProfile& profile,
							Visitor&& visitor) const
		{
			const auto startVoxel = ToVoxel(start);
			const auto goalVoxel = ToVoxel(goal);
			const auto startTile = ToTile(startVoxel);
			const auto goalTile = ToTile(goalVoxel);
			if (std::abs(static_cast<std::int64_t>(goalTile.Z) - startTile.Z) > 1)
				return;

			const auto deltaX = static_cast<std::int64_t>(goalTile.X) - startTile.X;
			const auto deltaY = static_cast<std::int64_t>(goalTile.Y) - startTile.Y;
			const auto steps = static_cast<std::size_t>(std::max(std::abs(deltaX), std::abs(deltaY)));
			const auto interpolate =
				[](const std::int32_t from, const std::int32_t to, const std::size_t index, const std::size_t count)
			{
				if (count == 0)
					return from;
				const auto delta = static_cast<std::int64_t>(to) - from;
				return static_cast<std::int32_t>(static_cast<std::int64_t>(from) +
												 delta * static_cast<std::int64_t>(index) /
													 static_cast<std::int64_t>(count));
			};
			for (std::size_t index = 0; index <= steps; ++index)
			{
				const auto x = interpolate(startTile.X, goalTile.X, index, steps);
				const auto y = interpolate(startTile.Y, goalTile.Y, index, steps);
				const auto footZ = interpolate(startVoxel.Z, goalVoxel.Z, index, steps);
				const auto minimumZ = FloorDiv(footZ - static_cast<std::int32_t>(profile.MaximumDrop));
				const auto maximumZ = FloorDiv(footZ + static_cast<std::int32_t>(profile.MaximumRise));
				for (auto z = minimumZ; z <= maximumZ; ++z)
					for (std::int32_t offsetY = -1; offsetY <= 1; ++offsetY)
						for (std::int32_t offsetX = -1; offsetX <= 1; ++offsetX)
							visitor(ProfileTileKey{profile.Id, {x + offsetX, y + offsetY, z}});
			}
		}

		void QueueRouteHint(const Spatial::Api::Position start,
							const Spatial::Api::Position goal,
							const Movement::Api::GroundedProfile& profile)
		{
			VisitRouteHint(start, goal, profile, [&](const ProfileTileKey key) { QueueTile(key); });
		}

		[[nodiscard]] bool ProjectionReady(const Spatial::Api::Position position,
										   const Movement::Api::GroundedProfile& profile) const
		{
			const auto origin = ToVoxel(position);
			if (!IsTileReady({profile.Id, ToTile(origin)}))
				return false;
			const auto verticalLimit = std::max<std::int32_t>(profile.MaximumDrop, profile.MaximumRise + 2);
			for (std::int32_t distance = 1; distance <= verticalLimit; ++distance)
				if (!IsTileReady({profile.Id, ToTile({origin.X, origin.Y, origin.Z + distance})}) ||
					!IsTileReady({profile.Id, ToTile({origin.X, origin.Y, origin.Z - distance})}))
					return false;
			return true;
		}

		void UpdateTopology()
		{
			std::size_t standablePositions{};
			for (std::size_t built = 0;
				 built < TileBuildsPerTopologyUpdate && standablePositions < TopologyStandablePositionsPerUpdate;)
			{
				std::optional<ProfileTileKey> next;
				while (!UrgentTileBuildOrder.empty() && !next)
				{
					const auto key = UrgentTileBuildOrder.front();
					UrgentTileBuildOrder.pop_front();
					if (PendingTileBuilds.contains(key) && UrgentTileBuilds.contains(key))
						next = key;
				}
				while (!next && !BackgroundTileBuildOrder.empty())
				{
					const auto activeNavigation =
						std::ranges::any_of(Requests,
											[](const auto& request)
											{ return request.State == NavigationApi::PlanState::Pending; }) ||
						std::ranges::any_of(ReachabilityRequests,
											[](const auto& request) { return !request.Result->IsComplete(); });
					if (activeNavigation)
						break;
					const auto key = BackgroundTileBuildOrder.front();
					BackgroundTileBuildOrder.pop_front();
					if (PendingTileBuilds.contains(key) && !UrgentTileBuilds.contains(key))
						next = key;
				}
				if (!next)
					break;
				const auto key = *next;
				PendingTileBuilds.erase(key);
				UrgentTileBuilds.erase(key);
				const auto* profile = Profile(key.Profile);
				if (profile && !Tiles.contains(key))
				{
					const auto [tile, inserted] = Tiles.emplace(key, BuildTile(key.Tile, *profile));
					if (inserted)
						standablePositions += tile->second.StandablePositions;
				}
				TopologyBuiltSincePlannerAdvance = true;
				++built;
			}
			UNREALVOXELSIM_PROFILE_PLOT(Profiling, "Planner topology standable positions", standablePositions);
		}

		void QueueAffectedRegions(const std::span<const VoxelApi::Region> regions)
		{
			for (const auto& profile : Profiles)
				for (const auto region : regions)
				{
					const auto horizontalBeforeX = static_cast<std::int32_t>((profile.Width - 1) / 2);
					const auto horizontalAfterX = static_cast<std::int32_t>(profile.Width / 2);
					const auto horizontalBeforeY = static_cast<std::int32_t>((profile.Length - 1) / 2);
					const auto horizontalAfterY = static_cast<std::int32_t>(profile.Length / 2);
					const auto verticalClearance = static_cast<std::int32_t>(profile.Height) +
						static_cast<std::int32_t>(std::max(profile.MaximumRise, profile.MaximumDrop));
					const VoxelApi::Region affected{
						{region.Min.X - horizontalBeforeX,
						 region.Min.Y - horizontalBeforeY,
						 region.Min.Z - verticalClearance + 1},
						{region.Max.X + horizontalAfterX, region.Max.Y + horizontalAfterY, region.Max.Z + 1}};
					const auto minimum = ToTile(affected.Min);
					const auto maximum = ToTile({affected.Max.X - 1, affected.Max.Y - 1, affected.Max.Z - 1});
					for (auto z = minimum.Z; z <= maximum.Z; ++z)
						for (auto y = minimum.Y; y <= maximum.Y; ++y)
							for (auto x = minimum.X; x <= maximum.X; ++x)
								QueueTile({profile.Id, {x, y, z}}, false);
				}
		}

		[[nodiscard]] bool IsStandable(const VoxelApi::Position position, const Movement::Api::GroundedProfile& profile)
		{
			if (!Environment.Bounds().Contains(position))
				return false;
			const auto key = ProfileTileKey{profile.Id, ToTile(position)};
			auto iterator = Tiles.find(key);
			if (iterator == Tiles.end())
			{
				QueueTile(key);
				return false;
			}
			return iterator->second.Standable[LocalIndex(position, key.Tile)] != 0;
		}

		[[nodiscard]] std::uint16_t Clearance(const VoxelApi::Position position,
											  const Movement::Api::GroundedProfile& profile)
		{
			if (!Environment.Bounds().Contains(position))
				return 0;
			const auto key = ProfileTileKey{profile.Id, ToTile(position)};
			auto iterator = Tiles.find(key);
			if (iterator == Tiles.end())
			{
				QueueTile(key);
				return 0;
			}
			return iterator->second.Clearance[LocalIndex(position, key.Tile)];
		}

		[[nodiscard]] std::optional<VoxelApi::Position> Project(const Spatial::Api::Position position,
																const Movement::Api::GroundedProfile& profile)
		{
			const auto origin = ToVoxel(position);
			if (IsStandable(origin, profile))
				return origin;
			const auto verticalLimit = std::max<std::int32_t>(profile.MaximumDrop, profile.MaximumRise + 2);
			for (std::int32_t distance = 1; distance <= verticalLimit; ++distance)
			{
				const VoxelApi::Position upward{origin.X, origin.Y, origin.Z + distance};
				if (IsStandable(upward, profile))
					return upward;
				const VoxelApi::Position downward{origin.X, origin.Y, origin.Z - distance};
				if (IsStandable(downward, profile))
					return downward;
			}
			return std::nullopt;
		}

		[[nodiscard]] std::optional<VoxelApi::Position> Neighbor(const VoxelApi::Position current,
																 const int dx,
																 const int dy,
																 const Movement::Api::GroundedProfile& profile)
		{
			const auto test = [&](const std::int32_t z)
			{ return VoxelApi::Position{current.X + dx, current.Y + dy, z}; };
			auto candidate = test(current.Z);
			if (!IsStandable(candidate, profile))
			{
				bool found{};
				for (std::int32_t rise = 1; rise <= profile.MaximumRise; ++rise)
				{
					candidate = test(current.Z + rise);
					if (IsStandable(candidate, profile))
					{
						found = true;
						break;
					}
				}
				if (!found)
					for (std::int32_t drop = 1; drop <= profile.MaximumDrop; ++drop)
					{
						candidate = test(current.Z - drop);
						if (IsStandable(candidate, profile))
						{
							found = true;
							break;
						}
					}
				if (!found)
					return std::nullopt;
			}
			if (dx != 0 && dy != 0)
			{
				if (!IsStandable({current.X + dx, current.Y, candidate.Z}, profile) ||
					!IsStandable({current.X, current.Y + dy, candidate.Z}, profile))
					return std::nullopt;
			}
			const auto elevation = candidate.Z - current.Z;
			if (elevation > 0 && Clearance(current, profile) < profile.Height + elevation)
				return std::nullopt;
			if (elevation < 0 && Clearance(candidate, profile) < profile.Height - elevation)
				return std::nullopt;
			return candidate;
		}

		[[nodiscard]] std::optional<ComponentKey> ComponentAt(const VoxelApi::Position position,
															  const Movement::Api::GroundedProfile& profile)
		{
			if (!IsStandable(position, profile))
				return std::nullopt;
			const auto tileKey = ToTile(position);
			const auto tile = Tiles.find(ProfileTileKey{profile.Id, tileKey});
			if (tile == Tiles.end())
				return std::nullopt;
			const auto component = tile->second.ComponentByCell[LocalIndex(position, tileKey)];
			if (component == NoComponent)
				return std::nullopt;
			return ComponentKey{profile.Id, tileKey, component};
		}

		[[nodiscard]] bool OutgoingCellDependenciesReady(const VoxelApi::Position current,
														 const Movement::Api::GroundedProfile& profile)
		{
			auto ready = true;
			const auto bounds = Environment.Bounds();
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx)
				{
					if (dx == 0 && dy == 0)
						continue;
					for (auto elevation = -static_cast<std::int32_t>(profile.MaximumDrop);
						 elevation <= static_cast<std::int32_t>(profile.MaximumRise);
						 ++elevation)
					{
						const VoxelApi::Position candidate{current.X + dx, current.Y + dy, current.Z + elevation};
						if (!bounds.Contains(candidate))
							continue;
						const ProfileTileKey dependency{profile.Id, ToTile(candidate)};
						QueueTile(dependency);
						ready = IsTileReady(dependency) && ready;
					}
				}
			return ready;
		}

		[[nodiscard]] static bool IsHorizontalTileBoundary(const VoxelApi::Position position,
														   const TileKey tile) noexcept
		{
			const auto x = Local(position.X, tile.X);
			const auto y = Local(position.Y, tile.Y);
			return x == 0 || x == TileEdge - 1 || y == 0 || y == TileEdge - 1;
		}

		[[nodiscard]] static bool IsOutgoingTileBoundary(const VoxelApi::Position position,
														 const TileKey tile,
														 const Movement::Api::GroundedProfile& profile) noexcept
		{
			const auto z = Local(position.Z, tile.Z);
			return IsHorizontalTileBoundary(position, tile) || z < static_cast<std::int32_t>(profile.MaximumDrop) ||
				z + static_cast<std::int32_t>(profile.MaximumRise) >= TileEdge;
		}

		[[nodiscard]] static bool IsIncomingTileBoundary(const VoxelApi::Position position,
														 const TileKey tile,
														 const Movement::Api::GroundedProfile& profile) noexcept
		{
			const auto z = Local(position.Z, tile.Z);
			return IsHorizontalTileBoundary(position, tile) || z < static_cast<std::int32_t>(profile.MaximumRise) ||
				z + static_cast<std::int32_t>(profile.MaximumDrop) >= TileEdge;
		}

		[[nodiscard]] bool AdvanceOutgoingEdges(const ComponentKey key, std::size_t& remainingCells)
		{
			if (ComponentEdgeCache.contains(key))
				return true;
			const auto* profile = Profile(key.Profile);
			const auto tile = Tiles.find(ProfileTileKey{key.Profile, key.Tile});
			if (!profile || tile == Tiles.end() || key.Component >= tile->second.ComponentCells.size())
			{
				ComponentEdgeCache.emplace(key, std::vector<ComponentKey>{});
				return true;
			}

			auto [jobIterator, inserted] = PendingComponentEdgeBuilds.try_emplace(key);
			auto& job = jobIterator->second;
			if (inserted)
				for (const auto local : tile->second.LocalEdges[key.Component])
					job.Edges.push_back({key.Profile, key.Tile, local});

			const auto& cells = tile->second.ComponentCells[key.Component];
			while (job.NextCell < cells.size())
			{
				const auto current = cells[job.NextCell];
				if (!IsOutgoingTileBoundary(current, key.Tile, *profile))
				{
					++job.NextCell;
					continue;
				}
				if (remainingCells == 0)
					return false;
				--remainingCells;
				if (!OutgoingCellDependenciesReady(current, *profile))
					return false;
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
					{
						if (dx == 0 && dy == 0)
							continue;
						const auto neighbor = Neighbor(current, dx, dy, *profile);
						if (!neighbor || ToTile(*neighbor) == key.Tile)
							continue;
						const auto target = ComponentAt(*neighbor, *profile);
						if (target)
							job.Edges.push_back(*target);
					}
				++job.NextCell;
			}
			if (job.NextCell != cells.size())
				return false;
			std::ranges::sort(job.Edges);
			job.Edges.erase(std::unique(job.Edges.begin(), job.Edges.end()), job.Edges.end());
			ComponentEdgeCache.emplace(key, std::move(job.Edges));
			PendingComponentEdgeBuilds.erase(jobIterator);
			return true;
		}

		[[nodiscard]] bool IncomingCellDependenciesReady(const VoxelApi::Position target,
														 const Movement::Api::GroundedProfile& profile)
		{
			auto ready = true;
			const auto bounds = Environment.Bounds();
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx)
				{
					if (dx == 0 && dy == 0)
						continue;
					for (auto elevation = -static_cast<std::int32_t>(profile.MaximumRise);
						 elevation <= static_cast<std::int32_t>(profile.MaximumDrop);
						 ++elevation)
					{
						const VoxelApi::Position predecessor{target.X - dx, target.Y - dy, target.Z + elevation};
						if (!bounds.Contains(predecessor))
							continue;
						const ProfileTileKey dependency{profile.Id, ToTile(predecessor)};
						QueueTile(dependency);
						ready = IsTileReady(dependency) && ready;
					}
				}
			return ready;
		}

		[[nodiscard]] bool AdvanceIncomingEdges(const ComponentKey key, std::size_t& remainingCells)
		{
			if (ComponentIncomingCache.contains(key))
				return true;
			const auto* profile = Profile(key.Profile);
			const auto tile = Tiles.find({key.Profile, key.Tile});
			if (!profile || tile == Tiles.end() || key.Component >= tile->second.ComponentCells.size())
			{
				ComponentIncomingCache.emplace(key, std::vector<ComponentKey>{});
				return true;
			}

			auto [jobIterator, inserted] = PendingComponentIncomingBuilds.try_emplace(key);
			auto& job = jobIterator->second;
			if (inserted)
				for (std::size_t source = 0; source < tile->second.LocalEdges.size(); ++source)
					if (std::ranges::binary_search(tile->second.LocalEdges[source], key.Component))
						job.Edges.push_back({key.Profile, key.Tile, static_cast<std::uint16_t>(source)});
			const auto& cells = tile->second.ComponentCells[key.Component];
			const auto bounds = Environment.Bounds();
			while (job.NextCell < cells.size())
			{
				const auto target = cells[job.NextCell];
				if (!IsIncomingTileBoundary(target, key.Tile, *profile))
				{
					++job.NextCell;
					continue;
				}
				if (remainingCells == 0)
					return false;
				--remainingCells;
				if (!IncomingCellDependenciesReady(target, *profile))
					return false;
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
					{
						if (dx == 0 && dy == 0)
							continue;
						for (auto elevation = -static_cast<std::int32_t>(profile->MaximumRise);
							 elevation <= static_cast<std::int32_t>(profile->MaximumDrop);
							 ++elevation)
						{
							const VoxelApi::Position predecessor{target.X - dx, target.Y - dy, target.Z + elevation};
							if (!bounds.Contains(predecessor) || !IsStandable(predecessor, *profile))
								continue;
							const auto transition = Neighbor(predecessor, dx, dy, *profile);
							if (!transition || *transition != target)
								continue;
							const auto source = ComponentAt(predecessor, *profile);
							if (source && *source != key)
								job.Edges.push_back(*source);
						}
					}
				++job.NextCell;
			}
			if (job.NextCell != cells.size())
				return false;
			std::ranges::sort(job.Edges);
			job.Edges.erase(std::unique(job.Edges.begin(), job.Edges.end()), job.Edges.end());
			ComponentIncomingCache.emplace(key, std::move(job.Edges));
			PendingComponentIncomingBuilds.erase(jobIterator);
			return true;
		}

		[[nodiscard]] auto FindComponentSearch(const ComponentKey source) noexcept
		{
			return std::ranges::lower_bound(ComponentSearches, source, {}, &ComponentSearch::Source);
		}

		[[nodiscard]] auto FindComponentSearch(const ComponentKey source) const noexcept
		{
			return std::ranges::lower_bound(ComponentSearches, source, {}, &ComponentSearch::Source);
		}

		ComponentSearch& EnsureComponentSearch(const ComponentKey source)
		{
			auto iterator = FindComponentSearch(source);
			if (iterator == ComponentSearches.end() || iterator->Source != source)
			{
				ComponentSearch search{source};
				search.Open.push_back(source);
				search.Visited.insert(source);
				iterator = ComponentSearches.insert(iterator, std::move(search));
				ActiveComponentSearches.push_back(source);
			}
			else if (!iterator->Complete &&
					 std::ranges::find(ActiveComponentSearches, source) == ActiveComponentSearches.end())
				ActiveComponentSearches.push_back(source);
			return *iterator;
		}

		[[nodiscard]] bool HasComponentDemand(const ComponentSearch& search) const
		{
			for (const auto& request : Requests)
				if (request.PendingPlan && request.ReachabilitySource == search.Source && request.ReachabilityGoal &&
					!search.Visited.contains(*request.ReachabilityGoal))
					return true;
			for (const auto& request : ReachabilityRequests)
			{
				if (request.Source != search.Source || request.Result->IsComplete())
					continue;
				for (std::size_t index = 0; index < request.Goals.size(); ++index)
					if (request.Result->Destinations[index] == NavigationApi::ReachabilityState::Pending &&
						request.Goals[index] && !search.Visited.contains(*request.Goals[index]))
						return true;
			}
			return false;
		}

		void AdvanceComponentSearches()
		{
			auto remaining = ReachabilityComponentExpansionsPerTick;
			auto remainingCells = ComponentCellsPerTick;
			while (remaining != 0 && !ActiveComponentSearches.empty())
			{
				if (ActiveComponentCursor >= ActiveComponentSearches.size())
					ActiveComponentCursor = 0;
				const auto source = ActiveComponentSearches[ActiveComponentCursor];
				auto search = FindComponentSearch(source);
				if (search == ComponentSearches.end() || search->Complete || search->Open.empty() ||
					!HasComponentDemand(*search))
				{
					if (search != ComponentSearches.end() && search->Open.empty())
						search->Complete = true;
					ActiveComponentSearches.erase(ActiveComponentSearches.begin() +
												  static_cast<std::ptrdiff_t>(ActiveComponentCursor));
					if (ActiveComponentCursor >= ActiveComponentSearches.size())
						ActiveComponentCursor = 0;
					continue;
				}
				const auto component = search->Open.front();
				if (!AdvanceOutgoingEdges(component, remainingCells))
					return;
				const auto edges = ComponentEdgeCache.find(component);
				assert(edges != ComponentEdgeCache.end());
				search->Open.pop_front();
				for (const auto target : edges->second)
					if (search->Visited.insert(target).second)
						search->Open.push_back(target);
				--remaining;
				if (search->Open.empty())
				{
					search->Complete = true;
					ActiveComponentSearches.erase(ActiveComponentSearches.begin() +
												  static_cast<std::ptrdiff_t>(ActiveComponentCursor));
					if (ActiveComponentCursor >= ActiveComponentSearches.size())
						ActiveComponentCursor = 0;
				}
				else
					ActiveComponentCursor = (ActiveComponentCursor + 1) % ActiveComponentSearches.size();
			}
		}

		[[nodiscard]] std::optional<std::vector<TileKey>> TileNeighbors(const TileKey key,
																		const Movement::Api::GroundedProfile& profile)
		{
			const auto cacheKey = ProfileTileKey{profile.Id, key};
			if (const auto cached = TileNeighborCache.find(cacheKey); cached != TileNeighborCache.end())
				return cached->second;

			const auto demandGeneration = TopologyDemandGeneration;
			const auto region = TileRegion(key);
			std::vector<TileKey> result;
			const auto consider = [&](const VoxelApi::Position source, const int dx, const int dy)
			{
				if (!IsStandable(source, profile))
					return;
				const auto target = Neighbor(source, dx, dy, profile);
				if (!target)
					return;
				const auto targetTile = ToTile(*target);
				if (targetTile != key)
					result.push_back(targetTile);
			};

			for (auto z = region.Min.Z; z < region.Max.Z; ++z)
				for (auto y = region.Min.Y; y < region.Max.Y; ++y)
				{
					consider({region.Min.X, y, z}, -1, 0);
					consider({region.Max.X - 1, y, z}, 1, 0);
				}
			for (auto z = region.Min.Z; z < region.Max.Z; ++z)
				for (auto x = region.Min.X; x < region.Max.X; ++x)
				{
					consider({x, region.Min.Y, z}, 0, -1);
					consider({x, region.Max.Y - 1, z}, 0, 1);
				}
			std::ranges::sort(result);
			result.erase(std::unique(result.begin(), result.end()), result.end());
			if (TopologyDemandGeneration != demandGeneration)
				return std::nullopt;
			TileNeighborCache.emplace(cacheKey, result);
			return result;
		}

		[[nodiscard]] std::optional<VoxelApi::Position> FindPortal(const TileKey from,
																   const TileKey to,
																   const VoxelApi::Position goal,
																   const Movement::Api::GroundedProfile& profile)
		{
			const auto dx = to.X < from.X ? -1 : to.X > from.X ? 1 : 0;
			const auto dy = to.Y < from.Y ? -1 : to.Y > from.Y ? 1 : 0;
			if ((dx == 0) == (dy == 0))
				return std::nullopt;
			const auto region = TileRegion(from);
			std::optional<VoxelApi::Position> best;
			const auto consider = [&](const VoxelApi::Position source)
			{
				if (!IsStandable(source, profile))
					return;
				const auto target = Neighbor(source, dx, dy, profile);
				if (!target || ToTile(*target) != to)
					return;
				if (!best || Heuristic(*target, goal) < Heuristic(*best, goal) ||
					(Heuristic(*target, goal) == Heuristic(*best, goal) && *target < *best))
					best = *target;
			};
			if (dx != 0)
				for (auto z = region.Min.Z; z < region.Max.Z; ++z)
					for (auto y = region.Min.Y; y < region.Max.Y; ++y)
						consider({dx < 0 ? region.Min.X : region.Max.X - 1, y, z});
			else
				for (auto z = region.Min.Z; z < region.Max.Z; ++z)
					for (auto x = region.Min.X; x < region.Max.X; ++x)
						consider({x, dy < 0 ? region.Min.Y : region.Max.Y - 1, z});
			return best;
		}

		[[nodiscard]] std::optional<CoarseCorridor> BuildCorridor(const VoxelApi::Position start,
																  const VoxelApi::Position goal,
																  const Movement::Api::GroundedProfile& profile)
		{
			const CorridorKey key{profile.Id, ToTile(start), ToTile(goal)};
			if (const auto cached = Corridors.find(key); cached != Corridors.end())
				return cached->second;
			const auto demandGeneration = TopologyDemandGeneration;

			std::map<TileKey, CoarseRecord> records;
			std::priority_queue<CoarseOpenEntry, std::vector<CoarseOpenEntry>, CoarseOpenWorse> open;
			records[key.Start].Cost = 0;
			const auto initialHeuristic = CoarseHeuristic(key.Start, key.Goal);
			open.push({initialHeuristic, initialHeuristic, key.Start});
			const auto expansionLimit = std::clamp<std::size_t>((static_cast<std::size_t>(initialHeuristic) + 1) *
																	CoarseExpansionDistanceFactor,
																MinimumCoarseExpansions,
																MaximumCoarseExpansions);
			std::size_t expansions{};
			bool reached{};
			while (!open.empty() && expansions < expansionLimit)
			{
				const auto entry = open.top();
				open.pop();
				auto record = records.find(entry.Tile);
				if (record == records.end() || record->second.Closed)
					continue;
				if (entry.Score != record->second.Cost + CoarseHeuristic(entry.Tile, key.Goal))
					continue;
				record->second.Closed = true;
				++expansions;
				if (entry.Tile == key.Goal)
				{
					reached = true;
					break;
				}
				const auto neighbors = TileNeighbors(entry.Tile, profile);
				if (!neighbors)
					return std::nullopt;
				for (const auto neighbor : *neighbors)
				{
					const auto candidateCost = static_cast<std::uint32_t>(record->second.Cost + 1);
					auto [neighborRecord, inserted] = records.try_emplace(neighbor);
					if (!inserted && candidateCost >= neighborRecord->second.Cost)
						continue;
					neighborRecord->second.Cost = candidateCost;
					neighborRecord->second.Parent = entry.Tile;
					neighborRecord->second.HasParent = true;
					neighborRecord->second.Closed = false;
					const auto heuristic = CoarseHeuristic(neighbor, key.Goal);
					open.push({candidateCost + heuristic, heuristic, neighbor});
				}
			}

			CoarseCorridor corridor;
			for (const auto& [tile, record] : records)
				if (record.Closed)
					corridor.Dependencies.insert(tile);
			if (reached)
			{
				std::vector<TileKey> centerLine{key.Goal};
				auto current = key.Goal;
				while (current != key.Start)
				{
					const auto iterator = records.find(current);
					if (iterator == records.end() || !iterator->second.HasParent)
						break;
					current = iterator->second.Parent;
					centerLine.push_back(current);
				}
				std::ranges::reverse(centerLine);
				for (std::size_t index = 0; index < centerLine.size(); ++index)
				{
					const auto tile = centerLine[index];
					corridor.Allowed.push_back(tile);
					const auto target = index + 1 < centerLine.size()
						? FindPortal(tile, centerLine[index + 1], goal, profile).value_or(goal)
						: goal;
					corridor.Guidance.emplace(
						tile, TileGuidance{target, static_cast<std::uint32_t>(centerLine.size() - index - 1)});
				}
				std::ranges::sort(corridor.Allowed);
				corridor.Allowed.erase(std::unique(corridor.Allowed.begin(), corridor.Allowed.end()),
									   corridor.Allowed.end());
			}
			if (TopologyDemandGeneration != demandGeneration)
				return std::nullopt;
			Corridors.emplace(key, corridor);
			return corridor;
		}

		[[nodiscard]] static std::uint64_t GuidedHeuristic(const Search& search,
														   const VoxelApi::Position position) noexcept
		{
			const auto direct = Heuristic(position, search.Goal);
			const auto guidance = search.Guidance.find(ToTile(position));
			if (guidance == search.Guidance.end())
				return direct;
			const auto guided = Heuristic(position, guidance->second.Target) +
				static_cast<std::uint64_t>(guidance->second.RemainingTiles) * TileEdge * CardinalCost;
			return std::max(direct, guided);
		}

		void PublishPath(Request& request,
						 const std::span<const VoxelApi::Position> positions,
						 const Movement::Api::ProfileId profile)
		{
			auto path = std::make_shared<NavigationApi::Path>();
			path->EnvironmentRevision = Revision;
			path->ValidationToken = NextPathValidationToken++;
			path->Waypoints.reserve(positions.size());
			for (std::size_t index = 0; index < positions.size(); ++index)
			{
				auto primitive = NavigationApi::StandardPrimitives::Traverse;
				if (index != 0 && positions[index].Z > positions[index - 1].Z)
					primitive = NavigationApi::StandardPrimitives::Rise;
				if (index != 0 && positions[index].Z < positions[index - 1].Z)
					primitive = NavigationApi::StandardPrimitives::Drop;
				path->Waypoints.push_back({ToContinuous(positions[index]), primitive});
			}
			PathValidation validation{profile};
			for (const auto position : positions)
				validation.Dependencies.insert(ToTile(position));
			PathValidations.emplace(path->ValidationToken, std::move(validation));
			request.Path = std::move(path);
			request.State = NavigationApi::PlanState::Complete;
			request.SearchState.reset();
		}

		void Finish(Request& request, const VoxelApi::Position goal)
		{
			auto positions = std::vector<VoxelApi::Position>{goal};
			auto current = goal;
			while (true)
			{
				const auto iterator = request.SearchState->Records.find(current);
				if (iterator == request.SearchState->Records.end() || !iterator->second.HasParent)
					break;
				current = iterator->second.Parent;
				positions.push_back(current);
			}
			std::ranges::reverse(positions);
			PublishPath(request, positions, request.SearchState->Profile);
		}

		[[nodiscard]] DirectPathState TryDirectPath(Request& request, const Movement::Api::GroundedProfile& profile)
		{
			assert(request.PendingPlan && request.ProjectedStart && request.ProjectedGoal);
			if (request.ProjectedStart->Z != request.ProjectedGoal->Z)
				return DirectPathState::Blocked;

			const auto goal = *request.ProjectedGoal;
			const auto maximumSteps = static_cast<std::size_t>(
				std::max(std::abs(static_cast<std::int64_t>(goal.X) - request.ProjectedStart->X),
						 std::abs(static_cast<std::int64_t>(goal.Y) - request.ProjectedStart->Y)));
			auto current = *request.ProjectedStart;
			std::set<TileKey> dependencies{ToTile(current)};
			std::array<std::optional<std::pair<ProfileTileKey, const Tile*>>, 8> tileCache;
			std::size_t nextCacheEntry{};
			const auto bounds = Environment.Bounds();
			const auto standable = [&](const VoxelApi::Position position) -> std::optional<bool>
			{
				if (!bounds.Contains(position))
					return false;
				const ProfileTileKey key{profile.Id, ToTile(position)};
				for (const auto& entry : tileCache)
					if (entry && entry->first == key)
						return entry->second->Standable[LocalIndex(position, key.Tile)] != 0;
				if (!IsTileReady(key))
					return std::nullopt;
				const auto tile = Tiles.find(key);
				if (tile == Tiles.end())
					return false;
				tileCache[nextCacheEntry] = std::pair{key, &tile->second};
				nextCacheEntry = (nextCacheEntry + 1) % tileCache.size();
				return tile->second.Standable[LocalIndex(position, key.Tile)] != 0;
			};
			std::size_t steps{};
			while (current != goal)
			{
				const auto sign = [](const std::int32_t value) { return value < 0 ? -1 : value > 0 ? 1 : 0; };
				const auto dx = sign(goal.X - current.X);
				const auto dy = sign(goal.Y - current.Y);
				if (dx == 0 && dy == 0)
					return DirectPathState::Blocked;
				const VoxelApi::Position next{current.X + dx, current.Y + dy, current.Z};
				const auto nextStandable = standable(next);
				if (!nextStandable)
					return DirectPathState::Pending;
				if (!*nextStandable)
					return DirectPathState::Blocked;
				if (dx != 0 && dy != 0)
				{
					const auto xSide = standable({current.X + dx, current.Y, current.Z});
					const auto ySide = standable({current.X, current.Y + dy, current.Z});
					if (!xSide || !ySide)
						return DirectPathState::Pending;
					if (!*xSide || !*ySide)
						return DirectPathState::Blocked;
				}
				current = next;
				dependencies.insert(ToTile(current));
				++steps;
				if (steps > maximumSteps || current.Z != goal.Z)
					return DirectPathState::Blocked;
			}
			auto path = std::make_shared<NavigationApi::Path>();
			path->EnvironmentRevision = Revision;
			path->ValidationToken = NextPathValidationToken++;
			path->Waypoints.push_back(
				{ToContinuous(*request.ProjectedStart), NavigationApi::StandardPrimitives::Traverse});
			if (*request.ProjectedStart != goal)
				path->Waypoints.push_back({ToContinuous(goal), NavigationApi::StandardPrimitives::Traverse});
			PathValidations.emplace(path->ValidationToken, PathValidation{profile.Id, std::move(dependencies)});
			request.Path = std::move(path);
			request.State = NavigationApi::PlanState::Complete;
			request.SearchState.reset();
			return DirectPathState::Complete;
		}

		[[nodiscard]] bool PositionDependenciesReady(const VoxelApi::Position current,
													 const Movement::Api::GroundedProfile& profile)
		{
			auto ready = true;
			const auto bounds = Environment.Bounds();
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx)
				{
					if (dx == 0 && dy == 0)
						continue;
					for (auto elevation = -static_cast<std::int32_t>(profile.MaximumDrop);
						 elevation <= static_cast<std::int32_t>(profile.MaximumRise);
						 ++elevation)
					{
						const VoxelApi::Position candidate{current.X + dx, current.Y + dy, current.Z + elevation};
						if (!bounds.Contains(candidate))
							continue;
						const ProfileTileKey dependency{profile.Id, ToTile(candidate)};
						QueueTile(dependency);
						ready = IsTileReady(dependency) && ready;
					}
				}
			return ready;
		}

		[[nodiscard]] bool ExpandOne(Request& request)
		{
			auto& search = *request.SearchState;
			const auto* profile = Profile(search.Profile);
			assert(profile != nullptr);
			while (!search.Open.empty())
			{
				const auto entry = search.Open.top();
				search.Open.pop();
				auto record = search.Records.find(entry.Position);
				if (record == search.Records.end() || record->second.Closed)
					continue;
				const auto expectedHeuristic = GuidedHeuristic(search, entry.Position);
				const auto expectedScore = record->second.Cost + expectedHeuristic * 5 / 4;
				if (entry.Score != expectedScore)
					continue;
				if (!PositionDependenciesReady(entry.Position, *profile))
				{
					search.Open.push(entry);
					return false;
				}
				record->second.Closed = true;
				if (search.ExpandedNodes == MaximumExpansionsPerRequest)
				{
					request.State = NavigationApi::PlanState::Unreachable;
					request.SearchState.reset();
					return true;
				}
				++search.ExpandedNodes;
				if (entry.Position == search.Goal)
				{
					Finish(request, entry.Position);
					return true;
				}

				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
					{
						if (dx == 0 && dy == 0)
							continue;
						const auto neighbor = Neighbor(entry.Position, dx, dy, *profile);
						if (!neighbor)
							continue;
						if (!search.Corridor.empty() && !search.Corridor.contains(ToTile(*neighbor)))
							continue;
						const auto transitionCost = dx != 0 && dy != 0 ? DiagonalCost : CardinalCost;
						const auto verticalCost =
							static_cast<std::uint64_t>(std::abs(neighbor->Z - entry.Position.Z)) * 250;
						const auto candidateCost = record->second.Cost + transitionCost + verticalCost;
						auto [neighborRecord, inserted] = search.Records.try_emplace(*neighbor);
						if (!inserted && candidateCost >= neighborRecord->second.Cost)
							continue;
						neighborRecord->second.Cost = candidateCost;
						neighborRecord->second.Parent = entry.Position;
						neighborRecord->second.HasParent = true;
						neighborRecord->second.Closed = false;
						const auto heuristic = GuidedHeuristic(search, *neighbor);
						search.Open.push({candidateCost + heuristic * 5 / 4, heuristic, *neighbor});
					}
				return true;
			}
			if (!search.Corridor.empty() && !search.FallbackUsed)
			{
				search.Corridor.clear();
				search.Guidance.clear();
				search.FallbackUsed = true;
				search.Records.clear();
				while (!search.Open.empty())
					search.Open.pop();
				search.Records[search.Start].Cost = 0;
				const auto heuristic = GuidedHeuristic(search, search.Start);
				search.Open.push({heuristic * 5 / 4, heuristic, search.Start});
				return true;
			}
			request.State = NavigationApi::PlanState::Unreachable;
			request.SearchState.reset();
			return true;
		}

		[[nodiscard]] auto FindReachabilityRequest(const NavigationApi::ReachabilityRequestId id) noexcept
		{
			return std::ranges::lower_bound(
				ReachabilityRequests, id, {}, [](const ReachabilityRequest& request) { return request.Query.Request; });
		}

		[[nodiscard]] auto FindReachabilityRequest(const NavigationApi::ReachabilityRequestId id) const noexcept
		{
			return std::ranges::lower_bound(
				ReachabilityRequests, id, {}, [](const ReachabilityRequest& request) { return request.Query.Request; });
		}

		void InitializePendingTopology()
		{
			auto remainingPlanProjections = PlanEndpointProjectionsPerTick;
			const auto maximumChecks = std::min(PlanEndpointChecksPerTick, Requests.size());
			std::size_t checked{};
			while (remainingPlanProjections != 0 && checked < maximumChecks && !Requests.empty())
			{
				if (PendingEndpointCursor >= Requests.size())
					PendingEndpointCursor = 0;
				auto& request = Requests[PendingEndpointCursor];
				PendingEndpointCursor = (PendingEndpointCursor + 1) % Requests.size();
				++checked;
				if (!request.PendingPlan)
					continue;
				const auto* profile = Profile(request.PendingPlan->Profile);
				assert(profile != nullptr);
				if (!request.ProjectedStart)
				{
					if (!ProjectionReady(request.PendingPlan->Start, *profile))
						continue;
					request.ProjectedStart = Project(request.PendingPlan->Start, *profile);
					--remainingPlanProjections;
					if (!request.ProjectedStart)
					{
						request.State = NavigationApi::PlanState::Unreachable;
						request.PendingPlan.reset();
						continue;
					}
				}
				if (remainingPlanProjections == 0)
					break;
				if (!request.ProjectedGoal)
				{
					if (!ProjectionReady(request.PendingPlan->Goal, *profile))
						continue;
					request.ProjectedGoal = Project(request.PendingPlan->Goal, *profile);
					--remainingPlanProjections;
					if (!request.ProjectedGoal)
					{
						request.State = NavigationApi::PlanState::Unreachable;
						request.PendingPlan.reset();
						continue;
					}
				}
			}
			UNREALVOXELSIM_PROFILE_PLOT(Profiling, "Planner endpoint checks", checked);

			auto remaining = ReachabilityComponentExpansionsPerTick;
			for (auto& request : ReachabilityRequests)
			{
				if (remaining == 0)
					break;
				if (request.Initialized || request.Result->IsComplete())
					continue;
				const auto* profile = Profile(request.Query.Profile);
				assert(profile != nullptr);
				if (!request.SourceResolved)
				{
					if (!ProjectionReady(request.Query.Start, *profile))
						continue;
					const auto source = Project(request.Query.Start, *profile);
					--remaining;
					request.SourceResolved = true;
					if (source)
						request.Source = ComponentAt(*source, *profile);
					if (!request.Source)
					{
						std::ranges::fill(request.Result->Destinations, NavigationApi::ReachabilityState::InvalidStart);
						request.Initialized = true;
						continue;
					}
					EnsureComponentSearch(*request.Source);
				}
				while (remaining != 0 && request.NextDestination < request.Query.Destinations.size())
				{
					if (!ProjectionReady(request.Query.Destinations[request.NextDestination], *profile))
						break;
					const auto destination = Project(request.Query.Destinations[request.NextDestination], *profile);
					--remaining;
					if (destination)
						request.Goals[request.NextDestination] = ComponentAt(*destination, *profile);
					if (!request.Goals[request.NextDestination])
						request.Result->Destinations[request.NextDestination] =
							NavigationApi::ReachabilityState::InvalidGoal;
					++request.NextDestination;
				}
				request.Initialized = request.NextDestination == request.Query.Destinations.size();
				if (request.Source)
					EnsureComponentSearch(*request.Source);
			}
		}

		[[nodiscard]] bool ResolveReachability(const bool allowColdWork)
		{
			auto builtColdCorridor = false;
			auto incomingBuildBudget = allowColdWork ? ColdIncomingComponentBuildsPerTick : 0;
			auto incomingCellBudget = ComponentCellsPerTick;
			const auto incomingEmpty = [&](const ComponentKey key) -> std::optional<bool>
			{
				if (!ComponentIncomingCache.contains(key))
				{
					if (incomingBuildBudget == 0)
						return std::nullopt;
					--incomingBuildBudget;
					if (!AdvanceIncomingEdges(key, incomingCellBudget))
						return std::nullopt;
				}
				const auto cached = ComponentIncomingCache.find(key);
				assert(cached != ComponentIncomingCache.end());
				return cached->second.empty();
			};
			for (auto& request : ReachabilityRequests)
			{
				if (!request.Initialized || !request.Source || request.Result->IsComplete())
					continue;
				const auto search = FindComponentSearch(*request.Source);
				if (search == ComponentSearches.end())
					continue;
				for (std::size_t index = 0; index < request.Goals.size(); ++index)
				{
					if (request.Result->Destinations[index] != NavigationApi::ReachabilityState::Pending ||
						!request.Goals[index])
						continue;
					if (search->Visited.contains(*request.Goals[index]))
						request.Result->Destinations[index] = NavigationApi::ReachabilityState::Reachable;
					else
					{
						if (incomingEmpty(*request.Goals[index]).value_or(false))
							request.Result->Destinations[index] = NavigationApi::ReachabilityState::Unreachable;
						if (search->Complete)
							request.Result->Destinations[index] = NavigationApi::ReachabilityState::Unreachable;
					}
				}
			}

			auto promotionBudget = PlanPromotionsPerTick;
			auto coldCorridorBudget = allowColdWork ? ColdCorridorBuildsPerTick : 0;
			for (auto& request : Requests)
			{
				if (promotionBudget == 0)
					break;
				if (!request.PendingPlan || !request.ProjectedStart || !request.ProjectedGoal)
					continue;

				const auto* profile = Profile(request.PendingPlan->Profile);
				assert(profile != nullptr);
				const auto direct = TryDirectPath(request, *profile);
				if (direct == DirectPathState::Pending)
					continue;
				if (direct == DirectPathState::Complete)
				{
					request.PendingPlan.reset();
					--promotionBudget;
					continue;
				}
				const CorridorKey corridorKey{
					profile->Id, ToTile(*request.ProjectedStart), ToTile(*request.ProjectedGoal)};
				const auto corridorCached = Corridors.contains(corridorKey);
				if (!corridorCached && coldCorridorBudget == 0)
					continue;
				const auto corridor = BuildCorridor(*request.ProjectedStart, *request.ProjectedGoal, *profile);
				if (!corridor)
					continue;
				request.SearchState.emplace(
					Search{request.PendingPlan->Profile, *request.ProjectedStart, *request.ProjectedGoal});
				request.SearchState->Corridor.insert(corridor->Allowed.begin(), corridor->Allowed.end());
				request.SearchState->Guidance = corridor->Guidance;
				request.SearchState->FallbackUsed = corridor->Allowed.empty();
				request.SearchState->Records[*request.ProjectedStart].Cost = 0;
				const auto heuristic = GuidedHeuristic(*request.SearchState, *request.ProjectedStart);
				request.SearchState->Open.push({heuristic * 5 / 4, heuristic, *request.ProjectedStart});
				request.PendingPlan.reset();
				AddActive(request.Id);
				--promotionBudget;
				if (!corridorCached)
				{
					--coldCorridorBudget;
					builtColdCorridor = true;
				}
			}
			return builtColdCorridor;
		}

		const EnvironmentApi::IEnvironment& Environment;
		ProfilingApi::NullRecorder NullProfiling;
		ProfilingApi::IRecorder& Profiling;
		std::vector<Movement::Api::GroundedProfile> Profiles;
		std::size_t ExpansionsPerTick;
		std::size_t MaximumExpansionsPerRequest;
		std::size_t ReachabilityComponentExpansionsPerTick;
		std::size_t TileBuildsPerTopologyUpdate;
		std::size_t ComponentCellsPerTick;
		std::map<ProfileTileKey, Tile> Tiles;
		std::set<ProfileTileKey> PendingTileBuilds;
		std::set<ProfileTileKey> UrgentTileBuilds;
		std::deque<ProfileTileKey> UrgentTileBuildOrder;
		std::deque<ProfileTileKey> BackgroundTileBuildOrder;
		std::uint64_t TopologyDemandGeneration{};
		bool TopologyBuiltSincePlannerAdvance{};
		std::map<ProfileTileKey, std::vector<TileKey>> TileNeighborCache;
		std::map<CorridorKey, CoarseCorridor> Corridors;
		std::map<ComponentKey, std::vector<ComponentKey>> ComponentEdgeCache;
		std::map<ComponentKey, std::vector<ComponentKey>> ComponentIncomingCache;
		std::map<ComponentKey, ComponentEdgeBuild> PendingComponentEdgeBuilds;
		std::map<ComponentKey, ComponentEdgeBuild> PendingComponentIncomingBuilds;
		std::vector<ComponentSearch> ComponentSearches;
		std::vector<ComponentKey> ActiveComponentSearches;
		std::size_t ActiveComponentCursor{};
		std::vector<ReachabilityRequest> ReachabilityRequests;
		std::vector<Request> Requests;
		std::size_t PendingEndpointCursor{};
		std::vector<NavigationApi::RequestId> ActiveRequests;
		std::size_t ActiveCursor{};
		std::map<std::uint64_t, PathValidation> PathValidations;
		std::uint64_t NextPathValidationToken{1};
		std::uint64_t Revision{1};
		std::thread::id OwnerThread{std::this_thread::get_id()};
	};

	Planner::Planner(const EnvironmentApi::IEnvironment& environment,
					 const std::span<const Movement::Api::GroundedProfile> profiles,
					 const std::size_t expansionsPerTick,
					 const std::size_t maximumExpansionsPerRequest,
					 const std::size_t reachabilityComponentExpansionsPerTick,
					 const std::size_t tileBuildsPerTopologyUpdate,
					 const std::size_t componentCellsPerTick) :
		m_Impl(std::make_unique<Impl>(environment,
									  profiles,
									  nullptr,
									  expansionsPerTick,
									  maximumExpansionsPerRequest,
									  reachabilityComponentExpansionsPerTick,
									  tileBuildsPerTopologyUpdate,
									  componentCellsPerTick))
	{
	}

	Planner::Planner(const EnvironmentApi::IEnvironment& environment,
					 const std::span<const Movement::Api::GroundedProfile> profiles,
					 ProfilingApi::IRecorder& profiling,
					 const std::size_t expansionsPerTick,
					 const std::size_t maximumExpansionsPerRequest,
					 const std::size_t reachabilityComponentExpansionsPerTick,
					 const std::size_t tileBuildsPerTopologyUpdate,
					 const std::size_t componentCellsPerTick) :
		m_Impl(std::make_unique<Impl>(environment,
									  profiles,
									  &profiling,
									  expansionsPerTick,
									  maximumExpansionsPerRequest,
									  reachabilityComponentExpansionsPerTick,
									  tileBuildsPerTopologyUpdate,
									  componentCellsPerTick))
	{
	}

	Planner::~Planner() = default;

	std::expected<void, NavigationApi::PlanError> Planner::Begin(const NavigationApi::PlanRequest request)
	{
		m_Impl->AssertOwnerThread();
		if (!request.Request.IsValid())
			return std::unexpected{NavigationApi::PlanError::InvalidRequest};
		const auto* profile = m_Impl->Profile(request.Profile);
		if (!profile)
			return std::unexpected{NavigationApi::PlanError::UnknownProfile};
		const auto iterator = m_Impl->FindRequest(request.Request);
		if (iterator != m_Impl->Requests.end() && iterator->Id == request.Request)
			return std::unexpected{NavigationApi::PlanError::DuplicateRequest};
		Request planned{request.Request};
		planned.Plan = request;
		planned.PendingPlan = request;
		const auto requestIndex = static_cast<std::size_t>(std::distance(m_Impl->Requests.begin(), iterator));
		if (requestIndex < m_Impl->PendingEndpointCursor)
			++m_Impl->PendingEndpointCursor;
		m_Impl->Requests.insert(iterator, std::move(planned));
		m_Impl->QueueProjection(request.Start, *profile);
		m_Impl->QueueProjection(request.Goal, *profile);
		m_Impl->QueueRouteHint(request.Start, request.Goal, *profile);
		return {};
	}

	void Planner::Cancel(const NavigationApi::RequestId request) noexcept
	{
		m_Impl->AssertOwnerThread();
		const auto iterator = m_Impl->FindRequest(request);
		if (iterator != m_Impl->Requests.end() && iterator->Id == request)
		{
			if (iterator->Path)
				m_Impl->PathValidations.erase(iterator->Path->ValidationToken);
			m_Impl->RemoveActive(request);
			const auto requestIndex = static_cast<std::size_t>(std::distance(m_Impl->Requests.begin(), iterator));
			if (requestIndex < m_Impl->PendingEndpointCursor)
				--m_Impl->PendingEndpointCursor;
			m_Impl->Requests.erase(iterator);
			if (m_Impl->PendingEndpointCursor >= m_Impl->Requests.size())
				m_Impl->PendingEndpointCursor = 0;
		}
	}

	void Planner::Advance(const Simulation::Api::StepContext context)
	{
		UNREALVOXELSIM_PROFILE_ZONE(m_Impl->Profiling, "Voxel planner advance");
		static_cast<void>(context);
		m_Impl->AssertOwnerThread();
		{
			UNREALVOXELSIM_PROFILE_ZONE(m_Impl->Profiling, "Resolve request endpoints");
			m_Impl->InitializePendingTopology();
		}
		const auto topologyBuilt = m_Impl->TopologyBuiltSincePlannerAdvance;
		m_Impl->TopologyBuiltSincePlannerAdvance = false;
		if (!topologyBuilt)
		{
			UNREALVOXELSIM_PROFILE_ZONE(m_Impl->Profiling, "Expand reachability components");
			m_Impl->AdvanceComponentSearches();
		}
		bool builtColdCorridor{};
		{
			UNREALVOXELSIM_PROFILE_ZONE(m_Impl->Profiling, "Resolve reachability and corridors");
			builtColdCorridor = m_Impl->ResolveReachability(!topologyBuilt);
		}
		std::size_t remaining = builtColdCorridor || topologyBuilt
			? std::min(m_Impl->ExpansionsPerTick, FineExpansionsAfterColdCorridor)
			: m_Impl->ExpansionsPerTick;
		{
			UNREALVOXELSIM_PROFILE_ZONE(m_Impl->Profiling, "Expand fine paths");
			while (remaining != 0 && !m_Impl->ActiveRequests.empty())
			{
				if (m_Impl->ActiveCursor >= m_Impl->ActiveRequests.size())
					m_Impl->ActiveCursor = 0;
				const auto requestId = m_Impl->ActiveRequests[m_Impl->ActiveCursor];
				const auto iterator = m_Impl->FindRequest(requestId);
				if (iterator == m_Impl->Requests.end() || iterator->Id != requestId ||
					iterator->State != NavigationApi::PlanState::Pending || !iterator->SearchState)
				{
					m_Impl->RemoveActive(requestId);
					continue;
				}
				if (!m_Impl->ExpandOne(*iterator))
					break;
				--remaining;
				if (iterator->State == NavigationApi::PlanState::Pending)
					m_Impl->ActiveCursor = (m_Impl->ActiveCursor + 1) % m_Impl->ActiveRequests.size();
				else
					m_Impl->RemoveActive(requestId);
			}
		}
		UNREALVOXELSIM_PROFILE_PLOT(m_Impl->Profiling, "Planner requests", m_Impl->Requests.size());
		UNREALVOXELSIM_PROFILE_PLOT(m_Impl->Profiling, "Planner active fine requests", m_Impl->ActiveRequests.size());
		UNREALVOXELSIM_PROFILE_PLOT(
			m_Impl->Profiling, "Planner reachability requests", m_Impl->ReachabilityRequests.size());
		UNREALVOXELSIM_PROFILE_PLOT(
			m_Impl->Profiling, "Planner active component searches", m_Impl->ActiveComponentSearches.size());
		UNREALVOXELSIM_PROFILE_PLOT(m_Impl->Profiling, "Planner cached navigation tiles", m_Impl->Tiles.size());
		UNREALVOXELSIM_PROFILE_PLOT(
			m_Impl->Profiling, "Planner pending navigation tiles", m_Impl->PendingTileBuilds.size());
	}

	std::uint64_t Planner::CurrentEnvironmentRevision() const noexcept
	{
		m_Impl->AssertOwnerThread();
		return m_Impl->Revision;
	}

	bool Planner::IsPathCurrent(const NavigationApi::Path& path) const noexcept
	{
		m_Impl->AssertOwnerThread();
		const auto validation = m_Impl->PathValidations.find(path.ValidationToken);
		return validation != m_Impl->PathValidations.end() && validation->second.Current;
	}

	NavigationApi::PlanState Planner::State(const NavigationApi::RequestId request) const noexcept
	{
		m_Impl->AssertOwnerThread();
		const auto iterator = m_Impl->FindRequest(request);
		return iterator != m_Impl->Requests.end() && iterator->Id == request ? iterator->State
																			 : NavigationApi::PlanState::Cancelled;
	}

	std::shared_ptr<const NavigationApi::Path> Planner::ReadPath(const NavigationApi::RequestId request) const noexcept
	{
		m_Impl->AssertOwnerThread();
		const auto iterator = m_Impl->FindRequest(request);
		return iterator != m_Impl->Requests.end() && iterator->Id == request ? iterator->Path : nullptr;
	}

	std::expected<void, NavigationApi::ReachabilityError>
	Planner::BeginReachability(NavigationApi::ReachabilityQuery query)
	{
		m_Impl->AssertOwnerThread();
		if (!query.Request.IsValid())
			return std::unexpected{NavigationApi::ReachabilityError::InvalidRequest};
		if (!m_Impl->Profile(query.Profile))
			return std::unexpected{NavigationApi::ReachabilityError::UnknownProfile};
		if (query.Destinations.empty())
			return std::unexpected{NavigationApi::ReachabilityError::EmptyDestinations};
		const auto iterator = m_Impl->FindReachabilityRequest(query.Request);
		if (iterator != m_Impl->ReachabilityRequests.end() && iterator->Query.Request == query.Request)
			return std::unexpected{NavigationApi::ReachabilityError::DuplicateRequest};
		auto result = std::make_shared<NavigationApi::ReachabilityResult>();
		result->Request = query.Request;
		result->EnvironmentRevision = m_Impl->Revision;
		result->Destinations.resize(query.Destinations.size(), NavigationApi::ReachabilityState::Pending);
		ReachabilityRequest request{std::move(query), std::move(result)};
		request.Goals.resize(request.Query.Destinations.size());
		m_Impl->QueueProjection(request.Query.Start, *m_Impl->Profile(request.Query.Profile));
		for (const auto destination : request.Query.Destinations)
			m_Impl->QueueProjection(destination, *m_Impl->Profile(request.Query.Profile));
		m_Impl->ReachabilityRequests.insert(iterator, std::move(request));
		return {};
	}

	void Planner::CancelReachability(const NavigationApi::ReachabilityRequestId request) noexcept
	{
		m_Impl->AssertOwnerThread();
		const auto iterator = m_Impl->FindReachabilityRequest(request);
		if (iterator == m_Impl->ReachabilityRequests.end() || iterator->Query.Request != request)
			return;
		for (auto& state : iterator->Result->Destinations)
			state = NavigationApi::ReachabilityState::Cancelled;
		iterator->Initialized = true;
	}

	std::shared_ptr<const NavigationApi::ReachabilityResult>
	Planner::ReadReachability(const NavigationApi::ReachabilityRequestId request) const noexcept
	{
		m_Impl->AssertOwnerThread();
		const auto iterator = m_Impl->FindReachabilityRequest(request);
		return iterator != m_Impl->ReachabilityRequests.end() && iterator->Query.Request == request ? iterator->Result
																									: nullptr;
	}

	void Planner::Invalidate(const std::span<const VoxelApi::Region> regions)
	{
		UNREALVOXELSIM_PROFILE_ZONE(m_Impl->Profiling, "Invalidate navigation topology");
		m_Impl->AssertOwnerThread();
		if (regions.empty())
			return;
		++m_Impl->Revision;
		std::set<ProfileTileKey> affectedTiles;
		for (auto tile = m_Impl->Tiles.begin(); tile != m_Impl->Tiles.end();)
		{
			if (!std::ranges::any_of(regions,
									 [&](const auto region) { return Intersects(tile->second.Dependency, region); }))
			{
				++tile;
				continue;
			}
			affectedTiles.insert(tile->first);
			tile = m_Impl->Tiles.erase(tile);
		}
		m_Impl->QueueAffectedRegions(regions);

		std::erase_if(m_Impl->TileNeighborCache,
					  [&](const auto& entry)
					  { return m_Impl->GraphTileAffected(entry.first.Profile, entry.first.Tile, affectedTiles); });
		std::erase_if(m_Impl->Corridors,
					  [&](const auto& entry)
					  {
						  return std::ranges::any_of(
							  entry.second.Dependencies,
							  [&](const auto tile)
							  { return m_Impl->GraphTileAffected(entry.first.Profile, tile, affectedTiles); });
					  });
		const auto componentAffected = [&](const auto& entry)
		{ return m_Impl->ComponentAffected(entry.first, affectedTiles); };
		std::erase_if(m_Impl->ComponentEdgeCache, componentAffected);
		std::erase_if(m_Impl->ComponentIncomingCache, componentAffected);
		std::erase_if(m_Impl->PendingComponentEdgeBuilds, componentAffected);
		std::erase_if(m_Impl->PendingComponentIncomingBuilds, componentAffected);

		std::set<ComponentKey> invalidatedSearches;
		std::erase_if(m_Impl->ComponentSearches,
					  [&](const ComponentSearch& search)
					  {
						  const auto affected =
							  std::ranges::any_of(search.Visited,
												  [&](const auto component)
												  { return m_Impl->ComponentAffected(component, affectedTiles); });
						  if (affected)
							  invalidatedSearches.insert(search.Source);
						  return affected;
					  });
		std::erase_if(m_Impl->ActiveComponentSearches,
					  [&](const auto source) { return invalidatedSearches.contains(source); });
		if (m_Impl->ActiveComponentCursor >= m_Impl->ActiveComponentSearches.size())
			m_Impl->ActiveComponentCursor = 0;

		for (auto& entry : m_Impl->PathValidations)
		{
			auto& validation = entry.second;
			if (validation.Current &&
				std::ranges::any_of(validation.Dependencies,
									[&](const auto tile)
									{ return m_Impl->GraphTileAffected(validation.Profile, tile, affectedTiles); }))
				validation.Current = false;
		}

		for (auto& request : m_Impl->ReachabilityRequests)
		{
			const auto cancelled = std::ranges::all_of(
				request.Result->Destinations,
				[](const auto state) { return state == NavigationApi::ReachabilityState::Cancelled; });
			if (cancelled)
				continue;
			request.Result->EnvironmentRevision = m_Impl->Revision;
			const auto endpointAffected =
				m_Impl->GraphTileAffected(request.Query.Profile, ToTile(ToVoxel(request.Query.Start)), affectedTiles) ||
				std::ranges::any_of(request.Query.Destinations,
									[&](const auto destination)
									{
										return m_Impl->GraphTileAffected(
											request.Query.Profile, ToTile(ToVoxel(destination)), affectedTiles);
									});
			const auto topologyAffected = (request.Source &&
										   (m_Impl->ComponentAffected(*request.Source, affectedTiles) ||
											invalidatedSearches.contains(*request.Source))) ||
				std::ranges::any_of(request.Goals,
									[&](const auto& goal)
									{ return goal && m_Impl->ComponentAffected(*goal, affectedTiles); });
			if (!endpointAffected && !topologyAffected)
				continue;
			std::ranges::fill(request.Result->Destinations, NavigationApi::ReachabilityState::Pending);
			request.Source.reset();
			std::ranges::fill(request.Goals, std::nullopt);
			request.NextDestination = 0;
			request.SourceResolved = false;
			request.Initialized = false;
			const auto* profile = m_Impl->Profile(request.Query.Profile);
			assert(profile != nullptr);
			m_Impl->QueueProjection(request.Query.Start, *profile);
			for (const auto destination : request.Query.Destinations)
				m_Impl->QueueProjection(destination, *profile);
		}
		for (auto& request : m_Impl->Requests)
		{
			if (request.State != NavigationApi::PlanState::Pending)
				continue;
			const auto endpointAffected =
				m_Impl->GraphTileAffected(request.Plan.Profile, ToTile(ToVoxel(request.Plan.Start)), affectedTiles) ||
				m_Impl->GraphTileAffected(request.Plan.Profile, ToTile(ToVoxel(request.Plan.Goal)), affectedTiles);
			const auto topologyAffected = (request.ReachabilitySource &&
										   (m_Impl->ComponentAffected(*request.ReachabilitySource, affectedTiles) ||
											invalidatedSearches.contains(*request.ReachabilitySource))) ||
				(request.ReachabilityGoal && m_Impl->ComponentAffected(*request.ReachabilityGoal, affectedTiles)) ||
				(request.SearchState &&
				 std::ranges::any_of(request.SearchState->Records,
									 [&](const auto& record)
									 {
										 return m_Impl->GraphTileAffected(
											 request.SearchState->Profile, ToTile(record.first), affectedTiles);
									 }));
			if (!endpointAffected && !topologyAffected)
				continue;
			m_Impl->RemoveActive(request.Id);
			request.SearchState.reset();
			request.PendingPlan = request.Plan;
			request.ProjectedStart.reset();
			request.ProjectedGoal.reset();
			request.ReachabilitySource.reset();
			request.ReachabilityGoal.reset();
			m_Impl->QueueProjection(request.Plan.Start, *m_Impl->Profile(request.Plan.Profile));
			m_Impl->QueueProjection(request.Plan.Goal, *m_Impl->Profile(request.Plan.Profile));
			m_Impl->QueueRouteHint(request.Plan.Start, request.Plan.Goal, *m_Impl->Profile(request.Plan.Profile));
		}
	}

	void Planner::Prepare(const std::span<const VoxelApi::Region> regions)
	{
		UNREALVOXELSIM_PROFILE_ZONE(m_Impl->Profiling, "Prepare navigation topology");
		m_Impl->AssertOwnerThread();
		m_Impl->QueueAffectedRegions(regions);
	}

	void Planner::UpdateTopology(const Simulation::Api::StepContext context)
	{
		UNREALVOXELSIM_PROFILE_ZONE(m_Impl->Profiling, "Voxel topology update");
		static_cast<void>(context);
		m_Impl->AssertOwnerThread();
		m_Impl->UpdateTopology();
		UNREALVOXELSIM_PROFILE_PLOT(
			m_Impl->Profiling, "Planner urgent navigation tiles", m_Impl->UrgentTileBuilds.size());
	}
}
