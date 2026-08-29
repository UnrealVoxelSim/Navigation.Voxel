# UnrealVoxelSim.Navigation.Voxel

Deterministic, incremental grounded navigation over a batched voxel-environment projection. `Planner` implements
`Navigation::Api::IPlanner`, `Navigation::Api::IReachability`, and the preparation, invalidation, and topology-update
contracts from `Navigation.Voxel.Api`.

V1 supports profile-sized grounded bodies, eight-way horizontal travel, adjacent rises up to `MaximumRise`, and adjacent
drops up to `MaximumDrop`. It does not jump across gaps. Start and goal positions are continuous; planning projects them
to nearby standable voxel positions, while returned waypoints convert back to deterministic continuous coordinates.

## Derived topology

Topology is cached in profile-keyed 16 x 16 x 16 standability tiles. A tile is derived from one batched environment read
covering the tile plus the profile footprint, body height, rise headroom, and drop shaft dependencies. For every candidate
foot position it stores:

- whether every footprint cell has supporting voxels beneath it;
- the vertical occupancy clearance available to the body; and
- a directed local connectivity component for standable positions.

Different body dimensions and rise/drop capabilities therefore produce different topology while sharing the same
navigation-oriented environment projection. Derived tiles, component edges, incoming edges, coarse corridors, and
component searches are reconstructible caches, never authoritative world state.

`Prepare(regions)` queues affected tiles for background construction. `Invalidate(regions)` increments the environment
revision, removes every tile whose dependency region intersects a change, selectively removes graph/search caches that
can depend on those tiles, and queues affected replacements. Published paths retain tile dependency tokens, so active
followers replan only when a changed region can affect their path.

`UpdateTopology` builds a configurable number of tiles per call. Tiles required by active queries are urgent and take
priority; background preparation yields while navigation demand exists. `Advance` never synchronously builds a tile, so
cold requests pause for deterministic simulation steps instead of causing an unbounded presentation-frame stall.
The default topology budget builds sixteen tiles per update. Unsupported candidate positions bypass occupancy-clearance
scanning, keeping empty and sparse tiles cheap enough to process in batches. A second deterministic standable-position
budget prevents a batch containing many populated walking surfaces from producing the same spike as sixteen worst-case
tiles.
New path requests also enqueue a narrow deterministic topology hint around the direct tile route when both endpoints are
at plausible grounded elevations. This removes the one-tile-at-a-time discovery chain without treating the hint as a
valid path; coarse and fine searches still validate all movement against constructed topology.

## Transition rules

A node represents a standable foot position, not an empty voxel in isolation. For each of the eight horizontal neighbor
directions, the planner tests same-height travel, then the nearest allowed rise, then the nearest allowed drop.

A transition is accepted only when:

- the destination supports the whole profile footprint and has at least body-height occupancy clearance;
- diagonal movement also has both orthogonal side positions standable, preventing corner cutting;
- a rise has enough clearance above the launch column for the swept body; and
- a drop has enough destination-column clearance for the full descent shaft.

These conservative swept-clearance checks reject passages whose endpoints look valid but which continuous movement
cannot traverse. Movement primitives on the resulting waypoints label level traversal, rise, and drop; the movement
domain remains responsible for actual jump, gravity, and collision integration.

## Hierarchical planning

Path requests move through bounded stages:

1. Project the continuous endpoints to profile-valid standable positions once their tiles are available.
2. Build or reuse a bounded tile-level portal corridor between endpoint tiles.
3. Run weighted eight-way voxel A* inside the corridor, using stable integer costs and deterministic tie-breaking.
4. If corridor restriction is insufficient, retry the fine search without the restriction.
5. Publish an immutable revision-tagged `Path` with an opaque dependency-validation token.

Ordinary path requests do not run a separate component-reachability flood before corridor construction. That duplicated
most of the path search for every distinct starting component and could materialize large areas unrelated to the route.
The bounded coarse and fine searches already determine whether a path exists.

When the projected endpoints share an elevation, the planner first validates the deterministic direct route using the
same grounded transition rules as A*. Clear routes publish immediately without allocating per-node A* records or
constructing a coarse corridor. A missing topology dependency keeps the request pending; a genuine obstruction falls
through to the hierarchical planner.

Fine search uses cardinal cost `1000`, diagonal cost `1414`, a vertical penalty, and a 1.25 weighted heuristic. The
current planner does not apply `Navigation.Voxel.Api::Cell::TraversalCost`. Strict global optimality is intentionally not
promised; bounded latency, deterministic behavior, and shared work are favored for large entity counts and distances.

## Reachability

Reachability queries project one source and many destinations, then use the directed component graph and cached source
searches. Results are updated independently as components are discovered. Direction matters:
with asymmetric rise and drop limits, A reaching B does not imply B can reach A.

Source-component searches are shared across simultaneous and later path/reachability requests for the same profile.
Invalidation retains searches whose explored tiles are spatially independent of the change. Incoming-component checks
can prove isolated goals unreachable early. This makes AI feasibility queries substantially cheaper than constructing a
full path per candidate.

## Deterministic work scheduling

All progression is driven by explicit simulation calls and integer work counts, never elapsed wall-clock time. Constructor
parameters configure the principal budgets:

- `expansionsPerTick`: shared fine-search node expansions;
- `maximumExpansionsPerRequest`: deterministic per-request failure ceiling;
- `reachabilityComponentExpansionsPerTick`: completed component-graph expansions;
- `tileBuildsPerTopologyUpdate`: derived-topology construction work; and
- `componentCellsPerTick`: boundary cells examined while incrementally constructing outgoing or incoming component
  edges.

Defaults are public `constexpr` values on `Planner`. Active fine searches and component searches are advanced in stable
round-robin order. Coarse work, endpoint promotion, and cold-cache construction also have deterministic internal limits.
Changing simulation pacing or time compression changes how quickly ticks are presented, not which budget a tick receives
or the simulation result for the same ordered inputs.

Path endpoint admission has a separate deterministic budget from component reachability. Up to 1,024 pending path
requests and 2,048 cached endpoint projections are processed per tick, so low reachability budgets cannot accidentally
turn a large pawn population into a many-second admission queue.

The current implementation is thread-affine. Construct it and invoke all interfaces on one simulation thread; background
workers must operate on immutable snapshots and return through a future deterministic commit boundary rather than mutate
planner state directly.

The constructor overload accepting `Profiling::Api::IRecorder &` exposes batch-level zones for invalidation, bounded
tile construction, endpoint projection, component reachability, corridor resolution, and fine path expansion. The
original constructor owns a no-op recorder for compatibility. Profiling does not change deterministic work budgets or
add zones per voxel, pawn, request, or expanded node.

## API usage

- Task executors and input adapters should use `Navigation.Api::ICommandSink` and read-only
  `NavigationExecutionComponent` access, normally provided by `Navigation.Following` and the composition root.
- AI systems needing feasibility should use `Navigation.Api::IReachability` and tolerate pending results.
- Following implementations use `Navigation.Api::IPlanner` and immutable `Path` values.
- World adapters call `Prepare` after loading relevant regions and send committed changes through `Invalidate`.
- The composition root calls `UpdateTopology` and `Advance` at explicit positions in its tick pipeline.

The environment and movement profiles must outlive `Planner`. Request identifiers must be valid and unique while their
results are retained. Unknown profiles and duplicate identifiers are rejected synchronously; expensive work remains
asynchronous.
