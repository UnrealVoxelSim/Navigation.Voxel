# UnrealVoxelSim.Navigation.Voxel

Deterministic grounded planner over shared profile-keyed 16³ standability tiles. Layer payload is captured through the
batched `Navigation.Voxel.Api` environment contract; A* never dispatches into a voxel layer per node. Requests advance
under an exact shared node-expansion budget and use stable integer costs and tie-breaking.

V1 uses a cached tile-portal hierarchy to guide weighted eight-way voxel A*. Coarse corridors are shared by profile and
endpoint tiles; exact searches follow deterministic boundary portals and fall back to an unconstrained search if a coarse
corridor is insufficient. Adjacent rise and drop limits come from the movement profile. Strict global optimality is
intentionally not promised.

Grounded transitions validate conservative swept clearance, including launch-column headroom for rises and destination
shaft clearance for drops. Endpoint standability alone is not considered sufficient.

Each standability tile also carries profile-specific directed connectivity components. Component searches are cached by
source component, shared by path requests and batched reachability queries, and retain directionality for asymmetric rise
and drop rules. Impossible requests therefore terminate at the component graph without entering fine A*.

Cold topology is never constructed by `Planner::Advance`. Region preparation, voxel invalidation, and query demand enqueue
profile-tile work for the separately scheduled topology updater. The composition root decides when maintenance runs;
queries remain pending until their immutable tiles are ready. Active query dependencies take priority over background
preparation, while voxel changes proactively queue affected tiles.

Topology maintenance, component traversal, incoming-component checks, cold corridor promotion, and fine A* all use
deterministic, configurable work limits. The fine-search ceiling is also configurable; reaching it fails the request
deterministically instead of allowing an order to monopolize the simulation or presentation thread.
