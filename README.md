# UnrealVoxelSim.Navigation.Voxel

Voxel-backed incremental path planning and reachability. The implementation owns its voxel environment projection,
solid-change subscription, topology invalidation, topology maintenance, and planning lifecycle.

`Planner` exposes representation-independent planning capabilities through `Navigation.Api` and participates in the
generic simulation pipeline. `SolidEnvironment` and `SolidInvalidation` are Navigation-owned adapters over
`Voxel.Solid.Api`; the Voxel domain does not depend on Navigation.

`Follower` translates active paths into synchronous calls to `Movement.Api::IIntentReceiver`. It has no write or
structural authority over Movement's public `InputComponent`.
