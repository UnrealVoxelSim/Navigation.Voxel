# UnrealVoxelSim AGENTS.md

This repository is a module of UnrealVoxelSim, a modular C++ project for a deterministic, high-performance simulation
that can be represented by Unreal Engine or other adapters.

The simulation core is independent of presentation, input, persistence, networking, and engine integrations. Unreal
Engine is a primary representation layer, not the owner of simulation rules or state.

## Project structure

### Repository model

UnrealVoxelSim is split across multiple Git repositories. Each repository contains one narrowly scoped module and
normally produces one reusable CMake library target. Repository names omit the `UnrealVoxelSim` organization prefix;
the full CMake target and C++ namespace retain it.

Modules use this repository as their structural template. Shared CMake conventions are maintained in the private
`UnrealVoxelSim/Build.CMake` repository.

### Module directories

Modules use the following directory structure:

- **public**: public headers. Other modules may include these headers and link the module target.
- **src**: reusable production implementation. This is compiled into the main module library.
- **mock**: reusable test doubles for contracts from **public**. Other modules may use this target in tests.
- **test**: unit and contract tests. Test targets are never production dependencies.
- **app**: optional executables closely associated with the module, such as tools, examples, or composition roots.
  Application targets are never linked as dependencies.
- **benchmark**: optional performance benchmarks. Benchmark targets are never production dependencies.
- **docs**: optional module-specific design and usage documentation.

Directory paths mirror C++ namespaces. For example, `UnrealVoxelSim::ModuleTemplate::ModuleTemplate` is located at:

```text
public/UnrealVoxelSim/ModuleTemplate/ModuleTemplate.h
src/UnrealVoxelSim/ModuleTemplate/ModuleTemplate.cpp
test/UnrealVoxelSim/ModuleTemplate/ModuleTemplateTest.cpp
```

Header-only modules still define a CMake `INTERFACE` target and keep public headers under **public**. Empty directories
must not be added merely to reproduce the full template structure.

### Module responsibilities

- Every module has one clear responsibility.
- Avoid generic `Utility`, `Common`, `Helpers`, or `Misc` modules.
- Split modules when responsibilities or dependency reasons diverge.
- If a contract is shared by multiple modules, place it in a dedicated `*.Api` module.
- Implementation modules depend on API modules; API modules never depend on implementations.
- Avoid dependency cycles at all costs.

## Build system

### CMake

- CMake is the canonical build description for standalone modules.
- Always configure, build, and test through checked-in CMake presets. Do not use undocumented ad-hoc configure flags.
- Shared target configuration comes from `UnrealVoxelSim/Build.CMake`.
- Use target-based modern CMake. Do not use directory-wide include paths, link paths, compiler flags, or definitions.
- Every production target uses C++23 with extensions disabled unless a documented platform adapter requires otherwise.
- Prefer static linking for internal and external dependencies whenever the platform and dependency support it. Interface
  libraries remain header-only.
- Windows presets use the `x64-windows-static` vcpkg host and target triplets, set `BUILD_SHARED_LIBS=OFF`, and select the
  matching static MSVC runtime (`/MT` or `/MTd`). Do not override the runtime or linkage policy on individual targets.
- Generated build directories and dependency source trees are not authoritative and must never be edited.

### Dependency management

Dependency ownership has a strict boundary:

- **vcpkg** manages external C and C++ packages.
- **FetchContent** fetches internal repositories from the `UnrealVoxelSim` organization.
- Do not fetch external packages with FetchContent.
- Do not declare internal UnrealVoxelSim modules as vcpkg packages.

External dependencies are declared in `vcpkg.json` and resolved through the pinned registry baseline in
`vcpkg-configuration.json`. Use `find_package` and imported targets after vcpkg resolves them.

FetchContent is also used to obtain `Build.CMake`. Shared build code owns canonical internal dependency identities,
source overrides, Git references, and compact dependency directory names; individual modules must not bypass it for
ordinary internal dependencies.

Internal FetchContent identities are derived from canonical repository names, not caller-selected dependency aliases.
On Windows, presets shorten those identities with a deterministic hash suffix and CMake policy `CMP0168` remains `NEW`
to avoid redundant FetchContent sub-build paths. Do not work around path limits by inventing module-specific build-tree
layouts or weakening canonical target and namespace names.

Internal dependencies may follow a moving development reference or use an explicit immutable pin. An explicit
per-dependency Git reference takes precedence over the configured project-wide reference, which defaults to `main` for
active development. Use immutable commit hashes when reproducibility is required. The build system must provide a safe,
documented way to refresh generated checkouts used by moving references.

For simultaneous work on sibling repositories, use `UNREALVOXELSIM_INTERNAL_SOURCE_ROOT` or a documented per-module
source override. Never introduce a committed relative dependency path to a developer's workspace. Never modify the copy
under `o` or `_deps`; modify the authoritative repository instead.

Repositories are private unless explicitly changed. Never place access tokens, credentials, or authenticated URLs in
CMake files, presets, manifests, or documentation.

## C++ formatting

The canonical C++ formatter is the CLion `PiSubmarine` code-style scheme. Its exact project-level export is tracked in
`ModuleTemplate/.idea/codeStyles/Project.xml`; new modules copy the tracked `.idea/codeStyles` directory from the
template. Reformat changed C++ with CLion or its command-line formatter using that XML. `.clang-format` is a fallback
for tools that cannot run CLion and is not authoritative where its output differs from the project scheme.

## Naming conventions

- Repository names describe the primary responsibility, for example `Ecs.Api` or `Ecs.EnTT`.
- CMake targets include the organization prefix, for example `UnrealVoxelSim.Ecs.Api`.
- C++ namespaces mirror target segments, for example `UnrealVoxelSim::Ecs::Api`.
- Modules containing contracts and shared value types use the `.Api` suffix.
- Implementation suffixes identify the implementation or integration, such as `.EnTT`, `.Unreal`, or `.InMemory`.
- Treat namespaces as part of a type's fully qualified name. Do not repeat domain or module qualifiers already supplied
  by namespaces, including semantic repetitions that are not exact spelling matches. Prefer
  `UnrealVoxelSim::Voxel::Solid::Api::IReader` to `ISolidVoxelReader`, and
  `UnrealVoxelSim::Voxel::Solid::Controller` to `SolidVoxels`.
- Retain a qualifier when it distinguishes sibling concepts in the same namespace or removing it would make the role
  ambiguous, for example `Ecs::Api::EntityOperationError` and `Ecs::Api::ComponentOperationError`.
- Dynamically polymorphic interfaces are prefixed with `I`.
- Concept names describe requirements and are not prefixed with `I`.
- Types stored directly as entity components use the `Component` suffix. Reserve that suffix for component types;
  value types merely contained by a component do not acquire it.
- Private data members use `m_CamelCase`; public data members use unprefixed `CamelCase`.
- Use explicit domain types instead of primitive parameters when values have different meanings or invariants.

## Software architecture

Clarity, maintainability, and predictable performance are more important than brevity. Follow SOLID principles and
prefer explicit dependencies and data flow.

- High-level policy depends on abstractions, not low-level implementations.
- All dependencies between reusable modules cross through API contracts. Runtime module capabilities use dynamically
  polymorphic interfaces; reusable implementation targets never depend on another module's implementation target.
- Compile-time abstractions must remain behind module API boundaries unless the module exists specifically to define a
  compile-time contract, such as an ECS API or composition binding.
- Prefer composition over inheritance unless runtime polymorphism is required.
- Public interfaces are small and segregated by responsibility.
- Ownership and lifetime are explicit at every API boundary.
- Avoid service locators, mutable global state, and hidden singletons.
- Reusable production targets depend only on other modules' API targets, never on their implementation targets.
  Composition roots select and wire concrete implementations across module boundaries, own shared infrastructure, and
  inject dependencies through API contracts. An implementation may construct its own private concrete types. Tests and
  benchmarks may instantiate and depend on the concrete implementations they verify; integration tests may act as
  test-only composition roots.
- Infrastructure objects must not be passed through unrelated domain APIs.
- Engine- and platform-specific types must not leak into portable modules.

### Engine independence

Portable modules must not include Unreal Engine headers or depend on Unreal reflection, object lifetime, containers,
allocators, math types, logging, task systems, or global engine state. Unreal adapters translate between core contracts
and Unreal representation types.

Do not copy Unreal conventions into the core when the standard library provides the appropriate portable abstraction.
An integration adapter may follow host requirements within its own boundary.

### Entity-component storage

Entity-component storage is shared infrastructure owned by a composition root. Dynamically abstracted domain APIs do not
exchange an unrestricted ECS registry, queries, storage references, or backend-specific handles. Opaque entity
identifiers may cross module boundaries when identity is part of the contract. Compile-time ECS binding and capability
declarations belong in ECS-aware implementation or composition code, not in dynamically abstracted interfaces.

Domain components are private storage and processing details by default. A stable component may be a public inter-system
data contract when cross-domain composition or querying is intentional; public visibility does not grant mutation.
Every ECS-aware system declares its read, existing-value write, structural, and entity-lifecycle authority, and the
composition root grants a narrow, non-escalating access object. Public state normally has one authoritative writer or
structural owner. External mutation is limited to explicit input or coordination components with documented lifetime
and arbitration. Invariant-rich changes still use domain commands. Persistence needs alone do not make a component
public.

Large homogeneous data sets are not automatically modeled as ECS entities. Choose storage according to access patterns,
locality, mutation behavior, and measured performance.

### Derived state

Derived representations are allowed and often necessary for performance, but they never become independent authorities
for the same domain fact merely because they are cached or incrementally maintained.

- Every derived representation has an explicit authoritative source.
- Every derived representation defines its invalidation, rebuild, and synchronization rules.
- Reconstructible derived state is not persisted unless an explicit, documented requirement justifies doing so.
- Prefer reusable geometric, spatial, topological, or other domain-neutral derived properties over gameplay-specific
  precomputed classifications.
- Cache or materialize expensive semantic queries only when measurements justify the additional invalidation and memory
  complexity.

### Events and inter-module communication

Use an injected API interface when one module directly requests behavior from another. Use an event when a producer
announces a fact without depending on the consumers or their reactions.

- Commands request state-changing behavior. Queries request information and do not mutate authoritative simulation state.
  Events announce facts that have already occurred.
- Do not replace a required synchronous command or query with an event merely to avoid a direct dependency.
- Define one narrow `I<Name>EventSource` source interface for each event type. Do not group unrelated subscriptions in a
  broad observer, event-manager, or event-bus interface.
- Consumers register functors; consuming an event does not require inheritance from a producer-defined observer
  interface.
- Event source interfaces, listener types, and move-only RAII subscription handles come from the shared event API.
  Listener storage, safe mutation during dispatch, and event delivery use the shared event implementation. Domain
  modules must not create their own subscription or broadcast infrastructure.
- Event infrastructure is agnostic of simulation ticks, phases, ECS, and persistence. It exposes only its documented
  publication, subscription, queueing, dispatch, and pending-delivery capabilities. A scheduler or composition root
  decides when queued events are dispatched.
- A producer owns its event sources and exposes only the specific source interfaces required by its consumers. Do not
  introduce a global dispatcher, service locator, string-keyed channel, or hidden type-indexed event bus.
- The composition root constructs event infrastructure and injects the required source interfaces. Consumers receive
  mandatory event sources explicitly through dependency injection and own the resulting subscriptions; composition code
  wires only optional or configuration-dependent relationships.
- Normally, an event source outlives its subscriptions. The shared implementation must safely invalidate outstanding
  handles if a source is destroyed first. Store subscription handles so they are destroyed before the consumer state
  and dependencies used by their callbacks.
- Subscribing must not invoke the listener synchronously. Destroying a subscription prevents subsequent or queued
  delivery to that listener.
- Delivery order, reentrancy, mutation during dispatch, exception behavior, thread affinity, and allocation behavior
  are part of an event implementation's public contract and must be deterministic and tested.
- Prefer queued delivery when immediate callbacks could cause reentrancy, invalidate iteration, or make execution order
  implicit. The owning scheduler invokes generic dispatch operations at its chosen boundaries. Cross-thread publication
  requires an explicit adapter or implementation; it is never an accidental property of the default event source.

## Determinism and time

- Simulation behavior must be reproducible from the same initial state and inputs.
- Time-based logic receives explicit durations or tick information. Do not read wall-clock time in simulation logic.
- Randomness is injected, seeded, and reproducible. Do not use hidden global random-number generators.
- Iteration order must not accidentally define behavior. When order matters, define and test it explicitly.
- Floating-point assumptions that affect reproducibility must be documented and tested on supported platforms.
- Avoid blocking I/O, unbounded work, and unpredictable allocation in time-critical update paths.

## Persistence

Persistence follows a snapshot-based ports-and-adapters architecture. Keep state capture and restoration, binary
encoding, and durable storage as separate responsibilities. Domain implementations must not depend on a serialization
library, archive type, filesystem, database, Unreal API, or generated schema type.

- The shared persistence API defines only mechanism-neutral snapshot metadata, stable section identifiers, schema
  versions, encoded sections or blocks, narrow capture and restore ports, storage ports, and error types. It must not
  know about games, ECS, Unreal Engine, a particular domain, or a specific encoding or storage implementation.
- Keep capture and restore capabilities segregated. Do not require domain objects to implement a universal
  `IPersistable`, expose `Serialize(Archive&)`, or turn their in-memory C++ layout into the save format.
- Persistence adapters depend on the relevant domain APIs and the persistence API; domain modules do not depend on
  persistence adapters. Domain state crosses this boundary only through narrow, dynamically abstracted capture and
  restore capabilities using format-neutral, domain-owned logical value types. Keep these capabilities segregated and
  batch-oriented where state volume makes per-element dynamic dispatch or monolithic allocation inappropriate.
- Generated encoding types and third-party serialization APIs remain inside persistence adapters. Encoding formats may
  differ between sections when justified and must not leak through domain or shared persistence contracts.
- Canonical saves are versioned logical snapshots, not serialized event queues or an implicit event-sourced history.
  Domain-level scheduled intentions and future actions, simulation tick state, reproducible random state, and other
  behavior-affecting state are durable; scheduler queues, worker jobs, pending callbacks, transient notifications, and
  reconstructible caches are not.
- The simulation engine defines persistence checkpoint boundaries. It may initiate capture only after every update,
  structural-mutation, and event-delivery phase belonging to the captured state has completed and all transient event
  queues are empty. Future behavior is durable domain state, not an event retained across checkpoints.
- A checkpoint describes one coherent simulation state. Expensive encoding, compression, and storage may continue
  asynchronously only against immutable captured data.
- Restoration operates on an inactive staging session and must not emit ordinary domain events or expose partially
  restored state. Decode and migrate sections, create entity mappings, restore authoritative state, resolve references,
  rebuild derived state, and validate the complete result before installing and activating the session. Ticks, input,
  event delivery, presentation, and networking remain disabled until activation. A failed load leaves the current
  session intact.

ECS persistence is a dedicated integration concern, but it does not inspect, discover, allowlist, or serialize private
domain components. Generic ECS persistence is limited to snapshot-local entity identity, fresh runtime entity creation,
runtime-to-snapshot and snapshot-to-runtime identifier mapping, and reference validation. The shared persistence API
remains ECS-agnostic.

- Runtime entity identifiers are never serialized as durable identity. Domain persistence adapters translate entity
  references in their logical state through the snapshot-local mapping. Use an explicit domain identifier when identity
  must remain stable across saves.
- Restore entities and references in deterministic passes so decoding never depends accidentally on registry allocation
  or iteration order.
- Transient, presentation-only, cached, and reconstructible derived state is excluded. Rebuild required derived state
  inside the inactive staging session before validation and activation, using direct domain capabilities rather than
  replayed events.
- Persist large homogeneous state according to its own storage model. In particular, voxel data should be partitioned
  into bounded chunks rather than represented or serialized as individual entities.

Every save container has a versioned manifest and independently versioned sections. Section identifiers are stable,
removed schema fields or identifiers are never silently reused, and incompatible data is rejected explicitly. Schema
migrations are owned by persistence adapters and covered by golden-file and corruption tests.

- Treat all loaded data as untrusted. Validate versions, identifiers, counts, sizes, numeric ranges, references,
  checksums, and allocation limits before committing restored state.
- Durable writes are transactional. Never overwrite the last valid save in place; completely write and verify a new
  representation before atomically publishing it. Interrupted writes must leave a recoverable valid state.
- The container and storage abstractions support bounded sections or blocks so large worlds can later use dirty-chunk
  tracking, immutable revisions, incremental saves, and deduplication without changing domain APIs. Add those
  optimizations only when measurements justify their complexity.
- Persistence is not a multiplayer replication protocol. Persistent IDs, schemas, compatibility policies, manifests,
  and storage APIs are distinct from network IDs, messages, per-connection baselines, relevance, and transport.
  Persistence and networking may share pure domain projections or conversion primitives only when their semantics are
  genuinely identical.

## Concurrency

- Thread-safety is explicit for every public type that can cross threads.
- Do not add hidden locking to low-level data structures.
- Prefer phase-based execution, immutable inputs, message passing, and command buffers over shared mutable state.
- Background work must not mutate simulation state without an explicit synchronization and commit boundary.
- Structural mutations produced in parallel must be committed in a deterministic order.
- Use RAII for synchronization primitives and task lifetime.
- Do not hold locks while calling an injected dependency or user-provided callback.

## Performance and memory

Performance is a functional requirement, but optimization must be evidence-driven.

- Design hot data around locality and access patterns.
- Keep allocation behavior explicit. Reuse or preallocate memory in repeated time-critical paths.
- Avoid virtual dispatch, type erasure, hashing, and pointer chasing inside per-element loops unless benchmarks justify
  them.
- Place dynamic dispatch at coarse module, task, or batch boundaries when practical.
- Do not sacrifice correctness or module boundaries for an unmeasured optimization.
- Add representative release-build benchmarks for data structures and algorithms on critical paths.
- Record the scenario, data size, compiler, configuration, and relevant hardware with benchmark conclusions.

## Error handling

- Use `std::expected<Value, ErrorEnum>` for recoverable failures.
- Error enums are strongly typed and specific to the operation or responsibility.
- Do not return generic integer error codes or error strings as the primary error representation.
- Use exceptions for failures that cannot be handled locally or for initialization failures where stack unwinding is
  useful.
- Never mix exception and `std::expected` semantics at the same API boundary.
- Preconditions representing programmer errors may fail fast.
- A validated hot path may expose direct values or references when an earlier operation already guarantees validity.
- Never silently ignore an error merely to keep an update loop running.

## API design

- Keep APIs minimal, explicit, and difficult to misuse.
- Public interfaces describe capabilities using the provider module's domain vocabulary. Do not introduce
  consumer-specific use cases into lower-level provider contracts.
- Place runtime abstraction at module capability boundaries rather than primitive per-element operations. Prefer
  batch-oriented contracts when consumers naturally issue many equivalent operations, while allowing implementations to
  use non-polymorphic, data-oriented kernels internally.
- Prefer multiple narrow interfaces over a broad manager interface.
- State ownership, borrowing, lifetime, invalidation, and thread-safety rules in public documentation.
- Avoid boolean parameters when an enum or explicit option type communicates intent.
- Avoid implicit conversions that weaken type or unit safety.
- Do not expose third-party dependency types unless wrapping them would provide no meaningful boundary and the exposure is
  an explicit architectural decision.
- Do not return writable access when read-only access is sufficient.
- Avoid callbacks that can outlive borrowed state unless lifetime is enforced by the type system.

## Testing strategy

- All business and simulation logic has automated unit tests.
- Test externally observable behavior and contracts, not private implementation details.
- Use GoogleTest and GoogleMock from vcpkg for runtime-polymorphic contracts.
- Use small fakes and conformance tests for compile-time abstractions when mocks are not appropriate.
- Public mocks live in **mock** only when other modules genuinely need them.
- Non-trivial `constexpr` behavior is verified with `static_assert` where practical.
- Bugs receive regression tests.
- Tests must be deterministic and independent of execution order.
- Tests must not require Unreal Engine for portable modules.
- Platform adapters may use integration tests in addition to portable unit tests.

## Logging and diagnostics

- Logging is an injected capability; modules do not use global loggers.
- The composition root owns sinks, formatting, and output destinations.
- Modules do not create logging sinks in reusable production code.
- Do not log in per-element or time-critical loops unless the diagnostic value justifies the measured cost.
- Logging must not influence simulation behavior or timing decisions.
- Diagnostic messages include sufficient operation and identifier context without exposing secrets.

## Code style

- Prefer one primary type per file. Do not optimize for a smaller file count by packing multiple independently meaningful
  classes, structs, interfaces, enums, or other types into the same header or source file.
- Put multiple types in one file only when they are genuinely inseparable parts of one abstraction and separating them
  would make the code harder to understand or use. Small private implementation-only helper types may remain next to
  the owning type when they have no independent meaning.
- File names should normally match the primary type they contain and follow the namespace-mirroring directory rules.
- Do not extract very small functions or methods when replacing the call with the function body is equally or more
  readable and the function introduces no meaningful abstraction, invariant, policy, reuse, or test seam. Prefer the
  direct expression over a named wrapper that merely forwards arguments, returns a constant, or constructs a simple
  value.
- A function name should communicate intent that is materially clearer than its body. For example, prefer
  `Error{ErrorType::Access}` at the call site over a helper such as `ConstructAccessError()` whose implementation is
  only that expression.
- Small functions remain appropriate when the function itself is part of a public contract, required by a callback or
  concept, centralizes a non-trivial invariant or policy, removes meaningful duplication, or gives a domain operation
  a clearer name than its implementation details.

## Code review expectations

- Review architecture, dependency direction, ownership, determinism, and performance characteristics as well as syntax.
- Reject hidden coupling, cyclic dependencies, engine leakage, and unbounded work in critical paths.
- Prefer self-explanatory code. Comments explain non-obvious intent, invariants, or trade-offs rather than restating code.
- Keep changes focused. Unrelated cleanup belongs in a separate change.

## Do

- Use standard C++23 facilities when they satisfy the requirement.
- Use `constexpr`, concepts, ranges, RAII, smart pointers, and strong types where they improve clarity or correctness.
- Use references for required non-owning dependencies and pointers when null is meaningful.
- Prefer value semantics and contiguous storage.
- Include what a file uses.
- Keep public headers small and stable.
- Build and test Debug and Release configurations when changing performance-sensitive behavior.
- Point out code smells and architectural violations encountered during work.

## Do not

- Do not use macros when a language or build-system feature can express the same intent.
- Do not use owning raw pointers or unmanaged resource lifetime.
- Do not use C-style casts or C APIs without a necessary interoperability boundary.
- Do not use `std::cin`, console interaction, file I/O, networking, or engine calls in reusable simulation logic.
- Do not expose mutable containers as a shortcut for designing an interface.
- Do not add a dependency to avoid implementing a small, well-defined operation with the standard library.
- Do not edit generated files or downloaded dependency sources.
- Do not commit credentials, local machine paths, build artifacts, or IDE state.

## When in doubt

If a change affects module boundaries, public contracts, dependency direction, determinism, persistent data, threading,
or a critical-path allocation pattern, present the alternatives and trade-offs before implementation.
