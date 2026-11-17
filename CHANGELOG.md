# Changelog

## [3.1.1]

### Changed
- A body's blocked cells are made in three Burst passes, one per axis (across x and z for its ring, up y for its height), each sliding a running count along the grid's lines, instead of one pass reading the whole box round every cell. The cost no longer grows with the body's size: a big flyer (a ring of two or three cells) takes about what a person-sized one does. Same cells as 3.1.0. The passes share one scratch array per grid.

## [3.1.0]

### Added
- `PathBody` and `GridAStarAgent.body`: a flyer bigger than one free cell (its width and height in metres) searches only where it fits: the cells above its lowest one for its height, and a ring of cells round them for its width, never out of the grid's sides; the open sky above the grid's top counts as free (a body standing on a roof near the top has its head above it). Its start, goal and path points are its centre. A start or goal it doesn't fit at moves to the nearest cell it fits in (two cells each way at most).
- The blocked cells for each body size are made from the grid's own in a Burst job (`BodyMaskJob`), again only when the grid's blocking changed; line-of-sight pruning of a sized body's path uses them.
- `GridAStarAgent.GetPathAsync(start, target, points, ct)`: a search from any point, not only the agent's transform (a sized body's centre, a leg planned from somewhere else).
- `GridExtension.ProbeMargin`: the bake's margin past half a cell, named (0.1 m, as before).

## [3.0.0]

### Removed
- Walking agents (2.2.0): `PathMovement`, `WalkSettings`, `GridVolume.movement` and `.walking`, the walking members of `GridCollection` (`Movement`, `Walk`, `IsWalking`, `IsStandable`, `TryGetStandable`, `HasWalkableLineOfSight`), walking searches and island fills, and `PathFollower.Steer`. Walkers on a flyer grid miss whatever is shorter than a cell and climb stairs as cell-high steps; ground agents belong on a NavMesh. The package is for flying agents again.

### Kept from 2.2.0
- `GridAStarAgent.SetGrid(GridCollection)`: search a given grid instead of the active one of the agent's type. The tests use it instead of reflection.
- The fix: `GetPath` and `GetPathAsync` returned the start cell first and left the goal cell out. The path runs from the cell after the start to the goal.

## [2.2.0]

### Added
- Walking agents (removed in 3.0.0), `PathFollower.Steer`, and `GridAStarAgent.SetGrid`.

### Fixed
- `GetPath` and `GetPathAsync` returned the start cell first and left the goal cell out.

## [2.1.0]

### Changed
- Island maps fill in a Burst job (`IslandFillJob`) instead of a time-sliced flood fill on the main thread. The dynamic refill that follows obstacle changes runs on a worker thread from its own copy of the blocking, and swaps in when done; queries keep the last complete map meanwhile. Before, it took up to 10 ms of every frame until done and allocated a new map each time (a 300k-cell grid dropped 2-3 frames on Quest 3 per refill). The maps are native and reused, released by `GridCollection.Dispose`.
- `ComputeIslands` and `ComputeDynamicIslands` complete before they return.

## [2.0.0]

### Changed
- `PathAgentType` is a byte, named like a layer in `Project Settings > Pathfinding Agent Types` (editor-only), instead of a ScriptableObject. An identity asset referenced from both the player build and Addressables was loaded twice, and agents never found their grid. Re-pick the type on every `GridVolume` and agent field.

## [1.1.1]

### Fixed
- `PathFollower` flew back to a corner it had cut: it only moved on to the next segment once it projected past the end of the current one, so a body beside its start cell (a short first leg, then a turn) lost progress and turned round. It now moves on when the next segment is nearer, and its progress along the path never goes back.

## [1.0.0]

### Added
- Weighted A* search over uniform 3D grids: indexed binary min-heap with decrease-key, real diagonal step costs, no-corner-cutting rule, pooled version-stamped search contexts (no per-request allocation), synchronous and frame-sliced async APIs.
- Island reachability: static flood-filled region IDs for O(1) rejection of unreachable goals, plus a background-refreshed dynamic island map (version-stamped, coalesced refills) and a bounded reverse goal probe for the stale window.
- Static/dynamic blocking split: baked static flags plus refcounted dynamic blocks (`GridObstacle`), so obstacle movement can never corrupt baked geometry.
- Exact line-of-sight (Amanatides-Woo voxel traversal) and LOS-based waypoint pruning with corner-knot retention.
- Baked data format v2: magic/version/dims/origin header + one blocked bit per cell, validated on load. Addressables-based loading; editor baker with automatic addressable registration and obstacle-vs-bake-layer validation.
- `PathAgentType` ScriptableObject identity for agent categories.
- `PathFollower`: pure-pursuit velocity follower with acceleration-limited steering, braking-distance arrival and repath velocity continuity; `ComputeAvoidance` predictive (closest-point-of-approach) pairwise avoidance.
- `PathfindingTicker` auto-driver for obstacle ticks and island maintenance.
- EditMode test suite: search, islands, seal probe, line-of-sight, pruning, follower continuity, avoidance.
