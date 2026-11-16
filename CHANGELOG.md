# Changelog

## [2.2.0]

### Added
- Walking agents. `GridVolume.movement` (`PathMovement.Fly` or `Walk`) and `GridVolume.walking` (`WalkSettings`: body height, step height, drop height). On a walking grid a walker stands in a free cell with a blocked cell under it and its body height free above, and moves to the 8 columns around it, climbing up to a step and dropping up to a drop where its body fits through. Searches (`AStarJob`), island maps (`IslandFillJob`, drops counted both ways so a reachable goal is never rejected), start/goal snapping (`GridCollection.TryGetStandable`, `IsStandable`) and pruning (`HasWalkableLineOfSight`) all walk. The baked data is unchanged.
- `PathFollower.Steer(position, deltaTime, planar, out velocity)`: the steering velocity without moving the body, for hosts that move it themselves; `planar` follows the path in plan, for walkers.
- `GridAStarAgent.SetGrid(GridCollection)`: search a given grid instead of the active one of the agent's type. The tests use it instead of reflection.

### Fixed
- `GetPath` and `GetPathAsync` returned the start cell first and left the goal cell out (the search lists the path goal first, and the copy skipped the wrong end). The path now runs from the cell after the start to the goal: agents no longer turn back to the centre of the cell they start in, and arrive at the goal instead of a cell short of it.

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
