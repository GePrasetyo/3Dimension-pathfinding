# Changelog

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
