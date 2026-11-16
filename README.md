# 🚁 Majinfwork - 3D Pathfinding

<p align="center">
  <img src="https://img.shields.io/badge/Unity-6.0%2B-black?style=flat-square&logo=unity" alt="Unity 6.0+"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=flat-square" alt="MIT License"/>
</p>

<p align="center">
  <b>Grid-based 3D A* navigation for flying and walking agents</b><br/>
  Bake a volume, request a path, follow it smoothly - built for VR/mobile budgets.
</p>

---

## 🛠️ Features

- **Weighted A\*** - fast 3D grid search with real diagonal costs and no corner cutting
- **Instant unreachable checks** - island maps reject sealed-off goals in O(1) instead of scanning the whole grid
- **Dynamic obstacles** - `GridObstacle` blocks cells at runtime without ever corrupting baked geometry
- **Straight flight paths** - exact line-of-sight pruning turns grid staircases into clean segments
- **Smooth movement** - `PathFollower` steers a persistent velocity under an acceleration limit; repaths never snap direction
- **Local avoidance** - predictive agent-vs-agent dodging built into the follower
- **Tiny baked data** - one bit per cell (a 100k-cell grid is ~13 KB), validated on load
- **Frame-friendly** - searches and island refills run as Burst jobs on worker threads; obstacle changes never stall a frame

## 🚀 Getting Started

### 1. Name your agent types

`Project Settings > Pathfinding Agent Types` - one entry per kind of flying agent (e.g. `Drone`). Like layers, a type is just a byte; the names are editor-only.

### 2. Set up the grid volume

Add a **GridVolume** to your level: pick the agent type, grid size, cell size and the layer mask of static geometry. Press **Bake grid data** in the inspector.

### 3. Initialize on level load

```csharp
foreach(GridVolume volume in FindObjectsByType<GridVolume>(FindObjectsInactive.Include, FindObjectsSortMode.None)) {
    await volume.InitializeGrid();
}
```

### 4. Path and move an agent

```csharp
using Majinfwork.Pathfinding;

public class Drone : MonoBehaviour {
    [SerializeField] private PathAgentType agentType;
    [SerializeField] private PathFollower follower;

    private GridAStarAgent agent;

    void Start() {
        agent = new GridAStarAgent(transform, agentType);
        agent.AgentState(true);
        follower.Initialize(transform);
    }

    async void GoTo(Vector3 target) {
        var (success, waypoints) = await agent.GetPathAsync(target);
        if(success) {
            follower.InputPathToMove(waypoints);
        }
    }

    void Update() {
        follower.UpdateSpeed(1f);
        follower.Move(Time.deltaTime, out Vector3? direction);
    }
}
```

### 5. Walking agents

A grid can be walked instead of flown: set the **GridVolume**'s *Movement* to **Walk** and fill its *Walking* settings (body height, step height, drop height, in metres). A walker stands in a free cell with a blocked cell under it and its body height of free cells above, and moves to the 8 columns around it, climbing up to a step and dropping up to a drop, wherever its body fits through (no squeezing past corners). The baked data is the same blocked bits, so a walking volume is baked like any other; pick a cell size that resolves the step height.

Start and goal snap to the cell a walker would stand in there, so points on the floor work. Waypoints are the centers of those cells, just above the floor: steer along them in plan and keep the body on the ground with your own controller.

```csharp
void Update() {
    // Steer gives the velocity without moving the body; planar follows the path in plan.
    if(follower.Steer(transform.position, Time.deltaTime, planar: true, out Vector3 velocity)) {
        characterController.Move((velocity + Vector3.down * fallSpeed) * Time.deltaTime);
    }
}
```

A drop in the path (the next waypoint below by more than a step) is where the walker steps or jumps off a ledge. Island checks treat drops as two-way, so they never reject a reachable goal; a goal only reachable by climbing what a walker can only drop is found unreachable by the search.

### 6. Dynamic obstacles

Add a **GridObstacle** component to anything that moves - its transform scale defines the blocking box. Ticking is automatic.

> ⚠️ One blocking channel per object: static geometry uses the bake layer mask, movable things use `GridObstacle` - never both. The baker warns if you mix them.

## ⚙️ Tuning

| Knob | Where | Effect |
|------|-------|--------|
| `pointDistance` | GridVolume | Cell size: resolution vs memory |
| `movement` / `walking` | GridVolume | Flown or walked; a walker's body height, step and drop |
| `heuristicWeight` | GridAStarAgent | 1 = optimal paths, higher = faster searches |
| `maxSpeed` / `acceleration` / `lookaheadDistance` | PathFollower | Cruise speed, turn tightness, corner behavior |
| `speedFactor` | PathFollower | Runtime multiplier (difficulty, buffs, slows) |

## 🧪 Tests

EditMode test suite included - add `"testables": ["com.majingari.pathfinding3d"]` to your project manifest and run *Window > General > Test Runner*.

## ⚙️ Requirements

- Unity 6.0 or later
- [UniTask](https://github.com/Cysharp/UniTask) (`com.cysharp.unitask`)
- Addressables (`com.unity.addressables`, auto-resolved)

## 📜 License

MIT License - see [LICENSE](LICENSE) for details.

---
