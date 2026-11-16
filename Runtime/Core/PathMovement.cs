using System;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    /// <summary>How the agents of a grid move through it.</summary>
    public enum PathMovement : byte {
        /// <summary>Through any free cell, in all 26 directions (drones, birds).</summary>
        Fly,
        /// <summary>
        /// On floors: from a cell they can stand in to one of the 8 around it, climbing or dropping as far as their
        /// <see cref="WalkSettings"/> allow (people, robots).
        /// </summary>
        Walk
    }

    /// <summary>
    /// What a walking agent's body allows, in metres; a grid turns them into whole cells. A walker stands in a free cell
    /// with a blocked cell under it and its body height of free cells above. It steps to a neighbouring column as long as
    /// its body fits through the move, climbing at most a step and dropping at most a drop.
    /// </summary>
    [Serializable]
    public struct WalkSettings {
        [Tooltip("Height (m) the walker needs free above the floor.")]
        public float bodyHeight;
        [Tooltip("Highest (m) the walker climbs from one cell to the next (steps, kerbs, stairs).")]
        public float stepHeight;
        [Tooltip("Deepest (m) the walker steps or jumps down from one cell to the next (a ledge, a roof edge).")]
        public float dropHeight;

        /// <summary>A person: 1.8 m tall, climbs 0.5 m, drops 1 m.</summary>
        public static WalkSettings Default => new WalkSettings { bodyHeight = 1.8f, stepHeight = 0.5f, dropHeight = 1f };

        /// <summary>Free cells a walker needs above the floor (at least one).</summary>
        public int ClearanceCells(float cellSize) => Mathf.Max(1, Mathf.CeilToInt(bodyHeight / cellSize - 0.001f));

        /// <summary>Cells a walker climbs in one step.</summary>
        public int StepCells(float cellSize) => Mathf.Max(0, Mathf.FloorToInt(stepHeight / cellSize + 0.001f));

        /// <summary>Cells a walker drops in one step.</summary>
        public int DropCells(float cellSize) => Mathf.Max(0, Mathf.FloorToInt(dropHeight / cellSize + 0.001f));
    }
}
