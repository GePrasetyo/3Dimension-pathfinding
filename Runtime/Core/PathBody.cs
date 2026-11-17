using System;
using UnityEngine;

namespace Majinfwork.Pathfinding {
    /// <summary>
    /// How much room a flyer needs: its width (across) and height, in metres. A grid's free cell holds a box half a cell
    /// plus <see cref="GridExtension.ProbeMargin"/> each way from its centre (the bake blocks a cell when anything comes
    /// nearer), so a flyer that fits in that box needs one free cell (<see cref="Point"/>, the default). A bigger one needs
    /// the cells above its lowest one free for its height (or the open sky over the grid's top), and a ring of cells round
    /// them for its width (never out of the grid's sides). A sized flyer's positions (where it starts, its goal, its path
    /// points) are its centre.
    /// </summary>
    [Serializable]
    public struct PathBody : IEquatable<PathBody> {
        [Tooltip("Width (m) across the flyer. Zero: it fits in one free cell.")]
        [SerializeField, Min(0f)] private float width;
        [Tooltip("Height (m) of the flyer. Zero: it fits in one free cell.")]
        [SerializeField, Min(0f)] private float height;

        public PathBody(float width, float height) {
            this.width = Mathf.Max(0f, width);
            this.height = Mathf.Max(0f, height);
        }

        /// <summary>A flyer that fits in one free cell.</summary>
        public static PathBody Point => default;

        public float Width => width;
        public float Height => height;

        /// <summary>
        /// The cells it needs on a grid with cells <paramref name="pointDistance"/> apart: <paramref name="up"/> cells above
        /// its lowest one, and <paramref name="ring"/> cells round them each way.
        /// </summary>
        public void Cells(float pointDistance, out int up, out int ring) {
            float free = pointDistance + 2f * GridExtension.ProbeMargin;
            // A column of n free cells holds (n - 1) cells plus one free box.
            up = Mathf.Max(0, Mathf.CeilToInt((height - free) / pointDistance - 0.0001f));
            ring = Mathf.Max(0, Mathf.CeilToInt((width - free) * 0.5f / pointDistance - 0.0001f));
        }

        /// <summary>From its lowest cell up to its centre, on a grid with cells <paramref name="pointDistance"/> apart.</summary>
        public Vector3 Lift(float pointDistance) {
            Cells(pointDistance, out int up, out _);
            return Vector3.up * (up * pointDistance * 0.5f);
        }

        public bool Equals(PathBody other) => width == other.width && height == other.height;
        public override bool Equals(object obj) => obj is PathBody other && Equals(other);
        public override int GetHashCode() => width.GetHashCode() * 397 ^ height.GetHashCode();
    }
}
