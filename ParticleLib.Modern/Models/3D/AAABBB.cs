using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace ParticleLib.Modern.Models._3D
{
    /// <summary>
    /// A memory-efficient Axis-Aligned Bounding Box implementation
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct AAABBB : IEquatable<AAABBB>
    {
        public readonly Point3D Min;
        public readonly Point3D Max;

        /// <summary>
        /// Creates a new AAABBB with the specified minimum and maximum points
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public AAABBB(Point3D min, Point3D max)
        {
            Min = min;
            Max = max;
        }

        /// <summary>
        /// Creates a new AAABBB from two points (not necessarily min/max)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static AAABBB FromPoints(Point3D a, Point3D b)
        {
            return new AAABBB(
                Point3D.Min(a, b),
                Point3D.Max(a, b)
            );
        }

        /// <summary>
        /// Gets the center point of this bounding box
        /// </summary>
        public Point3D Center => (Min + Max) / 2f;

        /// <summary>
        /// Gets the size/extents of this bounding box
        /// </summary>
        public Point3D Size => Max - Min;

        /// <summary>
        /// Gets the half-size/half-extents of this bounding box
        /// </summary>
        public Point3D HalfSize => Size / 2f;

        /// <summary>
        /// Checks if this bounding box contains the specified point
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(Point3D point)
        {
            return point.X >= Min.X && point.X <= Max.X &&
                   point.Y >= Min.Y && point.Y <= Max.Y &&
                   point.Z >= Min.Z && point.Z <= Max.Z;
        }

        /// <summary>
        /// Checks if this bounding box intersects with another
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(AAABBB other)
        {
            return Min.X <= other.Max.X && Max.X >= other.Min.X &&
                   Min.Y <= other.Max.Y && Max.Y >= other.Min.Y &&
                   Min.Z <= other.Max.Z && Max.Z >= other.Min.Z;
        }

        /// <summary>
        /// Creates a new bounding box that contains both this box and another
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public AAABBB Union(AAABBB other)
        {
            return new AAABBB(
                Point3D.Min(Min, other.Min),
                Point3D.Max(Max, other.Max)
            );
        }

        /// <summary>
        /// Creates a new bounding box that is the intersection of this box and another
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public AAABBB Intersection(AAABBB other)
        {
            return new AAABBB(
                Point3D.Max(Min, other.Min),
                Point3D.Min(Max, other.Max)
            );
        }

        /// <summary>
        /// Splits this bounding box into eight equal octants
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Span<AAABBB> Split()
        {
            AAABBB[] octants = new AAABBB[8];
            var center = Center;
            
            // Create the 8 octants
            octants[0] = new AAABBB(Min, center); // 000
            octants[1] = new AAABBB(new Point3D(Min.X, Min.Y, center.Z), new Point3D(center.X, center.Y, Max.Z)); // 001
            octants[2] = new AAABBB(new Point3D(Min.X, center.Y, Min.Z), new Point3D(center.X, Max.Y, center.Z)); // 010
            octants[3] = new AAABBB(new Point3D(Min.X, center.Y, center.Z), new Point3D(center.X, Max.Y, Max.Z)); // 011
            octants[4] = new AAABBB(new Point3D(center.X, Min.Y, Min.Z), new Point3D(Max.X, center.Y, center.Z)); // 100
            octants[5] = new AAABBB(new Point3D(center.X, Min.Y, center.Z), new Point3D(Max.X, center.Y, Max.Z)); // 101
            octants[6] = new AAABBB(new Point3D(center.X, center.Y, Min.Z), new Point3D(Max.X, Max.Y, center.Z)); // 110
            octants[7] = new AAABBB(center, Max); // 111
            
            return octants;
        }

        /// <summary>
        /// Gets the volume of this bounding box
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float Volume()
        {
            var size = Size;
            return size.X * size.Y * size.Z;
        }

        /// <summary>
        /// Gets the surface area of this bounding box
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float SurfaceArea()
        {
            var size = Size;
            return 2f * (size.X * size.Y + size.X * size.Z + size.Y * size.Z);
        }

        /// <summary>
        /// Determines which octant a point belongs to relative to the center of this box
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public byte GetOctant(Point3D point)
        {
            var center = Center;
            byte octant = 0;
            
            if (point.X >= center.X) octant |= 0b100;
            if (point.Y >= center.Y) octant |= 0b010;
            if (point.Z >= center.Z) octant |= 0b001;
            
            return octant;
        }

        /// <summary>
        /// Checks if this bounding box equals another
        /// </summary>
        public bool Equals(AAABBB other)
        {
            return Min.Equals(other.Min) && Max.Equals(other.Max);
        }

        /// <summary>
        /// Checks if this bounding box equals another object
        /// </summary>
        public override bool Equals(object? obj)
        {
            return obj is AAABBB other && Equals(other);
        }

        /// <summary>
        /// Gets a hash code for this bounding box
        /// </summary>
        public override int GetHashCode()
        {
            return HashCode.Combine(Min, Max);
        }

        /// <summary>
        /// Returns a string representation of this bounding box
        /// </summary>
        public override string ToString()
        {
            return $"Min: {Min}, Max: {Max}";
        }

        /// <summary>
        /// Checks if two bounding boxes are equal
        /// </summary>
        public static bool operator ==(AAABBB left, AAABBB right)
        {
            return left.Equals(right);
        }

        /// <summary>
        /// Checks if two bounding boxes are not equal
        /// </summary>
        public static bool operator !=(AAABBB left, AAABBB right)
        {
            return !left.Equals(right);
        }
    }
}
