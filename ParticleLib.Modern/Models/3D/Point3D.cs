using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace ParticleLib.Modern.Models._3D
{
    /// <summary>
    /// A high-performance 3D point structure that uses SIMD operations for vector math
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Point3D : IEquatable<Point3D>
    {
        public readonly float X;
        public readonly float Y;
        public readonly float Z;

        /// <summary>
        /// Creates a new Point3D with the specified coordinates
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Point3D(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        /// <summary>
        /// Creates a new Point3D from a Vector3
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Point3D(Vector3 vector)
        {
            X = vector.X;
            Y = vector.Y;
            Z = vector.Z;
        }

        /// <summary>
        /// Converts this Point3D to a Vector3 for SIMD operations
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3 ToVector3() => new Vector3(X, Y, Z);

        /// <summary>
        /// Subtracts one point from another
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Point3D operator -(Point3D a, Point3D b)
        {
            return new Point3D(Vector3.Subtract(a.ToVector3(), b.ToVector3()));
        }

        /// <summary>
        /// Adds two points together
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Point3D operator +(Point3D a, Point3D b)
        {
            return new Point3D(Vector3.Add(a.ToVector3(), b.ToVector3()));
        }

        /// <summary>
        /// Divides a point by a scalar value
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Point3D operator /(Point3D point, float scalar)
        {
            return new Point3D(Vector3.Divide(point.ToVector3(), scalar));
        }

        /// <summary>
        /// Multiplies a point by a scalar value
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Point3D operator *(Point3D point, float scalar)
        {
            return new Point3D(Vector3.Multiply(point.ToVector3(), scalar));
        }

        /// <summary>
        /// Calculates the distance between two points
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Distance(Point3D a, Point3D b)
        {
            return Vector3.Distance(a.ToVector3(), b.ToVector3());
        }

        /// <summary>
        /// Calculates the squared distance between two points (faster than Distance)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float DistanceSquared(Point3D a, Point3D b)
        {
            return Vector3.DistanceSquared(a.ToVector3(), b.ToVector3());
        }

        /// <summary>
        /// Linearly interpolates between two points
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Point3D Lerp(Point3D a, Point3D b, float t)
        {
            return new Point3D(Vector3.Lerp(a.ToVector3(), b.ToVector3(), t));
        }

        /// <summary>
        /// Returns the minimum point (component-wise)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Point3D Min(Point3D a, Point3D b)
        {
            return new Point3D(Vector3.Min(a.ToVector3(), b.ToVector3()));
        }

        /// <summary>
        /// Returns the maximum point (component-wise)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Point3D Max(Point3D a, Point3D b)
        {
            return new Point3D(Vector3.Max(a.ToVector3(), b.ToVector3()));
        }

        /// <summary>
        /// Checks if two points are equal
        /// </summary>
        public bool Equals(Point3D other)
        {
            return X.Equals(other.X) && Y.Equals(other.Y) && Z.Equals(other.Z);
        }

        /// <summary>
        /// Checks if this point equals another object
        /// </summary>
        public override bool Equals(object? obj)
        {
            return obj is Point3D other && Equals(other);
        }

        /// <summary>
        /// Gets a hash code for this point
        /// </summary>
        public override int GetHashCode()
        {
            return HashCode.Combine(X, Y, Z);
        }

        /// <summary>
        /// Returns a string representation of this point
        /// </summary>
        public override string ToString()
        {
            return $"({X}, {Y}, {Z})";
        }

        /// <summary>
        /// Checks if two points are equal
        /// </summary>
        public static bool operator ==(Point3D left, Point3D right)
        {
            return left.Equals(right);
        }

        /// <summary>
        /// Checks if two points are not equal
        /// </summary>
        public static bool operator !=(Point3D left, Point3D right)
        {
            return !left.Equals(right);
        }

        // Common predefined points
        public static Point3D Zero => new Point3D(0, 0, 0);
        public static Point3D One => new Point3D(1, 1, 1);
        public static Point3D UnitX => new Point3D(1, 0, 0);
        public static Point3D UnitY => new Point3D(0, 1, 0);
        public static Point3D UnitZ => new Point3D(0, 0, 1);
    }
}
