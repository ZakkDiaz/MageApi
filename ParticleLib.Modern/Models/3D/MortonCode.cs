//using System;
//using System.Numerics;
//using System.Runtime.CompilerServices;
//using System.Runtime.Intrinsics.X86;

//namespace ParticleLib.Modern.Models._3D
//{
//    /// <summary>
//    /// A high-performance utility for computing Morton codes (Z-order curves)
//    /// for efficient O(1) sector lookups in 3D space.
//    /// </summary>
//    public static class MortonCode
//    {
//        // Pre-computed lookup tables for 8-bit values
//        private static readonly ushort[] MortonTable256X = new ushort[256];
//        private static readonly ushort[] MortonTable256Y = new ushort[256];
//        private static readonly ushort[] MortonTable256Z = new ushort[256];

//        // Static constructor to initialize lookup tables
//        static MortonCode()
//        {
//            for (int i = 0; i < 256; i++)
//            {
//                MortonTable256X[i] = (ushort)(Expand3D(i) << 0);
//                MortonTable256Y[i] = (ushort)(Expand3D(i) << 1);
//                MortonTable256Z[i] = (ushort)(Expand3D(i) << 2);
//            }
//        }

//        /// <summary>
//        /// Expands a 8-bit integer into a 24-bit integer by inserting 2 zeros after each bit
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        private static uint Expand3D(int value)
//        {
//            uint x = (uint)value & 0x000000FF;
//            x = (x | (x << 8)) & 0x0000F00F;
//            x = (x | (x << 4)) & 0x000C30C3;
//            x = (x | (x << 2)) & 0x00249249;
//            return x;
//        }

//        /// <summary>
//        /// Computes a 64-bit Morton code for the given 3D point using lookup tables
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static ulong Encode(float x, float y, float z, float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
//        {
//            // Normalize coordinates to [0, 1] range
//            float normalizedX = (x - minX) / (maxX - minX);
//            float normalizedY = (y - minY) / (maxY - minY);
//            float normalizedZ = (z - minZ) / (maxZ - minZ);

//            // Scale to [0, 2^21-1] range (21 bits per dimension)
//            uint scaledX = (uint)(normalizedX * 2097151.0f);
//            uint scaledY = (uint)(normalizedY * 2097151.0f);
//            uint scaledZ = (uint)(normalizedZ * 2097151.0f);

//            return Encode(scaledX, scaledY, scaledZ);
//        }

//        /// <summary>
//        /// Computes a 64-bit Morton code for the given 3D point using lookup tables
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static ulong Encode(Point3D point, AAABBB bounds)
//        {
//            return Encode(point.X, point.Y, point.Z, bounds.Min.X, bounds.Min.Y, bounds.Min.Z, bounds.Max.X, bounds.Max.Y, bounds.Max.Z);
//        }

//        /// <summary>
//        /// Computes a 64-bit Morton code for the given 3D integer coordinates
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static ulong Encode(uint x, uint y, uint z)
//        {
//            // Use hardware acceleration if available
//            if (Bmi2.IsSupported)
//            {
//                return EncodeHardwareAccelerated(x, y, z);
//            }
            
//            // Fall back to lookup tables
//            return EncodeLookupTable(x, y, z);
//        }

//        /// <summary>
//        /// Computes a 64-bit Morton code using BMI2 hardware instructions (much faster)
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        private static ulong EncodeHardwareAccelerated(uint x, uint y, uint z)
//        {
//            if (System.Runtime.Intrinsics.X86.Bmi2.X64.IsSupported)
//            {
//                const ulong XMask = 0x9249249249249249UL; // bits 0,3,6,...
//                const ulong YMask = 0x2492492492492492UL; // bits 1,4,7,...
//                const ulong ZMask = 0x4924924924924924UL; // bits 2,5,8,...

//                ulong answer = 0;
//                answer |= System.Runtime.Intrinsics.X86.Bmi2.X64.ParallelBitDeposit(x, XMask);
//                answer |= System.Runtime.Intrinsics.X86.Bmi2.X64.ParallelBitDeposit(y, YMask);
//                answer |= System.Runtime.Intrinsics.X86.Bmi2.X64.ParallelBitDeposit(z, ZMask);
//                return answer;
//            }

//            // Fallback (correct) if only 32-bit BMI2 is available
//            return EncodeLookupTable(x, y, z);
//        }


//        /// <summary>
//        /// Computes a 64-bit Morton code using lookup tables (fallback method)
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        private static ulong EncodeLookupTable(uint x, uint y, uint z)
//        {
//            ulong answer = 0;
            
//            // Process all bytes of the input coordinates
//            for (int i = 0; i < 7; i++) // 7 bytes = 21 bits per dimension
//            {
//                answer |= ((ulong)MortonTable256X[(x >> (i * 8)) & 0xFF]) << (3 * i);
//                answer |= ((ulong)MortonTable256Y[(y >> (i * 8)) & 0xFF]) << (3 * i);
//                answer |= ((ulong)MortonTable256Z[(z >> (i * 8)) & 0xFF]) << (3 * i);
//            }
            
//            return answer;
//        }

//        /// <summary>
//        /// Decodes a 64-bit Morton code into 3D coordinates
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static (uint x, uint y, uint z) Decode(ulong code)
//        {
//            // Use hardware acceleration if available
//            if (Bmi2.IsSupported)
//            {
//                return DecodeHardwareAccelerated(code);
//            }
            
//            // Fall back to bit manipulation
//            return DecodeBitManipulation(code);
//        }

//        /// <summary>
//        /// Decodes a 64-bit Morton code using BMI2 hardware instructions
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        private static (uint x, uint y, uint z) DecodeHardwareAccelerated(ulong code)
//        {
//            // Use BMI2 PEXT (Parallel Bits Extract) instruction for optimal performance
//            uint x = Bmi2.ParallelBitExtract((uint)code, 0x49249249);
//            uint y = Bmi2.ParallelBitExtract((uint)(code >> 1), 0x49249249);
//            uint z = Bmi2.ParallelBitExtract((uint)(code >> 2), 0x49249249);
//            return (x, y, z);
//        }

//        /// <summary>
//        /// Decodes a 64-bit Morton code using bit manipulation (fallback method)
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        private static (uint x, uint y, uint z) DecodeBitManipulation(ulong code)
//        {
//            uint x = 0, y = 0, z = 0;
            
//            // Extract bits for each coordinate
//            for (int i = 0; i < 21; i++)
//            {
//                x |= (uint)((code & (1UL << (3 * i + 0))) >> (2 * i + 0));
//                y |= (uint)((code & (1UL << (3 * i + 1))) >> (2 * i + 1));
//                z |= (uint)((code & (1UL << (3 * i + 2))) >> (2 * i + 2));
//            }
            
//            return (x, y, z);
//        }

//        /// <summary>
//        /// Decodes a 64-bit Morton code into normalized 3D coordinates [0,1]
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static (float x, float y, float z) DecodeNormalized(ulong code)
//        {
//            var (x, y, z) = Decode(code);
            
//            // Convert from integer to normalized float [0,1]
//            return (
//                x / 2097151.0f,
//                y / 2097151.0f,
//                z / 2097151.0f
//            );
//        }

//        /// <summary>
//        /// Decodes a 64-bit Morton code into a 3D point in the given bounds
//        /// </summary>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static Point3D DecodeToPoint(ulong code, AAABBB bounds)
//        {
//            var (normalizedX, normalizedY, normalizedZ) = DecodeNormalized(code);
            
//            // Denormalize to original coordinate space
//            float x = normalizedX * (bounds.Max.X - bounds.Min.X) + bounds.Min.X;
//            float y = normalizedY * (bounds.Max.Y - bounds.Min.Y) + bounds.Min.Y;
//            float z = normalizedZ * (bounds.Max.Z - bounds.Min.Z) + bounds.Min.Z;
            
//            return new Point3D(x, y, z);
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static int CommonPrefixLength(ulong a, ulong b)
//        {
//            ulong xor = a ^ b;
//            // leading equal bits, not trailing!
//            return xor == 0 ? 64 : BitOperations.LeadingZeroCount(xor);
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static int GetDivergenceLevel(ulong a, ulong b)
//        {
//            // 3 bits per octree level
//            return CommonPrefixLength(a, b) / 3;
//        }

//    }
//}
