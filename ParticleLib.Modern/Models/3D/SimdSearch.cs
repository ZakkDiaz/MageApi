using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace ParticleLib.Modern.Models._3D
{
    /// <summary>
    /// Helpers that use System.Numerics.Vector to speed up radial searches.
    /// </summary>
    internal static class SimdSearch
    {
        public static void AppendHitsInRadius(
            Point3D center,
            float radius,
            List<int> particleIndices,
            List<Point3D> particleStore,
            List<int> hits)
        {
            int vecWidth = Vector<float>.Count;           // 4 on SSE, 8 on AVX2
            float r2 = radius * radius;

            // vector constants
            Vector<float> vCenterX = new Vector<float>(center.X);
            Vector<float> vCenterY = new Vector<float>(center.Y);
            Vector<float> vCenterZ = new Vector<float>(center.Z);
            Vector<float> vRadius2 = new Vector<float>(r2);

            // scratch arrays reused for each block
            float[] tmpX = new float[vecWidth];
            float[] tmpY = new float[vecWidth];
            float[] tmpZ = new float[vecWidth];

            int count = particleIndices.Count;
            int i = 0;

            // process full SIMD blocks
            while (i + vecWidth <= count)
            {
                for (int lane = 0; lane < vecWidth; lane++)
                {
                    var p = particleStore[particleIndices[i + lane]];
                    tmpX[lane] = p.X;
                    tmpY[lane] = p.Y;
                    tmpZ[lane] = p.Z;
                }

                var vx = new Vector<float>(tmpX);
                var vy = new Vector<float>(tmpY);
                var vz = new Vector<float>(tmpZ);

                var dx = vx - vCenterX;
                var dy = vy - vCenterY;
                var dz = vz - vCenterZ;

                var dist2 = dx * dx + dy * dy + dz * dz;
                var mask = Vector.LessThanOrEqual(dist2, vRadius2);

                // mask lanes are 0xFFFFFFFF (true) or 0 (false)
                for (int lane = 0; lane < vecWidth; lane++)
                    if (mask[lane] != 0f)
                        hits.Add(particleIndices[i + lane]);

                i += vecWidth;
            }

            // tail (scalar)
            for (; i < count; i++)
            {
                var p = particleStore[particleIndices[i]];
                if (Point3D.DistanceSquared(center, p) <= r2)
                    hits.Add(particleIndices[i]);
            }
        }
    }
}
