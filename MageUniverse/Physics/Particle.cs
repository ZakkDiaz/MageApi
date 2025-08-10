using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MageUniverse.Physics
{
    public class Particle
    {
        public float Mass { get; set; }
        public float LocationX { get; set; }
        public float LocationY { get; set; }
        public float LocationZ { get; set; }

        public float VelocityX { get; set; }
        public float VelocityY { get; set; }
        public float VelocityZ { get; set; }

        /// <summary>
        /// Index of this particle within the spatial octree.
        /// </summary>
        public int OctreeIndex { get; set; }

        /// <summary>
        /// Collision radius derived from mass.
        /// </summary>
        public float Radius => MathF.Cbrt(Mass);
    }
}
