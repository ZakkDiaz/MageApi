using System;
using System.Diagnostics;

namespace ParticleLib.Modern.Models._3D
{
    /// <summary>Very lightweight toggle-able trace logger for the octree.</summary>
    internal static class OctreeDebug
    {
        /// <summary>Set to true at runtime to turn on tracing.</summary>
        public static volatile bool Enabled = false;

        [Conditional("DEBUG")]
        public static void Log(string msg)
        {
            if (!Enabled) return;
            Console.WriteLine(msg);          // feel free to switch to Debug.WriteLine
        }
    }
}
