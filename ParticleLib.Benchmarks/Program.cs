using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using ModAAABBB = ParticleLib.Modern.Models._3D.AAABBB;
using ModOctree = ParticleLib.Modern.Models._3D.Octree;
using ModPoint = ParticleLib.Modern.Models._3D.Point3D;

namespace ParticleLib.Benchmarks
{
    public static class Program { public static void Main() => Bench.Run(); }

    internal struct Phase
    {
        public double WallMs;
    }

    internal static class Bench
    {
        // -------- fixed sweep axes --------
        private static readonly int[] ParticleSizes =
            { 10_000, 20_000, 40_000, 80_000, 160_000 };

        private static (string Label, int LeafCap, int Depth)[] BuildConfigs()
        {
            var cfgs = new List<(string, int, int)>();
            for (int d = 1; d <= 16; d <<= 1)        // depth 1,2,4,8,16
                for (int l = 1; l <= 512; l <<= 1)   // leaf 1..512
                    cfgs.Add(($"D{d}L{l}", l, d));
            return cfgs.ToArray();
        }

        private const int RandSeed = 42;
        private const float Min = -100f, Max = 100f, Radius = 3f;

        private static readonly Process Proc = Process.GetCurrentProcess();

        public static void Run()
        {
            var cfgs = BuildConfigs();

            Console.WriteLine("Modern Octree 2-D sweep (particle size × tree params)\n");

            foreach (int count in ParticleSizes)
            {
                // generate points for this size
                var rnd = new Random(RandSeed);
                var pts = Enumerable.Range(0, count)
                                    .Select(_ =>
                                    {
                                        float x = (float)(rnd.NextDouble() * (Max - Min) + Min);
                                        float y = (float)(rnd.NextDouble() * (Max - Min) + Min);
                                        float z = (float)(rnd.NextDouble() * (Max - Min) + Min);
                                        return new ModPoint(x, y, z);
                                    }).ToList();

                // phase → best (cfg, time)
                var best = new Dictionary<string, (string cfg, double ms)>
                {
                    ["CON"] = ("", double.MaxValue),
                    ["INS"] = ("", double.MaxValue),
                    ["QRY"] = ("", double.MaxValue),
                    ["UPD"] = ("", double.MaxValue),
                    ["REM"] = ("", double.MaxValue),
                    ["TOT"] = ("", double.MaxValue)
                };

                foreach (var (label, leafCap, depth) in cfgs)
                {
                    double total = 0;

                    // CON
                    var (tree, p) = Timed(() => new ModOctree(
                        new ModAAABBB(new ModPoint(Min, Min, Min),
                                      new ModPoint(Max, Max, Max)),
                        leafCap, depth));
                    UpdateBest("CON", label, p.WallMs); total += p.WallMs;

                    // INS
                    p = Timed(() => tree.AddParticles(pts)).phase;
                    UpdateBest("INS", label, p.WallMs); total += p.WallMs;

                    // QRY  (1% of points as centres)
                    var centres = pts.Where((_, i) => i % 100 == 0).ToList();
                    p = Timed(() =>
                    {
                        foreach (var c in centres) tree.GetParticlesInRadius(c, Radius);
                    }).phase;
                    UpdateBest("QRY", label, p.WallMs); total += p.WallMs;

                    // UPD  (random reposition)
                    p = Timed(() =>
                    {
                        foreach (var (pt, idx) in pts.Select((pnt, idx) => (pnt, idx)))
                            tree.UpdateParticle(idx, pt);
                        tree.ProcessParticleReflow();
                    }).phase;
                    UpdateBest("UPD", label, p.WallMs); total += p.WallMs;

                    // REM  (delete 50 %)
                    p = Timed(() =>
                    {
                        for (int i = 0; i < count; i += 2) tree.RemoveParticle(i);
                    }).phase;
                    UpdateBest("REM", label, p.WallMs); total += p.WallMs;

                    UpdateBest("TOT", label, total);

                    // uncomment for verbose per-config output
                    //Console.WriteLine($"{count,7} {label} total {total:F1} ms");
                }

                // ---- summary for this particle count ----
                Console.WriteLine($"\n=== {count:N0} particles ===");
                foreach (var phase in new[] { "TOT", "CON", "INS", "QRY", "UPD", "REM" })
                    Console.WriteLine($"{phase}: {best[phase].cfg,-6}  {best[phase].ms,8:F1} ms");

                void UpdateBest(string key, string cfg, double ms)
                {
                    if (ms < best[key].ms) best[key] = (cfg, ms);
                }
            }
        }

        // ------------- timed helper -------------
        private static (T result, Phase phase) Timed<T>(Func<T> work)
        {
            var sw = Stopwatch.StartNew();
            T val = work();
            sw.Stop();
            return (val, new Phase { WallMs = sw.Elapsed.TotalMilliseconds });
        }
        private static (object _, Phase phase) Timed(Action act) =>
            Timed(() => { act(); return (object)null; });
    }
}
