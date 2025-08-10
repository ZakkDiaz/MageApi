using System.Collections.Generic;
using ParticleLib.Modern.Models._3D;

namespace MageUniverse.Physics;

public interface IPhysicsEngine
{
    /// <summary>
    /// Advances the physics simulation for the provided particles using the supplied spatial index.
    /// </summary>
    /// <param name="particles">Collection of particles to simulate.</param>
    /// <param name="octree">Spatial index containing particle positions.</param>
    /// <param name="deltaTime">Time step in seconds.</param>
    void TimeStep(List<Particle> particles, Octree octree, float deltaTime);
}

