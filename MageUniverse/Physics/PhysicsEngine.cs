using System;
using System.Collections.Generic;
using ParticleLib.Modern.Models._3D;

namespace MageUniverse.Physics;

/// <summary>
/// Basic physics engine that uses an octree to reduce the number of particle interactions.
/// </summary>
public class PhysicsEngine : IPhysicsEngine
{
    const float minDistance = 1f;  // Minimum distance threshold to avoid extremely high forces
    const float maxForce = 1000f;  // Maximum gravitational force allowed

    // Radius used when querying the octree for potential interactions
    const float neighborRadius = 100f;

    public void TimeStep(List<Particle> particles, Octree octree, float deltaTime)
    {
        for (int i = 0; i < particles.Count; i++)
        {
            var particle = particles[i];

            UpdateParticlePosition(particle, deltaTime);

            var neighbors = octree.GetParticlesInRadius(
                new Point3D(particle.LocationX, particle.LocationY, particle.LocationZ),
                neighborRadius);

            foreach (var neighborId in neighbors)
            {
                if (neighborId <= i) continue; // prevent double processing

                var particle1 = particles[neighborId];
                float distance = CalculateDistance(particle, particle1);
                float radiusParticle = (float)Math.Sqrt(particle.Mass);
                float radiusParticle1 = (float)Math.Sqrt(particle1.Mass);

                if (distance < (radiusParticle + radiusParticle1) / 2)
                {
                    // Future: collision handling could be added here.
                }

                ApplyGravitationalForce(particle, particle1, distance, deltaTime);
            }
        }
    }

    private void UpdateParticlePosition(Particle particle, float deltaTime)
    {
        particle.LocationX += particle.VelocityX * deltaTime;
        particle.LocationY += particle.VelocityY * deltaTime;
        particle.LocationZ += particle.VelocityZ * deltaTime;
    }

    private float CalculateDistance(Particle particle, Particle particle1)
    {
        float distanceX = particle1.LocationX - particle.LocationX;
        float distanceY = particle1.LocationY - particle.LocationY;
        float distanceZ = particle1.LocationZ - particle.LocationZ;

        return (float)Math.Sqrt(distanceX * distanceX + distanceY * distanceY + distanceZ * distanceZ);
    }

    private void ApplyGravitationalForce(Particle particle, Particle particle1, float distance, float deltaTime)
    {
        distance = Math.Max(distance, minDistance);
        float force = particle.Mass * particle1.Mass / (distance * distance);
        force = Math.Min(force, maxForce);  // Limit the force to avoid extreme values

        float distanceX = particle1.LocationX - particle.LocationX;
        float distanceY = particle1.LocationY - particle.LocationY;
        float distanceZ = particle1.LocationZ - particle.LocationZ;

        particle.VelocityX += force * distanceX / particle.Mass * deltaTime;
        particle.VelocityY += force * distanceY / particle.Mass * deltaTime;
        particle.VelocityZ += force * distanceZ / particle.Mass * deltaTime;
    }
}

