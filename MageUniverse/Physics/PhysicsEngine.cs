namespace MageUniverse.Physics
{
    public class PhysicsEngine : IPhysicsEngine
    {
        const float minDistance = 1f;  // Minimum distance threshold to avoid extremely high forces
        const float maxForce = 1000f;  // Maximum gravitational force allowed
        const float coefficientOfRestitution = 0.1f; // Coefficient for semi-inelastic collision

        public void TimeStep(List<Particle> particles, float deltaTime)
        {
            // Using a for-loop with indices to avoid creating a new list
            for (int i = 0; i < particles.Count; i++)
            {
                var particle = particles[i];

                //if (particle.Mass <= 499)
                {
                    // Update positions based on current velocities
                    UpdateParticlePosition(particle, deltaTime);

                    // Apply gravity and check for collisions
                    for (int j = i + 1; j < particles.Count; j++)
                    {
                        var particle1 = particles[j];

                        // Calculate distance and check for collision
                        float distance = CalculateDistance(particle, particle1);
                        float radiusParticle = (float)Math.Sqrt(particle.Mass);
                        float radiusParticle1 = (float)Math.Sqrt(particle1.Mass);

                        if (distance < (radiusParticle + radiusParticle1)/2)
                        {
                            //ResolveCollisionRev2(particle, particle1);
                        }
                        ApplyGravitationalForce(particle, particle1, distance, deltaTime);
                    }
                }
            }
        }
        private void ResolveCollisionRev2(Particle particle, Particle particle1)
        {
            // Calculate the displacement vector (normal) between particles
            float dx = particle1.LocationX - particle.LocationX;
            float dy = particle1.LocationY - particle.LocationY;
            float dz = particle1.LocationZ - particle.LocationZ;

            float distance = (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
            if (distance == 0) return; // Avoid division by zero

            // Normalize the displacement vector
            dx /= distance;
            dy /= distance;
            dz /= distance;

            // Calculate the total mass
            float totalMass = particle.Mass + particle1.Mass;

            // Calculate the velocity difference
            float dvx = particle.VelocityX - particle1.VelocityX;
            float dvy = particle.VelocityY - particle1.VelocityY;
            float dvz = particle.VelocityZ - particle1.VelocityZ;

            // Calculate the component of the velocity difference along the normal
            float dot = dvx * dx + dvy * dy + dvz * dz;

            // Skip if particles are moving apart
            if (dot > 0) return;

            // Calculate the collision impulse
            float j = (dot / totalMass)/2;

            // Apply the impulse
            float impulseX = dx * j;
            float impulseY = dy * j;
            float impulseZ = dz * j;

            particle.VelocityX -= impulseX * particle1.Mass / totalMass;
            particle.VelocityY -= impulseY * particle1.Mass / totalMass;
            particle.VelocityZ -= impulseZ * particle1.Mass / totalMass;

            particle1.VelocityX += impulseX * particle.Mass / totalMass;
            particle1.VelocityY += impulseY * particle.Mass / totalMass;
            particle1.VelocityZ += impulseZ * particle.Mass / totalMass;

            // Adjust positions to prevent overlap
            //float overlap = ((float)Math.Sqrt(particle.Mass) + (float)Math.Sqrt(particle1.Mass) - distance) / 2;
            //particle.LocationX -= dx * overlap / particle.Mass;
            //particle.LocationY -= dy * overlap / particle.Mass;
            //particle.LocationZ -= dz * overlap / particle.Mass;

            //particle1.LocationX += dx * overlap / particle1.Mass;
            //particle1.LocationY += dy * overlap / particle1.Mass;
            //particle1.LocationZ += dz * overlap / particle1.Mass;
        }

        private void ResolveCollisionRev(Particle particle, Particle particle1)
        {
            // Calculate the normal vector components of the collision
            float normalX = particle1.LocationX - particle.LocationX;
            float normalY = particle1.LocationY - particle.LocationY;
            float normalZ = particle1.LocationZ - particle.LocationZ;

            float normalLength = (float)Math.Sqrt(normalX * normalX + normalY * normalY + normalZ * normalZ);
            if (normalLength == 0) return; // Avoid division by zero

            normalX /= normalLength;
            normalY /= normalLength;
            normalZ /= normalLength;

            // Calculate relative velocity in the direction of the normal
            float relVelX = particle.VelocityX - particle1.VelocityX;
            float relVelY = particle.VelocityY - particle1.VelocityY;
            float relVelZ = particle.VelocityZ - particle1.VelocityZ;
            float relVel = relVelX * normalX + relVelY * normalY + relVelZ * normalZ;

            if (relVel > 0) return; // They are moving apart, no need to resolve collision

            // Adjust velocities
            float massTotal = particle.Mass + particle1.Mass;
            float particleMassRatio = 2 * particle1.Mass / massTotal;
            float particle1MassRatio = 2 * particle.Mass / massTotal;

            particle.VelocityX -= particleMassRatio * relVelX;
            particle.VelocityY -= particleMassRatio * relVelY;
            particle.VelocityZ -= particleMassRatio * relVelZ;

            particle1.VelocityX += particle1MassRatio * relVelX;
            particle1.VelocityY += particle1MassRatio * relVelY;
            particle1.VelocityZ += particle1MassRatio * relVelZ;

            // Prevent overlapping
            float overlap = (float)Math.Sqrt(particle.Mass) + (float)Math.Sqrt(particle1.Mass) - normalLength;
            float overlapCorrection = overlap / normalLength;
            particle.LocationX -= normalX * overlapCorrection * particle1.Mass / massTotal;
            particle.LocationY -= normalY * overlapCorrection * particle1.Mass / massTotal;
            particle.LocationZ -= normalZ * overlapCorrection * particle1.Mass / massTotal;

            particle1.LocationX += normalX * overlapCorrection * particle.Mass / massTotal;
            particle1.LocationY += normalY * overlapCorrection * particle.Mass / massTotal;
            particle1.LocationZ += normalZ * overlapCorrection * particle.Mass / massTotal;
        }

        private void ResolveCollision(Particle particle, Particle particle1)
        {
            // Calculate the normal vector components of the collision
            float normalX = particle1.LocationX - particle.LocationX;
            float normalY = particle1.LocationY - particle.LocationY;
            float normalZ = particle1.LocationZ - particle.LocationZ;

            // Normalize the normal vector
            float normalLength = (float)Math.Sqrt(normalX * normalX + normalY * normalY + normalZ * normalZ);
            if (normalLength == 0) return; // Avoid division by zero

            normalX /= normalLength;
            normalY /= normalLength;
            normalZ /= normalLength;

            // Diffuse a fraction of the velocities based on their mass
            float transferFraction = .2f; // Fraction of velocity to transfer

            float velocityTransferX = normalX * transferFraction;
            float velocityTransferY = normalY * transferFraction;
            float velocityTransferZ = normalZ * transferFraction;

            // Nudge particles towards each other
            particle.VelocityX -= velocityTransferX * particle1.Mass / particle.Mass;
            particle.VelocityY -= velocityTransferY * particle1.Mass / particle.Mass;
            particle.VelocityZ -= velocityTransferZ * particle1.Mass / particle.Mass;

            particle1.VelocityX += velocityTransferX * particle.Mass / particle1.Mass;
            particle1.VelocityY += velocityTransferY * particle.Mass / particle1.Mass;
            particle1.VelocityZ += velocityTransferZ * particle.Mass / particle1.Mass;
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
}
