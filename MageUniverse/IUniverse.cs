using MageUniverse.Physics;

namespace MageUniverse
{
    public interface IUniverse
    {
        // Adds a single particle to the universe
        void AddParticle(Particle particle);

        // Adds multiple particles to the universe
        void AddParticles(IEnumerable<Particle> particles);

        // Advances the universe state by a given time step
        void TimeStep(float deltaTime);

        // Retrieves the current state of all particles in the universe
        List<Particle> GetCurrentState();

        void Clear();
    }
}