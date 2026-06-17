using UnityEngine;
using System.Collections.Generic;

public class PowderCalculation : MonoBehaviour
{
    public void UpdateParticles(List<Particle> particles, PhysicsManager manager, float dt)
    {
        foreach (Particle particle in particles)
        {
            if (particle.material != MaterialType.Powder) continue;

            Vector3 fi = SPHPhysics.CalculateGravity(particle.position, particle.mass,
                                                   particle.rigidBodyId, particles, SimulationManager.gravityCoef);
            particle.velocity += (fi / particle.mass) * dt;
            particle.position += particle.velocity * dt;
        }
    }
}