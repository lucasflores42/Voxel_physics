using UnityEngine;
using System.Collections.Generic;

public class SolidCalculation : MonoBehaviour
{
    public void UpdateParticles(List<Particle> particles, PhysicsManager manager, float dt)
    {
        foreach (Particle particle in particles)
        {
            // Rigid body solids are handled by PhysicsManager.UpdateRigidBodies
            if (particle.material != MaterialType.Solid || particle.rigidBodyId != 0) continue;

            Vector3 fi = manager.CalculateGravity(particle.position, particle.mass,
                                                   particle.rigidBodyId, particles);
            particle.velocity += (fi / particle.mass) * dt;
            particle.position += particle.velocity * dt;
        }
    }
}