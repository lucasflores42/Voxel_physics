using UnityEngine;
using System.Collections.Generic;

public class LiquidCalculation : MonoBehaviour
{

    public void UpdateParticles(List<Particle> particles, PhysicsManager manager, float dt)
    {
        foreach (Particle particle in particles)
        {
            if (particle.material != MaterialType.Liquid) continue;

            SPHPhysics.CalculateTemperature(particle, particles);
            SPHPhysics.CalculateDensityPressure(particle, particles);

            Vector3 gradPressure     = Vector3.zero;
            Vector3 laplacianVelocity = Vector3.zero;

            foreach (Particle other in particles)
            {
                if (particle == other) continue;

                Vector3 rVec = particle.position - other.position;
                float r = rVec.magnitude;

                if (r > SimulationManager.smoothingLength || r < 0.0001f) continue;

                Vector3 kernelGrad = SPHPhysics.KernelGradient(rVec, r, SimulationManager.smoothingLength);

                float pressureTerm = particle.pressure / (particle.density * particle.density)
                                   + other.pressure    / (other.density    * other.density);
                gradPressure += other.mass * pressureTerm * kernelGrad;

                Vector3 vIj        = particle.velocity - other.velocity;
                float dotRGrad     = Vector3.Dot(rVec, kernelGrad);
                float denominator  = Vector3.Dot(rVec, rVec) + 0.01f * SimulationManager.smoothingLength * SimulationManager.smoothingLength;

                if (denominator != 0f)
                    laplacianVelocity += 2f * (other.mass / other.density) * vIj * (dotRGrad / denominator);
            }

            Vector3 fiPressure  = -gradPressure;
            Vector3 fiViscosity = particle.mass * SimulationManager.liquidViscosityCoef * laplacianVelocity;
            Vector3 fiGravity   = manager.CalculateGravity(particle.position, particle.mass,
                                                           particle.rigidBodyId, particles);
            Vector3 fi = fiPressure + fiViscosity + fiGravity;

            particle.velocity += (fi / particle.mass) * dt;
            particle.position += particle.velocity * dt;
        }
    }
}