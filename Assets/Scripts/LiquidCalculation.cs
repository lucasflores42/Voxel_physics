using UnityEngine;
using System.Collections.Generic;

public class LiquidCalculation : MonoBehaviour
{
    public void UpdateParticles(List<Particle> particles, PhysicsManager manager, float dt, SpatialGrid grid)
    {
        foreach (Particle particle in particles)
        {
            if (particle.material != MaterialType.Liquid) continue;

            Vector3 fi = Vector3.zero; 

            if (particle.sph == 1)
            {
                // SPHPhysics.CalculateTemperature(particle, particles, grid);
                SPHPhysics.CalculateDensityPressure(particle, particles, grid);

                Vector3 gradPressure = Vector3.zero;
                Vector3 laplacianVelocity = Vector3.zero;

                foreach (Particle other in grid.GetNeighborCandidates(particle.position))
                // foreach (Particle other in particles)
                {
                    if (particle == other) continue;

                    Vector3 rVec = particle.position - other.position;
                    float r = rVec.magnitude;

                    if (r > SimulationManager.smoothingLength || r < 0.0001f) continue;

                    Vector3 kernelGrad = SPHPhysics.KernelGradient(rVec, r, SimulationManager.smoothingLength);
                    if (kernelGrad.sqrMagnitude < 0.0001f) continue;

                    float safeDensity = Mathf.Max(particle.density, 0.1f);
                    float otherDensity = Mathf.Max(other.density, 0.1f);
                    float pressureTerm = particle.pressure / (safeDensity * safeDensity)
                                    + other.pressure / (otherDensity * otherDensity);
                    gradPressure += other.mass * pressureTerm * kernelGrad;

                    Vector3 vIj = particle.velocity - other.velocity;
                    float dotRGrad = Vector3.Dot(rVec, kernelGrad);
                    float denominator = Vector3.Dot(rVec, rVec) + 0.01f * SimulationManager.smoothingLength * SimulationManager.smoothingLength;

                    if (denominator != 0f && otherDensity > 0.01f)
                        laplacianVelocity += 2f * (other.mass / otherDensity) * vIj * (dotRGrad / denominator);
                }

                Vector3 fiPressure = -gradPressure;
                Vector3 fiViscosity = particle.mass * SimulationManager.liquidViscosityCoef * laplacianVelocity;
                Vector3 fiGravity = manager.CalculateGravity(particle.position, particle.mass, particle.rigidBodyId, particles);
                fi = fiPressure + fiViscosity + fiGravity;
            }

            if (particle.sph == 0)
            {
                Vector3 fiGravity = manager.CalculateGravity(particle.position, particle.mass, particle.rigidBodyId, particles);
                fi = fiGravity;
            }

            particle.velocity += (fi / particle.mass) * dt;
            particle.position += particle.velocity * dt;

            // activate SPH calculations (use .y em vez de índice inválido)
            if (particle.position.y < 4f)
            {
                particle.sph = 1;
            }
            else
            {
                particle.sph = 0;
            }
        }
    }
}