using UnityEngine;
using System.Collections.Generic;

public class GasCalculation : MonoBehaviour
{

    public void UpdateParticles(List<Particle> particles, PhysicsManager manager, float dt, SpatialGrid grid)
    {
        foreach (Particle particle in particles)
        {
            if (particle.material != MaterialType.Gas) continue;

            SPHPhysics.CalculateDensityPressure(particle, particles, grid);

            Vector3 gradPressure      = Vector3.zero;
            Vector3 laplacianVelocity = Vector3.zero;

            foreach (Particle other in grid.GetNeighborCandidates(particle.position))
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
                                   + other.pressure    / (otherDensity    * otherDensity);
                gradPressure += other.mass * pressureTerm * kernelGrad;

                Vector3 vIj       = particle.velocity - other.velocity;
                float dotRGrad    = Vector3.Dot(rVec, kernelGrad);
                float denominator = Vector3.Dot(rVec, rVec) + 0.01f * SimulationManager.smoothingLength * SimulationManager.smoothingLength;

                if (denominator != 0f && otherDensity > 0.01f)
                    laplacianVelocity += 2f * (other.mass / otherDensity) * vIj * (dotRGrad / denominator);
            }

            Vector3 fiPressure  = -gradPressure;
            Vector3 fiViscosity = particle.mass * SimulationManager.gasViscosityCoef * laplacianVelocity;

            // Region confinement along Z
            Vector3 fExtra = Vector3.zero;
            if (particle.position.z < SimulationManager.gasRegion - SimulationManager.gasRegionWidth)
                fExtra = SimulationManager.gasRegionStrength * Vector3.forward;
            else if (particle.position.z > SimulationManager.gasRegion + SimulationManager.gasRegionWidth)
                fExtra = -SimulationManager.gasRegionStrength * Vector3.forward;
            else
                particle.velocity.z *= 0.8f;

            Vector3 fi = fiPressure + fiViscosity + fExtra; // no gravity for gas

            particle.velocity += (fi / particle.mass) * dt;
            particle.position += particle.velocity * dt;

            // Temperature fluctuation
            particle.temperature += 10f * Random.Range(-1f, 1f);
        }
    }
}