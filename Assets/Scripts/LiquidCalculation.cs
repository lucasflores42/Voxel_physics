using UnityEngine;
using System.Collections.Generic;

public class LiquidCalculation : MonoBehaviour
{
    [Header("SPH Parameters")]
    public float smoothingLength     = 0.1f;
    public float liquidTargetDensity = 1000f;
    public float liquidStiffCoef     = 10f;
    public float liquidViscosityCoef = 0.2f;

    public void UpdateParticles(List<Particle> particles, PhysicsManager manager, float dt)
    {
        foreach (Particle particle in particles)
        {
            if (particle.material != MaterialType.Liquid) continue;

            CalculateTemperature(particle, particles);
            CalculateDensityPressure(particle, particles);

            Vector3 gradPressure     = Vector3.zero;
            Vector3 laplacianVelocity = Vector3.zero;

            foreach (Particle other in particles)
            {
                if (particle == other) continue;

                Vector3 rVec = particle.position - other.position;
                float r = rVec.magnitude;

                if (r > smoothingLength || r < 0.0001f) continue;

                Vector3 kernelGrad = SPHPhysics.KernelGradient(rVec, r, smoothingLength);

                float pressureTerm = particle.pressure / (particle.density * particle.density)
                                   + other.pressure    / (other.density    * other.density);
                gradPressure += other.mass * pressureTerm * kernelGrad;

                Vector3 vIj        = particle.velocity - other.velocity;
                float dotRGrad     = Vector3.Dot(rVec, kernelGrad);
                float denominator  = Vector3.Dot(rVec, rVec) + 0.01f * smoothingLength * smoothingLength;

                if (denominator != 0f)
                    laplacianVelocity += 2f * (other.mass / other.density) * vIj * (dotRGrad / denominator);
            }

            Vector3 fiPressure  = -gradPressure;
            Vector3 fiViscosity = particle.mass * liquidViscosityCoef * laplacianVelocity;
            Vector3 fiGravity   = manager.CalculateGravity(particle.position, particle.mass,
                                                           particle.rigidBodyId, particles);
            Vector3 fi = fiPressure + fiViscosity + fiGravity;

            particle.velocity += (fi / particle.mass) * dt;
            particle.position += particle.velocity * dt;
        }
    }

    void CalculateTemperature(Particle particle, List<Particle> particles)
    {
        float neighborhoodTemp = 0f;
        int   neighbors        = 0;

        foreach (Particle other in particles)
        {
            if (other.material != MaterialType.Liquid) continue;

            float r = (particle.position - other.position).magnitude;
            if (r <= smoothingLength)
            {
                neighborhoodTemp += other.velocity.sqrMagnitude;
                neighbors++;
            }
        }

        if (neighbors > 0)
            particle.temperature = 20f * neighborhoodTemp / neighbors;
    }

    void CalculateDensityPressure(Particle particle, List<Particle> particles)
    {
        particle.density = 0f;

        foreach (Particle other in particles)
        {
            float r = (particle.position - other.position).magnitude;
            particle.density += other.mass * SPHPhysics.Kernel(r, smoothingLength);
        }

        particle.pressure = liquidStiffCoef *
                            (Mathf.Pow(particle.density / liquidTargetDensity, 7f) - 1f);
    }
}