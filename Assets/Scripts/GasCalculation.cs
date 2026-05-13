using UnityEngine;
using System.Collections.Generic;

public class GasCalculation : MonoBehaviour
{
    [Header("SPH Parameters")]
    public float smoothingLength   = 0.1f;
    public float gasTargetDensity  = 1000f;
    public float gasStiffCoef      = 100f;
    public float gasViscosityCoef  = 0.05f;

    [Header("Region Confinement")]
    public float gasRegion         = 7f;
    public float gasRegionWidth    = 1f;
    public float gasRegionStrength = 5f;

    public void UpdateParticles(List<Particle> particles, PhysicsManager manager, float dt)
    {
        foreach (Particle particle in particles)
        {
            if (particle.material != MaterialType.Gas) continue;

            CalculateDensityPressure(particle, particles);

            Vector3 gradPressure      = Vector3.zero;
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

                Vector3 vIj       = particle.velocity - other.velocity;
                float dotRGrad    = Vector3.Dot(rVec, kernelGrad);
                float denominator = Vector3.Dot(rVec, rVec) + 0.01f * smoothingLength * smoothingLength;

                if (denominator != 0f)
                    laplacianVelocity += 2f * (other.mass / other.density) * vIj * (dotRGrad / denominator);
            }

            Vector3 fiPressure  = -gradPressure;
            Vector3 fiViscosity = particle.mass * gasViscosityCoef * laplacianVelocity;

            // Region confinement along Z
            Vector3 fExtra = Vector3.zero;
            if (particle.position.z < gasRegion - gasRegionWidth)
                fExtra = gasRegionStrength * Vector3.forward;
            else if (particle.position.z > gasRegion + gasRegionWidth)
                fExtra = -gasRegionStrength * Vector3.forward;
            else
                particle.velocity.z *= 0.8f;

            Vector3 fi = fiPressure + fiViscosity + fExtra; // no gravity for gas

            particle.velocity += (fi / particle.mass) * dt;
            particle.position += particle.velocity * dt;

            // Temperature fluctuation
            particle.temperature += 10f * Random.Range(-1f, 1f);
        }
    }

    void CalculateDensityPressure(Particle particle, List<Particle> particles)
    {
        particle.density = 0f;

        foreach (Particle other in particles)
        {
            float r = (particle.position - other.position).magnitude;
            particle.density += other.mass * SPHPhysics.Kernel(r, smoothingLength);
        }

        particle.pressure = gasStiffCoef *
                            (Mathf.Pow(particle.density / gasTargetDensity, 7f) - 1f);
    }
}