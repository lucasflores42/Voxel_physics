using UnityEngine;
using System.Collections.Generic;

/// No MonoBehaviour — call from anywhere.

public static class SPHPhysics
{
    // -------------------------------------------------------------------------
    //  SPH Kernel  (cubic spline, matches Julia version)
    // -------------------------------------------------------------------------
    public static float Kernel(float r, float h)
    {
        float q = r / h;
        if (q <= 1f)
            return (1f - 1.5f * q * q + 0.75f * q * q * q) / (Mathf.PI * h * h * h);
        else if (q <= 2f)
            return 0.25f * Mathf.Pow(2f - q, 3f) / (Mathf.PI * h * h * h);
        else
            return 0f;
    }

    // -------------------------------------------------------------------------
    //  SPH Kernel Gradient
    // -------------------------------------------------------------------------
    public static Vector3 KernelGradient(Vector3 rVec, float r, float h)
    {
        if (r < 0.0001f) return Vector3.zero;

        float q = r / h;
        float dWdq;

        if (q <= 1f)
            dWdq = (-3f * q + 2.25f * q * q) / (Mathf.PI * h * h * h);
        else if (q <= 2f)
            dWdq = -0.75f * Mathf.Pow(2f - q, 2f) / (Mathf.PI * h * h * h);
        else
            return Vector3.zero;

        return (dWdq / (r * h)) * rVec;
    }

    // -------------------------------------------------------------------------
    //  Gravity  (N-body style, attracted toward solid particles)
    // -------------------------------------------------------------------------
    public static Vector3 CalculateGravity(Vector3 position, float mass, int ownRigidBodyId,
                                           List<Particle> particles, float gravityCoef)
    {
        Vector3 fGravity = Vector3.zero;

        foreach (Particle other in particles)
        {
            if (other.material != MaterialType.Solid) continue;

            Vector3 rVec = other.position - position;
            float r = rVec.magnitude;

            if (r > 0.0001f)
                fGravity += gravityCoef * mass * other.mass * rVec / (r * r * r);
        }

        return fGravity;
        //return mass * Vector3.down * 10; // uniform gravity for testing
    }

    // -------------------------------------------------------------------------
    //  Inertia Tensor  (3x3 as flat float[9], row-major)
    // -------------------------------------------------------------------------
    public static Matrix3x3 CalculateInertiaTensor(List<Particle> particles, RigidBodyData rb)
    {
        Matrix3x3 I = Matrix3x3.Zero;

        foreach (int idx in rb.particleIndices)
        {
            Particle p = particles[idx];
            Vector3 r = p.position - rb.centerOfMass;

            float r2 = Vector3.Dot(r, r);

            // I += m * (r²·Id - r⊗r)
            I.m00 += p.mass * (r2 - r.x * r.x);
            I.m11 += p.mass * (r2 - r.y * r.y);
            I.m22 += p.mass * (r2 - r.z * r.z);
            I.m01 += p.mass * (-r.x * r.y);
            I.m02 += p.mass * (-r.x * r.z);
            I.m12 += p.mass * (-r.y * r.z);
        }

        // symmetric
        I.m10 = I.m01;
        I.m20 = I.m02;
        I.m21 = I.m12;

        return I;
    }

    // -------------------------------------------------------------------------
    //  Center of Mass
    // -------------------------------------------------------------------------
    public static (Vector3 cm, float totalMass) CalculateCenterOfMass(List<Particle> particles,
                                                                        List<int> indices)
    {
        float totalMass = 0f;
        Vector3 cm = Vector3.zero;

        foreach (int idx in indices)
        {
            Particle p = particles[idx];
            totalMass += p.mass;
            cm += p.mass * p.position;
        }

        return (cm / totalMass, totalMass);
    }

    public static void CalculateDensityPressure(Particle particle, List<Particle> particles, SpatialGrid grid)
    {
        particle.density = 0f;

        foreach (Particle other in grid.GetNeighborCandidates(particle.position))
        {
            if (particle == other) continue;

            float r = (particle.position - other.position).magnitude;
            particle.density += other.mass * SPHPhysics.Kernel(r, SimulationManager.smoothingLength);
        }

        particle.pressure = SimulationManager.liquidStiffCoef *
                            (Mathf.Pow(particle.density / SimulationManager.liquidTargetDensity, 7f) - 1f);
    }

    public static void CalculateTemperature(Particle particle, List<Particle> particles, SpatialGrid grid)
    {
        float neighborhoodTemp = 0f;
        int   neighbors        = 0;

        foreach (Particle other in grid.GetNeighborCandidates(particle.position))
        {
            if (particle == other) continue;
            if (other.material != MaterialType.Liquid) continue;

            float r = (particle.position - other.position).magnitude;
            if (r <= SimulationManager.smoothingLength)
            {
                neighborhoodTemp += other.velocity.sqrMagnitude;
                neighbors++;
            }
        }

        if (neighbors > 0)
            particle.temperature = 20f * neighborhoodTemp / neighbors;
    }
}

// =============================================================================
//  Minimal 3×3 matrix (Unity doesn't expose one publicly)
// =============================================================================
public struct Matrix3x3
{
    public float m00, m01, m02;
    public float m10, m11, m12;
    public float m20, m21, m22;

    public static Matrix3x3 Zero => new Matrix3x3();

    public static Matrix3x3 Identity => new Matrix3x3
    {
        m00 = 1, m11 = 1, m22 = 1
    };

    public static Vector3 operator *(Matrix3x3 m, Vector3 v) => new Vector3(
        m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
        m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
        m.m20 * v.x + m.m21 * v.y + m.m22 * v.z
    );

    /// <summary>Inverse via Cramer's rule.</summary>
    public Matrix3x3 Inverse()
    {
        float det =
            m00 * (m11 * m22 - m12 * m21) -
            m01 * (m10 * m22 - m12 * m20) +
            m02 * (m10 * m21 - m11 * m20);

        if (Mathf.Abs(det) < 1e-10f)
        {
            Debug.LogWarning("Matrix3x3: near-singular inertia tensor, returning identity.");
            return Identity;
        }

        float invDet = 1f / det;

        return new Matrix3x3
        {
            m00 = invDet * (m11 * m22 - m21 * m12),
            m01 = invDet * (m02 * m21 - m01 * m22),
            m02 = invDet * (m01 * m12 - m02 * m11),
            m10 = invDet * (m12 * m20 - m10 * m22),
            m11 = invDet * (m00 * m22 - m02 * m20),
            m12 = invDet * (m02 * m10 - m00 * m12),
            m20 = invDet * (m10 * m21 - m20 * m11),
            m21 = invDet * (m20 * m01 - m00 * m21),
            m22 = invDet * (m00 * m11 - m10 * m01)
        };
    }
}