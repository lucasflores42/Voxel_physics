using UnityEngine;
using System.Collections.Generic;

/// Handles rigid body integration, collisions, and boundary conditions.
/// Called each step by SimulationManager.

public class PhysicsManager : MonoBehaviour
{
    
    //  Gravity wrapper (used by material calculators)
    public Vector3 CalculateGravity(Vector3 position, float mass, int rigidBodyId,
                                    List<Particle> particles)
        => SPHPhysics.CalculateGravity(position, mass, rigidBodyId, particles, SimulationManager.gravityCoef);


    //  Rigid body integration  (translation + rotation via Rodrigues)
    public void UpdateRigidBodies(List<Particle> particles, List<RigidBodyData> rigidbodies)
    {
        foreach (RigidBodyData rb in rigidbodies)
        {
            Vector3 cmOld = rb.centerOfMass;

            // --- Translation ---
            float M = TotalMass(particles, rb);
            Vector3 fGravity = CalculateGravity(rb.centerOfMass, M, rb.id, particles);
            Vector3 a = fGravity / M;

            rb.velocity      += a * SimulationManager.dt;
            rb.centerOfMass  += rb.velocity * SimulationManager.dt;

            // --- Rotate particles around new CM ---
            Vector3 omega = rb.angularVelocity;
            float   theta = omega.magnitude * SimulationManager.dt;

            foreach (int idx in rb.particleIndices)
            {
                Particle p = particles[idx];
                Vector3 r  = p.position - cmOld;

                if (theta > 0f)
                {
                    Vector3 k = omega / omega.magnitude;   // rotation axis
                    r = r * Mathf.Cos(theta)
                      + Vector3.Cross(k, r) * Mathf.Sin(theta)
                      + k * Vector3.Dot(k, r) * (1f - Mathf.Cos(theta));
                }

                p.position = rb.centerOfMass + r;
                p.velocity = rb.velocity + Vector3.Cross(omega, r);
            }
        }
    }

    public void CalculateCollisions(List<Particle> particles, List<RigidBodyData> rigidbodies)
    {
        for (int i = 0; i < particles.Count; i++)
            for (int j = i + 1; j < particles.Count; j++)
                ResolveCollision(particles[i], particles[j], particles, rigidbodies);
    }

    void ResolveCollision(Particle p1, Particle p2,
                          List<Particle> particles, List<RigidBodyData> rigidbodies)
    {
        Vector3 rVec = p1.position - p2.position;
        float r = rVec.magnitude;

        if (r >= p1.radius + p2.radius || r < 0.0001f) return;

        bool rb1Exists = p1.rigidBodyId != 0;
        bool rb2Exists = p2.rigidBodyId != 0;

        if (rb1Exists && rb2Exists)
            CollideRBvsRB(p1, p2, r, particles, rigidbodies);
        else if (!rb1Exists && !rb2Exists)
            CollideFreeVsFree(p1, p2, r);
        else if (rb1Exists && !rb2Exists)
            CollideRBvsFree(p1, p2, r, particles, rigidbodies);
        else
            CollideRBvsFree(p2, p1, r, particles, rigidbodies); // swap so rb is first arg
    }

    void CollideFreeVsFree(Particle p1, Particle p2, float r)
    {
        Vector3 x1 = p1.position, x2 = p2.position;
        Vector3 v1 = p1.velocity, v2 = p2.velocity;
        float m1 = p1.mass, m2 = p2.mass;

        Vector3 normal = (x1 - x2) / r;
        float overlap  = p1.radius + p2.radius - r;
        float totalMass = m1 + m2;
        float cr = SimulationManager.collisionRestitutionCoefficient;
        float contactR = p1.radius + p2.radius;

        Vector3 dv1 = -(1f + cr) * m2 / totalMass
                      * Vector3.Dot(v1 - v2, x1 - x2) * (x1 - x2) / (contactR * contactR);
        Vector3 dv2 = -(1f + cr) * m1 / totalMass
                      * Vector3.Dot(v2 - v1, x2 - x1) * (x2 - x1) / (contactR * contactR);

        p1.position += overlap * normal * (m2 / totalMass);
        p1.velocity += dv1;
        p2.position -= overlap * normal * (m1 / totalMass);
        p2.velocity += dv2;
    }

    void CollideRBvsRB(Particle p1, Particle p2, float r,
                       List<Particle> particles, List<RigidBodyData> rigidbodies)
    {
        RigidBodyData rb1 = FindRB(rigidbodies, p1.rigidBodyId);
        RigidBodyData rb2 = FindRB(rigidbodies, p2.rigidBodyId);

        Vector3 x1 = p1.position, x2 = p2.position;
        Vector3 v1 = p1.velocity, v2 = p2.velocity;
        float m1 = TotalMass(particles, rb1);
        float m2 = TotalMass(particles, rb2);

        Vector3 normal = (x1 - x2) / r;
        float overlap  = p1.radius + p2.radius - r;
        float contactR = p1.radius + p2.radius;
        float cr = SimulationManager.collisionRestitutionCoefficient;

        Vector3 dv1 = -(1f + cr) * m2 / (m1 + m2)
                      * Vector3.Dot(v1 - v2, x1 - x2) * (x1 - x2) / (contactR * contactR);
        Vector3 dv2 = -(1f + cr) * m1 / (m1 + m2)
                      * Vector3.Dot(v2 - v1, x2 - x1) * (x2 - x1) / (contactR * contactR);

        Vector3 shift1 =  overlap * normal * (m2 / (m1 + m2));
        Vector3 shift2 = -overlap * normal * (m1 / (m1 + m2));

        ShiftRB(particles, rb1,  shift1);
        ShiftRB(particles, rb2, -shift2);

        ApplyImpulseToRB(particles, rb1, p1, m1 * dv1);
        ApplyImpulseToRB(particles, rb2, p2, m2 * dv2);
    }

    void CollideRBvsFree(Particle rbParticle, Particle freeParticle, float r,
                         List<Particle> particles, List<RigidBodyData> rigidbodies)
    {
        RigidBodyData rb = FindRB(rigidbodies, rbParticle.rigidBodyId);

        Vector3 x1 = rbParticle.position, x2 = freeParticle.position;
        Vector3 v1 = rbParticle.velocity, v2 = freeParticle.velocity;
        float m1 = TotalMass(particles, rb);
        float m2 = freeParticle.mass;

        Vector3 normal = (x1 - x2) / r;
        float overlap  = rbParticle.radius + freeParticle.radius - r;
        float contactR = rbParticle.radius + freeParticle.radius;
        float cr = SimulationManager.collisionRestitutionCoefficient;

        Vector3 dv1 = -(1f + cr) * m2 / (m1 + m2)
                      * Vector3.Dot(v1 - v2, x1 - x2) * (x1 - x2) / (contactR * contactR);
        Vector3 dv2 = -(1f + cr) * m1 / (m1 + m2)
                      * Vector3.Dot(v2 - v1, x2 - x1) * (x2 - x1) / (contactR * contactR);

        Vector3 shift = overlap * normal * (m2 / (m1 + m2));
        ShiftRB(particles, rb, shift);

        ApplyImpulseToRB(particles, rb, rbParticle, m1 * dv1);

        freeParticle.position -= overlap * normal * (m1 / (m1 + m2));
        freeParticle.velocity += dv2;
    }

    // -------------------------------------------------------------------------
    //  Boundary conditions
    // -------------------------------------------------------------------------
    public void ApplyBoundaryConditions(List<Particle> particles)
    {
        foreach (Particle p in particles)
        {
            if (p.rigidBodyId != 0) continue;

            ClampAxis(ref p.position.x, ref p.velocity.x, p.radius);
            ClampAxis(ref p.position.y, ref p.velocity.y, p.radius);
            ClampAxis(ref p.position.z, ref p.velocity.z, p.radius);
        }
    }

    public void ApplyBoundaryConditionsRigidBodies(List<Particle> particles,
                                                    List<RigidBodyData> rigidbodies)
    {
        foreach (RigidBodyData rb in rigidbodies)
        {
            float M = TotalMass(particles, rb);
            Matrix3x3 invI = SPHPhysics.CalculateInertiaTensor(particles, rb).Inverse();

            foreach (int idx in rb.particleIndices)
            {
                Particle p = particles[idx];

                for (int d = 0; d < 3; d++)
                {
                    float pos   = GetComponent(p.position, d);
                    float lower = p.radius;
                    float upper = SimulationManager.boxSize - p.radius;

                    if (pos < lower)
                        ResolveRBWall(particles, rb, p, d, lower, 1f, M, invI);
                    else if (pos > upper)
                        ResolveRBWall(particles, rb, p, d, upper, -1f, M, invI);
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    //  Internal helpers
    // -------------------------------------------------------------------------
    void ResolveRBWall(List<Particle> particles, RigidBodyData rb, Particle p,
                       int d, float wall, float normalSign, float M, Matrix3x3 invI)
    {
        float shift = wall - GetComponent(p.position, d);
        Vector3 rRel = p.position - rb.centerOfMass;

        foreach (int idx in rb.particleIndices)
            SetComponent(ref particles[idx].position, d,
                         GetComponent(particles[idx].position, d) + shift);
        SetComponent(ref rb.centerOfMass, d, GetComponent(rb.centerOfMass, d) + shift);

        Vector3 normal = Vector3.zero;
        SetComponent(ref normal, d, normalSign);

        Vector3 vContact = rb.velocity + Vector3.Cross(rb.angularVelocity, rRel);
        float vn = Vector3.Dot(vContact, normal);

        if (vn < 0f)
        {
            Vector3 dv  = -(1f + SimulationManager.damping) * vn * normal;
            Vector3 imp = p.mass * dv;
            rb.velocity        += imp / M;
            rb.angularVelocity += invI * Vector3.Cross(rRel, imp);
        }
    }

    void ApplyImpulseToRB(List<Particle> particles, RigidBodyData rb, Particle contactParticle,
                          Vector3 impulse)
    {
        float M = TotalMass(particles, rb);
        Matrix3x3 invI = SPHPhysics.CalculateInertiaTensor(particles, rb).Inverse();
        Vector3 rRel = contactParticle.position - rb.centerOfMass;

        rb.velocity        += impulse / M;
        rb.angularVelocity += invI * Vector3.Cross(rRel, impulse);
    }

    void ShiftRB(List<Particle> particles, RigidBodyData rb, Vector3 shift)
    {
        rb.centerOfMass += shift;
        foreach (int idx in rb.particleIndices)
            particles[idx].position += shift;
    }

    static float TotalMass(List<Particle> particles, RigidBodyData rb)
    {
        float m = 0f;
        foreach (int idx in rb.particleIndices) m += particles[idx].mass;
        return m;
    }

    static RigidBodyData FindRB(List<RigidBodyData> rigidbodies, int id)
    {
        foreach (var rb in rigidbodies) if (rb.id == id) return rb;
        return null;
    }

    static float GetComponent(Vector3 v, int d) => d == 0 ? v.x : d == 1 ? v.y : v.z;

    static void SetComponent(ref Vector3 v, int d, float val)
    {
        if (d == 0) v.x = val;
        else if (d == 1) v.y = val;
        else v.z = val;
    }

    void ClampAxis(ref float pos, ref float vel, float radius)
    {
        if (pos < radius)        { pos =  radius;        vel *= -SimulationManager.damping; }
        else if (pos > SimulationManager.boxSize - radius) { pos = SimulationManager.boxSize - radius; vel *= -SimulationManager.damping; }
    }
}