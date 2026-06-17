using UnityEngine;
using System.Collections.Generic;

public class RigidBodyData
{
    public int id;
    public List<int> particleIndices;

    public Vector3 centerOfMass;
    public Vector3 velocity;
    public Vector3 angularVelocity;
    public int physics;

    public float totalMass;
    public Matrix3x3 inertia;
    public Matrix3x3 invInertia;

    public RigidBodyData(int id,
                        List<int> particleIndices,
                        Vector3 centerOfMass,
                        Vector3 velocity,
                        Vector3 angularVelocity,
                        int physics,
                        float totalMass)
    {
        this.id              = id;
        this.particleIndices = particleIndices;
        this.centerOfMass    = centerOfMass;
        this.velocity        = velocity;
        this.angularVelocity = angularVelocity;
        this.physics = physics;
        this.totalMass = totalMass;
        this.inertia = Matrix3x3.Zero;
        this.invInertia = Matrix3x3.Identity;
    }

    // Recompute mass, center of mass and inertia from current particle positions
    public void RecomputeFromParticles(List<Particle> particles)
    {
        (Vector3 cm, float mass) = SPHPhysics.CalculateCenterOfMass(particles, particleIndices);
        centerOfMass = cm;
        totalMass = mass;

        Matrix3x3 I = SPHPhysics.CalculateInertiaTensor(particles, this);
        inertia = I;
        invInertia = I.Inverse();
    }
}