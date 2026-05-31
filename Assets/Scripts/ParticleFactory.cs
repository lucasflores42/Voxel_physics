using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Creates particles and rigid bodies — mirrors Julia's functions.cs / voxel.jl factory functions.
/// </summary>
public static class ParticleFactory
{
    // -------------------------------------------------------------------------
    //  Single free solid particle
    // -------------------------------------------------------------------------
    public static void CreateParticleSolid(List<Particle> particles,
                                      float mass, float radius,
                                      Vector3 position, Vector3 velocity)
    {
        particles.Add(new Particle(position, velocity, mass, MaterialType.Solid, radius));
    }

    // -------------------------------------------------------------------------
    //  Liquid cloud
    // -------------------------------------------------------------------------
    public static void CreateLiquid(List<Particle> particles, int numParticles)
    {
        for (int i = 0; i < numParticles; i++)
        {
            Vector3 pos = new Vector3(
                Random.value * SimulationManager.boxSize,
                9f,
                Random.value * SimulationManager.boxSize
            );

            particles.Add(new Particle(pos, Vector3.zero, 0.1f, MaterialType.Liquid, 0.1f,
                                       temperature: 1f));
        }
    }

    // -------------------------------------------------------------------------
    //  Rigid Cube  (8 corner particles)
    // -------------------------------------------------------------------------
    public static void CreateCube(List<Particle> particles, List<RigidBodyData> rigidbodies,
                                  int id, float mass,Vector3 offset, Vector3 initVelocity, Vector3 initAngular)
    {
        float r  = 0.4f;
        float d  = 2f * r;

        Vector3[] localPositions =
        {
            new Vector3(0,0,0), new Vector3(d,0,0), new Vector3(d,d,0), new Vector3(0,d,0),
            new Vector3(0,0,d), new Vector3(d,0,d), new Vector3(d,d,d), new Vector3(0,d,d)
        };

        BuildRigidBody(particles, rigidbodies, id, offset, initVelocity, initAngular,
                       localPositions, mass: mass, radius: r, material: MaterialType.Solid);
    }

    // -------------------------------------------------------------------------
    //  Rigid Sphere  (shell of particles)
    // -------------------------------------------------------------------------
    public static void CreateSphere(List<Particle> particles, List<RigidBodyData> rigidbodies,
                                    int id, float mass, Vector3 offset, Vector3 initVelocity, Vector3 initAngular)
    {
        float particleRadius = 0.15f;
        float sphereRadius   = 0.5f;
        float thickness      = 0.1f;

        var localPositions = new List<Vector3>();

        for (float z = -sphereRadius; z <= sphereRadius; z += particleRadius)
        for (float x = -sphereRadius; x <= sphereRadius; x += particleRadius)
        for (float y = -sphereRadius; y <= sphereRadius; y += particleRadius)
        {
            float r2 = x * x + y * y + z * z;
            float sr2 = sphereRadius * sphereRadius;
            if (r2 >= sr2 - thickness && r2 <= sr2 + thickness)
                localPositions.Add(new Vector3(x, y, z));
        }

        BuildRigidBody(particles, rigidbodies, id, offset, initVelocity, initAngular,
                       localPositions.ToArray(), mass: mass, radius: particleRadius,
                       material: MaterialType.Solid);
    }

    // -------------------------------------------------------------------------
    //  Rigid Disk
    // -------------------------------------------------------------------------
    public static void CreateDisk(List<Particle> particles, List<RigidBodyData> rigidbodies,
                                  int id, float mass, float diskRadius, Vector3 offset,
                                  Vector3 initVelocity, Vector3 initAngular)
    {
        float particleRadius = 0.2f;
        float thickness      = 0.01f;

        var localPositions = new List<Vector3>();

        for (float z = -thickness; z <= thickness; z += particleRadius)
        for (float x = -diskRadius; x <= diskRadius; x += particleRadius)
        for (float y = -diskRadius; y <= diskRadius; y += particleRadius)
        {
            if (x * x + y * y + z * z <= diskRadius * diskRadius)
                localPositions.Add(new Vector3(x, y, z));
        }

        BuildRigidBody(particles, rigidbodies, id, offset, initVelocity, initAngular,
                       localPositions.ToArray(), mass: mass, radius: particleRadius,
                       material: MaterialType.Liquid);
    }


    // -------------------------------------------------------------------------
    //  Shared builder
    // -------------------------------------------------------------------------
    static void BuildRigidBody(List<Particle> particles, List<RigidBodyData> rigidbodies,
                               int id, Vector3 offset, Vector3 initVelocity, Vector3 initAngular,
                               Vector3[] localPositions, float mass, float radius,
                               MaterialType material)
    {
        var indices = new List<int>();

        foreach (Vector3 localPos in localPositions)
        {
            particles.Add(new Particle(offset + localPos, Vector3.zero, mass,
                                       material, radius, rigidBodyId: id));
            indices.Add(particles.Count - 1);
        }

        (Vector3 cm, float _) = SPHPhysics.CalculateCenterOfMass(particles, indices);

        // Apply initial angular velocity to each particle's velocity
        foreach (int idx in indices)
        {
            Vector3 r = particles[idx].position - cm;
            particles[idx].velocity = initVelocity + Vector3.Cross(initAngular, r);
        }

        rigidbodies.Add(new RigidBodyData(id, indices, cm, initVelocity, initAngular));
    }
}