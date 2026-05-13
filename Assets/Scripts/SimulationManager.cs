using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Main entry point. Attach to a GameObject in your Unity scene.
/// Wires all subsystems together and drives the simulation loop via FixedUpdate.
/// Mirrors Julia's main() and simulate_step!().
/// </summary>
public class SimulationManager : MonoBehaviour
{
    // -----------------------------------------------------------------------
    //  Inspector-exposed configuration
    // -----------------------------------------------------------------------
    [Header("World")]
    public float tmax = 100f;
    public float dt = 0.01f;
    public float boxSize = 10f;
    public float damping = 0f;

    [Header("SPH")]
    public float smoothingLength = 0.1f;

    [Header("Liquid")]
    public float liquidTargetDensity = 1000f;
    public float liquidStiffCoef = 10f;
    public float liquidViscosityCoef = 0.2f;

    [Header("Gas")]
    public float gasTargetDensity = 1000f;
    public float gasStiffCoef = 100f;
    public float gasViscosityCoef = 0.05f;

    [Header("Collision")]
    public float collisionRestitutionCoefficient = 0f;

    [Header("Physics")]
    public float gravityCoef = 0.1f;

    [Header("Spawn")]
    public int   liquidParticleCount = 500;
    public bool  spawnSolidParticle  = true;
    public bool  spawnCube           = false;
    public bool  spawnSphere         = false;

    // -----------------------------------------------------------------------
    //  Subsystem references (assign in Inspector or auto-found via GetComponent)
    // -----------------------------------------------------------------------
    PhysicsManager    physicsManager;
    LiquidCalculation liquidCalc;
    GasCalculation    gasCalc;
    PowderCalculation powderCalc;
    SolidCalculation  solidCalc;

    // -----------------------------------------------------------------------
    //  Simulation state
    // -----------------------------------------------------------------------
    List<Particle>     particles   = new List<Particle>();
    List<RigidBodyData> rigidbodies = new List<RigidBodyData>();

    // -----------------------------------------------------------------------
    //  Unity lifecycle
    // -----------------------------------------------------------------------
    void Awake()
    {
        // Grab or auto-create subsystems on the same GameObject
        physicsManager = GetOrAdd<PhysicsManager>();
        liquidCalc     = GetOrAdd<LiquidCalculation>();
        gasCalc        = GetOrAdd<GasCalculation>();
        powderCalc     = GetOrAdd<PowderCalculation>();
        solidCalc      = GetOrAdd<SolidCalculation>();

        // Sync world parameters to PhysicsManager
        physicsManager.boxSize = boxSize;
        physicsManager.dt = dt;
        physicsManager.damping = damping;
        physicsManager.collisionRestitution = collisionRestitutionCoefficient;
        physicsManager.gravityCoef = gravityCoef;

        // Sync SPH parameters to calculators
        liquidCalc.smoothingLength = smoothingLength;
        liquidCalc.liquidTargetDensity = liquidTargetDensity;
        liquidCalc.liquidStiffCoef = liquidStiffCoef;
        liquidCalc.liquidViscosityCoef = liquidViscosityCoef;

        gasCalc.smoothingLength = smoothingLength;
        gasCalc.gasTargetDensity = gasTargetDensity;
        gasCalc.gasStiffCoef = gasStiffCoef;
        gasCalc.gasViscosityCoef = gasViscosityCoef;
    }

    void Start()
    {
        SpawnScene();
    }

    void FixedUpdate()
    {
        SimulateStep();
    }

    // -----------------------------------------------------------------------
    //  Scene setup — mirrors Julia's main()
    // -----------------------------------------------------------------------
    void SpawnScene()
    {
        if (spawnSolidParticle)
            ParticleFactory.CreateParticle(particles, 1000f, 0.5f,
                                           new Vector3(5, 5, 5), Vector3.zero);

        ParticleFactory.CreateLiquid(particles, liquidParticleCount, boxSize);

        if (spawnCube)
        {
            ParticleFactory.CreateCube(particles, rigidbodies, 1,
                                       new Vector3(5 - 0.8f, 5, 8),
                                       new Vector3(0, 0, -20), Vector3.zero);
            ParticleFactory.CreateCube(particles, rigidbodies, 2,
                                       new Vector3(5, 5, 3),
                                       Vector3.zero, Vector3.zero);
        }

        if (spawnSphere)
        {
            ParticleFactory.CreateSphere(particles, rigidbodies, 1,
                                         new Vector3(5, 5, 5), Vector3.zero, Vector3.zero);
        }
    }

    // -----------------------------------------------------------------------
    //  simulate_step!() equivalent
    // -----------------------------------------------------------------------
    void SimulateStep()
    {
        liquidCalc.UpdateParticles(particles, physicsManager, dt);
        gasCalc.UpdateParticles(particles, physicsManager, dt);
        powderCalc.UpdateParticles(particles, physicsManager, dt);
        solidCalc.UpdateParticles(particles, physicsManager, dt);

        physicsManager.CalculateCollisions(particles, rigidbodies);
        physicsManager.UpdateRigidBodies(particles, rigidbodies);
        physicsManager.ApplyBoundaryConditions(particles);
        physicsManager.ApplyBoundaryConditionsRigidBodies(particles, rigidbodies);
    }

    // -----------------------------------------------------------------------
    //  Public access for renderers / UI
    // -----------------------------------------------------------------------
    public IReadOnlyList<Particle>     Particles   => particles;
    public IReadOnlyList<RigidBodyData> RigidBodies => rigidbodies;

    // -----------------------------------------------------------------------
    //  Helper
    // -----------------------------------------------------------------------
    T GetOrAdd<T>() where T : Component
    {
        T c = GetComponent<T>();
        if (c == null) c = gameObject.AddComponent<T>();
        return c;
    }
}