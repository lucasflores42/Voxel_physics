using UnityEngine;
using System.Collections.Generic;

/// Main. Attach to a GameObject in your Unity scene.

public class SimulationManager : MonoBehaviour
{
    // -----------------------------------------------------------------------
    //  Inspector parameters
    // -----------------------------------------------------------------------

    [Header("World")]
    public float tmaxInspector = 100f;
    public float dtInspector = 0.01f;
    public float boxSizeInspector = 10f;
    public float dampingInspector = 0f;

    [Header("SPH")]
    public float smoothingLengthInspector = 0.1f;

    [Header("Liquid")]
    public float liquidTargetDensityInspector = 1000f;
    public float liquidStiffCoefInspector = 10f;
    public float liquidViscosityCoefInspector = 0.2f;

    [Header("Gas")]
    public float gasTargetDensityInspector = 1000f;
    public float gasStiffCoefInspector = 100f;
    public float gasViscosityCoefInspector = 0.05f;

    public float gasRegionInspector = 5f;
    public float gasRegionWidthInspector = 2f;
    public float gasRegionStrengthInspector = 10f;

    [Header("Collision")]
    public float collisionRestitutionCoefficientInspector = 0f;

    [Header("Physics")]
    public float gravityCoefInspector = 0.1f;

    [Header("Spawn")]
    public int  liquidParticleCount = 500;
    public bool spawnSolidParticle  = true;
    public bool spawnCube           = false;
    public bool spawnSphere         = false;

    // -----------------------------------------------------------------------
    //  Global static parameters
    // -----------------------------------------------------------------------

    public static float tmax;
    public static float dt;
    public static float boxSize;
    public static float damping;

    public static float smoothingLength;

    public static float liquidTargetDensity;
    public static float liquidStiffCoef;
    public static float liquidViscosityCoef;

    public static float gasTargetDensity;
    public static float gasStiffCoef;
    public static float gasViscosityCoef;

    public static float gasRegion;
    public static float gasRegionWidth;
    public static float gasRegionStrength;

    public static float collisionRestitutionCoefficient;

    public static float gravityCoef;

    // -----------------------------------------------------------------------
    //  Subsystem references
    // -----------------------------------------------------------------------

    PhysicsManager    physicsManager;
    LiquidCalculation liquidCalc;
    GasCalculation    gasCalc;
    PowderCalculation powderCalc;
    SolidCalculation  solidCalc;

    // -----------------------------------------------------------------------
    //  Simulation state
    // -----------------------------------------------------------------------

    List<Particle> particles = new List<Particle>();
    List<RigidBodyData> rigidbodies = new List<RigidBodyData>();

    // -----------------------------------------------------------------------
    //  Unity lifecycle
    // -----------------------------------------------------------------------

    void Awake()
    {
        // ---------------------------------------------------------------
        // Copy inspector values into global static variables
        // ---------------------------------------------------------------

        tmax = tmaxInspector;
        dt = dtInspector;
        boxSize = boxSizeInspector;
        damping = dampingInspector;

        smoothingLength = smoothingLengthInspector;

        liquidTargetDensity = liquidTargetDensityInspector;
        liquidStiffCoef = liquidStiffCoefInspector;
        liquidViscosityCoef = liquidViscosityCoefInspector;

        gasTargetDensity = gasTargetDensityInspector;
        gasStiffCoef = gasStiffCoefInspector;
        gasViscosityCoef = gasViscosityCoefInspector;

        gasRegion = gasRegionInspector;
        gasRegionWidth = gasRegionWidthInspector;
        gasRegionStrength = gasRegionStrengthInspector;

        collisionRestitutionCoefficient = collisionRestitutionCoefficientInspector;

        gravityCoef = gravityCoefInspector;

        // ---------------------------------------------------------------
        // Get subsystems
        // ---------------------------------------------------------------

        physicsManager = GetOrAdd<PhysicsManager>();
        liquidCalc     = GetOrAdd<LiquidCalculation>();
        gasCalc        = GetOrAdd<GasCalculation>();
        powderCalc     = GetOrAdd<PowderCalculation>();
        solidCalc      = GetOrAdd<SolidCalculation>();
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
    //  Scene setup
    // -----------------------------------------------------------------------

    void SpawnScene()
    {
        if (spawnSolidParticle)
        {
            ParticleFactory.CreateParticle(
                particles,
                1000f,
                0.5f,
                new Vector3(5, 5, 5),
                Vector3.zero
            );
        }

        ParticleFactory.CreateLiquid(
            particles,
            liquidParticleCount,
            boxSize
        );

        if (spawnCube)
        {
            ParticleFactory.CreateCube(
                particles,
                rigidbodies,
                1,
                new Vector3(5 - 0.8f, 5, 8),
                new Vector3(0, 0, -20),
                Vector3.zero
            );

            ParticleFactory.CreateCube(
                particles,
                rigidbodies,
                2,
                new Vector3(5, 5, 3),
                Vector3.zero,
                Vector3.zero
            );
        }

        if (spawnSphere)
        {
            ParticleFactory.CreateSphere(
                particles,
                rigidbodies,
                1,
                new Vector3(5, 5, 5),
                Vector3.zero,
                Vector3.zero
            );
        }
    }

    // -----------------------------------------------------------------------
    //  Simulation step
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
    //  Public access
    // -----------------------------------------------------------------------

    public IReadOnlyList<Particle> Particles => particles;
    public IReadOnlyList<RigidBodyData> RigidBodies => rigidbodies;

    // -----------------------------------------------------------------------
    //  Helper
    // -----------------------------------------------------------------------

    T GetOrAdd<T>() where T : Component
    {
        T c = GetComponent<T>();

        if (c == null)
            c = gameObject.AddComponent<T>();

        return c;
    }
}