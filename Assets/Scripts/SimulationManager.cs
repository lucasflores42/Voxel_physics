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
    public float dtInspector = 0.1f;
    public float boxSizeInspector = 10f;
    public float dampingInspector = 1.0f;

    [Header("SPH")]
    public float smoothingLengthInspector = 0.1f;

    [Header("Liquid")]
    public float liquidTargetDensityInspector = 100f;
    public float liquidStiffCoefInspector = 1f;
    public float liquidViscosityCoefInspector = 0.5f;

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
    public float cellSize = 0.5f;
    private SpatialGrid grid;


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
        grid = new SpatialGrid(cellSize);

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

    void Update()
    {
        SimulateStep();
    }

    // -----------------------------------------------------------------------
    //  Scene setup
    // -----------------------------------------------------------------------

    void SpawnScene()
    {
        //ParticleFactory.CreateParticleSolid(particles, 1000f, 0.5f, new Vector3(5, 5, 5), Vector3.zero);
        ParticleFactory.CreateLiquid(particles, 200);
        //ParticleFactory.CreateCube(particles, rigidbodies, 1, 100, new Vector3(5, 5, 5), new Vector3(0, 0, 0), Vector3.zero);
        //ParticleFactory.CreateCube(particles, rigidbodies, 2, new Vector3(5, 5, 3), new Vector3(0.5f, 0, 0), Vector3.zero);
        //ParticleFactory.CreateSphere(particles, rigidbodies, 1, 100, new Vector3(5, 5, 5), Vector3.zero, Vector3.zero);

        // Create scene 1
        // Generate solid terrain (single rigid body) with a hill and a valley
        Vector3 terrainCenterOffset = new Vector3(0f, 0f, 0f);
        float startX = 0f, endX = 9f;
        float startZ = 0f, endZ = 9f;
        float particleRadius = 0.15f;
        float spacing = 2f * particleRadius;

        int terrainId = 1;
        var terrainIndices = new List<int>();

        // heightmap parameters
        float baseY = 2.0f;
        // hill
        Vector2 hillCenter = new Vector2(5f, 6f);
        float hillAmp = 0.9f;
        float hillSigma = 1.2f;
        // valley
        Vector2 valleyCenter = new Vector2(7f, 4f);
        float valleyAmp = -0.8f;
        float valleySigma = 1.0f;

        for (float x = startX; x <= endX; x += spacing)
        for (float z = startZ; z <= endZ; z += spacing)
        {
            float dxh = x - hillCenter.x;
            float dzh = z - hillCenter.y;
            float hill = hillAmp * Mathf.Exp(-(dxh * dxh + dzh * dzh) / (2f * hillSigma * hillSigma));

            float dxv = x - valleyCenter.x;
            float dzv = z - valleyCenter.y;
            float valley = valleyAmp * Mathf.Exp(-(dxv * dxv + dzv * dzv) / (2f * valleySigma * valleySigma));

            float y = baseY + hill + valley;

            // add many layers for stability - create a solid block
            for (int layer = 0; layer < 4; layer++)
            {
                Vector3 pos = new Vector3(x, y - layer * spacing, z) + terrainCenterOffset;
                particles.Add(new Particle(pos, Vector3.zero, 100f, MaterialType.Solid, particleRadius, rigidBodyId: terrainId));
                terrainIndices.Add(particles.Count - 1);
            }
        }

        (Vector3 terrainCM, float _) = SPHPhysics.CalculateCenterOfMass(particles, terrainIndices);
        foreach (int idx in terrainIndices)
            particles[idx].velocity = Vector3.zero;

        rigidbodies.Add(new RigidBodyData(terrainId, terrainIndices, terrainCM, Vector3.zero, Vector3.zero));

            
    }

    // -----------------------------------------------------------------------
    //  Simulation step
    // -----------------------------------------------------------------------

    void SimulateStep()
    {
        grid.Rebuild(particles);
        liquidCalc.UpdateParticles(particles, physicsManager, dt, grid);
        gasCalc.UpdateParticles(particles, physicsManager, dt, grid);
        powderCalc.UpdateParticles(particles, physicsManager, dt);
        solidCalc.UpdateParticles(particles, physicsManager, dt);

        physicsManager.CalculateCollisions(particles, rigidbodies, grid);
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