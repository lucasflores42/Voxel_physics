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
    // profiling
    int _profileFrame = 0;
    const int PROFILE_INTERVAL = 60;
    double _accGrid = 0, _accLiquid = 0, _accGas = 0, _accPowder = 0, _accSolid = 0, _accCollisions = 0, _accRB = 0, _accBC = 0, _accBCRB = 0;


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
        Vector2 hillCenter = new Vector2(boxSize * 1/4f, boxSize * 1/4f);
        float hillAmp = 2.9f;
        float hillSigma = 1.2f;
        // valley
        Vector2 valleyCenter = new Vector2(boxSize * 1/2f, boxSize * 1/2f);
        float valleyAmp = -1.8f;
        float valleySigma = 1.5f;

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
                particles.Add(new Particle(pos, Vector3.zero, 100f, MaterialType.Solid, particleRadius, rigidBodyId: terrainId, sph: 0));
                terrainIndices.Add(particles.Count - 1);
            }
        }

        (Vector3 terrainCM, float terrainMass) = SPHPhysics.CalculateCenterOfMass(particles, terrainIndices);
        foreach (int idx in terrainIndices)
            particles[idx].velocity = Vector3.zero;

        var terrainRb = new RigidBodyData(terrainId, terrainIndices, terrainCM, Vector3.zero, Vector3.zero, 0, terrainMass);
        Matrix3x3 terrainI = SPHPhysics.CalculateInertiaTensor(particles, terrainRb);
        terrainRb.inertia = terrainI;
        terrainRb.invInertia = terrainI.Inverse();
        rigidbodies.Add(terrainRb);

        // ------------------------------------------------------------------
        // Create a player rigid body and wire PlayerRBController(s)
        // ------------------------------------------------------------------
        int playerId = 2;
        // per-particle mass for the player's rigid body (tune as needed)
        float playerParticleMass = 5f;
        Vector3 playerPos = new Vector3(boxSize * 0.5f, 6f, boxSize * 0.5f);
        ParticleFactory.CreatePlayerStack(particles, rigidbodies, playerId, playerParticleMass, playerPos, Vector3.zero, Vector3.zero);

        // compute inertia for the newly created player RB (ParticleFactory already computes it, but ensure it's present)
        foreach (var rb in rigidbodies)
        {
            if (rb.id == playerId)
            {
                Matrix3x3 Iplayer = SPHPhysics.CalculateInertiaTensor(particles, rb);
                rb.inertia = Iplayer;
                rb.invInertia = Iplayer.Inverse();
                break;
            }
        }

        // Wire any PlayerRBController components to use this RB
        var controllers = UnityEngine.Object.FindObjectsByType<PlayerRBController>(FindObjectsSortMode.None);
        foreach (var c in controllers)
        {
            c.simManager = this;
            c.playerRbId = playerId;
        }

            
    }

    // -----------------------------------------------------------------------
    //  Simulation step
    // -----------------------------------------------------------------------

    void SimulateStep()
    {
        double t0 = Time.realtimeSinceStartupAsDouble;
        grid.Rebuild(particles);
        double tGrid = (Time.realtimeSinceStartupAsDouble - t0) * 1000.0;

        double t1 = Time.realtimeSinceStartupAsDouble;
        liquidCalc.UpdateParticles(particles, physicsManager, dt, grid);
        double tLiquid = (Time.realtimeSinceStartupAsDouble - t1) * 1000.0;

        double t2 = Time.realtimeSinceStartupAsDouble;
        gasCalc.UpdateParticles(particles, physicsManager, dt, grid);
        double tGas = (Time.realtimeSinceStartupAsDouble - t2) * 1000.0;

        double t3 = Time.realtimeSinceStartupAsDouble;
        powderCalc.UpdateParticles(particles, physicsManager, dt);
        double tPowder = (Time.realtimeSinceStartupAsDouble - t3) * 1000.0;

        double t4 = Time.realtimeSinceStartupAsDouble;
        solidCalc.UpdateParticles(particles, physicsManager, dt);
        double tSolid = (Time.realtimeSinceStartupAsDouble - t4) * 1000.0;

        double t5 = Time.realtimeSinceStartupAsDouble;
        physicsManager.CalculateCollisions(particles, rigidbodies, grid);
        double tColl = (Time.realtimeSinceStartupAsDouble - t5) * 1000.0;

        double t6 = Time.realtimeSinceStartupAsDouble;
        physicsManager.UpdateRigidBodies(particles, rigidbodies);
        double tRB = (Time.realtimeSinceStartupAsDouble - t6) * 1000.0;

        double t7 = Time.realtimeSinceStartupAsDouble;
        physicsManager.ApplyBoundaryConditions(particles);
        double tBC = (Time.realtimeSinceStartupAsDouble - t7) * 1000.0;

        double t8 = Time.realtimeSinceStartupAsDouble;
        physicsManager.ApplyBoundaryConditionsRigidBodies(particles, rigidbodies);
        double tBCRB = (Time.realtimeSinceStartupAsDouble - t8) * 1000.0;

        _profileFrame++;
        _accGrid += tGrid; _accLiquid += tLiquid; _accGas += tGas; _accPowder += tPowder; _accSolid += tSolid; _accCollisions += tColl; _accRB += tRB; _accBC += tBC; _accBCRB += tBCRB;

        if (_profileFrame % PROFILE_INTERVAL == 0)
        {
            double avgGrid = _accGrid / PROFILE_INTERVAL;
            double avgLiquid = _accLiquid / PROFILE_INTERVAL;
            double avgGas = _accGas / PROFILE_INTERVAL;
            double avgPowder = _accPowder / PROFILE_INTERVAL;
            double avgSolid = _accSolid / PROFILE_INTERVAL;
            double avgColl = _accCollisions / PROFILE_INTERVAL;
            double avgRB = _accRB / PROFILE_INTERVAL;
            double avgBC = _accBC / PROFILE_INTERVAL;
            double avgBCRB = _accBCRB / PROFILE_INTERVAL;

            int cellCount = grid.CellCount();
            int totalParticlesInCells = grid.TotalParticlesCount();
            double avgParticlesPerCell = cellCount > 0 ? (double)totalParticlesInCells / cellCount : 0.0;

            Debug.Log(string.Format("Sim profile (ms avg): grid={0:F2}, liquid={1:F2}, gas={2:F2}, powder={3:F2}, solid={4:F2}, collisions={5:F2}, rbs={6:F2}, bc={7:F2}, bcrb={8:F2} | cells={9}, avgParticles/cell={10:F2}", avgGrid, avgLiquid, avgGas, avgPowder, avgSolid, avgColl, avgRB, avgBC, avgBCRB, cellCount, avgParticlesPerCell));

            _accGrid = _accLiquid = _accGas = _accPowder = _accSolid = _accCollisions = _accRB = _accBC = _accBCRB = 0;
            _profileFrame = 0;
        }
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