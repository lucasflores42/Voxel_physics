using UnityEngine;

public enum MaterialType { Solid, Liquid, Gas, Powder }

public class Particle
{
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 acceleration;

    public float density;
    public float pressure;
    public float mass;

    public MaterialType material;
    public float radius;
    public float temperature;

    // 0 = free particle, >0 = belongs to a rigid body
    public int rigidBodyId;

    public Particle(Vector3 position, 
                    Vector3 velocity, 
                    float mass,
                    MaterialType material, 
                    float radius,
                    float temperature = 1f, 
                    int rigidBodyId = 0
                    )
    {
        this.position    = position;
        this.velocity    = velocity;
        this.acceleration = Vector3.zero;
        this.density     = 1000f;
        this.pressure    = 0f;
        this.mass        = mass;
        this.material    = material;
        this.radius      = radius;
        this.temperature = temperature;
        this.rigidBodyId = rigidBodyId;
    }
} 
