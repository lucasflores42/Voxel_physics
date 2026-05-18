using UnityEngine;
using System.Collections.Generic;

public class RigidBodyData
{
    public int id;
    public List<int> particleIndices;

    public Vector3 centerOfMass;
    public Vector3 velocity;
    public Vector3 angularVelocity;

    public RigidBodyData(int id, 
                        List<int> particleIndices,
                        Vector3 centerOfMass, 
                        Vector3 velocity, 
                        Vector3 angularVelocity
                        )
    {
        this.id              = id;
        this.particleIndices = particleIndices;
        this.centerOfMass    = centerOfMass;
        this.velocity        = velocity;
        this.angularVelocity = angularVelocity;
    }
}
