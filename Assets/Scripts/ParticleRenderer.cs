using UnityEngine;
using System.Collections.Generic;

public class ParticleRenderer : MonoBehaviour
{
    public GameObject particlePrefab;
    public Material solidMaterial;
    public Material liquidMaterial;

    SimulationManager manager;
    List<GameObject> particleObjects = new List<GameObject>();

    void Start()
    {
        manager = FindFirstObjectByType<SimulationManager>();
    }

    void Update()
    {
        IReadOnlyList<Particle> particles = manager.Particles;

        while (particleObjects.Count < particles.Count)
        {
            int i = particleObjects.Count;

            GameObject obj = Instantiate(particlePrefab,
                particles[i].position, Quaternion.identity);

            obj.transform.localScale =
                Vector3.one * (2f * particles[i].radius);

            var rend = obj.GetComponent<Renderer>();
            if (rend != null)
            {
                if (particles[i].material == MaterialType.Liquid)
                    rend.material = liquidMaterial;
                else
                    rend.material = solidMaterial;
            }

            particleObjects.Add(obj);
        }

        for (int i = 0; i < particles.Count; i++)
        {
            particleObjects[i].transform.position = particles[i].position;
        }
    }
}