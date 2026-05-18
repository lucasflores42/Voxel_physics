using UnityEngine;
using System.Collections.Generic;

public class ParticleRenderer : MonoBehaviour
{
    public GameObject particlePrefab;

    SimulationManager manager;

    List<GameObject> particleObjects = new List<GameObject>();

    void Start()
    {
        manager = FindFirstObjectByType<SimulationManager>();
    }

    void Update()
    {
        IReadOnlyList<Particle> particles = manager.Particles;

        // Cria objetos faltantes
        while (particleObjects.Count < particles.Count)
        {
            int i = particleObjects.Count;

            GameObject obj = Instantiate(
                particlePrefab,
                particles[i].position,
                Quaternion.identity
            );

            obj.transform.localScale =
                Vector3.one * (2f * particles[i].radius);

            particleObjects.Add(obj);
        }

        // Atualiza posições
        for (int i = 0; i < particles.Count; i++)
        {
            particleObjects[i].transform.position =
                particles[i].position;
        }
    }
}