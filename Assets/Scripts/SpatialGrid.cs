using System.Collections.Generic;
using UnityEngine;

public class SpatialGrid
{
    private readonly float cellSize;
    private readonly Dictionary<Vector3Int, List<Particle>> cells;

    public SpatialGrid(float cellSize)
    {
        this.cellSize = Mathf.Max(1e-4f, cellSize);
        cells = new Dictionary<Vector3Int, List<Particle>>();
    }

    public void Clear()
    {
        cells.Clear();
    }

    public void Rebuild(IEnumerable<Particle> particles)
    {
        Clear();
        foreach (var particle in particles)
        {
            AddParticle(particle);
        }
    }

    public void AddParticle(Particle particle)
    {
        var cell = GetCell(particle.position);
        if (!cells.TryGetValue(cell, out var list))
        {
            list = new List<Particle>();
            cells[cell] = list;
        }

        list.Add(particle);
    }

    public Vector3Int GetCell(Vector3 position)
    {
        return new Vector3Int(
            Mathf.FloorToInt(position.x / cellSize),
            Mathf.FloorToInt(position.y / cellSize),
            Mathf.FloorToInt(position.z / cellSize)
        );
    }

    public IEnumerable<Particle> GetNeighborCandidates(Vector3 position)
    {
        var center = GetCell(position);

        for (int x = center.x - 1; x <= center.x + 1; x++)
        {
            for (int y = center.y - 1; y <= center.y + 1; y++)
            {
                for (int z = center.z - 1; z <= center.z + 1; z++)
                {
                    var neighborCell = new Vector3Int(x, y, z);
                    if (cells.TryGetValue(neighborCell, out var list))
                    {
                        foreach (var other in list)
                        {
                            yield return other;
                        }
                    }
                }
            }
        }
    }

    public IEnumerable<Particle> GetParticlesInCell(Vector3Int cell)
    {
        if (cells.TryGetValue(cell, out var list))
        {
            foreach (var particle in list)
            {
                yield return particle;
            }
        }
    }
}
