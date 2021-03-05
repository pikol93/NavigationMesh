using System;
using System.Numerics;

namespace Pikol93.NavigationMesh
{
    public static class NavMeshFactory
    {
        public static NavMesh Create(Vector2[] bounds, Vector2[][] shapes, double agentSize)
        {
            // Bounds need to form a polygon
            if (bounds.Length < 3)
                throw new ArgumentException("Bounds need to form a polygon.");

            // agentSize less than 1.0 leads to undefined behaviour due to polygon inflation
            if (Math.Round(agentSize) < 1.0)
                throw new ArgumentException("Agent size has to be greater than 1.0.");

            Vector2[][] inflatedPolygons = PolygonInflation.InflatePolygons(bounds, shapes, agentSize);

            NavigationMeshGenerator generator = new NavigationMeshGenerator(inflatedPolygons);
            return generator.GenerateNavMesh();
        }
    }
}