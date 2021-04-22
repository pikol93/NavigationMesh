using System;
using System.Collections.Generic;
using System.Numerics;

namespace Pikol93.NavigationMesh
{
    public static class NavMeshFactory
    {
        // public static List<Vector2[]> portals;
        // public static List<Vector2[]> edges;

        /// <summary>
        /// Tries to create a new NavMesh.
        /// </summary>
        /// <param name="bounds">The bounds of the NavMesh, the passed array needs to contain at least 3 vertices, so that it can form a polygon.</param>
        /// <param name="shapes">The collision shapes that the agents need to avoid.</param>
        /// <param name="agentSize">Size of the agent that the NavMesh is created for.</param>
        /// <returns>The created NavMesh.</returns>
        public static NavMesh Create(Vector2[] bounds, Vector2[][] shapes, double agentSize)
        {
            // Bounds need to form a polygon
            if (bounds == null || bounds.Length < 3)
                throw new ArgumentException("Bounds need to form a polygon.");

            // agentSize less than 1.0 leads to undefined behaviour due to polygon inflation
            if (Math.Round(agentSize) < 1.0)
                throw new ArgumentException("Agent size has to be greater than 1.0.");

            Vector2[][] inflatedPolygons = PolygonInflation.InflatePolygons(bounds, shapes, agentSize);

            NavigationMeshGenerator generator = new NavigationMeshGenerator(inflatedPolygons);

            var result = generator.GenerateNavMesh();

            // portals = new List<Vector2[]>();
            // foreach (NavigationMeshGenerator.IIntTuple tuple in generator.Portals)
            // {
            //     portals.Add(new Vector2[] { generator.Vertices[tuple.Item1].Position, generator.Vertices[tuple.Item2].Position } );
            // }

            // edges = new List<Vector2[]>();
            // foreach (NavigationMeshGenerator.IIntTuple tuple in generator.Edges)
            // {
            //     edges.Add(new Vector2[] { generator.Vertices[tuple.Item1].Position, generator.Vertices[tuple.Item2].Position } );
            // }

            return result;
        }
    }
}