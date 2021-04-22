using System;
using System.Numerics;

namespace Pikol93.NavigationMesh
{
    public static class NavMeshFactory
    {
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
            {
                throw new ArgumentException("Bounds need to form a polygon.");
            }

            // agentSize less than 1.0 leads to undefined behaviour due to polygon inflation
            if (Math.Round(agentSize) < 1.0)
            {
                throw new ArgumentException("Agent size has to be greater than 1.0.");
            }

            Vector2[][] inflatedPolygons = PolygonInflation.InflatePolygons(bounds, shapes, agentSize);

            NavigationMeshGenerator generator = new NavigationMeshGenerator();

            return generator.GenerateNavMesh(inflatedPolygons);
        }

        /// <summary>
        /// Sets the bounds of the NavMesh to be a rectangle which contains all the shapes
        /// </summary>
        /// <param name="shapes">The shapes that agents need to avoid.</param>
        /// <param name="agentSize">Size of the agent.</param>
        /// <param name="padding">Padding of the NavMesh bounds.</param>
        /// <returns></returns>
        public static NavMesh CreateWithPadding(Vector2[][] shapes, double agentSize, float padding)
        {
            float minX = float.PositiveInfinity;
            float maxX = float.NegativeInfinity;
            float minY = float.PositiveInfinity;
            float maxY = float.NegativeInfinity;

            foreach (Vector2[] shape in shapes)
            {
                foreach (Vector2 vertex in shape)
                {
                    if (vertex.X < minX)
                    {
                        minX = vertex.X;
                    }
                    if (vertex.X > maxX)
                    {
                        maxX = vertex.X;
                    }
                    if (vertex.Y < minY)
                    {
                        minY = vertex.Y;
                    }
                    if (vertex.Y > maxY)
                    {
                        maxY = vertex.Y;
                    }
                }
            }
            if (minX == float.PositiveInfinity)
            {
                // If one of them was not set then all the others
                // were not set as well
                minX = 0f;
                maxX = 0f;
                minY = 0f;
                maxY = 0f;
            }

            Vector2[] bounds = new Vector2[]
            {
                new Vector2(minX - padding, minY - padding),
                new Vector2(minX - padding, maxY + padding),
                new Vector2(maxX + padding, maxY + padding),
                new Vector2(maxX + padding, minY - padding),
            };

            return Create(bounds, shapes, agentSize);
        }
    }
}