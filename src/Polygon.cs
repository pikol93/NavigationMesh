using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using PortalInt = System.Tuple<int, int>;

namespace Pikol93.NavigationMesh
{
    public class Polygon
    {
        public int[] Vertices { get; }
        public Vector2 Center { get; }
        public float PolygonRadiusSqr { get; }

        public PolygonNeighbour[] Neighbours { get; private set; }

        public Polygon(int[] vertices, Vector2 center, float polygonRadiusSqr)
        {
            Vertices = vertices;
            Center = center;
            PolygonRadiusSqr = polygonRadiusSqr;
        }

        public void SetNeighbours(Vector2[] vertices, List<Polygon> neighbours)
        {
            Neighbours = new PolygonNeighbour[neighbours.Count];
            for (int i = 0; i < neighbours.Count; i++)
            {
                PortalInt portal = GetConnectingPortal(neighbours[i]);

                // Get center of portal
                Vector2 portalCenter = (vertices[portal.Item1] + vertices[portal.Item2]) / 2f;

                Neighbours[i] = new PolygonNeighbour(neighbours[i], portal, portalCenter);
            }
        }

        public bool IsPointInside(Vector2[] vertices, Vector2 point)
        {
            // Check if the point lies in range of the radius
            if ((point - Center).LengthSquared() > PolygonRadiusSqr)
            {
                return false;
            }

            for (int i = 0; i < Vertices.Length; i++)
            {
                Vector2 linePointA = vertices[Vertices[i]];
                Vector2 linePointB = vertices[Vertices[(i + 1) % Vertices.Length]];

                if (!point.IsToRightOfLine(linePointA, linePointB))
                {
                    return false;
                }
            }

            return true;
        }

        public Vector2 FindNearestPoint(Vector2[] vertices, Vector2 point)
        {
            Vector2 result = Center;
            float maxDistance = float.PositiveInfinity;

            for (int i = 0; i < Vertices.Length; i++)
            {
                Vector2 linePointA = vertices[Vertices[i]];
                Vector2 linePointB = vertices[Vertices[(i + 1) % Vertices.Length]];

                Vector2 pointOnLine = point.GetClosestPointOnLine(linePointA, linePointB, true);
                float distance = Vector2.DistanceSquared(point, pointOnLine);
                if (distance < maxDistance)
                {
                    result = pointOnLine;
                    maxDistance = distance;
                }
            }

            return result;
        }

        private PortalInt GetConnectingPortal(Polygon polygon)
        {
            IEnumerable<int> commonVertices = Vertices.Intersect(polygon.Vertices);
#if DEBUG
            // commonVertices is an IEnumerable with at least two elements
            // it's quaranteed by the process that's responsible for selecting neighbours
            if (commonVertices.Count() != 2)
            {
                throw new ApplicationException($"Invalid `commonVertices` length ({commonVertices.Count()}) for cells [{string.Join(", ", Vertices)}] and [{string.Join(", ", polygon.Vertices)}].");
            }
#endif

            // Portals have their indexes ordered clockwise, so in case 
            // polygons are connected with the index where the loop occures, the order needs to be reversed 
            if (Vertices[Vertices.Length - 1] == commonVertices.ElementAt(1) && Vertices[0] == commonVertices.ElementAt(0))
            {
                commonVertices = commonVertices.Reverse();
            }
            return new PortalInt(commonVertices.ElementAt(0), commonVertices.ElementAt(1));
        }
    }
}