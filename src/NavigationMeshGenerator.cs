using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace Pikol93.NavigationMesh
{
    internal class NavigationMeshGenerator
    {
        private List<Vertex> Vertices { get; }
        private List<IIntTuple> Edges { get; }
        private List<IIntTuple> Portals { get; }
        private List<int[]> Cells { get; }

        internal NavigationMeshGenerator(IEnumerable<Vector2[]> shapes)
        {
            Vertices = new List<Vertex>();
            Edges = new List<IIntTuple>();
            Portals = new List<IIntTuple>();
            Cells = new List<int[]>();

            InitializeShapes(shapes);
            CreatePortals();
            CreateCells();
        }

        internal NavMesh GenerateNavMesh()
        {
            List<Polygon> polygons = new List<Polygon>(Cells.Count);

            foreach (int[] cell in Cells)
            {
                // Find the center of the polygon
                Vector2[] polygonVertices = new Vector2[cell.Length];
                for (int i = 0; i < polygonVertices.Length; i++)
                {
                    polygonVertices[i] = Vertices[cell[i]].Position;
                }
                Vector2 polygonCenter = polygonVertices.GetPolygonCentroid();

                // Find furthest distance from center to vertices in a polygon
                float maxDistance = 0f;
                foreach (Vector2 vertex in polygonVertices)
                {
                    float distance = Vector2.DistanceSquared(polygonCenter, vertex);
                    if (distance > maxDistance)
                    {
                        maxDistance = distance;
                    }
                }

                Polygon polygon = new Polygon(cell, polygonCenter, maxDistance);
                polygons.Add(polygon);
            }

            if (polygons.Count < 1)
                throw new ApplicationException("Not enough polygons created.");

            Vector2[] vertices = new Vector2[Vertices.Count];
            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i] = Vertices[i].Position;
            }

            foreach (Polygon polygon in polygons)
            {
                // Find connected polygons
                List<Polygon> neighbours = new List<Polygon>();
                foreach (Polygon neighbour in polygons)
                {
                    if (polygon == neighbour)
                        continue;

                    if (polygon.Vertices.HasAmountOfCommonElements(neighbour.Vertices, 2))
                    {
                        neighbours.Add(neighbour);
                    }
                }

                polygon.SetNeighbours(vertices, neighbours);
            }

            Vector2[] verticesArray = new Vector2[Vertices.Count];
            for (int i = 0; i < verticesArray.Length; i++)
            {
                verticesArray[i] = Vertices[i].Position;
            }

            return new NavMesh(verticesArray, polygons.ToArray());
        }

        /// <summary>
        /// Fills Vertices and Edges with given data for Portal and Cell generation.
        /// </summary>
        /// <param name="shapes">Shapes (convex, and concave) to create the NavMesh with.</param>
        private void InitializeShapes(IEnumerable<Vector2[]> shapes)
        {
            // Fill Vertices with data from shapes
            foreach (Vector2[] shape in shapes)
            {
                int shapeIndex = Vertices.Count;
                for (int i = 0; i < shape.Length; i++)
                {
                    Vertex vertex = new Vertex(shape[i], Vertices.Count)
                    {
                        PreviousIndex = shapeIndex + (shape.Length + i - 1) % shape.Length,
                        NextIndex = shapeIndex + (i + 1) % shape.Length
                    };
                    Vertices.Add(vertex);

                    Edges.Add(new Edge(vertex.Index, vertex.NextIndex));
                }
            }
        }

        private void CreatePortals()
        {
            Queue<int> notches = new Queue<int>();
            foreach (Vertex vertex in Vertices)
            {
                // TODO: Add convexity relaxation
                if (IsVertexConcave(vertex))
                    notches.Enqueue(vertex.Index);
            }

            while (notches.Count > 0)
            {
                Vertex notch = Vertices[notches.Dequeue()];
                if (notch.Processed)
                    continue;

                ProcessNotch(notch);
            }
            notches.Clear();
        }

        private void CreateCells()
        {
            // Assuming that there's at least one portal in the geometry
            foreach (Portal portal in Portals)
            {
                // Create cells on both sides of the portal
                Vertex a = Vertices[portal.Item1];
                Vertex b = Vertices[portal.Item2];
                AddToCells(GenerateCell(a, b));
                AddToCells(GenerateCell(b, a));
            }

            if (Cells.Count == 0)
                throw new ApplicationException("No pathfinding cells could be found.");
        }

        private void ProcessNotch(Vertex notch)
        {
            float minDistance = float.PositiveInfinity;
            object target = null;
            Vector2 position = Vector2.Zero;

            foreach (Vertex vertex in Vertices)
            {
                if (vertex.Equals(notch))
                    continue;

                if (!IsPointInIA(notch, vertex.Position))
                    continue;

                float distance = notch.GetDistanceSquared(vertex.Position);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    target = vertex;
                }
            }

            foreach (Edge edge in Edges)
            {
                // Don't process edges where the notch is a point in it
                if (edge.ContainsIndex(notch.Index))
                    continue;

                if (!IsEdgeInIA(notch, edge, out Vector2 result))
                    continue;

                float distance = notch.GetDistanceSquared(result);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    target = edge;
                    position = result;
                }
            }

            foreach (Portal portal in Portals)
            {
                // A Portal can have an index of vertex that still needs to be processed
                // Refer to 3.2.1 Vertex-Vertex Portals for more information
                if (portal.ContainsIndex(notch.Index))
                    continue;

                if (!IsPortalInIA(notch, portal, out Vector2 intersection))
                    continue;

                float distance = notch.GetDistanceSquared(intersection);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    target = portal;
                }
            }

            if (target is Vertex targetVertex)
            {
                // If both points are in both IAs then 
                // there's no need to process them twice
                if (IsPointInIA(targetVertex, notch.Position))
                    targetVertex.Processed = true;

                // Add new portal connecting to the vertex
                Portal portal = new Portal(notch.Index, targetVertex.Index);
                Portals.Add(portal);
            }
            else if (target is Edge targetEdge)
            {
                // A new vertex representing the point on the edge is created
                int edgeVertexIndex = Vertices.Count;
                Vertex edgeVertex = new Vertex(position, edgeVertexIndex)
                {
                    PreviousIndex = targetEdge.Item1,
                    NextIndex = targetEdge.Item2
                };
                Vertices.Add(edgeVertex);

                // Update the vertexIndexes on previous and next vertices and on edges
                Vertices[targetEdge.Item1].NextIndex = edgeVertexIndex;
                Vertices[targetEdge.Item2].PreviousIndex = edgeVertexIndex;

                // Create new edge and add it to the list
                Edge newEdge = new Edge(edgeVertexIndex, targetEdge.Item2);
                Edges.Add(newEdge);

                targetEdge.Item2 = edgeVertexIndex;

                // Add new portal connecting to the point on the edge
                Portal portal = new Portal(notch.Index, edgeVertexIndex);
                Portals.Add(portal);
            }
            else if (target is Portal targetPortal)
            {
                if (!IsPointInIA(notch, Vertices[targetPortal.Item2].Position))
                {
                    // Create new portal with first vertex
                    Portal newPortal = new Portal(notch.Index, targetPortal.Item1);
                    Portals.Add(newPortal);
                }
                if (!IsPointInIA(notch, Vertices[targetPortal.Item1].Position))
                {
                    // Create portal with second vertex
                    Portal newPortal = new Portal(notch.Index, targetPortal.Item2);
                    Portals.Add(newPortal);
                }
            }
            else
            {
                // This happens when no object can be found within notch's IA
                // Usually this means that shape is out of bounds
                throw new ApplicationException("Invalid shape.");
            }
        }

        private bool IsPointInIA(Vertex iaVertex, Vector2 point)
        {
            Vector2 prev = Vertices[iaVertex.PreviousIndex].Position;
            Vector2 next = Vertices[iaVertex.NextIndex].Position;
            return point.IsToLeftOfLine(prev, iaVertex.Position) &&
                point.IsToRightOfLine(next, iaVertex.Position);

            /* The polygons are quaranteed to be clockwise due to polygon inflation
            bool clockwise = next.IsToRightOfLine(prev, iaVertex.Position);

            if (clockwise)
            {
                return point.IsToLeftOfLine(prev, iaVertex.Position) &&
                    point.IsToRightOfLine(next, iaVertex.Position);
            }
            return point.IsToRightOfLine(prev, iaVertex.Position) &&
                point.IsToLeftOfLine(next, iaVertex.Position);
            */
        }

        private bool IsEdgeInIA(Vertex iaVertex, Edge edge, out Vector2 result)
        {
            Vector2 pointA = Vertices[edge.Item1].Position;
            Vector2 pointB = Vertices[edge.Item2].Position;

            // Check if candidate q is within IA
            Vector2 projectedPoint = iaVertex.Position.GetClosestPointOnLine(pointA, pointB, true);

            // Damn, float precision errors make this algorithm slower than it can be
            if (IsPointInIA(iaVertex, projectedPoint))
            {
                result = projectedPoint;
                return true;
            }

            // Intersect lines to check if edge falls onto IA at all
            float minDistance = float.PositiveInfinity;
            result = MathExtensions.Vector2Inf;
            Vector2[] vertexPositions = new Vector2[]
            {
                Vertices[iaVertex.PreviousIndex].Position,
                Vertices[iaVertex.NextIndex].Position
            };

            foreach (Vector2 item in vertexPositions)
            {
                // Can't use IsPointInIA here due to float precision errors
                Vector2 intersection = MathExtensions.CastRay(item, iaVertex.Position, pointA, pointB);
                if (intersection != MathExtensions.Vector2Inf)
                {
                    float distance = iaVertex.GetDistanceSquared(intersection);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        result = intersection;
                    }
                }
            }

            // If result has been found then it's not equal to MathExtensions.Vector2Inf
            return result != MathExtensions.Vector2Inf;
        }

        private bool IsPortalInIA(Vertex iaVertex, Portal portal, out Vector2 intersection)
        {
            Vector2 prev = Vertices[iaVertex.PreviousIndex].Position;
            Vector2 next = Vertices[iaVertex.NextIndex].Position;
            Vector2 portalPoint1 = Vertices[portal.Item1].Position;
            Vector2 portalPoint2 = Vertices[portal.Item2].Position;

            // Find if portal lies within IA
            Vector2 intersectionA = MathExtensions.CastRay(prev, iaVertex.Position, portalPoint1, portalPoint2);
            Vector2 intersectionB = MathExtensions.CastRay(next, iaVertex.Position, portalPoint1, portalPoint2);

            if (intersectionA != MathExtensions.Vector2Inf || intersectionB != MathExtensions.Vector2Inf)
            {
                float distanceToIntersectionA = iaVertex.GetDistanceSquared(intersectionA);
                float distanceToIntersectionB = iaVertex.GetDistanceSquared(intersectionB);
                intersection = distanceToIntersectionA < distanceToIntersectionB ?
                    intersectionA : intersectionB;

                return true;
            }

            intersection = MathExtensions.Vector2Inf;
            return false;
        }

        private bool IsVertexConcave(Vertex vertex) =>
            Vertices[vertex.NextIndex].Position.IsToRightOfLine(Vertices[vertex.PreviousIndex].Position, vertex.Position);

        private int[] GenerateCell(Vertex first, Vertex second)
        {
            // Get vertices in a cell
            List<Vertex> output = new List<Vertex>() { first, second };
            GetCellRecursive(output);

            // Convert the vertices to indexes
            int[] cell = new int[output.Count];
            for (int i = 0; i < cell.Length; i++)
            {
                cell[i] = output[i].Index;
            }

            return cell;
        }

        private void GetCellRecursive(List<Vertex> vertices)
        {
            // TODO: This is confusing. Consider remaking this into a while loop
            Vertex startVertex = vertices[0];
            Vertex lastIterationVertex = vertices[vertices.Count - 2];
            Vertex currentIterationVertex = vertices[vertices.Count - 1];

            // Find connected vertices
            List<int> connectedVertices = new List<int>()
                { currentIterationVertex.PreviousIndex, currentIterationVertex.NextIndex };

            foreach (Portal portal in Portals)
            {
                if (!portal.ContainsIndex(currentIterationVertex.Index))
                    continue;

                int target = currentIterationVertex.Index == portal.Item1 ? portal.Item2 : portal.Item1;
                if (target == lastIterationVertex.Index)
                    continue;

                connectedVertices.Add(target);
            }

            // Remove vertices that already have been processed
            for (int i = connectedVertices.Count - 1; i >= 0; i--)
            {
                if (connectedVertices[i] == startVertex.Index)
                    return;

                foreach (Vertex vertex in vertices)
                {
                    if (connectedVertices[i] == vertex.Index)
                    {
                        connectedVertices.RemoveAt(i);
                        break;
                    }
                }
            }

            foreach (int index in connectedVertices)
            {
                if (index == startVertex.Index)
                    return;
            }

            // Call GetCell recursively on the best candidate
            Vertex bestCandidate = null;
            double candidateAngle = float.PositiveInfinity;

            foreach (int candidateIndex in connectedVertices)
            {
                Vertex candidate = Vertices[candidateIndex];
                Vector2 a = lastIterationVertex.Position - currentIterationVertex.Position;
                Vector2 b = candidate.Position - currentIterationVertex.Position;
                double angle = a.GetCounterClockwiseAngle(b);

                if (angle < candidateAngle)
                {
                    bestCandidate = candidate;
                    candidateAngle = angle;
                }
            }

            if (bestCandidate != null)
            {
                vertices.Add(bestCandidate);
                GetCellRecursive(vertices);
            }
        }

        private void AddToCells(int[] cell)
        {
            foreach (int[] item in Cells)
            {
                if (cell.Length != item.Length)
                    continue;

                // No two cells should have the same 3 indexes in them
                if (item.Contains(cell[0]) && item.Contains(cell[1]) && item.Contains(cell[2]))
                    return;
            }

            Cells.Add(cell);
        }

        private class Vertex
        {
            public Vector2 Position { get; }
            public int Index { get; }
            public int PreviousIndex { get; set; }
            public int NextIndex { get; set; }
            public bool Processed { get; set; }

            public Vertex(Vector2 position, int index)
            {
                Position = position;
                Index = index;
            }

            public float GetDistanceSquared(Vector2 target) =>
                Vector2.Distance(Position, target);
        }

        private class Edge : IIntTuple
        {
            // These properties have to be mutable
            // so I can't use Tuple<int, int> here
            public int Item1 { get; set; }
            public int Item2 { get; set; }

            public Edge(int item1, int item2)
            {
                Item1 = item1;
                Item2 = item2;
            }

            public bool ContainsIndex(int index) =>
                Item1 == index || Item2 == index;
        }

        private class Portal : Tuple<int, int>, IIntTuple
        {
            public Portal(int item1, int item2)
                : base(item1, item2) { }

            public bool ContainsIndex(int index) =>
                Item1 == index || Item2 == index;
        }

        private interface IIntTuple
        {
            int Item1 { get; }
            int Item2 { get; }

            bool ContainsIndex(int index);
        }
    }
}