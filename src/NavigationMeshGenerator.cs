using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

using Portal = System.Tuple<int, int>;

namespace Pikol93.NavigationMesh
{
    internal class NavigationMeshGenerator
    {
        private List<Vector2> vertices = new List<Vector2>();
        private List<Point> points = new List<Point>();
        private List<Edge> edges = new List<Edge>();
        private List<Portal> portals = new List<Portal>();
        private List<int[]> cells = new List<int[]>();

        internal NavMesh GenerateNavMesh(IEnumerable<Vector2[]> shapes)
        {
            InitializeShapes(shapes);
            ProcessNotches();
            CreateCells();

            List<Polygon> polygons = new List<Polygon>(cells.Count);

            foreach (int[] cell in cells)
            {
#if DEBUG
                if (cell.Length < 3)
                {
                    Console.WriteLine($"Vertex amount for cell: {cell.Length}. Vertices: {String.Join(", ", cell)}");
                    throw new ApplicationException("Invalid vertex amount.");
                }
#endif

                // Find the center of the polygon
                Vector2[] polygonVertices = new Vector2[cell.Length];
                for (int i = 0; i < polygonVertices.Length; i++)
                {
                    polygonVertices[i] = vertices[cell[i]];
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
            {
                throw new ApplicationException("Not enough polygons created.");
            }

            Vector2[] verticesArray = vertices.ToArray();

            foreach (Polygon polygon in polygons)
            {
                // Find connected polygons
                List<Polygon> neighbours = new List<Polygon>();
                foreach (Polygon neighbour in polygons)
                {
                    if (polygon == neighbour)
                    {
                        continue;
                    }

                    if (polygon.Vertices.HasAmountOfCommonElements(neighbour.Vertices, 2))
                    {
                        neighbours.Add(neighbour);
                    }
                }

                polygon.SetNeighbours(verticesArray, neighbours);
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
                int shapeIndex = points.Count;
                for (int i = 0; i < shape.Length; i++)
                {
                    vertices.Add(shape[i]);

                    Point vertex = new Point(points.Count)
                    {
                        PreviousIndex = shapeIndex + (shape.Length + i - 1) % shape.Length,
                        NextIndex = shapeIndex + (i + 1) % shape.Length
                    };
                    points.Add(vertex);

                    edges.Add(new Edge(vertex.Index, vertex.NextIndex));
                }
            }
        }

        /// <summary>
        /// Processes all the notches, in effect filling up the Portals list
        /// </summary>
        private void ProcessNotches()
        {
            // The points list is modified during this loop, hence no foreach
            int maxIndex = points.Count;
            for (int i = 0; i < maxIndex; i++)
            {
                // TODO: Add convexity relaxation
                if (points[i].Processed || !IsVertexConcave(points[i]))
                {
                    continue;
                }

                ProcessNotch(points[i]);
            }

        }

        private void CreateCells()
        {
            // Assuming that there's at least one portal in the geometry
            foreach (Portal portal in portals)
            {
                // Create cells on both sides of the portal
                Point a = points[portal.Item1];
                Point b = points[portal.Item2];
                int[] cellA = GenerateCell(a, b);
                if (cellA != null)
                {
                    cells.Add(cellA);
                }

                int[] cellB = GenerateCell(b, a);
                if (cellB != null)
                {
                    cells.Add(cellB);
                }
            }

            if (cells.Count == 0)
            {
                // No pathfinding cells. Create only a single cell that spans across the whole bounds
                int[] cell = new int[points.Count];
                for (int i = 0; i < cell.Length; i++)
                {
                    cell[i] = i;
                }

                cells.Add(cell);
            }
        }

        private void ProcessNotch(Point notch)
        {
            // TODO: Clean this up, this is trash
            float minDistance = float.PositiveInfinity;
            object target = null;
            Vector2 position = Vector2.Zero;

            foreach (Point vertex in points)
            {
                if (vertex.Equals(notch))
                {
                    continue;
                }

                if (!IsPointInIA(notch, vertices[vertex.Index]))
                {
                    continue;
                }

                float distance = Vector2.DistanceSquared(vertices[notch.Index], vertices[vertex.Index]);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    target = vertex;
                }
            }

            foreach (Edge edge in edges)
            {
                // Don't process edges where the notch is a point in it
                if (edge.ContainsIndex(notch.Index))
                {
                    continue;
                }

                if (!IsEdgeInIA(notch, edge, out Vector2 result))
                {
                    continue;
                }

                float distance = Vector2.DistanceSquared(vertices[notch.Index], result);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    target = edge;
                    position = result;
                }
            }

            foreach (Portal portal in portals)
            {
                // A Portal can have an index of vertex that still needs to be processed
                // Refer to 3.2.1 Vertex-Vertex Portals for more information
                if (portal.Item1 == notch.Index || portal.Item2 == notch.Index)
                {
                    continue;
                }

                if (!IsPortalInIA(notch, portal, out Vector2 intersection))
                {
                    continue;
                }

                float distance = Vector2.DistanceSquared(vertices[notch.Index], intersection);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    target = portal;
                }
            }

            if (target is Point targetVertex)
            {
                // If both points are in both IAs then 
                // there's no need to process them twice
                if (IsPointInIA(targetVertex, vertices[notch.Index]))
                {
                    targetVertex.Processed = true;
                }

                // Add new portal connecting to the vertex
                Portal portal = new Portal(notch.Index, targetVertex.Index);
                portals.Add(portal);
            }
            else if (target is Edge targetEdge)
            {
                // A new vertex representing the point on the edge is created
                int edgeVertexIndex = points.Count;
                vertices.Add(position);
                Point edgeVertex = new Point(edgeVertexIndex)
                {
                    PreviousIndex = targetEdge.Item1,
                    NextIndex = targetEdge.Item2
                };
                points.Add(edgeVertex);

                // Update the vertexIndexes on previous and next vertices and on edges
                points[targetEdge.Item1].NextIndex = edgeVertexIndex;
                points[targetEdge.Item2].PreviousIndex = edgeVertexIndex;

                // Create new edge and add it to the list
                Edge newEdge = new Edge(edgeVertexIndex, targetEdge.Item2);
                edges.Add(newEdge);

                targetEdge.Item2 = edgeVertexIndex;

                // Add new portal connecting to the point on the edge
                Portal portal = new Portal(notch.Index, edgeVertexIndex);
                portals.Add(portal);
            }
            else if (target is Portal targetPortal)
            {
                if (!IsPointInIA(notch, vertices[targetPortal.Item2]))
                {
                    // Create new portal with first vertex
                    Portal newPortal = new Portal(notch.Index, targetPortal.Item1);
                    portals.Add(newPortal);
                }

                if (!IsPointInIA(notch, vertices[targetPortal.Item1]))
                {
                    // Create portal with second vertex
                    Portal newPortal = new Portal(notch.Index, targetPortal.Item2);
                    portals.Add(newPortal);
                }
            }
#if DEBUG
            else
            {
                // This happens when no object can be found within notch's IA
                throw new ApplicationException("Invalid shape.");
            }
#endif
        }

        private bool IsPointInIA(Point iaPoint, Vector2 point)
        {
            Vector2 vertex = vertices[iaPoint.Index];

            // Instead of doing IsToLeftOfLine(a, b), I have to do !IsToRightOfLine(a, b)
            // because sometimes the points are placed exactly on the line in question
            return !point.IsToRightOfLine(vertices[iaPoint.PreviousIndex], vertex) &&
                !point.IsToLeftOfLine(vertices[iaPoint.NextIndex], vertex);
        }

        private bool IsEdgeInIA(Point iaPoint, Edge edge, out Vector2 result)
        {
            Vector2 pointA = vertices[edge.Item1];
            Vector2 pointB = vertices[edge.Item2];

            // Check if candidate q is within IA
            Vector2 projectedPoint = vertices[iaPoint.Index].GetClosestPointOnLine(pointA, pointB, true);

            // Damn, float precision errors make this algorithm slower than it can be
            if (IsPointInIA(iaPoint, projectedPoint))
            {
                result = projectedPoint;
                return true;
            }

            // Intersect lines to check if edge falls onto IA at all
            float minDistance = float.PositiveInfinity;
            result = MathExtensions.Vector2Inf;
            Vector2[] vertexPositions = new Vector2[]
            {
                vertices[iaPoint.PreviousIndex],
                vertices[iaPoint.NextIndex]
            };

            foreach (Vector2 item in vertexPositions)
            {
                // Can't use IsPointInIA here due to float precision errors
                if (MathExtensions.CastRay(item, vertices[iaPoint.Index], pointA, pointB, out Vector2 intersection))
                {
                    float distance = Vector2.DistanceSquared(vertices[iaPoint.Index], intersection);
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

        private bool IsPortalInIA(Point iaPoint, Portal portal, out Vector2 intersection)
        {
            Vector2 vertex = vertices[iaPoint.Index];
            Vector2 portalPoint1 = vertices[portal.Item1];
            Vector2 portalPoint2 = vertices[portal.Item2];

            intersection = MathExtensions.Vector2Inf;

            // Find if portal lies within IA
            float distanceA = float.PositiveInfinity;
            if (MathExtensions.CastRay(vertices[iaPoint.PreviousIndex], vertex, portalPoint1, portalPoint2, out Vector2 intersectionA))
            {
                distanceA = Vector2.DistanceSquared(vertex, intersectionA);
                intersection = intersectionA;
            }
            if (MathExtensions.CastRay(vertices[iaPoint.NextIndex], vertex, portalPoint1, portalPoint2, out Vector2 intersectionB))
            {
                if (Vector2.DistanceSquared(vertex, intersectionB) < distanceA)
                {
                    intersection = intersectionB;
                }
            }

            return intersection != MathExtensions.Vector2Inf;
        }

        private bool IsVertexConcave(Point point) =>
            vertices[point.NextIndex].IsToRightOfLine(vertices[point.PreviousIndex], vertices[point.Index]);

        private int[] GenerateCell(Point first, Point second)
        {
            List<Point> path = new List<Point>() { first, second };

            while (true)
            {
                if (path.Count == 3)
                {
                    // No two cells should have the same 3 indexes in them
                    // There probably is a better place to put this code...
                    foreach (int[] item in cells)
                    {
                        if (item.Contains(path[0].Index) && item.Contains(path[1].Index) && item.Contains(path[2].Index))
                        {
                            return null;
                        }
                    }
                }

                // Find connected vertices to the last point of path
                IEnumerable<Point> connectedPoints = GetConnectedVertices(path[path.Count - 1]);

                Point bestCandidate = null;
                foreach (Point point in connectedPoints)
                {
                    // Check if path is found
                    if (point == first && point != path[path.Count - 2])
                    {
                        return VertexPathToCell(path);
                    }

                    // Check if point has already been processed
                    // It's iterated backwards because most of the time
                    // we're dealing with the index of recently processed vertex
                    bool skip = false;
                    for (int i = path.Count - 1; i >= 0; i--)
                    {
                        if (path[i] == point)
                        {
                            skip = true;
                            break;
                        }
                    }
                    if (skip)
                    {
                        continue;
                    }

                    Vector2 lastIteratedVertex = vertices[path[path.Count - 2].Index];
                    Vector2 vertex = vertices[point.Index];
                    // Don't ever accept points that could result in a concave shape
                    if (vertex.IsToLeftOfLine(lastIteratedVertex, vertices[path[path.Count - 1].Index]))
                    {
                        continue;
                    }

                    if (bestCandidate == null || vertex.IsToRightOfLine(lastIteratedVertex, vertices[bestCandidate.Index]))
                    {
                        // New best candidate
                        bestCandidate = point;
                    }
                }

                if (bestCandidate == null)
                {
                    break;
                }

                path.Add(bestCandidate);
            }

            // Something went wrong
            return null;
        }

        private IEnumerable<Point> GetConnectedVertices(Point vertex)
        {
            int index = vertex.Index;

            // Get edge connected vertices
            yield return points[vertex.PreviousIndex];
            yield return points[vertex.NextIndex];

            // Get portal connected vertices
            foreach (Portal portal in portals)
            {
                if (portal.Item1 == index)
                {
                    yield return points[portal.Item2];
                    continue;
                }

                if (portal.Item2 == index)
                {
                    yield return points[portal.Item1];
                    continue;
                }
            }
        }

        private int[] VertexPathToCell(List<Point> path)
        {
            int[] cell = new int[path.Count];
            for (int i = 0; i < cell.Length; i++)
            {
                cell[i] = path[i].Index;
            }

            return cell;
        }

        private class Point
        {
            public int Index { get; }
            public int PreviousIndex { get; set; }
            public int NextIndex { get; set; }
            public bool Processed { get; set; }

            public Point(int index)
            {
                Index = index;
            }
        }

        private class Edge
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
    }
}