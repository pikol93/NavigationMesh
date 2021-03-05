using System.Collections.Generic;
using System.Numerics;
using PortalInt = System.Tuple<int, int>;
using Portal = System.Tuple<System.Numerics.Vector2, System.Numerics.Vector2>;

namespace Pikol93.NavigationMesh
{
    public class NavMesh
    {
        private Vector2[] Vertices { get; }
        private Polygon[] Polygons { get; }

        internal NavMesh(Vector2[] vertices, Polygon[] polygons)
        {
            Vertices = vertices;
            Polygons = polygons;
        }

        public List<Vector2> FindPath(Vector2 start, Vector2 end, bool limitToNavMesh = false)
        {
            Polygon startPolygon = FindNearestPolygon(start);
            Polygon endPolygon = FindNearestPolygon(end);

            if (startPolygon == endPolygon)
            {
                if (limitToNavMesh && !endPolygon.IsPointInside(Vertices, end))
                {
                    end = endPolygon.FindNearestPoint(Vertices, end);
                }
                return new List<Vector2>() { start, end };
            }

            List<PortalNode> open = new List<PortalNode>();
            List<PortalNode> closed = new List<PortalNode>();

            foreach (PolygonNeighbour neighbour in startPolygon.Neighbours)
            {
                Vector2 point = MathExtensions.GetClosestPointOnLine(start,
                    Vertices[neighbour.Portal.Item1], Vertices[neighbour.Portal.Item2], true);
                var node = new PortalNode(neighbour.Portal, neighbour.Polygon, (end - point).Length());
                node.Update(null, point, (point - start).Length());
                open.Add(node);
            }

            while (open.Count > 0)
            {
                // Find node with lowest cost
                PortalNode node = open[0];
                for (int i = 1; i < open.Count; i++)
                {
                    if (open[i].FScore < node.FScore)
                    {
                        node = open[i];
                    }
                }

                if (node.ToPolygon == endPolygon)
                {
                    // Found path
                    List<Portal> portals = RestructurePath(node);
                    if (limitToNavMesh && !endPolygon.IsPointInside(Vertices, end))
                    {
                        end = endPolygon.FindNearestPoint(Vertices, end);
                    }
                    return StringPulling(start, end, portals);
                }

                open.Remove(node);
                closed.Add(node);

                foreach (PolygonNeighbour newNeighbour in node.ToPolygon.Neighbours)
                {
                    // Check if the portal has been added to closed already
                    if (closed.Find(x => newNeighbour.IsTheSamePortal(x.Portal)) != null)
                    {
                        continue;
                    }

                    Vector2 point = MathExtensions.GetClosestPointOnLine(node.Point,
                        Vertices[newNeighbour.Portal.Item1], Vertices[newNeighbour.Portal.Item2], true);

                    // If the node already exists in the open set, then there might be a better route to it
                    PortalNode processedNode = open.Find(x => newNeighbour.IsTheSamePortal(x.Portal));
                    if (processedNode == null)
                    {
                        processedNode = new PortalNode(newNeighbour.Portal, newNeighbour.Polygon, (end - point).Length());
                        open.Add(processedNode);
                    }

                    float newGScore = node.GScore + (point - node.Point).Length();
                    if (newGScore < processedNode.GScore)
                    {
                        processedNode.Update(node, point, newGScore);
                    }
                }
            }

            // Console.WriteLine("Path not found.");
            return new List<Vector2>() { start };
        }

        private List<Portal> RestructurePath(PortalNode node)
        {
            List<PortalNode> nodes = new List<PortalNode>();
            while (node != null)
            {
                nodes.Add(node);
                node = node.Previous;
            }

            nodes.Reverse();
            List<Portal> output = new List<Portal>(nodes.Count);
            foreach (PortalNode iteration in nodes)
            {
                output.Add(new Portal(Vertices[iteration.Portal.Item1], Vertices[iteration.Portal.Item2]));
            }
            return output;
        }

        private Polygon FindPolygon(Vector2 point)
        {
            foreach (Polygon polygon in Polygons)
            {
                // If there's a point inside then don't iterate further
                if (polygon.IsPointInside(Vertices, point))
                    return polygon;
            }

            return null;
        }

        private Polygon FindNearestPolygon(Vector2 point)
        {
            Polygon findPolygonResult = FindPolygon(point);
            if (findPolygonResult != null)
                return findPolygonResult;

            // This is VERY slow, consider using it rarely
            Polygon result = null;
            float maxDistance = float.PositiveInfinity;

            foreach (Polygon polygon in Polygons)
            {
                Vector2 nearestPoint = polygon.FindNearestPoint(Vertices, point);
                float distance = Vector2.DistanceSquared(point, nearestPoint);

                if (distance < maxDistance)
                {
                    result = polygon;
                    maxDistance = distance;
                }
            }

            return result;
        }

        public static List<Vector2> StringPulling(Vector2 start, Vector2 end, List<Portal> portals)
        {
            // Portals are constructed so that Item1 of Tuple<Vector2, Vector2>
            // is the "left" edge of the portal, and Item2 is the "right" edge
            if (portals.Count <= 0)
            {
                // No portals to pull string through
                return new List<Vector2>() { start, end };
            }

            // path[path.Count - 1] is the apex point
            List<Vector2> path = new List<Vector2>() { start };
            int indexLeft = 0;
            int indexRight = 0;
            int currentIteration = 0;

            while (++currentIteration <= portals.Count && indexLeft < portals.Count && indexRight < portals.Count)
            {
                Vector2 portalLeft;
                Vector2 portalRight;

                if (currentIteration < portals.Count)
                {
                    // Use portals
                    portalLeft = portals[currentIteration].Item1;
                    portalRight = portals[currentIteration].Item2;
                }
                else // if (currentIteration == portals.Count)
                {
                    // Use end point
                    portalLeft = end;
                    portalRight = end;
                }

                if (portalLeft.PerpDotProduct(path[path.Count - 1], portals[indexLeft].Item1) < 0f)
                {
                    if (portalLeft.PerpDotProduct(path[path.Count - 1], portals[indexRight].Item2) <= 0f)
                    {
                        // Next left portal point falls out of funnel
                        path.Add(portals[indexRight].Item2);

                        currentIteration = indexRight;
                        indexLeft = currentIteration + 1;
                        indexRight = currentIteration + 1;
                        continue;
                    }
                    else
                    {
                        // Narrowing the funnel
                        // It's clamped here because the index can go out of bounds on last iteration
                        indexLeft = currentIteration < portals.Count ? currentIteration : portals.Count - 1;
                    }
                }

                if (portalRight.PerpDotProduct(path[path.Count - 1], portals[indexRight].Item2) > 0f)
                {
                    if (portalRight.PerpDotProduct(path[path.Count - 1], portals[indexLeft].Item1) >= 0f)
                    {
                        path.Add(portals[indexLeft].Item1);

                        currentIteration = indexLeft;
                        indexLeft = currentIteration + 1;
                        indexRight = currentIteration + 1;
                        continue;
                    }
                    else
                    {
                        // Narrowing the funnel
                        indexRight = currentIteration < portals.Count ? currentIteration : portals.Count - 1;
                    }
                }
            }

            path.Add(end);

            return path;
        }

        private class PortalNode
        {
            public PortalInt Portal { get; }
            public Polygon ToPolygon { get; }

            public float HScore { get; }
            public PortalNode Previous { get; private set; }
            public Vector2 Point { get; private set; }
            public float GScore { get; private set; }

            public float FScore { get => HScore + GScore; }

            public PortalNode(PortalInt portal, Polygon toPolygon, float hScore)
            {
                Portal = portal;
                ToPolygon = toPolygon;
                HScore = hScore;

                GScore = float.PositiveInfinity;
            }

            public void Update(PortalNode previous, Vector2 point, float newGScore)
            {
                this.Previous = previous;
                this.Point = point;
                this.GScore = newGScore;
            }
        }
    }
}