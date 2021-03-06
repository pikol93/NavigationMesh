using System.Numerics;
using PortalInt = System.Tuple<int, int>;

namespace Pikol93.NavigationMesh
{
    public class PolygonNeighbour
    {
        public Polygon Polygon { get; }
        public PortalInt Portal { get; }
        public Vector2 PortalCenter { get; }

        public PolygonNeighbour(Polygon polygon, PortalInt portal, Vector2 portalCenter)
        {
            this.Polygon = polygon;
            this.Portal = portal;
            this.PortalCenter = portalCenter;
        }

        /// <summary> Checks if the has the same indexes as the given one. </br>
        /// A portal is considered to be equal with another one 
        /// when their indexes match (reversed portals match too). </summary>
        public bool IsTheSamePortal(PortalInt second)
        {
            if (Portal.Item1 == second.Item1)
            {
                if (Portal.Item2 == second.Item2)
                    return true;
                return false;
            }

            if (Portal.Item1 == second.Item2)
            {
                if (Portal.Item2 == second.Item1)
                    return true;
            }

            return false;
        }
    }
}