using PortalInt = System.Tuple<int, int>;

namespace Pikol93.NavigationMesh
{
    internal class PolygonNeighbour
    {
        internal Polygon Polygon { get; }
        internal PortalInt Portal { get; }
        internal float Distance { get; }

        public PolygonNeighbour(Polygon polygon, PortalInt portal)
        {
            this.Polygon = polygon;
            this.Portal = portal;
        }

        /// <summary> Checks if the has the same indexes as the given one. </br>
        /// A portal is considered to be equal with another one 
        /// when their indexes match (reversed portals match too). </summary>
        public bool IsTheSamePortal(PortalInt second)
        {
            if (Portal.Item1 == second.Item1 && Portal.Item2 == second.Item2)
                return true;
            if (Portal.Item1 == second.Item2 && Portal.Item2 == second.Item1)
                return true;

            return false;
        }
    }
}