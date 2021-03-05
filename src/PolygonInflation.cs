using System;
using System.Collections.Generic;
using System.Numerics;
using ClipperLib;
using Path = System.Collections.Generic.List<ClipperLib.IntPoint>;

namespace Pikol93.NavigationMesh
{
    internal static class PolygonInflation
    {
        internal static Vector2[][] InflatePolygons(Vector2[] bounds, Vector2[][] shapes, double agentSize)
        {
            // Limit the passed shapes to bounds
            Clipper clipper = new Clipper(Clipper.ioStrictlySimple);
            clipper.AddPath(ToPath(bounds), PolyType.ptSubject, true);
            clipper.AddPaths(ToPathList(shapes), PolyType.ptClip, true);

            List<Path> solution = new List<Path>();
            if (!clipper.Execute(ClipType.ctDifference, solution, PolyFillType.pftNonZero))
                throw new ApplicationException("Could not limit passed shapes to bounds.");

            // Inflate the polygons with the given agentSize
            ClipperOffset clipperOffset = new ClipperOffset();
            clipperOffset.AddPath(solution[0], JoinType.jtMiter, EndType.etClosedLine);
            for (int i = 1; i < solution.Count; i++)
            {
                clipperOffset.AddPath(solution[i], JoinType.jtMiter, EndType.etClosedPolygon);
            }

            List<Path> offsetSolution = new List<Path>();
            clipperOffset.Execute(ref offsetSolution, agentSize);

            // The first item in a solution is a polygon facing outwards
            // I don't know if this will break but better be careful with this
            offsetSolution.RemoveAt(0);

            return ToArrayOfVector2Array(offsetSolution);
        }

        private static Vector2[] ToVector2Array(Path shape)
        {
            Vector2[] result = new Vector2[shape.Count];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = new Vector2(shape[i].X, shape[i].Y);
            }
            
            return result;
        }

        private static Vector2[][] ToArrayOfVector2Array(List<Path> shapes)
        {
            Vector2[][] result = new Vector2[shapes.Count][];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = ToVector2Array(shapes[i]);
            }

            return result;
        }

        private static Path ToPath(Vector2[] shape)
        {
            Path result = new Path(shape.Length);
            for (int i = 0; i < shape.Length; i++)
            {
                result.Add(new IntPoint(shape[i].X, shape[i].Y));
            }

            return result;
        }

        private static List<Path> ToPathList(Vector2[][] shapes)
        {
            List<Path> result = new List<Path>(shapes.Length);
            for (int i = 0; i < shapes.Length; i++)
            {
                result.Add(ToPath(shapes[i]));
            }

            return result;
        }
    }
}