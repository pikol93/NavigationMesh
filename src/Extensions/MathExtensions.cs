using System;
using System.Numerics;

namespace Pikol93.NavigationMesh
{
    internal static class MathExtensions
    {
        public const double Tau = Math.PI * 2.0;
        public readonly static Vector2 Vector2Inf = new Vector2(float.PositiveInfinity);

        public static float PerpDotProduct(this Vector2 point, Vector2 lineA, Vector2 lineB) =>
            (point.X - lineA.X) * (lineB.Y - lineA.Y) - (point.Y - lineA.Y) * (lineB.X - lineA.X);

        public static bool IsToLeftOfLine(this Vector2 point, Vector2 lineA, Vector2 lineB) =>
            PerpDotProduct(point, lineA, lineB) > 0f;

        public static bool IsToRightOfLine(this Vector2 point, Vector2 lineA, Vector2 lineB) =>
            PerpDotProduct(point, lineA, lineB) < 0f;

        public static double GetClockwiseAngle(this Vector2 vector, Vector2 with)
        {
            double dot = Vector2.Dot(with, vector);
            double determinant = with.X * vector.Y - with.Y * vector.X;
            // Allowed return value is greater or equal to 0 and less than Tau
            return (Tau + Math.Atan2(determinant, dot)) % Tau;
        }

        public static double GetCounterClockwiseAngle(this Vector2 vector, Vector2 with) =>
            GetClockwiseAngle(with, vector);

        public static Vector2 GetClosestPointOnLine(this Vector2 point, Vector2 lineA, Vector2 lineB, bool limitToSegment = false)
        {
            // https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
            float a = point.X - lineA.X;
            float b = point.Y - lineA.Y;
            float c = lineB.X - lineA.X;
            float d = lineB.Y - lineA.Y;

            float dot = a * c + d * b;
            float lineLength = c * c + d * d;
            float fraction = dot / lineLength;

            if (limitToSegment)
            {
                if (fraction < 0f)
                {
                    return lineA;
                }
                else if (fraction > 1f)
                {
                    return lineB;
                }
            }

            return new Vector2(lineA.X + fraction * c, lineA.Y + fraction * d);
        }

        public static Vector2 GetPolygonCentroid(this Vector2[] polygon, bool clockwise = false)
        {
            float area = 0f;
            Vector2 result = Vector2.Zero;

            for (int i = 0; i < polygon.Length; i++)
            {
                int nextIndex = (i + 1) % polygon.Length;
                float secondFactor =
                    polygon[i].X * polygon[nextIndex].Y -
                    polygon[nextIndex].X * polygon[i].Y;

                area +=
                    (polygon[nextIndex].X - polygon[i].X) *
                    (polygon[nextIndex].Y + polygon[i].Y) / 2f;

                result.X += (polygon[i].X + polygon[nextIndex].X) * secondFactor;
                result.Y += (polygon[i].Y + polygon[nextIndex].Y) * secondFactor;
            }

            result /= 6 * area;

            return clockwise ? result : -result;
        }

        /// <summary>
        /// Casts a ray from lineA and intersects it with line specified with p3 and p4.
        /// The ray is casted in the direction of p1 -> p2 and 
        /// will return only results outside of the segment.
        /// </summary>
        /// <param name="p1">First point of LineA</param>
        /// <param name="p2">Second point of LineA</param>
        /// <param name="p3">Fist point of LineB</param>
        /// <param name="p4">Second point of LineB</param>
        /// <returns></returns>
        public static Vector2 CastRay(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4)
        {
            float dx12 = p2.X - p1.X;
            float dy12 = p2.Y - p1.Y;
            float dx34 = p4.X - p3.X;
            float dy34 = p4.Y - p3.Y;

            float denominator = dy12 * dx34 - dx12 * dy34;
            float t1 = ((p1.X - p3.X) * dy34 + (p3.Y - p1.Y) * dx34) / denominator;

            // Check if the lines are parallel
            if (float.IsInfinity(t1))
            {
                return Vector2Inf;
            }

            // Don't allow returning point that's on the first line
            if (t1 <= 1f)
            {
                return Vector2Inf;
            }

            // t2 is a fraction of second line on which the intersection occurs
            float t2 = ((p3.X - p1.X) * dy12 + (p1.Y - p3.Y) * dx12) / -denominator;
            if (t2 <= 0f || t2 >= 1f)
            {
                // Intersection is outside p3 -> p4
                return Vector2Inf;
            }

            return new Vector2(p3.X + dx34 * t2, p3.Y + dy34 * t2);
        }

        public static float GetIntersectionSecondLineFraction(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4)
        {
            float dx12 = p2.X - p1.X;
            float dy12 = p2.Y - p1.Y;
            float dx34 = p4.X - p3.X;
            float dy34 = p4.Y - p3.Y;

            float denominator = dy12 * dx34 - dx12 * dy34;

            // t2 is a fraction of second line on which the intersection occurs
            return ((p3.X - p1.X) * dy12 + (p1.Y - p3.Y) * dx12) / -denominator;
        }

        public static bool HasAmountOfCommonElements(this int[] arrayA, int[] arrayB, int requiredElements)
        {
            int count = 0;
            for (int i = 0; i < arrayA.Length; i++)
            {
                for (int j = 0; j < arrayB.Length; j++)
                {
                    if (arrayA[i] == arrayB[j])
                    {
                        count++;
                        if (count >= requiredElements)
                            return true;
                    }
                }
            }
            return false;
        }
    }
}