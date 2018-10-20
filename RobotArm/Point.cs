using System;

namespace RobotArm
{
    public struct Point
    {
        public double X;
        public double Y;
        public double Z;

        public double DistanceFromOtherPoint(Point point)
        {
            return Math.Sqrt(Math.Pow(point.Y - Y, 2) + Math.Pow(point.X - X, 2) + Math.Pow(point.Z - Z, 2));
        }

    }
}