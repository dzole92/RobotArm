using System;

namespace RobotArm.Interfaces
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

		public override string ToString() { return $"({X}, {Y}, {Z})"; }

		public Point Clone() {
			return new Point {
				X = this.X,
				Y = this.Y,
				Z = this.Z,
			};
		}

	}
}