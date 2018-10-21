using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotArm
{
    public class RobotArm
    {
        private double[] x1d;
        private double[] y1d;
        public double L1 { get; }

        public double L2 { get; }
        public double Theta2Min { get; }

        public double Theta1Min { get; }

        public double Theta1Max { get; }

        public double Theta2Max { get; }

        public double[] Theta1Vector { get; private set; }
        public double[] Theta2Vector { get; private set; }
        private IEnumerable<double[,]> AnglesGrid { get; set; }

        private double[,] Y2D { get; set; }

        private double[,] X2D { get; set; }

        private readonly FuzzyHelper _fuzzyHelper;

        public double[] X
        {
            get
            {
                if (x1d != null) return x1d;
                if (X2D == null || X2D.Length < 2) return null;
                var rowsNum = X2D.GetLength(0);
                var colsNum = X2D.GetLength(1);
                var tempArray = new List<double>();
                for (int i = 0; i < rowsNum; i++)
                {
                    for (int j = 0; j < colsNum; j++)
                    {
                        tempArray.Add(X2D[i, j]);
                    }
                }
                x1d = tempArray.ToArray();
                return x1d;
            }
        }

        public double[] Y
        {
            get
            {
                if (y1d != null) return y1d;
                if (Y2D == null || Y2D.Length < 2) return null;
                var rowsNum = Y2D.GetLength(0);
                var colsNum = Y2D.GetLength(1);
                var tempArray = new List<double>();
                for (int i = 0; i < rowsNum; i++)
                {
                    for (int j = 0; j < colsNum; j++)
                    {
                        tempArray.Add(Y2D[i, j]);
                    }
                }
                y1d = tempArray.ToArray();
                return y1d;
            }
        }

        public IEnumerable<Point> Positions { get; private set; }

        public RobotArm(double l1, double l2, double theta1Min, double theta1Max, double theta2Min, double theta2Max)
        {
            L1 = l1;
            L2 = l2;
            Theta1Max = theta1Max;
            Theta2Max = theta2Max;
            Theta1Min = theta1Min;
            Theta2Min = theta2Min;
            _fuzzyHelper = new FuzzyHelper();
        }
        public RobotArm(double l1, double l2, double theta1Min, double theta1Max, double theta2Min, double theta2Max, FuzzyHelper fuzzyHelper)
        {
            L1 = l1;
            L2 = l2;
            Theta1Max = theta1Max;
            Theta2Max = theta2Max;
            Theta1Min = theta1Min;
            Theta2Min = theta2Min;
            _fuzzyHelper = fuzzyHelper;
        }
        

        public async Task<bool> CalculateWholeDataSet(double agleStep)
        {
            try
            {
                var calculateVector1Task = _fuzzyHelper.CalculateVector(Theta1Min, Theta1Max, agleStep);
                var calculateVector2Task = _fuzzyHelper.CalculateVector(Theta2Min, Theta2Max, agleStep);

                Theta1Vector = await calculateVector1Task;
                Theta2Vector = await calculateVector2Task;

                if (Theta1Vector == null || !Theta1Vector.Any()) throw new Exception("Theta1Max Vector is empty.");
                if (Theta2Vector == null || !Theta2Vector.Any()) throw new Exception("Theta2Max Vector is empty.");

                AnglesGrid = await _fuzzyHelper.MeshGrid(Theta1Vector, Theta2Vector);
                if (AnglesGrid == null) throw new Exception("MeshGrid function not working");

                var coordinates = await _fuzzyHelper.CalculateCoordinates(L1, L2, AnglesGrid);
                X2D = coordinates.ToArray()[0];
                Y2D = coordinates.ToArray()[1];

                var t1 = Task.Run(() => X.ToList());
                var t2 = Task.Run(() => Y.ToList());

                Task.WaitAll(t1, t2);
                Positions = X.Select((t, i) => new Point{ X = t,  Y = Y[i], Z = 0 }).ToList();

                return true;
            }
            catch (Exception e)
            {
                Trace.WriteLine(e);
            }
            return false;
        }


        public async Task<IEnumerable<Point>> CalculateArmJoint(Point endPoint)
        {
            /* 
            Formulas for calculation:
                y1 =  (A + D)/B 
                y2 = (A - D)/B
                x1 = +-Sqrt(L1^2 - y1^2)
                x2 = +-Sqrt(L1^2 - y2^2)
                -----------------------------------------------------------------
                A = 2*c*y;
                B = 2*(y^2 + x^2);
                D = Sqrt(m - n*o);
                c = (L1^2 - L2^2 +x^2 +y^2)/2.0;
                m = 4*c^2*y^2
                n = 4*(y^2+x^2);
                o = (c^2-(x^2*L1^2));
           */
            await Task.Yield();
            var x = endPoint.X;
            var y = endPoint.Y;
            var c = (Math.Pow(L1, 2) - Math.Pow(L2, 2) + Math.Pow(x, 2) + Math.Pow(y, 2)) / 2.0;
            var m = 4 * (Math.Pow(c, 2) * Math.Pow(y, 2));
            var n = 4 * (Math.Pow(y, 2) + Math.Pow(x, 2));
            var o = Math.Pow(c, 2) - x * x * Math.Pow(L1, 2);
            var A = 2.0 * c * y;
            var B = 2 * (Math.Pow(y, 2) + Math.Pow(x, 2));
            var d = m - n * o;
            // *** Error allowed of 10^-7 if d is negative
            if (d < 0) {
                if (d * Math.Pow(10, 7) < 0) d = 0;
                else throw new Exception($"Cannot calculate joint point for this end point: x: {x} y: {y}");
            }
            d = Math.Sqrt(d);
            var y1 = (A + d) / B;
            var y2 = (A - d) / B;
            var x1 = Math.Sqrt(Math.Pow(L1, 2) - Math.Pow(y1, 2));
            var x2 = Math.Sqrt(Math.Pow(L1, 2) - Math.Pow(y2, 2));

            return new List<Point>
                {
                    new Point { X = x1, Y = y1, Z = 0 }, new Point { X = x2, Y = y2, Z = 0 },
                    new Point { X = -x1, Y = y1, Z = 0 }, new Point { X = -x2, Y = y2, Z = 0 }
                }
                    .Distinct().Where(point=> ValidJointPointWithEndAndZeroPoint(point, endPoint)).ToList();
        }

        private bool ValidJointPointWithEndAndZeroPoint(Point jointPoint, Point endPoint)
        {
            var zeroPoint = new Point {X = 0, Y = 0, Z = 0};
            var distanceFromZeroPoint = jointPoint.DistanceFromOtherPoint(zeroPoint);
            var distanceFromEndPoint = jointPoint.DistanceFromOtherPoint(endPoint);
            return !(Math.Abs(distanceFromZeroPoint - L1) > 0.0000001) && !(Math.Abs(distanceFromEndPoint - L2) > 0.0000001);
        }
    }
}
