using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using RobotArm.Interfaces;

namespace RobotArm.Helpers
{
    public class FuzzyHelper
    {

		private readonly Point zeroPoint = new Point() {X = 0, Y = 0, Z = 0};
		private const double TOLERANCE = 0.0000001;

		public async Task<IEnumerable<double[,]>> CalculateCoordinates(double l1, double l2, IEnumerable<double[,]> anglesGrid)
        {
            var xMatrix = anglesGrid.ToArray()[0];
            var yMatrix = anglesGrid.ToArray()[1];
            var sumMatrixTask = AdditionMatrices(xMatrix, yMatrix);

            var xTask = Task.Factory.StartNew(async () =>
            {
                Trace.WriteLine($"X Task Start. {DateTime.Now:HH:mm:ss ffff}");
               
                var cos1Task = CustomCos(l1, xMatrix);
                var cos2Task = CustomCos(l2, await sumMatrixTask);
                var cos1Result = await cos1Task;
                var cos2Result = await cos2Task;
                return await AdditionMatrices(cos1Result, cos2Result);
            }, TaskCreationOptions.AttachedToParent);

            var yTask = Task.Factory.StartNew(async () =>
            {
                Trace.WriteLine($"Y Task Start. {DateTime.Now:HH:mm:ss ffff}");
                
                var sin1Task = CustomSin(l1, xMatrix);
                var sin2Task = CustomSin(l2, await sumMatrixTask);
                var sin1Result = await sin1Task;
                var sin2Result = await sin2Task;
                return await AdditionMatrices(sin1Result, sin2Result);
            }, TaskCreationOptions.AttachedToParent);

            var tempX = await await xTask;
            var tempY = await await yTask;

            return new [] {tempX, tempY};
        }

        public async Task<double[,]> AdditionMatrices(double[,] xMatrix, double[,] yMatrix)
        {
            //await Task.Yield();
            Trace.WriteLine($"Addition Matrices Start. {DateTime.Now:HH:mm:ss ffff}");

            var rowsNum = xMatrix.GetLength(0);
            var colsNum = xMatrix.GetLength(1);
            var tempSumMatrix = new double[rowsNum, colsNum];
            for (int i = 0; i < rowsNum; i++)
            {
                for (int j = 0; j < colsNum; j++)
                {
                    tempSumMatrix[i, j] = xMatrix[i, j] + yMatrix[i, j];
                }
            }
            return tempSumMatrix;
        }

        private async Task<double[,]> CustomCos(double l, double[,] matrix)
        {
            //await Task.Yield();
            Trace.WriteLine($"Custom Cos Start. {DateTime.Now:HH:mm:ss ffff}");
            var rowsNum = matrix.GetLength(0);
            var colsNum = matrix.GetLength(1);
            var resultM = new double[rowsNum, colsNum];
            for (int i = 0; i < rowsNum; i++)
            {
                for (int j = 0; j < colsNum; j++)
                {
                    resultM[i, j] = (Math.Cos(matrix[i, j]) * l);
                }
            }
            return resultM;
        }

        private async Task<double[,]> CustomSin(double l, double[,] matrix)
        {
            //await Task.Yield();
            Trace.WriteLine($"Custom Sin Start. {DateTime.Now:HH:mm:ss ffff}");
            var rowsNum = matrix.GetLength(0);
            var colsNum = matrix.GetLength(1);
            var resultM = new double[rowsNum, colsNum];
            for (int i = 0; i < rowsNum; i++)
            {
                for (int j = 0; j < colsNum; j++)
                {
                    resultM[i, j] = (Math.Sin(matrix[i, j]) * l);
                }
            }
            return resultM;
        }

        public async Task<IEnumerable<double[,]>> MeshGrid(double[] theta1Vector, double[] theta2Vector)
        {
            try
            {
                Trace.WriteLine($"MeshGrid Start. {DateTime.Now:HH:mm:ss ffff}");
                
                var rowsNumber = theta2Vector.Length;
                var columnsNumber = theta1Vector.Length;
                var XMatrix = new double[rowsNumber, columnsNumber];
                var YMatrix = new double[rowsNumber, columnsNumber];
                var t = Task.Run(() =>
                {
                    for (int i = 0; i < rowsNumber; i++)
                    {
                        for (int j = 0; j < columnsNumber; j++)
                        {
                            XMatrix[i, j] = theta1Vector[j];
                            YMatrix[i, j] = theta2Vector[i];
                        }
                    }
                });
                await t;
                var result = new List<double[,]> { XMatrix, YMatrix };
                return result;
            }
            catch (Exception e)
            {
                Trace.WriteLine(e);
            }
            return null;
        }

        public async Task<double[]> CalculateVector(double minAngle, double maxnAgle, double step)
        {
            try
            {
                await Task.Yield();
                if (minAngle > maxnAgle) throw new ArgumentException("MinAgle is greater than MaxAgle");
                Trace.WriteLine($"Calculate vector for: 0:{step}:{maxnAgle}. {DateTime.UtcNow:HH:mm:ss ffff}");
                var vectorLength = (int)((maxnAgle-minAngle) / step);
                var vector = new double[vectorLength + 1];
                for (var i = 0; i <= vectorLength; i++)
                {
                    vector[i] = i * step;
                }
                return vector;
            }
            catch (Exception e)
            {
                Trace.WriteLine(e);
            }
            return null;

        }

		/// <summary>
		/// Angle between two crossing vectors.
		/// </summary>
		/// <param name="a">Vector a</param>
		/// <param name="b">Vector b</param>
		/// <returns></returns>
		public double AngleBetweenVectors(Point a, Point b) {
			var multiple = a.X * b.X + a.Y * b.Y;
			var magnitudeA = Math.Sqrt(Math.Pow(a.X, 2) + Math.Pow(a.Y, 2));
			var magnitudeB = Math.Sqrt(Math.Pow(b.X, 2) + Math.Pow(b.Y, 2));
			var cosAlpha = multiple / (magnitudeA * magnitudeB);
			cosAlpha = cosAlpha > 1 ? 1 : cosAlpha;
			cosAlpha = cosAlpha < -1 ? -1 : cosAlpha;
			var result = Math.Acos(cosAlpha);
			return cosAlpha < 0 ? Math.PI - result : result;
		}


		public Task<IEnumerable<MathErrorOutcome>> CalculateMathError(IEnumerable<IKinematicOutcome> mathematical, IEnumerable<IKinematicOutcome> anfis) {
			var expectedOutcomes = mathematical?.Where(x=> x != null).ToArray();
			var actualOutcomes = anfis?.Where(x => x != null).ToArray();
			if (expectedOutcomes == null || actualOutcomes == null || expectedOutcomes.Length != actualOutcomes.Length) throw new ArgumentException("Size of mathematical and anfis does not match.");
			return Task.Run(() => {
				return expectedOutcomes.Select((t, i) => CalculateMathError(t, actualOutcomes[i])).Where(x => x != null);
			});
		}

		public MathErrorOutcome CalculateMathError(IKinematicOutcome mathematical, IKinematicOutcome anfis) {
			if (double.IsNaN(mathematical.Theta1)
			|| double.IsNaN(mathematical.Theta2)
			|| double.IsNaN(anfis.Theta1)
			|| double.IsNaN(anfis.Theta2)) throw new Exception("Some angle is NaN");
			var result = new MathErrorOutcome {
				Theta1Error = (mathematical.Theta1 - anfis.Theta1), 
				Theta2Error = (mathematical.Theta2 - anfis.Theta2)
			};
			return result;
		}

		/// <summary>
		/// position = sign((Bx - Ax) * (Y - Ay) - (By - Ay) * (X - Ax))
		/// </summary>
		/// <param name="point"></param>
		/// <returns></returns>
		public Quadrant InWhichQuadrant(Point point) {
			var xVector = new Point {X = 100};
			var yVector = new Point {Y = 100};
			var positionByX = PositionSignByVector(zeroPoint, xVector, point);// Math.Sign((xVector.X - 0) * (point.Y - 0) - (xVector.Y - 0) * (point.X - 0));
			var positionByY = PositionSignByVector(zeroPoint, yVector, point); // Math.Sign((yVector.X - 0) * (point.Y - 0) - (yVector.Y - 0) * (point.X - 0));

			switch (positionByX) {
				case 0:
				case 1:
					return positionByY <= 0 ? Quadrant.I : Quadrant.II;
				case -1:
					return positionByY >= 0 ? Quadrant.III : Quadrant.IV;
				default: return Quadrant.None;
			}
		}

		/// <summary>
		/// position = sign((Bx - Ax) * (Y - Ay) - (By - Ay) * (X - Ax))
		/// </summary>
		/// <param name="jointPosition"></param>
		/// <param name="endPoint"></param>
		/// <returns></returns>
		public Quadrant InWhichQuadrant(Point jointPosition, Point endPoint) {
			var normalVectorPoint1 = jointPosition.Clone();
			var normalVectorPoint2 = CalculateNormalVectorPoint(normalVectorPoint1);
			var infinityFlag = false;
			if (double.IsInfinity(normalVectorPoint2.Y)) infinityFlag = true;


			var jointQuadrant = InWhichQuadrant(jointPosition);
			var pointX = 0;
			var pointXn = 0;

			switch (jointQuadrant) {
				case Quadrant.I:
					pointX = 50;
					pointXn = -50;
					break;
				case Quadrant.II:
					pointX = 50;
					pointXn = -50;
					break;
				case Quadrant.III:
					pointX = -50;
					pointXn = 50;
					break;
				case Quadrant.IV:
					pointX = -50;
					pointXn = 50;
					break;
			}

			var vectorXPoint1 = jointPosition;
			var vectorXPoint2 = CalculatePointFromEquation(pointX, zeroPoint, jointPosition);
			normalVectorPoint2 = infinityFlag ? new Point() {X= normalVectorPoint2.X, Y = jointPosition.X} : CalculatePointFromEquation(pointXn, normalVectorPoint1, normalVectorPoint2);
			if (Equals(vectorXPoint2, default(Point)) || Equals(normalVectorPoint2, default(Point))) return Quadrant.None;

			var positionByX = PositionSignByVector(vectorXPoint1, vectorXPoint2, endPoint);
			var positionByY = PositionSignByVector(normalVectorPoint1, normalVectorPoint2, endPoint);

			switch (positionByX) {
				case 0:
				case 1:
					return positionByY <= 0 ? Quadrant.I : Quadrant.II;
				case -1:
					return positionByY >= 0 ? Quadrant.III : Quadrant.IV;
				default: return Quadrant.None;
			}
		}

		private static Point CalculateNormalVectorPoint(Point normalVectorPoint1, bool increase = true) {
			return increase
				? new Point {
					X = normalVectorPoint1.X + 1, Y = (Math.Pow(normalVectorPoint1.Y, 2) - normalVectorPoint1.X) / normalVectorPoint1.Y
				}
				: new Point {
					X = normalVectorPoint1.X - 1, Y = (Math.Pow(normalVectorPoint1.Y, 2) + normalVectorPoint1.X) / normalVectorPoint1.Y
				};
		}

		private Point CalculatePointFromEquation(double x, Point pointFromLine1, Point pointFromLine2) {
			if (Math.Abs(pointFromLine2.X - pointFromLine1.X) < 0.000000001) return default(Point);
			var slope = (pointFromLine2.Y - pointFromLine1.Y) / (pointFromLine2.X - pointFromLine1.X);
			return new Point {X = x, Y = (-(slope * pointFromLine2.X) + slope * x) + pointFromLine2.Y};
		}

		private int PositionSignByVector(Point vectorPoint1, Point vectorPoint2, Point point) {
			var result = (vectorPoint2.X - vectorPoint1.X) * (point.Y - vectorPoint1.Y)
					- (vectorPoint2.Y - vectorPoint1.Y) * (point.X - vectorPoint1.X);
			result = Math.Abs(result) < TOLERANCE ? 0 : result;
			return Math.Sign(result);
		}

	}
}
