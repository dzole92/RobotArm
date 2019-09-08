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
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		public double AngleBetweenVectors(Point a, Point b) {
			var multiple = a.X * b.X + a.Y * b.Y;
			var magnitudeA = Math.Sqrt(Math.Pow(a.X, 2) + Math.Pow(a.Y, 2));
			var magnitudeB = Math.Sqrt(Math.Pow(b.X, 2) + Math.Pow(b.Y, 2));
			var cosAlpha = multiple / (magnitudeA * magnitudeB);
			cosAlpha = cosAlpha > 1 ? 1 : cosAlpha;
			cosAlpha = cosAlpha < -1 ? -1 : cosAlpha;
			return Math.Acos(cosAlpha);
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

	}
}
