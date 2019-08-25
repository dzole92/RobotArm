using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;

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

        public async Task<double[]> CalculateVector(double minAgle, double maxAgle, double step)
        {
            try
            {
                await Task.Yield();
                if (minAgle > maxAgle) throw new ArgumentException("MinAgle is greater than MaxAgle");
                Trace.WriteLine($"Calculate vector for: 0:{step}:{maxAgle}. {DateTime.UtcNow:HH:mm:ss ffff}");
                var vectorLength = (int)((maxAgle-minAgle) / step);
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

    }
}
