using System;
using System.Collections.Generic;
using System.Linq;
using RobotArm.Interfaces;

namespace RobotArm.Helpers
{
    public static class Helper
    {
        public static bool Between(this double num, double lower, double upper, bool inclusive = false)
        {
            return inclusive
                ? lower <= num && num <= upper
                : lower < num && num < upper;
        }

		public static IEnumerable<double> ConvertToSingleArray(this double[,] matrix) {
			var result = new List<double>();
			var rowsNum = matrix.GetLength(0);
			var colsNum = matrix.GetLength(1);
			for (int i = 0; i < rowsNum; i++) {
				for (int j = 0; j < colsNum; j++) {
					result.Add(matrix[i, j]);
				}
			}

			return result;
		}

		public static double[] CovertToANFISParameter(this Point point) {
			return new [] {point.X, point.Y};
		}
		public static double[][] ConvertToANFISParameter(this IEnumerable<Point> positions) {
			if (!positions.Any()) throw new ArgumentException("List is empty for positions");
			return positions.Select(x=> new [] {x.X, x.Y}).ToArray();
		}

		public static double[][] ConvertToANFISParameter(this double[,] angleMatrix) {
			if (angleMatrix == null || angleMatrix.GetLength(0)==0 ) throw new ArgumentException("List is empty for positions");
			return ConvertToSingleArray(angleMatrix).Select(x=> new [] {x}).ToArray();
		}
    }
}