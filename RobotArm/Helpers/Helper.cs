using System.Collections.Generic;

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
    }
}