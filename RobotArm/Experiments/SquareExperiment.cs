using System;
using System.Collections.Generic;
using System.Linq;
using RobotArm.Interfaces;

namespace RobotArm.Experiments {

	public class SquareExperiment : Experiment {

		public override IEnumerable<Point> GeneratePositions(double side, double step = 0.1745, double shiftX = 0, double shiftY = 0) {
			var result = new List<Point>();
			for(double i = -side; i <= side; i+=step) {
				var side1Point = new Point { X = i + shiftX, Y = side + shiftY };
				var side2Point = new Point { X = i + shiftX, Y = -side + shiftY };
				var side3Point = new Point { X = side + shiftX, Y = i + shiftY };
				var side4Point = new Point { X = -side + shiftX, Y = i + shiftY };

				result.AddRange(new[] { side1Point, side2Point, side3Point, side4Point });
			}

			ExperimentPositions = result;
			return result;
		}
	}
}