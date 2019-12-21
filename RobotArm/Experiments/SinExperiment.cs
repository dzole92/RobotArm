using System;
using System.Collections.Generic;
using RobotArm.Interfaces;

namespace RobotArm.Experiments {
	public class SinExperiment : Experiment {

		public override IEnumerable<Point> GeneratePositions(double length, double step = 0.1745, double shiftX = 0, double shiftY = 0) {
			var result = new List<Point>();

			for(double i = 0; i <= length; i+= step) {
				result.Add(new Point {
					X = i+shiftX,
					Y = Math.Sin(i)+shiftY
				});
			}

			ExperimentPositions = result;
			return result;
		}
	}
}