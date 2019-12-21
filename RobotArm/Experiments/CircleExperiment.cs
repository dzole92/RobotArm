using System;
using System.Collections.Generic;
using RobotArm.Interfaces;

namespace RobotArm.Experiments {
	public class CircleExperiment : Experiment {

		public override IEnumerable<Point> GeneratePositions(double radius, double step = 0.1745, double shiftX = 0, double shiftY = 0) {
			double counter = 0.0;
			var result = new List<Point>();
			while(counter < Math.PI * 2) {
				var x = (radius * Math.Cos(counter)) + shiftX;
				var y = (radius * Math.Sin(counter)) + shiftY;
				result.Add(new Point() { X = x, Y = y });
				counter += step;
			}
			ExperimentPositions = result;
			return result;
		}
	}
}