using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RobotArm.Interfaces;

namespace RobotArm {
	public class KinematicOutcome: IKinematicOutcome {

		public double Theta1 { get; }
		public double Theta2 { get; }
		public Point JointPosition { get; }

		public KinematicOutcome(double theta1, double theta2, Point jointPosition) {
			Theta1 = theta1;
			Theta2 = theta2;
			JointPosition = jointPosition;
		}

	}
}
