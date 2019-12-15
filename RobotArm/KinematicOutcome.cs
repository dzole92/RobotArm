using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RobotArm.Interfaces;

namespace RobotArm {
	public class KinematicOutcome: IEquatable<KinematicOutcome>, IKinematicOutcome {

		private const double TOLERANCE = 0.00001;

		public double Theta1 { get; }
		public double Theta2 { get; }
		public Point JointPosition { get; }

		public KinematicOutcome(double theta1, double theta2, Point jointPosition) {
			Theta1 = theta1;
			Theta2 = theta2;
			JointPosition = jointPosition;
		}

		public bool Equals(KinematicOutcome other) {
			if (other == null) return false;
			if (Math.Abs(Theta1 - other.Theta1) < TOLERANCE && Math.Abs(Theta2 - other.Theta2) < TOLERANCE) return true;
			return false;
		}


		public override int GetHashCode() {

			return (int)Theta1 ^ (int)Theta2;

		}

	}
}
