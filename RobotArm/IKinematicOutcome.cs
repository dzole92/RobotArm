using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RobotArm.Interfaces;

namespace RobotArm {
	public interface IKinematicOutcome {

		double Theta1 { get; }
		double Theta2 { get; }
		Point JointPosition { get; }

	}
}
