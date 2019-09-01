using System.Collections.Generic;
using System.Threading.Tasks;

namespace RobotArm.Interfaces {
	public interface IRobotArm {

		double L1 { get; }
		double L2 { get; }
		double Theta2Min { get; }
		double Theta1Min { get; }
		double Theta1Max { get; }
		double Theta2Max { get; }
		double[] Theta1Vector { get; }
		double[] Theta2Vector { get; }
		IEnumerable<double[,]> AnglesGrid { get; set; }
		double[] X { get; }
		double[] Y { get; }
		IEnumerable<Point> Positions { get; }
		bool IsDataSetCalculated { get; }
		bool IsANFISTrained { get; }

		Task<bool> CalculateWholeDataSet(double agleStep);

		Task<IEnumerable<KinematicOutcome>> CalculateArmJoint(Point endPoint);
		Task<bool> TrainANFIS(int ruleNumber, int maxIterations);
		Task<KinematicOutcome> CalculateAngelsUsingANFIS(Point endPoint);

	}
}