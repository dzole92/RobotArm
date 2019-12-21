using System.Collections.Generic;
using System.Threading.Tasks;
using RobotArm.Interfaces;

namespace RobotArm.Experiments {

	public interface IExperiment {
		IEnumerable<Point> ExperimentPositions { get; }
		IEnumerable<IKinematicOutcome> ActualOutputs { get; }

		IEnumerable<IKinematicOutcome> AnfisOutputs { get; }

		Task<IEnumerable<MathErrorOutcome>> CalculateError();

		IEnumerable<Point> GeneratePositions(double radius, double step = 0.1745, double shiftX = 0, double shiftY = 0);

		void SetActualOutputs(IEnumerable<IKinematicOutcome> actualOutputs);

		void SetAnfisOutputs(IEnumerable<IKinematicOutcome> outputs);
	}
}