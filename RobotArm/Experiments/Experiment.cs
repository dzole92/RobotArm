using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using RobotArm.Interfaces;

namespace RobotArm.Experiments {
	public abstract class Experiment : IExperiment {
		public IEnumerable<IKinematicOutcome> ActualOutputs { get; private set; }
		public IEnumerable<IKinematicOutcome> AnfisOutputs { get; private set; }
		public IEnumerable<Point> ExperimentPositions { get; protected set; }

		public Task<IEnumerable<MathErrorOutcome>> CalculateError() {
			var expectedOutcomes = ActualOutputs?.Where(x => x != null).ToArray();
			var actualOutcomes = AnfisOutputs?.Where(x => x != null).ToArray();
			if(expectedOutcomes == null || actualOutcomes == null || expectedOutcomes.Length != actualOutcomes.Length)
				throw new ArgumentException("Size of mathematical and anfis does not match.");
			return Task.Run(() => {
				return expectedOutcomes.Select((t, i) => CalculateMathError(t, actualOutcomes[i])).Where(x => x != null);
			});
		}

		public abstract IEnumerable<Point> GeneratePositions(double radius, double step = 0.1745, double shiftX = 0, double shiftY = 0);

		public void SetActualOutputs(IEnumerable<IKinematicOutcome> actualOutputs) {
			ActualOutputs = actualOutputs;
		}

		public void SetAnfisOutputs(IEnumerable<IKinematicOutcome> outputs) {
			AnfisOutputs = outputs;
		}

		protected virtual MathErrorOutcome CalculateMathError(IKinematicOutcome mathematical, IKinematicOutcome anfis) {
			if(double.IsNaN(mathematical.Theta1)
			|| double.IsNaN(mathematical.Theta2)
			|| double.IsNaN(anfis.Theta1)
			|| double.IsNaN(anfis.Theta2))
				throw new Exception("Some angle is NaN");
			var result = new MathErrorOutcome {
				Theta1Error = (mathematical.Theta1 - anfis.Theta1),
				Theta2Error = (mathematical.Theta2 - anfis.Theta2)
			};
			return result;
		}
	}
}