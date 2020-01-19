using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using RobotArm.Experiments;
using RobotArm.Factories;
using RobotArm.Helpers;
using RobotArm.Interfaces;
using RobotArm.MembershipRules;
using RobotArm.RuleExtractors;
using RobotArm.Training;

namespace RobotArm {

	public class RobotArm : IRobotArm {
		private const double TOLERANCE = 0.0000001;
		private double[] x1d;
		private double[] y1d;
		public double L1 { get; }

		public double L2 { get; }
		public double Theta2Min { get; }

		public double Theta1Min { get; }

		public double Theta1Max { get; }

		public double Theta2Max { get; }

		public double[] Theta1Vector { get; private set; }
		public double[] Theta2Vector { get; private set; }
		public IEnumerable<double[,]> AnglesGrid { get; set; }

		private double[,] Y2D { get; set; }

		private double[,] X2D { get; set; }

		private readonly FuzzyHelper _fuzzyHelper;
		private ANFIS Theta1ANFIS;
		private ANFIS Theta2ANFIS;

		private readonly Point zeroPoint = new Point { X = 0, Y = 0, Z = 0 };

		public double[] X {
			get {
				if(x1d != null)
					return x1d;
				if(X2D == null || X2D.Length < 2)
					return null;
				var rowsNum = X2D.GetLength(0);
				var colsNum = X2D.GetLength(1);
				var tempArray = new List<double>();
				for(int i = 0; i < rowsNum; i++) {
					for(int j = 0; j < colsNum; j++) {
						tempArray.Add(X2D[i, j]);
					}
				}
				x1d = tempArray.ToArray();
				return x1d;
			}
		}

		public double[] Y {
			get {
				if(y1d != null)
					return y1d;
				if(Y2D == null || Y2D.Length < 2)
					return null;
				var rowsNum = Y2D.GetLength(0);
				var colsNum = Y2D.GetLength(1);
				var tempArray = new List<double>();
				for(int i = 0; i < rowsNum; i++) {
					for(int j = 0; j < colsNum; j++) {
						tempArray.Add(Y2D[i, j]);
					}
				}
				y1d = tempArray.ToArray();
				return y1d;
			}
		}

		public IEnumerable<Point> Positions { get; private set; }
		public bool IsDataSetCalculated { get; private set; }
		public bool IsANFISTrained { get; private set; }

		public RobotArm(double l1, double l2, double theta1Min, double theta1Max, double theta2Min, double theta2Max) {
			L1 = l1;
			L2 = l2;
			Theta1Max = theta1Max;
			Theta2Max = theta2Max;
			Theta1Min = theta1Min;
			Theta2Min = theta2Min;
			_fuzzyHelper = new FuzzyHelper();
		}

		public RobotArm(double l1, double l2, double theta1Min, double theta1Max, double theta2Min, double theta2Max, FuzzyHelper fuzzyHelper) {
			L1 = l1;
			L2 = l2;
			Theta1Max = theta1Max;
			Theta2Max = theta2Max;
			Theta1Min = theta1Min;
			Theta2Min = theta2Min;
			_fuzzyHelper = fuzzyHelper;
		}

		public async Task<bool> CalculateWholeDataSet(double agleStep) {
			try {
				var calculateVector1Task = _fuzzyHelper.CalculateVector(Theta1Min, Theta1Max, agleStep);
				var calculateVector2Task = _fuzzyHelper.CalculateVector(Theta2Min, Theta2Max, agleStep);

				Theta1Vector = await calculateVector1Task;
				Theta2Vector = await calculateVector2Task;

				if(Theta1Vector == null || !Theta1Vector.Any())
					throw new Exception("Theta1Max Vector is empty.");
				if(Theta2Vector == null || !Theta2Vector.Any())
					throw new Exception("Theta2Max Vector is empty.");

				AnglesGrid = await _fuzzyHelper.MeshGrid(Theta1Vector, Theta2Vector);
				if(AnglesGrid == null)
					throw new Exception("MeshGrid function not working");

				var coordinates = await _fuzzyHelper.CalculateCoordinates(L1, L2, AnglesGrid);
				X2D = coordinates.ToArray()[0];
				Y2D = coordinates.ToArray()[1];

				var t1 = Task.Run(() => X.ToList());
				var t2 = Task.Run(() => Y.ToList());

				Task.WaitAll(t1, t2);
				Positions = X.Select((t, i) => new Point { X = t, Y = Y[i], Z = 0 }).ToList();
				IsDataSetCalculated = true;

				return true;
			} catch(Exception e) {
				Trace.WriteLine(e);
			}
			return false;
		}

		public Task<IEnumerable<KinematicOutcome>> CalculateArmJoint(Point endPoint) {
			/*
			Formulas for calculation:
				y1 =  (A + D)/B
				y2 = (A - D)/B
				x1 = +-Sqrt(L1^2 - y1^2)
				x2 = +-Sqrt(L1^2 - y2^2)
				-----------------------------------------------------------------
				A = 2*c*y;
				B = 2*(y^2 + x^2);
				D = Sqrt(m - n*o);
				c = (L1^2 - L2^2 +x^2 +y^2)/2.0;
				m = 4*c^2*y^2
				n = 4*(y^2+x^2);
				o = (c^2-(x^2*L1^2));
		   */
			return Task.Run(() => {
				var x = endPoint.X;
				var y = endPoint.Y;
				var c = (Math.Pow(L1, 2) - Math.Pow(L2, 2) + Math.Pow(x, 2) + Math.Pow(y, 2)) / 2.0;
				var m = 4 * (Math.Pow(c, 2) * Math.Pow(y, 2));
				var n = 4 * (Math.Pow(y, 2) + Math.Pow(x, 2));
				var o = Math.Pow(c, 2) - x * x * Math.Pow(L1, 2);
				var A = 2.0 * c * y;
				var B = 2 * (Math.Pow(y, 2) + Math.Pow(x, 2));
				var d = m - n * o;
				// *** Error allowed of 10^-7 if d is negative
				if(d < 0) {
					if(d * Math.Pow(10, 7) < 0)
						d = 0;
					else
						throw new Exception($"Cannot calculate joint point for this end point: x: {x} y: {y}");
				}
				d = Math.Sqrt(d);
				var y1 = (A + d) / B;
				var y2 = (A - d) / B;
				var x1 = Math.Sqrt(Math.Pow(L1, 2) - Math.Pow(y1, 2));
				var x2 = Math.Sqrt(Math.Pow(L1, 2) - Math.Pow(y2, 2));

				var listOfPoints = new List<Point> {
						new Point {X = x1.Round(), Y = y1.Round(), Z = 0}, new Point {X = x2.Round(), Y = y2.Round(), Z = 0},
						new Point {X = -x1.Round(), Y = y1.Round(), Z = 0}, new Point {X = -x2.Round(), Y = y2.Round(), Z = 0}
					}.Select(RoundWithTolerance).Distinct(new CustomPointEquality());

				//var t = listOfPoints.Select(point => new KinematicOutcome(FindTheta1WhenJointPointGiven(point), FindTheta2WhenJointPointGiven(point, endPoint), point)).ToList();
				return
					listOfPoints.Select(point =>
											new KinematicOutcome(FindTheta1WhenJointPointGiven(point), FindTheta2WhenJointPointGiven(point, endPoint), point))
											.Where(outcome =>
													outcome.Theta1.Between(Theta1Min, Theta1Max, true)
													&& outcome.Theta2.Between(Theta2Min, Theta2Max, true)
													&& ValidateJointPointWithEndAndZeroPoint(outcome.JointPosition, endPoint)
												);
			});
		}

		private Point RoundWithTolerance(Point point) {
			return new Point {
				X = Math.Abs(point.X) < TOLERANCE ? 0 : point.X,
				Y = Math.Abs(point.Y) < TOLERANCE ? 0 : point.Y,
				Z = Math.Abs(point.Z) < TOLERANCE ? 0 : point.Z,
			};
		}

		/// <summary>
		///
		/// </summary>
		/// <param name="ruleNumber"></param>
		/// <param name="maxIterations"></param>
		/// <returns></returns>
		public Task<bool> TrainANFIS(int ruleNumber, int maxIterations, bool useAnalicitalOutcomeForTraining = false) {
			return Task.Run(() => {
				if(!IsDataSetCalculated)
					throw new ApplicationException("DataSet is not calculated or provided.");

				var sampleSize = Positions.Count() - 1;
				var dynamicObj = useAnalicitalOutcomeForTraining ? Positions.Select(x => new { Point = x, KinematicOutCome = CalculateArmJoint(x).GetAwaiter().GetResult().FirstOrDefault() }) : null;

				var input = useAnalicitalOutcomeForTraining
								? dynamicObj.Select(x => x.Point).ConvertToANFISParameter()
								: Positions.ConvertToANFISParameter();
				var theta1ANFIS = Task.Run(() => {
					var sPropTheta1 = new StochasticQprop(sampleSize);
					var extractorForTheta1 = new KMEANSExtractorIO(ruleNumber);
					var expectedOutcome = useAnalicitalOutcomeForTraining
											? dynamicObj.Select(x => new[] { x.KinematicOutCome.Theta1.ConvertRadiansToDegrees() }).ToArray()
											: AnglesGrid.First().ConvertToANFISParameter();
					Theta1ANFIS = ANFISBuilder<GaussianRule>.Build(input,
																	expectedOutcome, extractorForTheta1, sPropTheta1, maxIterations);
				});
				var theta2ANFIS = Task.Run(() => {
					var sPropTheta2 = new StochasticQprop(sampleSize);
					var extractorForTheta2 = new KMEANSExtractorIO(ruleNumber);
					var expectedOutcome2 = useAnalicitalOutcomeForTraining
										? dynamicObj.Select(x => new[] { x.KinematicOutCome.Theta2.ConvertRadiansToDegrees() }).ToArray()
										: AnglesGrid.Last().ConvertToANFISParameter();

					Theta2ANFIS = ANFISBuilder<GaussianRule>.Build(input,
																	expectedOutcome2, extractorForTheta2, sPropTheta2, maxIterations);
				});

				Task.WaitAll(theta1ANFIS, theta2ANFIS);
				IsANFISTrained = true;
				return true;
			});
		}

		/// <summary>
		/// Return Theta1 and Theta2 also the joint point.
		/// </summary>
		/// <param name="endPoint"></param>
		/// <returns></returns>
		public Task<KinematicOutcome> CalculateAngelsUsingANFIS(Point endPoint) {
			if(!IsANFISTrained)
				throw new ApplicationException("ANFIS is not trained");
			return Task.Run(() => {
				var theta1 = Theta1ANFIS?.Inference(endPoint.CovertToANFISParameter()).FirstOrDefault() ?? 0;
				var theta2 = Theta2ANFIS?.Inference(endPoint.CovertToANFISParameter()).FirstOrDefault() ?? 0;

				var jointPoint = FindJointPointWhenAngleGiven(theta1.ConvertDegreesToRadians());

				return new KinematicOutcome(theta1.Round(), theta2.Round(), jointPoint);
			});
		}

		/// <summary>
		/// Calculate the error of ANFIS against the mathematical solution
		/// </summary>
		/// <returns></returns>
		public async Task<IEnumerable<MathErrorOutcome>> CalculateMathError() {
			if(!IsDataSetCalculated) throw new ApplicationException("DataSet is not calculated");
			if(!IsANFISTrained) throw new ApplicationException("ANFIS is not trained");
			var mathematicalOutcomes = new List<IKinematicOutcome>();
			var anfisOutcomes = new List<IKinematicOutcome>();
			Positions.ToList().ForEach((endPoint) => {
				var anfisOut = CalculateAngelsUsingANFIS(endPoint).GetAwaiter().GetResult();
				var mathOuts = CalculateArmJoint(endPoint).GetAwaiter().GetResult().Select(x => new KinematicOutcome(x.Theta1.ConvertRadiansToDegrees(), x.Theta2.ConvertRadiansToDegrees(), x.JointPosition));
				var mathOut = mathOuts.OrderBy(x => x, new CustomSort(anfisOut)).FirstOrDefault();

				anfisOutcomes.Add(anfisOut);
				mathematicalOutcomes.Add(mathOut);
			});

			return await _fuzzyHelper.CalculateMathError(mathematicalOutcomes, anfisOutcomes);
		}

		public async Task<IEnumerable<MathErrorOutcome>> DoExperiment(IExperiment experiment) {
			if(!(experiment.ExperimentPositions?.Any() ?? false)) throw new Exception("No Experiment Positions generated.");
			if(!IsDataSetCalculated) throw new ApplicationException("DataSet is not calculated");
			if(!IsANFISTrained) throw new ApplicationException("ANFIS is not trained");

			var mathematicalOutcomes = new List<IKinematicOutcome>();
			var anfisOutcomes = new List<IKinematicOutcome>();
			experiment.ExperimentPositions.ToList().ForEach(point => {
				var anfisOut = CalculateAngelsUsingANFIS(point).GetAwaiter().GetResult();
				var mathOuts = CalculateArmJoint(point).GetAwaiter().GetResult()
								.Select(x => new KinematicOutcome(x.Theta1.ConvertRadiansToDegrees(), x.Theta2.ConvertRadiansToDegrees(), x.JointPosition));
				var mathOut = mathOuts.OrderBy(x => x, new CustomSort(anfisOut)).FirstOrDefault();

				anfisOutcomes.Add(anfisOut);
				mathematicalOutcomes.Add(mathOut);
			});
			experiment.SetActualOutputs(mathematicalOutcomes);
			experiment.SetAnfisOutputs(anfisOutcomes);

			return await experiment.CalculateError();
		}

		private double FindTheta1WhenJointPointGiven(Point jointPoint) {
			var vectorX = new Point { X = 100, Y = 0 };
			var angle = _fuzzyHelper.AngleBetweenVectors(jointPoint, vectorX).Round();
			var quadrant = _fuzzyHelper.InWhichQuadrant(jointPoint);
			switch(quadrant) {
				//case Quadrant.II:
				//	return (Math.PI - angle).Round();
				case Quadrant.III:
				case Quadrant.IV:
					return (2 * Math.PI - angle).Round();

				case Quadrant.None:
				case Quadrant.I:
				default:
					return angle;
			}
		}

		private double FindTheta2WhenJointPointGiven(Point jointPoint, Point endPoint) {
			var vectorX = new Point { X = endPoint.X - jointPoint.X, Y = endPoint.Y - jointPoint.Y };
			var angle = _fuzzyHelper.AngleBetweenVectors(jointPoint, vectorX).Round();
			if(Math.Abs(angle) < TOLERANCE)
				return angle;
			var quadrant = _fuzzyHelper.InWhichQuadrant(jointPoint, endPoint);
			switch(quadrant) {
				//case Quadrant.II:
					//return (Math.PI - angle).Round();

				case Quadrant.III:
					//return (Math.PI + angle).Round();

				case Quadrant.IV:
					return (2 * Math.PI - angle).Round();

				case Quadrant.None:
				case Quadrant.I:
				default:
					return angle;
			}
		}

		private Point FindJointPointWhenAngleGiven(double theta1) {
			if(Math.Abs(L1) < 0.00000001 || Math.Abs(theta1) < 0.00000001)
				throw new Exception("L1 is not given");
			var x = Math.Cos(theta1) * L1;
			var y = Math.Sin(theta1) * L1;
			return new Point { X = x.Round(), Y = y.Round() };
		}

		private bool ValidateJointPointWithEndAndZeroPoint(Point jointPoint, Point endPoint) {
			var distanceFromZeroPoint = jointPoint.DistanceFromOtherPoint(zeroPoint).Round(4);
			var distanceFromEndPoint = jointPoint.DistanceFromOtherPoint(endPoint).Round(4);
			return !(Math.Abs(distanceFromZeroPoint - L1) > 0.0000001) && !(Math.Abs(distanceFromEndPoint - L2) > 0.0000001);
		}

		private bool ValidateJointPointAgainstAngle(Point jointPoint, Point endpoint) {
			var slope1 = CalculateSlopeOfLine(zeroPoint, jointPoint);
			var slope2 = CalculateSlopeOfLine(jointPoint, endpoint);

			var theta1 = Math.Round(Math.Atan(slope1), 7);
			var theta2 = Math.Round(Math.Atan((slope2 + slope1 * -1) / (1 + slope1 * slope2)), 7);

			if(jointPoint.X < 0 && jointPoint.Y > 0)
				theta1 = Math.PI + theta1;
			else if(jointPoint.X > 0 && jointPoint.Y < 0)
				theta1 = 2 * Math.PI + theta1;

			return theta1.Between(Theta1Min, Theta1Max, true); //&& theta2.Between(Theta2Min, Theta2Max, true);
		}

		private double CalculateSlopeOfLine(Point firstPoint, Point secondPoint) {
			return (secondPoint.Y + firstPoint.Y * -1) / (secondPoint.X + firstPoint.X * -1);
		}
	}

	internal class CustomSort : IComparer<KinematicOutcome> {
		private readonly KinematicOutcome anfis;

		public CustomSort(KinematicOutcome anfis) {
			this.anfis = anfis;
		}

		public int Compare(KinematicOutcome x, KinematicOutcome y) {
			if(x == null)
				return -1;
			if(y == null)
				return 1;
			var error1 = Math.Abs(x.Theta1 - anfis.Theta1);
			var error2 = Math.Abs(x.Theta2 - anfis.Theta2);
			var error11 = Math.Abs(y.Theta1 - anfis.Theta1);
			var error22 = Math.Abs(y.Theta2 - anfis.Theta2);
			var sum1 = error1 + error2;
			var sum2 = error11 + error22;
			return sum1 > sum2 ? 1 : sum1 < sum2 ? -1 : 0;
		}
	}

	internal class CustomPointEquality : IEqualityComparer<Point> {
		private const double TOLERANCE = 0.00001;

		public bool Equals(Point x, Point y) {
			return Math.Abs(x.X - y.X) < TOLERANCE && Math.Abs(x.Y - y.Y) < TOLERANCE;
		}

		public int GetHashCode(Point obj) {
			return obj.GetHashCode();
		}
	}
}