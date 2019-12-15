using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NUnit.Framework;
using NUnit.Framework.Internal;
using RobotArm;
using RobotArm.Factories;
using RobotArm.Helpers;
using RobotArm.Interfaces;
using RobotArm.MembershipRules;
using RobotArm.RuleExtractors;
using RobotArm.Training;
using Shouldly;
using RobotArm1 = RobotArm.RobotArm;

namespace TestRobotArm
{
    [TestFixture]
    public class RobotArmTests
    {
        [Test]
        public void TestName()
        {
            var a = 2 + 2;
            a.ShouldBe(4);
        }

        [Test]
        [TestCase(3,4, 1.5, 3.1)]
        public void initialize_fuzzy_helper(int l, int t, double theta1, double theta2)
        {
            var robotArm = new RobotArm1(l, t, 0, theta1, 0, theta2);
            robotArm.L1.ShouldNotBeNull();
            robotArm.L2.ShouldNotBeNull();
        }

        [Test]
        [TestCase(3, 4, 1.5, 3.1)]
        public async Task calculate_data_sets(int l, int t, double theta1, double theta2)
        {
            var robotArm = new RobotArm1(l, t, 0, theta1, 0, theta2);
            double agleStep = 0.1;
            var result = await robotArm.CalculateWholeDataSet(agleStep);

            Trace.WriteLine(string.Join("; ", robotArm.Theta1Vector));
            result.ShouldBe(true);
        }

        [Test]
        [TestCase(10, 7, 1.57, 3.1)]
        public async Task draw_graph_test(int l1, int l2, double theta1, double theta2)
        {
            var robotArm = new RobotArm1(l1, l2, 0, theta1, 0, theta2);
            double agleStep = 0.1;
            var result = await robotArm.CalculateWholeDataSet(agleStep);
            var result1 = "";
			var input = "";
			var outputTheta1 = "";
            for (int i = 0; i < robotArm.X.Length; i++)
            {
                result1 += $"({robotArm.X[i]},{robotArm.Y[i]})";
				input += $"new []{{{robotArm.X[i]}, {robotArm.Y[i]}}},{Environment.NewLine}";
			}

			outputTheta1 = robotArm.AnglesGrid.First().ConvertToSingleArray().Aggregate("", (s, d) => s + $"new double[]{{{d}}}, {Environment.NewLine}");

			var inp = robotArm.Positions.ConvertToANFISParameter()
				.Aggregate("", (a, x) => a + $"new []{{{string.Join(", ", x)}}}, {Environment.NewLine}");

			input.ShouldNotBeNullOrEmpty();
			outputTheta1.ShouldNotBeNullOrEmpty();

			Trace.WriteLine(string.Join("; ", robotArm.Theta1Vector));
            result.ShouldBe(true);
        }

        [Test]
        [TestCase(new double[] { 1, 2, 3 }, new double[] { 1, 2, 3, 4, 5 }, TestName = "Check MeshGrid")]
        public async Task checkMeshGrid(double[] theta1Vector, double[] theta2Vector)
        {
            var fuzzyHelper = new FuzzyHelper();
            var result = await fuzzyHelper.MeshGrid(theta1Vector, theta2Vector);

            result.ShouldNotBeNull();
            result.Count().ShouldBe(2);
            var g = (result.ToList()[0]);
            var g1 = (result.ToList()[1]);
            g[1,1].ShouldBe(2);
            g[1,2].ShouldBe(3);
            g1[1,1].ShouldBe(2);
            g1[1,2].ShouldBe(2);
        }

        [Test]
        [TestCase(10,7)]
        public async Task checkStrategyOfCalculationCoordinates(double l1, double l2)
        {
            var fuzzyHelper = new FuzzyHelper();
            var temp = new List<double[,]>();
            var temp1 = new double[2,3] { {1, 2, 3}, {1,2,3}};
            var temp2 = new double[2,3] { {1, 1, 1}, {2,2,2}};
            temp.Add(temp1);
            temp.Add(temp2);
            var result = await fuzzyHelper.CalculateCoordinates(l1, l2, temp);

        }

        [TestCase(5.3524493404594757, 4.0198682584939531, 0, 2)]
        [TestCase(-3.7377074746323506, 2.3615021832391703, 0, 1)]
        [TestCase(6.965029156946181, 0.698833916527797, 0, 1)]
        [TestCase(-0.5775140827585494, 6.433306965204406, 0, 1)]
        public async Task CalculateJointPostionWhenPointPostionKnown(double x, double y, double z, int expectingPositions)
        {
            var robotArm = new RobotArm1(3, 4, 0, 1.5, 0, 3.1);
            var agleStep = 0.1;
            var result = await robotArm.CalculateWholeDataSet(agleStep);
            result.ShouldBe(true);
            var zeroPoint = new Point { X = 0, Y = 0, Z = 0 };
            var endPoint = new Point { X = x, Y = y, Z = 0 };

            var res = await robotArm.CalculateArmJoint(endPoint);

            res.ShouldNotBeNull();
            res.Count().ShouldBe(expectingPositions);
            var points = res.Select(te => te.JointPosition).ToArray();
            points[0].X.ShouldBeGreaterThan(0);
            points[0].Y.ShouldBeGreaterThan(0);
            points[0].Z.ShouldBe(0);
            (Math.Abs(points[0].DistanceFromOtherPoint(zeroPoint)- robotArm.L1)< 0.0000001).ShouldBe(true);
            (Math.Abs(points[0].DistanceFromOtherPoint(endPoint)- robotArm.L2)< 0.0000001).ShouldBe(true);
            if (expectingPositions == 2) {
                (Math.Abs(points[1].DistanceFromOtherPoint(zeroPoint) - robotArm.L1) < 0.0000001).ShouldBe(true);
                (Math.Abs(points[1].DistanceFromOtherPoint(endPoint) - robotArm.L2) < 0.0000001).ShouldBe(true);
            }
        }

        [TestCase(3,4)]
        [TestCase(-3,4)]
        [TestCase(-3,-4)]
        [TestCase(3,-4)]
        public void CheckAgleCalculationFromJointPoint(double x, double y)
        {
            var jointPoint = new Point {X= x, Y=y, Z=0};
            var zeroPoint = new Point { X = 0, Y = 0, Z = 0 };
            var distanceFromZeroPoint = jointPoint.DistanceFromOtherPoint(zeroPoint);
            var alfa = Math.Acos(jointPoint.X / distanceFromZeroPoint);
        }


		[TestCase]
		public void ANFIS_OutputFromDataSet() {
			var sampleSize = RobotArmDataSet.Input.Length - 1;
			//StochasticBatch sprop = new StochasticBatch(sampleSize, 1e-5);
			//sprop.UnknownCaseFaced += AddRule<GaussianRule2>;
			//var sprop = new Backprop(1e-1);
			var sprop = new StochasticQprop(sampleSize);
			var extractor = new KMEANSExtractorIO(25);

			//ANFIS fis = ANFISBuilder<GaussianRule2>.Build(RobotArmDataSet.Input, RobotArmDataSet.OutputTheta1, extractor, sprop, 150);
			ANFIS fis = ANFISBuilder<GaussianRule>.Build(RobotArmDataSet.Input, RobotArmDataSet.OutputTheta1, extractor, sprop, 150);

			var output1 = fis.Inference(new[] { 1.10413546487088, 2.81104319371924 }).FirstOrDefault(); // 1.1
			var output2 = fis.Inference(new[] { 2.31665592712393, 1.9375717475909 }).FirstOrDefault();  // 0.6
			var output3 = fis.Inference(new[] { 2.88944142930409, 16.7526454098038 }).FirstOrDefault(); // 1.4
		}

		[TestCase]
		public async Task ANFIS_OutputFromFuzzy() {

			IRobotArm robotArm = new RobotArm1(10, 7, 0, 1.57, 0, 3.1);
			robotArm.IsDataSetCalculated.ShouldBeFalse();
			await robotArm.CalculateWholeDataSet(0.1);
			await robotArm.TrainANFIS(25, 150, false);
			robotArm.IsDataSetCalculated.ShouldBeTrue();

			var point1 = new Point {X = 1.10413546487088, Y = 2.81104319371924 };
			var point2 = new Point {X = 2.31665592712393, Y = 1.9375717475909 };
			var point3 = new Point {X = 2.88944142930409, Y = 16.7526454098038 };

			var output1 = await robotArm.CalculateAngelsUsingANFIS(point1); // 1.1
			var output2 = await robotArm.CalculateAngelsUsingANFIS(point2); // 0.6
			var output3 = await robotArm.CalculateAngelsUsingANFIS(point3); // 1.4
		}

		[TestCase]
		public async Task CalculateErrorBetweenANFIS_and_Mathematical() {

			IRobotArm robotArm = new RobotArm1(10, 7, 0, 1.57, 0, 3.1);
			robotArm.IsDataSetCalculated.ShouldBeFalse();
			await robotArm.CalculateWholeDataSet(0.1);
			await robotArm.TrainANFIS(25, 150, false);
			robotArm.IsDataSetCalculated.ShouldBeTrue();

			var result = await robotArm.CalculateMathError();
			result.ShouldNotBeNull();
		}

		[TestCase]
		public void CalculateQuadrant() {
			var firstQuadrant = new Point {X = 3, Y = 9};
			var secondQuadrant = new Point {X = -3, Y = 9};
			var thirdQuadrant = new Point {X = -3, Y = -9};
			var fourthQuadrant = new Point {X = 3, Y = -9};
			var fuzzyHelper = new FuzzyHelper();
			var quadrantI = fuzzyHelper.InWhichQuadrant(firstQuadrant);
			var quadrantII = fuzzyHelper.InWhichQuadrant(secondQuadrant);
			var quadrantIII = fuzzyHelper.InWhichQuadrant(thirdQuadrant);
			var quadrantIV = fuzzyHelper.InWhichQuadrant(fourthQuadrant);

			quadrantI.ShouldBe(Quadrant.I);
			quadrantII.ShouldBe(Quadrant.II);
			quadrantIII.ShouldBe(Quadrant.III);
			quadrantIV.ShouldBe(Quadrant.IV);
		}

		[TestCase(1, TestName = "Joint in I Quadrant")]
		[TestCase(2, TestName = "Joint in II Quadrant")]
		[TestCase(3, TestName = "Joint in III Quadrant")]
		[TestCase(4, TestName = "Joint in IV Quadrant")]
		public void CalculateQuadrantByVector(int jointLocation) {

			var jointPosition = new Point();
			var firstQuadrant = new Point { X = 4, Y = 9 };
			var secondQuadrant = new Point { X = 2, Y = 10 };
			var thirdQuadrant = new Point { X = 1, Y = 9 };
			var fourthQuadrant = new Point { X = 4, Y = 6 };

			switch (jointLocation) {
				case 1:
					jointPosition = new Point { X = 3, Y = 9 };
					firstQuadrant = new Point { X = 4, Y = 9 };
					secondQuadrant = new Point { X = 2, Y = 10 };
					thirdQuadrant = new Point { X = 1, Y = 9 };
					fourthQuadrant = new Point { X = 4, Y = 6 };
					break;
				case 2:
					jointPosition = new Point { X = -3, Y = 9 };
					firstQuadrant = new Point { X = -4, Y = 9 };
					secondQuadrant = new Point { X = -2, Y = 10 };
					thirdQuadrant = new Point { X = -1, Y = 9 };
					fourthQuadrant = new Point { X = -4, Y = 6 };
					break;
				case 3:
					jointPosition = new Point { X = -3, Y = -9 };
					firstQuadrant = new Point { X = -4, Y = -9 };
					secondQuadrant = new Point { X = -2, Y = -10 };
					thirdQuadrant = new Point { X = -1, Y = -9 };
					fourthQuadrant = new Point { X = -4, Y = -6 };
					break;
				case 4:
					jointPosition = new Point { X = 3, Y = -9 };
					firstQuadrant = new Point { X = 4, Y = -9 };
					secondQuadrant = new Point { X = 2, Y = -10 };
					thirdQuadrant = new Point { X = 1, Y = -9 };
					fourthQuadrant = new Point { X = 4, Y = -6 };
					break;
			}

			
			
			var fuzzyHelper = new FuzzyHelper();
			var quadrantI = fuzzyHelper.InWhichQuadrant(jointPosition, firstQuadrant);
			var quadrantII = fuzzyHelper.InWhichQuadrant(jointPosition, secondQuadrant);
			var quadrantIII = fuzzyHelper.InWhichQuadrant(jointPosition, thirdQuadrant);
			var quadrantIV = fuzzyHelper.InWhichQuadrant(jointPosition, fourthQuadrant);

			quadrantI.ShouldBe(Quadrant.IV);
			quadrantII.ShouldBe(Quadrant.I);
			quadrantIII.ShouldBe(Quadrant.II);
			quadrantIV.ShouldBe(Quadrant.III);
		}

	}
}
