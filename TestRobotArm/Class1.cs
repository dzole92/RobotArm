using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NUnit.Framework;
using NUnit.Framework.Internal;
using RobotArm;
using Shouldly;
using RobotArm1 = RobotArm.RobotArm;

namespace TestRobotArm
{
    [TestFixture]
    public class Class1
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
            for (int i = 0; i < robotArm.X.Length; i++)
            {
                result1 += $"({robotArm.X[i]},{robotArm.Y[i]})";
            }

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

        [Test]
        [TestCase(5.3524493404594757, 4.0198682584939531, 0)]
        public async Task CalculateJointPostionWhenPointPostionKnown(double x, double y, double z)
        {
            var robotArm = new RobotArm1(3, 4, 0, 1.5, 0, 3.1);
            var agleStep = 0.1;
            var result = await robotArm.CalculateWholeDataSet(agleStep);
            result.ShouldBe(true);
            var zeroPoint = new Point { X = 0, Y = 0, Z = 0 };
            var endPoint = new Point { X = x, Y = y, Z = 0 };

            var res = await robotArm.CalculateArmJoint(endPoint);


            res.ShouldNotBeNull();
            res.Count().ShouldBe(2);
            var points = res.ToArray();
            points[0].X.ShouldBeGreaterThan(0);
            points[0].Y.ShouldBeGreaterThan(0);
            points[0].Z.ShouldBe(0);
            points[0].DistanceFromOtherPoint(zeroPoint).ShouldBe(robotArm.L1);
            points[0].DistanceFromOtherPoint(endPoint).ShouldBe(robotArm.L2);
            points[1].DistanceFromOtherPoint(zeroPoint).ShouldBe(robotArm.L1);
            points[1].DistanceFromOtherPoint(endPoint).ShouldBe(robotArm.L2);
        }
    }
}
