using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Web;
using System.Web.Mvc;
using RobotArm;
using RobotArm.Experiments;
using RobotArm.Helpers;
using RobotArm.Interfaces;

namespace RobotArmUI.Controllers
{
    public class RobotArmController : Controller
    {
        private int _l1;
        private int _l2;
        private double _theta1Min;
        private double _theta1Max;
        private double _theta2Min;
        private double _theta2Max;
        private static double _agleStep;
        private static RobotArm.RobotArm _robotArm;

        // GET: RobotArm
        public ActionResult Index()
        {
            return View("RobotArm");
        }

        public async Task<JsonResult> GetCoordinates(int l1, int l2, double theta1Min, double theta1Max, double theta2Min, double theta2Max, double agleStep)
        {
            await Task.Yield();
            InitLocalRobotArmParameters(l1, l2, theta1Min, theta1Max, theta2Min, theta2Max, agleStep);
            var result = await _robotArm.CalculateWholeDataSet(agleStep);
            if (!result) return Json(new { Success = false, Message = "Something went wrong." });
            return Json(new { Success = true, Message = "OK", _robotArm.Positions });
        }

        private void InitLocalRobotArmParameters(int l1, int l2, double theta1Min, double theta1Max, double theta2Min, double theta2Max, double agleStep)
        {
            _l1 = l1;
            _l2 = l2;
            _theta1Max = theta1Max;
            _theta1Min = theta1Min;
            _theta2Min = theta2Min;
            _theta2Max = theta2Max;
            _agleStep = agleStep;
            _robotArm = new RobotArm.RobotArm(l1, l2, theta1Min, theta1Max, theta2Min, theta2Max);
        }

        /// <summary>
        /// Calculate Arm joint position, when given end point of the robot arm. (Pointing point given)
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public async Task<JsonResult> CalculateJointPostions(double x, double y)
        {
            try
            {
                if (_robotArm == null) throw new NullReferenceException("Robot Arm is not Initialized");
                var result = await _robotArm.CalculateArmJoint(new Point { X = x, Y = y, Z = 0 });
                if( result.Any(point=> double.IsNaN(point.JointPosition.X) || double.IsNaN(point.JointPosition.Y))) throw new Exception("Some of the coordinates is NaN.");
                return Json(new { Success = result.Any(), Outcomes = result.Select(p => new KinematicOutcome(p.Theta1.ConvertRadiansToDegrees(), p.Theta2.ConvertRadiansToDegrees(), p.JointPosition)) });
            }
            catch (Exception e)
            {
                Console.WriteLine(e);
                return Json(new { Success = false, e.Message});

            }
        }

		/// <summary>
		/// Calculate Arm Angels for given end position using trained ANFIS.
		/// </summary>
		/// <param name="x"></param>
		/// <param name="y"></param>
		/// <returns></returns>
		public async Task<JsonResult> CalculateAngelsUsingANFIS(double x, double y) {
			try {
				if (_robotArm == null) throw new NullReferenceException("Robot Arm is not Initialized");
				var result = await _robotArm.CalculateAngelsUsingANFIS(new Point() { X = x, Y = y, Z = 0 });
				if (double.IsNaN(result.Theta1) || double.IsNaN(result.Theta2) ) throw new Exception("Some of the coordinates is NaN.");

				return Json(new { Success = true, Outcome = result });
			} catch (Exception e) {
				Console.WriteLine(e);
				return Json(new { Success = false, e.Message });

			}
		}

		/// <summary>
		/// Train ANFIS networks after generating train data
		/// </summary>
		/// <param name="ruleNumber">Number of Rules using in the training</param>
		/// <param name="maxIterations">Number of iterations using in the training</param>
		/// <returns></returns>
		public async Task<JsonResult> TrainANFIS(int ruleNumber = 25, int maxIterations = 150) {
			try {
				if (_robotArm == null) throw new NullReferenceException("Robot Arm is not Initialized");
				var result = await _robotArm.TrainANFIS(ruleNumber, maxIterations, true);
				return Json(new { Success = result });
			} catch (Exception e) {
				Console.WriteLine(e);
				return Json(new { Success = false, e.Message });

			}
		}

		public async Task<JsonResult> CalculateError() {
			try {
				if (_robotArm == null) throw new NullReferenceException("Robot Arm is not Initialized");
				var results = await _robotArm.CalculateMathError();
				return Json(new {Success = true, Outcome = results});
			} catch (Exception e) {
				Console.WriteLine(e);
				return Json(new { Success = false, e.Message });
			}
		}

		public async Task<JsonResult> CircleExperiment(double radius, double step, double shiftX, double shiftY) {
			try {
				await Task.Yield();
				if(_robotArm == null) throw new NullReferenceException("Robot Arm is not Initialized");
				var circleExperiment = new CircleExperiment();
				circleExperiment.GeneratePositions(radius, step, shiftX, shiftY);
				var experimentResult = await _robotArm.DoExperiment(circleExperiment);
				return Json(new { Success = experimentResult.Any(), ExperimentPositions = circleExperiment.ExperimentPositions.ToList(), Outcome = experimentResult.ToList() });
			} catch (Exception e) {
				Console.WriteLine(e);
				return Json(new { Success = false, e.Message });
			}
		}

		public async Task<JsonResult> SquareExperiment(double side, double step, double shiftX, double shiftY) {
			try {
				await Task.Yield();
				if(_robotArm == null) throw new NullReferenceException("Robot Arm is not Initialized");
				var squareExperiment = new SquareExperiment();
				squareExperiment.GeneratePositions(side, step, shiftX, shiftY);
				var experimentResult = await _robotArm.DoExperiment(squareExperiment);

				return Json(new { Success = experimentResult.Any(), ExperimentPositions = squareExperiment.ExperimentPositions.ToList(), Outcome = experimentResult.ToList() });
			} catch(Exception e) {
				Console.WriteLine(e);
				return Json(new { Success = false, e.Message });
			}
		}

		public async Task<JsonResult> SinExperiment(double length, double step, double shiftX, double shiftY) {
			try {
				await Task.Yield();
				if(_robotArm == null) throw new NullReferenceException("Robot Arm is not Initialized");
				var sinExperiment = new SinExperiment();
				sinExperiment.GeneratePositions(length, step, shiftX, shiftY);
				var experimentResult = await _robotArm.DoExperiment(sinExperiment);
				return Json(new { Success = experimentResult.Any(), ExperimentPositions = sinExperiment.ExperimentPositions.ToList(), Outcome = experimentResult.ToList() });
			} catch(Exception e) {
				Console.WriteLine(e);
				return Json(new { Success = false, e.Message });
			}
		}



	}
}