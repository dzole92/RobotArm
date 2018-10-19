using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Web;
using System.Web.Mvc;
using RobotArm;

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
            if (!result) return Json(new { Success = false, Message = "Something whent wrong." });
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
            if(_robotArm == null) throw new NullReferenceException("Robot Arm is not Initialized");
            var result = await _robotArm.CalculateArmJoint(new Point() {X = x, Y = y, Z = 0});
            return Json(new {Success = result.Any(), Positions = result.ToList()});
        }

    }
}