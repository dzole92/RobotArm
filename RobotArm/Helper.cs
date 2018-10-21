namespace RobotArm
{
    public static class Helper
    {
        public static bool Between(this double num, double lower, double upper, bool inclusive = false)
        {
            return inclusive
                ? lower <= num && num <= upper
                : lower < num && num < upper;
        }
    }
}