using Microsoft.Owin;
using Owin;

[assembly: OwinStartupAttribute(typeof(RobotArmUI.Startup))]
namespace RobotArmUI
{
    public partial class Startup
    {
        public void Configuration(IAppBuilder app)
        {
            ConfigureAuth(app);
        }
    }
}
