#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <math.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace OpenRAVE;
using namespace std;


void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    RaveViewerBasePtr viewer = penv->CreateViewer(viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AttachViewer(viewer);

    // finally you call the viewer's infinite loop (this is why you need a separate thread):
    bool showgui = true;
    viewer->main(showgui);

}

int main(int argc, char ** argv)
{
   string envfile;

   if (argc==1)
     //-- Default file
     envfile="./models/Minicube-I.env.xml";
   else
     envfile = argv[1];

    // create the main environment
    EnvironmentBasePtr penv = CreateEnvironment(true);
    penv->StopSimulation();

    boost::thread thviewer(boost::bind(SetViewer,penv,"qtcoin"));
    {
        // lock the environment to prevent changes
        EnvironmentMutex::scoped_lock lock(penv->GetMutex());

        // load the scene
        penv->Load(envfile);
    }

    //-- Get the robot
    std::vector<RobotBasePtr> robots;
    penv->GetRobots(robots);

    //-- Robot 0
    RobotBasePtr probot = robots[0];
    cout << "Robot: " << probot->GetName() << endl;

    //-- Load the controller
    ControllerBasePtr pcontroller = penv->CreateController("sinoscontroller");
    probot->SetController(pcontroller,"");

    const dReal STEP = 0.005;
    penv->StartSimulation(STEP);

    stringstream is;
    stringstream os;

    while(1) {
      //is << "setpos 45 45 ";
      //pcontroller->SendCommand(os,is);
      sleep(1);

      //is << "setpos -45 -45 ";
      //pcontroller->SendCommand(os,is);
      sleep(1);
    }

    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroyThe
    return 0;
}





