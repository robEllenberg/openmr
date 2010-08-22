#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <math.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

using namespace OpenRAVE;
using namespace std;


ViewerBasePtr viewer;

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    viewer = penv->CreateViewer(viewername);
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
     envfile="./models/Cube3-I.env.xml";
   else
     envfile = argv[1];

    // create the main environment
    EnvironmentBasePtr penv = CreateEnvironment(true);
    penv->StopSimulation();
    penv->SetDebugLevel(Level_Debug);

    boost::thread thviewer(boost::bind(SetViewer,penv,"qtcoin"));
    // load the scene
    if( !penv->Load(envfile) ) {
        penv->Destroy();
        return 2;
    }

    //-- Set the transform for the camera view
    RaveVector<float> rotation(0.426572, 0.285257, 0.469795, 0.718301);
    RaveVector<float> translation(0.589395, 0.0782605, 0.262882);
    RaveTransform<float> T(rotation,translation);
    viewer->SetCamera(T);

    //-- Get the robot
    std::vector<RobotBasePtr> robots;
    penv->GetRobots(robots);

    //-- Robot 0
    RobotBasePtr probot = robots[0];
    cout << "Robot: " << probot->GetName() << endl;

    //-- Load the controller
    ControllerBasePtr pcontroller = penv->CreateController("sinoscontroller");
    probot->SetController(pcontroller,"");

    stringstream os,is;
    is << "setamplitude 60 60 60 ";
    pcontroller->SendCommand(os,is);

    is << "setinitialphase 0 120 240 ";
    pcontroller->SendCommand(os,is);

    is << "setoffset 0 0 0 ";
    pcontroller->SendCommand(os,is);

    is << "setperiod 1.5 ";
    pcontroller->SendCommand(os,is);

    is << "oscillation on ";
    pcontroller->SendCommand(os,is);

    const dReal STEP = 0.001;
    penv->StartSimulation(STEP);

    while(1) {
      sleep(1);
      //-- Debug! Show the viewer transformation
      //RaveTransform<float> t = viewer->GetCameraTransform();
      //cout << "Transform: " << t << endl;
    }

    thviewer.join();
    penv->Destroy();
    return 0;
}





