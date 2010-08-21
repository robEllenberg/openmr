/*------------------------------------------------------------------------*/
/* Test-servocontroller2                                                  */
/*------------------------------------------------------------------------*/
/* (c) Juan Gonzalez. July-2010                                           */
/*------------------------------------------------------------------------*/
/* GPL license                                                            */
/*------------------------------------------------------------------------*/
/* Testing the Servocontroller.                                           */
/* This example set the position of the servos 0 and 1 to 45 and -45      */
/* alternately                                                            */
/**************************************************************************/

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
     envfile="./models/Unimod2.env.xml";
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
    RaveVector<float> rotation(-0.255844, -0.19416, 0.54046, 0.777655);
    RaveVector<float> translation(-0.155229, 0.230973, 0.118655);
    RaveTransform<float> T(rotation,translation);
    viewer->SetCamera(T);

    //-- Get the robot
    std::vector<RobotBasePtr> robots;
    penv->GetRobots(robots);

    //-- Robot 0
    RobotBasePtr probot = robots[0];
    cout << "Robot: " << probot->GetName() << endl;

    //-- Load the controller
    ControllerBasePtr pcontroller = penv->CreateController("servocontroller");
    probot->SetController(pcontroller,"");

    const dReal STEP = 0.005;
    penv->StartSimulation(STEP);
    

    stringstream is;
    stringstream os;

    //-- Main loop
    while(1) {

      //-- Servo positioning example 1: Using the setpos command. The position
      //-- of the two servos is given: Servo 0 to 45 and servo 1 to -45
      is << "setpos 45 -45 ";
      pcontroller->SendCommand(os,is);

      //-- Wait one second
      sleep(1);

      //-- Servo positioning example 2:
      //-- One method: using the "setpos" command:
      //-- is << "setpos -45 45 ";
      //-- pcontroller->SendCommand(os,is);

      //-- Another method: using the setpos1 command:
      //-- Seting the servo 0 position:
      is << "setpos1 0 -45 ";
      pcontroller->SendCommand(os,is);

      //-- Seting the servo 1 position:
      is << "setpos1 1 45 ";
      pcontroller->SendCommand(os,is);

      //-- Wait one second
      sleep(1);

      //-- Debug! Show the viewer transformation
      //RaveTransform<float> t = viewer->GetCameraTransform();
      //cout << "Transform: " << t << endl;
    }

    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroyThe
    return 0;
}





