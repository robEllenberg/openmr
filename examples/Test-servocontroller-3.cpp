/*------------------------------------------------------------------------*/
/* Test-servocontroller3                                                  */
/*------------------------------------------------------------------------*/
/* (c) Juan Gonzalez. July-2010                                           */
/*------------------------------------------------------------------------*/
/* GPL license                                                            */
/*------------------------------------------------------------------------*/
/* Testing the Servocontroller.                                           */
/* Example of the record_on and record_off commands                       */
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
     envfile="./models/Unimod2.env.xml";
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

    //-- Set the transform matrix for the camera view
    RaveTransformMatrix<float> M;
    RaveVector<float> rotquad(0.505073, 0.268078, 0.395983, 0.718493);
    RaveVector<float> trans(0.412915, 0.156822, 0.285362);
    M.trans = trans;
    M.rotfromquat (rotquad);
    RaveTransform<float> Tcamera(M);

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
    penv->SetCamera (Tcamera);


    stringstream is;
    stringstream os;

    //-- Initially set the position to 0
    is << "setpos 0 0 ";
    pcontroller->SendCommand(os,is);
    sleep(1);


    //-- Start recording the servo's angle
    is << "record_on test1.m ";
    pcontroller->SendCommand(os,is);
    usleep(100000);

    is << "setpos 45 -45 ";
    pcontroller->SendCommand(os,is);
    sleep(1);

    is << "record_off ";
    pcontroller->SendCommand(os,is);


    cout << "FIN\n";



    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroyThe
    return 0;
}





