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
     envfile="./models/Tricube-I.env.xml";
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
    ControllerBasePtr pcontroller = penv->CreateController("sinoscontroller");
    probot->SetController(pcontroller,"");

    stringstream os,is;
    is << "setamplitude 30 30 30 ";
    pcontroller->SendCommand(os,is);

    is << "setinitialphase 0 150 300 ";
    pcontroller->SendCommand(os,is);

    is << "setoffset 0 0 0 ";
    pcontroller->SendCommand(os,is);

    is << "setperiod 2 ";
    pcontroller->SendCommand(os,is);

    const dReal STEP = 0.003;
    penv->StartSimulation(STEP);
    usleep(1000);
    penv->SetCamera (Tcamera);

    char key;
    while(1) {

      cin >> key;
      cout << penv->GetCameraTransform() << endl;
      //sleep(1);
    }


    thviewer.join();
    penv->Destroy();
    return 0;
}





