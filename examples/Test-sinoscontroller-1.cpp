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

    RaveTransformMatrix<float> M;
    M.m[0] = -0.34606934; M.m[1] = -0.51347446; M.m[2] =0.78522629 ;  M.m[3] = 0.49086213;
    M.m[4] = 0.93809277;  M.m[5] = -0.17619586; M.m[6] = 0.29822361;  M.m[7] = 0.21498504;
    M.m[8] = -0.01477666; M.m[9] = 0.83982098;  M.m[10] = 0.54266238; M.m[11] = 0.28141484;
    RaveVector<float> trans(0.412915, 0.156822, 0.285362);
    M.trans = trans;
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
    is << "setamplitude 45 45 ";
    pcontroller->SendCommand(os,is);

    is << "setinitialphase 0 120 ";
    pcontroller->SendCommand(os,is);

    is << "setoffset 0 0 ";
    pcontroller->SendCommand(os,is);

    is << "setperiod 2 ";
    pcontroller->SendCommand(os,is);

    const dReal STEP = 0.005;
    penv->StartSimulation(STEP);

    penv->SetCamera (Tcamera);

    char tecla;
    while(1) {

      cin >> tecla;
      //penv->SetCamera (Tcamera);
      cout << penv->GetCameraTransform() << endl;


      //sleep(1);
    }


    thviewer.join();
    penv->Destroy();
    return 0;
}





