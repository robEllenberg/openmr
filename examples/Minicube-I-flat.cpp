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
EnvironmentBasePtr penv;
RobotBasePtr probot;
ControllerBasePtr pcontroller;

const dReal STEP = 0.001;
const dReal PERIOD = 2;
const int SimulationCycles = 100;
const bool REALTIME = false;
const int REPETITIONS = 10;
#define VIEWER 1

void SetViewer(const string& viewername)
{
    viewer = penv->CreateViewer(viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AttachViewer(viewer);

    // finally you call the viewer's infinite loop (this is why you need a separate thread):
    bool showgui = true;
    viewer->main(showgui);

}

void InitRobot(dReal amplitude, dReal period, dReal pd)
{
    //-- Get the robot
    std::vector<RobotBasePtr> robots;
    penv->GetRobots(robots);

    //-- Robot 0
    probot = robots[0];
    cout << "Robot: " << probot->GetName() << endl;

    //-- Load the controller
    pcontroller = penv->CreateController("sinoscontroller");
    probot->SetController(pcontroller,"");

    stringstream os,is;
    is << "setamplitude " << amplitude << " " << amplitude << " ";
    pcontroller->SendCommand(os,is);

    is << "setinitialphase 0 " << pd << " ";
    pcontroller->SendCommand(os,is);

    is << "setoffset 0 0 ";
    pcontroller->SendCommand(os,is);

    is << "setperiod " << period << " ";
    pcontroller->SendCommand(os,is);

    is << "oscillation off ";
    pcontroller->SendCommand(os,is);
}

//-- Perform the simulation. Time: fraction of period (in seconds)
void SimulatePeriod(dReal period, bool realtime=false)
{
  int ticks = round(period/STEP);

  //cout << "ticks: " << ticks << endl;

  //-- The robot is set to its initial state!
  for (int n=0; n<ticks; n++) {
      penv->StepSimulation(STEP);
      if (realtime) usleep(STEP*1000000);
  } 
}

dReal Evaluation(bool realtime=false)
{
    
    Vector vfin, vini, dist;
    stringstream os,is;

    vini = probot->GetCenterOfMass();
    for (int n=0; n<SimulationCycles; n++) {
      //-- Put the robot in oscillation mode
      is << "oscillation on ";
      pcontroller->SendCommand(os,is);
      SimulatePeriod(PERIOD,realtime);
      
      //-- Wait for the servo to reach their final ref. positions
      is << "oscillation off ";
      pcontroller->SendCommand(os,is);
      SimulatePeriod(PERIOD*4/20,realtime);
    }
      
    vfin = probot->GetCenterOfMass();
    dist = vfin - vini;
    dReal step = round(dist.y/SimulationCycles*10000)/10;
    //cout << "Distance: " << round(dist.y*10000)/10 << endl;
    //cout << "Step: " << round(dist.y/SimulationCycles*10000)/10 << endl; 
    return step;
}

int main(int argc, char ** argv)
{
   string envfile;

   if (argc==1)
     //-- Default file
     envfile="models/Minicube-I-flat.env.xml";
   else
     envfile = argv[1];

    // create the main environment
    penv = CreateEnvironment(true);
    penv->StopSimulation();
    penv->SetDebugLevel(Level_Debug);

#ifdef VIEWER
      boost::thread thviewer(boost::bind(SetViewer,"qtcoin"));
#endif

    // load the scene
    if( !penv->Load(envfile) ) {
        penv->Destroy();
        return 2;
    }

#ifdef VIEWER   
      //-- Set the transform for the camera view
      RaveVector<float> rotation(0.426572, 0.285257, 0.469795, 0.718301);
      RaveVector<float> translation(0.337988, 0.179936, 0.165828);
      RaveTransform<float> T(rotation,translation);
      viewer->SetCamera(T);
#endif    

    //-- Amplitude, period, phase difference
    InitRobot(50, PERIOD, 120);

    //-- Wait for the robot to reach the initial state
    SimulatePeriod(PERIOD);
    Vector vorigin = probot->GetCenterOfMass();
    
    Transform t0=probot->GetTransform();
    //cout << "Transform: " << t0 << endl;

    cout << "Repetitions: " << REPETITIONS << endl;
    cout << "Period: " << PERIOD << " sec" << endl;
    cout << "Simulation Step: " << STEP << " sec" << endl;
    cout << "Simulation cycles: " << SimulationCycles << endl;

    //-- Evaluate the robot!
    dReal step;
    for (int n=0; n<REPETITIONS; n++) {
      step = Evaluation(REALTIME);
      cout << "n: " << n << ", Step: " << step << endl;

      //Transform t=probot->GetTransform();
      //cout << "Transform: " << t << endl;
      //probot->SetTransform (const Transform &transform)

      //char key;
      //cin >> key;
    
      probot->SetTransform(t0);
    }

    //-- Evaluate the robot!
    //step = Evaluation(true);
    //cout << "Step: " << step << endl;


    usleep(10000);
    cout << "FIN!" << endl;

#ifdef VIEWER
      thviewer.join();
#endif


    penv->Destroy();
    return 0;
}





