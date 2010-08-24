#include "TestBase.h"

#include <iostream>
#include <boost/bind.hpp>

//-- Constructor
TestBase::TestBase(string envfile, string controller, bool showgui)
{

  this->showgui = showgui;

  // create the main environment
  penv = CreateEnvironment(true);
  penv->StopSimulation();
  penv->SetDebugLevel(Level_Debug);

  // load the scene
  if( !penv->Load(envfile) ) {
      penv->Destroy();
      return;
  }

  //-- Start the viewer
  if (showgui)
    pthviewer = new boost::thread(boost::bind(&TestBase::SetViewer, this));

  //-- Get the robot
  std::vector<RobotBasePtr> robots;
  penv->GetRobots(robots);
  probot = robots[0];
  cout << "Robot: " << probot->GetName() << endl;

  //-- Load the controller
  pcontroller = penv->CreateController(controller);
  probot->SetController(pcontroller,"");

}

//-- Destructor
TestBase::~TestBase()
{
  if (showgui) {
    pthviewer->join();
    delete pthviewer;
  }

  //-- Destroy the environment
  penv->Destroy();
}

void TestBase::SetCamera(dReal q0, dReal q1, dReal q2, dReal q3, dReal tx, dReal ty, dReal tz)
{
  //-- Perform the transformation only if the gui is active
  if (!showgui) return;

  RaveVector<float> rotation(q0,q1,q2,q3);
  RaveVector<float> translation(tx,ty,tz);
  RaveTransform<float> T(rotation,translation);
  viewer->SetCamera(T);
}

void TestBase::SetViewer()
{
  viewer = penv->CreateViewer("qtcoin");
  BOOST_ASSERT(!!viewer);

  cout << "Viewer!!!!" << endl;

  // attach it to the environment:
  penv->AttachViewer(viewer);

  //-- Set the camera
  SetCamera(0.427, 0.285, 0.47, 0.718, 0.338, 0.18, 0.166);

  // finally you call the viewer's infinite loop
  // (this is why you need a separate thread):
  bool showgui = true;
  viewer->main(showgui);
}




