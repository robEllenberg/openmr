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


void TestBase::SetViewer()
{
  viewer = penv->CreateViewer("qtcoin");
  BOOST_ASSERT(!!viewer);

  cout << "Viewer!!!!" << endl;

  // attach it to the environment:
  penv->AttachViewer(viewer);

  //-- Set the camera
  RaveVector<float> rotation(0.426572, 0.285257, 0.469795, 0.718301);
  RaveVector<float> translation(0.337988, 0.179936, 0.165828);
  RaveTransform<float> T(rotation,translation);
  viewer->SetCamera(T);

  // finally you call the viewer's infinite loop
  // (this is why you need a separate thread):
  bool showgui = true;
  viewer->main(showgui);
}





