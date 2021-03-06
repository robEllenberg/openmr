#include "TestBase.h"
#include <iostream>

class Example : public TestBase
{
  public:
    Example(string envfile,string controller,bool showgui=true) :
       TestBase(envfile,controller,showgui) {};
    void run(dReal step, bool realtime=true);
};

void Example::run(dReal step, bool realtime)
{
  stringstream os,is;
  is << "setamplitude 60 60 60 ";
  pcontroller->SendCommand(os,is);

  is << "setinitialphase 0 120 240 ";
  pcontroller->SendCommand(os,is);

  is << "setoffset 0 0 0 ";
  pcontroller->SendCommand(os,is);

  is << "setperiod 3 ";
  pcontroller->SendCommand(os,is);

  is << "oscillation on ";
  pcontroller->SendCommand(os,is);

  char tecla;
  cout << "Press a key to start the simulation" << endl;

  cin >> tecla;

  penv->StartSimulation(step,realtime);

  while(1);
}

int main(int argc, char ** argv)
{
  string envfile;

  if (argc==1)
    //-- Default file
    envfile="./models/Cube3-I.env.xml";
  else
    envfile = argv[1];

  Example example(envfile,"sinoscontroller");
  usleep(100000);
  example.SetCamera(0.427, 0.285, 0.47, 0.718, 0.59, 0.078, 0.263);
  example.run(0.001,false);

  return 0;
}






