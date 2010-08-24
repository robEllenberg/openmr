#include <openrave-core.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace OpenRAVE;

class TestBase
{
  public:
    TestBase(string ,string,bool showgui=true);
    ~TestBase();

  protected:
    EnvironmentBasePtr penv;
    ViewerBasePtr viewer;
    boost::thread *pthviewer;
    RobotBasePtr probot;
    ControllerBasePtr pcontroller;
    bool showgui;

  private:
    void SetViewer();

};


