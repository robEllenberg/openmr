#include "hubo.h"
#include <map>
#include <string>
//Add some useful C++ stuff to Dan's hubo.h

namespace Hubo{

    typedef std::map<std::string,unsigned int> JointMap;
    JointMap name2jmc={

    /* Joint Numbers/Index values */
    std::make_pair("RHY",26),    //Right Hip Yaw
    std::make_pair("RHR",27),    //Right Hip Roll
    std::make_pair("RHP",28),    //Right Hip Pitch
    std::make_pair("RKN",29),    //Right Knee Pitch
    std::make_pair("RAP",30),    //Right Ankle Pitch
    std::make_pair("RAR",31),    //Right Ankle Roll

    std::make_pair("LHY",19),    //Left Hip Yaw
    std::make_pair("LHR",20),    //Left Hip Roll
    std::make_pair("LHP",21),    //Left Hip Pitch
    std::make_pair("LKN",22),    //Left Knee Pitch
    std::make_pair("LAP",23),    //Left Ankle Pitch
    std::make_pair("LAR",24),    //Left Ankle Roll

    std::make_pair("RSP",11),    //Right Shoulder Pitch
    std::make_pair("RSR",12),    //Right Shoulder Pitch
    std::make_pair("RSY",13),    //Right Shoulder Roll
    std::make_pair("REB",14),    //Right Elbow Pitch
    std::make_pair("RWY",15),    // right wrist yaw
    std::make_pair("RWR",16),    // right wrist roll
    std::make_pair("RWP",17),    // right wrist Pitch

    std::make_pair("LSP",4),    //Left Shoulder Pitch
    std::make_pair("LSR",5),    //Left Shoulder Yaw
    std::make_pair("LSY",6),    //Left Shoulder Roll
    std::make_pair("LEB",7),    //Left Elbow Pitch
    std::make_pair("LWY",8),    // left wrist yaw
    std::make_pair("LWR",9),    // left wrist roll
    std::make_pair("LWP",10),    // left wrist pitch

    std::make_pair("NKY",1),    // neck yaw
    std::make_pair("NK1",2),    // neck 1
    std::make_pair("NK2",3),    // neck 2

    std::make_pair("WST",0),    //Trunk Yaw

    std::make_pair("RF1",32),    //Right Finger
    std::make_pair("RF2",33),    //Right Finger
    std::make_pair("RF3",34),    //Right Finger
    std::make_pair("RF4",35),    //Right Finger
    std::make_pair("RF5",36),    //Right Finger
    std::make_pair("LF1",37),    //Left Finger
    std::make_pair("LF2",38),    //Left Finger
    std::make_pair("LF3",39),    //Left Finger
    std::make_pair("LF4",40),    //Left Finger
    std::make_pair("LF5",41)    //Left Finger
    };
}
