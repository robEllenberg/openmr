#include "hubo.h"
#include <map>
#include <string>
#include <openrave/openrave.h>
#include <openrave/utils.h>
//Add some useful C++ stuff to Dan's hubo.h
using OpenRAVE::RobotBasePtr;

namespace Hubo{

    typedef std::map<std::string,unsigned int> JointMap;
    typedef std::map<unsigned int,unsigned int> DirectJointMap;
    typedef boost::shared_ptr<std::map<std::string,unsigned int>> JointMapPtr;
    typedef boost::shared_ptr<std::map<unsigned int,unsigned int>> DirectJointMapPtr;
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


    /**
     * Create a direct index-to-index map of openHubo to Hubo joints.
     * Pass in a pointer to a robot, and the function will iterate over all the
     * robot's DOF and try to match up with the "official" hubo joint names.
     * This will be UGLY for now but may eventually be formalized. THis should
     * cut down on expensive heap allocations during simulation and realtime.
     * Obviously any structural changes to the robot made by the code will NOT
     * be updated unless you re-run this function.
     */
    DirectJointMapPtr MakeDirectJointMap(RobotBasePtr probot)
    {
        //TODO: Return constant pointer?
        DirectJointMapPtr pjointmap (new DirectJointMap());
        std::string name;
        for (size_t i=0; i < probot->GetDOF(); ++i) {
            name=probot->GetJointFromDOFIndex(i)->GetName();
            if (name2jmc.find(name) != name2jmc.end())
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc[name]));
            //Ugly special cases due to name changes
            //FIXME: without safety checks on names...
            else if (name == "LKP" || name == "RKP") {
                name[2]='N';
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc[name]));
            }
            else if (name == "LEP" || name == "REP") {
                name[2]='B';
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc[name]));
            }
            else if (name == "HPY" || name == "TSY") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["WST"]));
            }
            else if (name == "HNY") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["NKY"]));
            }
            else if (name == "HNR") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["NK1"]));
            }
            else if (name == "HNP") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["NK2"]));
            }
            //FIXME: Not sure if the numbers match the digits correctly here
            else if (name == "leftIndexKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF1"]));
            }
            else if (name == "leftMiddleKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF2"]));
            }
            else if (name == "leftRingKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF3"]));
            }
            else if (name == "leftPinkyKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF4"]));
            }
            else if (name == "leftThumbKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF5"]));
            }
            else if (name == "rightIndexKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF1"]));
            }
            else if (name == "rightMiddleKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF2"]));
            }
            else if (name == "rightRingKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF3"]));
            }
            else if (name == "rightPinkyKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF4"]));
            }
            else if (name == "rightThumbKnuckle1") {
                pjointmap->insert(std::pair<unsigned int,unsigned int>(i,name2jmc["LF5"]));
            }
        }
        return pjointmap;
    };

}
