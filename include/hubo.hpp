/*
Copyright (c) 2012, Robert W. Ellenberg
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may 
      be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "hubo.h"
#include <map>
#include <vector>
#include <string>
#include <openrave/openrave.h>
#include <openrave/utils.h>
#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
//Add some useful C++ stuff to Dan's hubo.h
using OpenRAVE::RobotBasePtr;

namespace Hubo{

    typedef std::map<std::string,int> JointMap;
    typedef std::vector<int> DirectJointMap;
    typedef boost::shared_ptr<std::map<std::string,int>> JointMapPtr;
    typedef boost::shared_ptr<std::vector<int>> DirectJointMapPtr;
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
        pjointmap->resize(probot->GetDOF());
        unsigned int dof=0;
        FOREACH(it,*pjointmap) {
            name=probot->GetJointFromDOFIndex(dof)->GetName();
            if (name2jmc.find(name) == name2jmc.end()){
                //Failed to find "standard" name, check for common mistakes:
                //Ugly special cases due to name changes
                //FIXME: without safety checks on names...
                if (name == "LKP" || name == "RKP") {
                    name[2]='N';
                }
                else if (name == "LEP" || name == "REP") {
                    name[2]='B';
                }
                else if (name == "HPY" || name == "TSY") {
                    name = "WST";
                }
                else if (name == "HNY") {
                    name="NKY";
                }
                else if (name == "HNR") {
                    name="NK1";
                }
                else if (name == "HNP") {
                    name="NK2";
                }
                else if (name == "leftIndexKnuckle1") {
                    name="LF2";
                }
                else if (name == "leftMiddleKnuckle1") {
                    name="LF3";
                }
                else if (name == "leftRingKnuckle1") {
                    name="LF4";
                }
                else if (name == "leftPinkyKnuckle1") {
                    name="LF5";
                }
                else if (name == "leftThumbKnuckle1") {
                    name="LF1";
                }
                else if (name == "rightIndexKnuckle1") {
                    name="RF2";
                }
                else if (name == "rightMiddleKnuckle1") {
                    name="RF3";
                }
                else if (name == "rightRingKnuckle1") {
                    name="RF4";
                }
                else if (name == "rightPinkyKnuckle1") {
                    name="RF5";
                }
                else if (name == "rightThumbKnuckle1") {
                    name="RF1";
                }
            }
            if (name2jmc.find(name) != name2jmc.end()){
                std::string oldname=probot->GetJointFromDOFIndex(dof)->GetName();
                RAVELOG_DEBUG("Joint %s mapped to %s\n",oldname.c_str(),name.c_str());
                *it=name2jmc[name];
            }
            else {
                *it=-1;
                RAVELOG_WARN("Joint %s not found!\n",name.c_str());
            }
            ++dof;
        }
        return pjointmap;
    };

}
