// Copyright (C) 2010 Juan Gonzalez-Gomez (juan@iearobotics.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef OPENRAVE_MODULAR_ROBOTS_CONTROLLERS_H
#define OPENRAVE_MODULAR_ROBOTS_CONTROLLERS_H

#include <math.h>

class ServoController : public ControllerBase
{
 public:
    ServoController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
        __description = "Servo controller by Juan Gonzalez-Gomez and Rosen Diankov";
    }
    virtual ~ServoController() {}

    virtual bool Init(RobotBasePtr robot, const std::string& args)
    {
        _probot = robot;

        //-- Initilialization of the odevelocity controller
        _pvelocitycontroller = GetEnv()->CreateController("odevelocity");
        _pvelocitycontroller->Init(_probot,"");

        //-- Get the robot joints. Are needed in every simulation step for reading the
        //-- joint angles and maxvelocities
        std::vector<KinBodyPtr> bodies;
        GetEnv()->GetBodies(bodies);
        _joints = bodies[0]->GetJoints();

        Reset(0);
        return true;
    }

    virtual void Reset(int options)
    {
        //-- Initially, the reference positions should be set to the joints position
        //-- in order for the servos to stay in the initial position
        _ref_pos.resize(_probot->GetDOF());
        std::vector<dReal> angle;
        for (size_t i=0; i<_joints.size(); i++) {
            _joints[i]->GetValues(angle);
            _ref_pos[i]=angle[0];
        }

        //-- Default value of the Proportional controller KP constant
        _KP=8.3;

    }

    virtual bool SetDesired(const std::vector<dReal>& values)
    {
        Reset(0);
        return false;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        Reset(0);
        return false;
    }

    virtual bool SimulationStep(dReal fTimeElapsed)
    {
        std::vector<dReal> angle;
        std::vector<dReal> error(_probot->GetDOF());
        std::vector<dReal> velocity(_probot->GetDOF());
        stringstream is;
        stringstream os;

        is << "setvelocity ";

        //-- K controller for all the joints
        for (size_t i=0; i<_joints.size(); i++) {

            //-- Get the current joint angle
            _joints[i]->GetValues(angle);

            //-- Calculate the distance to the reference position (error)
            //-- and the desired velocity
            error[i] = angle[0] - _ref_pos[i];
            velocity[i] = -error[i]*_KP;

            //-- Limit the velocity to its maximum
            dReal Maxvel = _joints[i]->GetMaxVel();
            if (velocity[i] > Maxvel) velocity[i] = Maxvel;
            if (velocity[i] < -Maxvel) velocity[i] = -Maxvel;

            is << velocity[i] << " ";
        }

        //-- Set the joints velocities
        _pvelocitycontroller->SendCommand(os,is);

        return true;
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        //-- Set position command. The joint angles are received in degrees
        if( cmd == "setpos" ) {
            for(size_t i = 0; i < _ref_pos.size(); ++i) {
                dReal pos;
                is >> pos;

                //-- Store the reference positions in radians
                _ref_pos[i]=pos*PI/180;

                if( !is )
                    return false;
            }
            return true;
        }

        return true;
    }

    virtual bool IsDone()
    {
        return false;
    }
    virtual dReal GetTime() const
    {
        return 0;
    }
    virtual RobotBasePtr GetRobot() const { return _probot; }

protected:
    RobotBasePtr _probot;
    ControllerBasePtr _pvelocitycontroller;
    std::vector<KinBody::JointPtr> _joints;
    std::vector<dReal> _ref_pos;  //-- Reference positions (in radians)
    dReal _KP;                    //-- P controller KP constant

};

#endif
