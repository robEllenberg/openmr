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
#ifndef OPENMR_SINOSCONTROLLER_H
#define OPENMR_SINOSCONTROLLER_H

#include <math.h>

class SinosController : public ControllerBase
{
 public:
    SinosController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
        __description = "Sinusoidal oscillator controller by Juan Gonzalez-Gomez";
    }
    virtual ~SinosController() {}

    virtual bool Init(RobotBasePtr robot, const std::string& args)
    {
        _probot = robot;

        //-- Initilialization of the servocontroller
        _pservocontroller = GetEnv()->CreateController("servocontroller");
        _pservocontroller->Init(_probot,"");

	cout << "OPENMR: Sinoscontroller" << endl;

        Reset(0);
        return true;
    }

    virtual void Reset(int options)
    {
        //_ref_pos.resize(_probot->GetDOF());
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
        //std::vector<dReal> error(_probot->GetDOF());
        stringstream is;
        stringstream os;

        is << "setpos ";


        //-- Set the joints velocities
        //_pservocontroller->SendCommand(os,is);

        return true;
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        //-- Set position command. The joint angles are received in degrees
        if( cmd == "setpos" ) {
            /*
            for(size_t i = 0; i < _ref_pos.size(); ++i) {
                dReal pos;
                is >> pos;

                //-- Store the reference positions in radians
                //_ref_pos[i]=pos*PI/180;

                if( !is )
                    return false;
            }
            */
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
    ControllerBasePtr _pservocontroller;

};

#endif
