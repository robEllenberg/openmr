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

        _ref_pos.resize(_probot->GetDOF());
        _amplitude.resize(_probot->GetDOF());
        _phase0.resize(_probot->GetDOF());
        _offset.resize(_probot->GetDOF());

        cout << "OPENMR: Sinoscontroller: INIT" << endl;

        Reset(0);
        return true;
    }

    virtual void Reset(int options)
    {
        _samplingtics=0;
        _period=1.5;
        _N=20;
        _samplingperiod=_period/_N;
        _phase=0;

        for (int i=0; i<_probot->GetDOF(); i++) {
          _ref_pos[i]=0;
          _amplitude[i]=0;
          _phase0[i]=0;
          _offset[i]=0;
        }

         cout<<"Sampling period: " << _samplingperiod << endl;
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
        _pservocontroller->SimulationStep(fTimeElapsed);
        _samplingtics += fTimeElapsed;
        if (_samplingtics < _samplingperiod) return true;

        _samplingtics=0;
        _phase += 360.0/_N;

        stringstream os, is;
        is << "setpos ";

        //-- Calculate the next samples
        for (size_t i=0; i<_ref_pos.size(); i++) {
            _ref_pos[i]=_amplitude[i]*sin(_phase*PI/180 + _phase0[i]*PI/180) + _offset[i];
            is<<_ref_pos[i]<<" ";
        }

        //-- Set the new servos reference positions
        _pservocontroller->SendCommand(os,is);

        return true;
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        //-- Set position command. The joint angles are received in degrees
        if( cmd == "setamplitude" ) {

            for(size_t i = 0; i < _amplitude.size(); ++i) {
                is >> _amplitude[i];

                if( !is )
                    return false;
            }
            return true;
        }
        else if ( cmd == "setinitialphase" ) {
            for(size_t i = 0; i < _phase0.size(); ++i) {
                is >> _phase0[i];

                if( !is )
                    return false;
            }
            return true;
        }
        else if ( cmd == "setoffset" ) {
            for(size_t i = 0; i < _offset.size(); ++i) {
                is >> _offset[i];

                if( !is )
                    return false;
            }
            return true;
        }
        else if ( cmd == "setperiod" ) {
            is >> _period;
            _samplingperiod=_period/_N;

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
    dReal _samplingtics;
    dReal _samplingperiod;
    int _N;                   //-- Number of samples
    dReal _period;            //-- Oscilation period in seconds
    dReal _phase;
    std::vector<dReal> _ref_pos;   //-- Reference positions for the servos (in degrees)
    std::vector<dReal> _amplitude; //-- Oscillation amplitudes
    std::vector<dReal> _phase0;    //-- Oscillation initial phase
    std::vector<dReal> _offset;    //-- Oscillation offset

};

#endif
