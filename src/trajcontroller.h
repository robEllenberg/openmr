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
#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include <math.h>

class TrajectoryController : public ControllerBase
{
    public:
        TrajectoryController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
        __description = "Trajectory controller based on Sinusoidal oscillator by Juan Gonzalez-Gomez";
    }
        virtual ~TrajectoryController() {}

        virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
        {
            _probot = robot;
            _dofindices = dofindices;
            _nControlTransformation = nControlTransformation;

            //-- Initilialization of the servocontroller
            _pservocontroller = RaveCreateController(GetEnv(),"servocontroller"); 
            _pservocontroller->Init(_probot,_dofindices, nControlTransformation);

            _ref_pos.resize(_probot->GetDOF());
            //Assume Affine DOF + time index gives 8 additional datapoints
            _pose.resize(_probot->GetDOF()+7+1);
            //TODO: deal with velocity groups properly

            RAVELOG_DEBUG("Trajectory Controller initialized\n");

            Reset(0);
            return true;
        }

        virtual void Reset(int options)
        {
            _samplingtics=0;
            _timestep=.01; //100 Hz default
            _cycletime=0;

            _running=false;
            _complete=false;

            _time=0.0;
            _runtime=0.0;

            for (int i=0; i<_probot->GetDOF(); i++) {
                //Initialize references to default
                _ref_pos[i]=0;

            }
            RAVELOG_INFO("Trajectory Controller Reset!\n");
        }

        virtual const std::vector<int>& GetControlDOFIndices() const { return _dofindices; }
        virtual int IsControlTransformation() const { return _nControlTransformation; }
        virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
        { 
            return _pservocontroller->SetDesired(values,trans); 
        }

        virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
        {
            //TODO: Error checking, for now assume a new trajectory means
            //completely trash everything.
            if (ptraj != NULL){
                Reset(0);
                _traj = ptraj;
                _spec = ptraj->GetConfigurationSpecification(); //Redundant?
                std::vector<dReal> waypoint;
                dReal dt=0;

                //KLUDGE: Extract total runtime so we know when sampling is complete.
                for (size_t i = 0; i<_traj->GetNumWaypoints();++i){
                    _traj->GetWaypoint(i,waypoint);
                    _itdata=waypoint.begin();
                    _spec.ExtractDeltaTime(dt,_itdata);
                    _runtime+=dt;
                }
                //Assume it's good for now
                return true;
            }
            return false;
        }

        virtual void SimulationStep(dReal fTimeElapsed)
        {
            _time+=fTimeElapsed;
            //RAVELOG_VERBOSE("Elapsed time is %f\n",_time);
            //-- Update the servos
            _pservocontroller->SimulationStep(fTimeElapsed);

            if (_time>_runtime) {
                //TODO: should be some kind of interlock here...
                _complete=true;
                _running=false;
            }

            if (_running && !_complete) {

                // Find the closest number of simulation steps needed to
                // approximate the desired timestep
                _samplingperiod = round(_timestep/fTimeElapsed);
                _samplingtics ++;
                //RAVELOG_VERBOSE("Estimated %d sim steps per trajectory sample\n",_samplingperiod);

                if (_samplingtics >= _samplingperiod) {
                    _samplingtics=0;

                    //-- Calculate the next positions
                    SetRefPos();
                }
            }

        }

        virtual bool SendCommand(std::ostream& os, std::istream& is)
        {
            string cmd;
            is >> cmd;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            //-- Set position command. The joint angles are received in degrees
            if ( cmd == "setperiod" ) {
                is >> _timestep;
                _samplingperiod=_timestep;
                return true;
            }
            else if ( cmd == "run" ) {
                string mode;
                is >> mode;

                if (mode=="on") _running=true;
                else _running=false;
                return true;
            } 
            else if ( cmd == "record_on" || cmd == "record_off") {
                stringstream is2;
                //Pass stream through to servocontroller directly

                is2 << cmd << is.rdbuf();
                _pservocontroller->SendCommand(os,is2);  
            }
            return true;
        }

        virtual bool IsDone()
        {
            return _complete;
        }
        virtual dReal GetTime() const
        {
            return _time;
        }
        virtual RobotBasePtr GetRobot() const { return _probot; }

    private:
        //-- Calculate the reference position and send to the servos
        void SetRefPos() 
        { 
            //TODO: use config specs to optionally control only a subset of the joints?
            // Sample a trajectory step and assign it to the current pose
            _traj->Sample(_pose,_time);

            _itdata=_pose.begin();
            _itref=_ref_pos.begin();

            //Extract all joint values from the current pose and store as the new reference
            _spec.ExtractJointValues(_itref,_itdata,_probot,_dofindices);
            RAVELOG_DEBUG("Setting Ref, sample time is %f\n",_time);
            RAVELOG_DEBUG("Reference position %d is %f\n",7,_ref_pos[7]);

            //-- Set the new servos reference positions
            _pservocontroller->SetDesired(_ref_pos);
        }

    protected:

        RobotBasePtr _probot;
        std::vector<int> _dofindices;
        int _nControlTransformation;
        //Low-level controller to track reference positions
        ControllerBasePtr _pservocontroller;
        int _samplingtics;
        int _samplingperiod;
        dReal _cycletime;

        bool _running;        
        bool _complete; // Trajectory complete flag
        //TODO: make enumerated run states

        /** Timestep for trajectory sampling */
        dReal _timestep;           

        dReal _time;
        dReal _runtime;
        std::vector<dReal> _ref_pos;   //-- Reference positions for the servos (in degrees)

        TrajectoryBaseConstPtr _traj; //Loaded trajectory from input command
        std::vector<dReal> _pose; // complete current timeslice of a loaded trajectory
        ConfigurationSpecification _spec; //COnfiguration spec for the trajectgory (not sure we need this)
        std::vector<dReal>::iterator _itdata; //Iterator for ConfigurationSpecification to extract data with.
        std::vector<dReal>::iterator _itref; //Iterator for ConfigurationSpecification to extract data with.

};

#endif
