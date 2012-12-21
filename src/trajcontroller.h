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
#ifndef OPENRAVE_TRAJECTORY_CONTROLLER_H
#define OPENRAVE_TRAJECTORY_CONTROLLER_H

#include <math.h>

class TrajectoryController : public ControllerBase
{
    public:
        TrajectoryController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
        __description = "Trajectory controller based on Sinusoidal oscillator by Juan Gonzalez-Gomez";
        RegisterCommand("set",boost::bind(&TrajectoryController::SetProperties,this,_1,_2),"Format: set property value\n Use this command to set miscellaneous properties, such as the interpolation direction (forward / reverse), trajectory recording, etc.");
        RegisterCommand("load",boost::bind(&TrajectoryController::DeserializeTrajectory,this,_1,_2),"Format: load <serialized trajectory>\n Pass in an entire serialized trajectory and load it into the controller.");
        RegisterCommand("start",boost::bind(&TrajectoryController::StartController,this,_1,_2),"Format: start\n Start sampling and executing the loaded trajectory.");
        RegisterCommand("stop",boost::bind(&TrajectoryController::StopController,this,_1,_2),"Format: stop\n Stop a running controller (may eventually include a time reset).");
        RegisterCommand("pause",boost::bind(&TrajectoryController::PauseController,this,_1,_2),"Format: pause\n Pause a running controller, preserving the trajectory state.");

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
            _forward=true;
            _softstart=0;

            _time=0.0;
            _runtime=0.0;
            _traj.reset();
            //NOTE: Smart pointers should demalloc when appropriate here
            boost::static_pointer_cast<ServoController>(_pservocontroller)->SetRadians();

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
            if (values.size()<_ref_pos.size()){
                RAVELOG_ERROR("Insufficient # of values provided\n");
                return false;
            }

            FOREACH(it,_dofindices){
                //TODO: check for bad references here
                _ref_pos[*it]=values[*it];
            }
            return _pservocontroller->SetDesired(values,trans); 
        }

        virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
        {
            //TODO: Error checking, for now assume a new trajectory means
            //completely trash everything.
            if ( !!ptraj){
                Reset(0);
                _traj = ptraj;
                return SetupTrajectory();
            }
            RAVELOG_WARN("No trajectory provided, ignoring...\n");
            return false;
        }

        virtual void SimulationStep(dReal fTimeElapsed)
        {
            //RAVELOG_VERBOSE("Elapsed time is %f\n",_time);

            if (_time>=_runtime || _time < 0.0) {
                //TODO: should be some kind of interlock here...
                _complete=true;
                _running=false;
            }

            if (_running && !_complete) {
                _forward ? _time+=fTimeElapsed : _time-=fTimeElapsed;

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

            _pservocontroller->SimulationStep(fTimeElapsed);

        }

        bool SetProperties(std::ostream& os, std::istream& is)
        {
            dReal temp;
            string cmd2;
            while (is){
                is >> cmd2;
                if ( cmd2 == "timestep") {
                    is >> temp;
                    if (temp > 0) {
                        _timestep=temp;
                    }
                    else RAVELOG_WARN("Timestep %f out of range, ignoring\n",_timestep);
                }
                else if ( cmd2 == "softstart" ) {
                    is >> temp;
                    //NOTE: This setting does not do anything yet
                    _softstart=temp;
                    RAVELOG_WARN("Softstart is not currently implemented...\n");
                }
                else if ( cmd2 == "forward" )  _forward=true;
                else if ( cmd2 == "reverse" )  _forward=false;
                //Pass any other commands through to the servocontroller
                else {
                    std::stringstream is2;

                    //Pass stream through to servocontroller directly
                    is2 << "set" << " " << cmd2 << " " << is.rdbuf();
                    _pservocontroller->SendCommand(os,is2);  
                    return true;
                }
            }
            return true;
        }

        bool DeserializeTrajectory(std::ostream& os, std::istream& is){

            Reset(0);

            TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(), "");

            ptraj->deserialize(is);

            _traj=ptraj;
            return SetupTrajectory();
        }

        bool StartController(std::ostream& os, std::istream& is){
            if ( !!_traj) {
                RAVELOG_DEBUG("Starting trajectorycontroller at sim time %f...\n",_probot->GetEnv()->GetSimulationTime());
                _running=true;
                return true;
            }
            RAVELOG_WARN("Trajectory not defined, ignoring...\n");
            return false;
        } 

        bool StopController(std::ostream& os, std::istream& is){
            //TODO: Address thread safety? probably not significant here.
            RAVELOG_DEBUG("Stopping trajectorycontroller at local time %f...\n",_time);
            _running=false;
            //TODO: Differentiate stop and pause in the future
            return true;
        } 

        bool PauseController(std::ostream& os, std::istream& is){
            _running=false;
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

        bool SetupTrajectory() {
            // TODO Add locks with timeout to prevent multiple trajectory assignments.
            std::vector<dReal> waypoint;
            dReal dt=0;
            //TODO: add initial pose as softstart?
            _spec = _traj->GetConfigurationSpecification(); //Redundant?
            //KLUDGE: Extract total runtime so we know when sampling is complete.
            for (size_t i = 0; i<_traj->GetNumWaypoints();++i){
                _traj->GetWaypoint(i,waypoint);
                _itdata=waypoint.begin();
                _spec.ExtractDeltaTime(dt,_itdata);
                _runtime+=dt;
            }
            RAVELOG_DEBUG("Runtime is %f",_runtime);

            //Extract DOF's used
            ConfigurationSpecification::Group jointvals=_spec.GetGroupFromName("joint_values");
            stringstream data;
            data << jointvals.name;
            RAVELOG_DEBUG(jointvals.name+"\n");
            string name,type;
            data >> type >> name;

            //Resize pose and index vectors to the new trajectory
            //make _pose the largest reasonable sample size until we can figure
            //out how to size it based on the config spec.
            _pose.resize(_probot->GetDOF()*3+7+1);
            _trajindices.resize(jointvals.dof);
            _activejointvalues.resize(jointvals.dof);

            //Read out the joint
            FOREACH(it,_trajindices){
                data >> *it;
                //RAVELOG_DEBUG("DOF Index: %d\n",*it);
            }

            if (_runtime>0 && _traj->GetNumWaypoints()>0)
            {
                RAVELOG_DEBUG("Found %d waypoints, runtime is %f.\n",_traj->GetNumWaypoints(),_runtime);
                return true;
            }
            else {
                RAVELOG_ERROR("Trajectory not usable, aborting...\n");
                Reset(0);
                return false;
            }

        }

        inline const char * const BoolToString(bool b)
        {
            return b ? "true" : "false";
        }

        //-- Calculate the reference position and send to the servos
        void SetRefPos() 
        { 
            _traj->Sample(_pose,_time);

            _itdata=_pose.begin();
            _itref=_activejointvalues.begin();

            //Extract all joint values from the current pose and store as the new reference
            //Note that not all DOF need to be specified in the trajectory here.
            _spec.ExtractJointValues(_itref,_itdata,_probot,_trajindices);

            size_t dof=0;
            FOREACH(it, _trajindices){
                //Update only controlled DOF
                _ref_pos[*it]=_activejointvalues[dof++];
            }
            //if (_time<.02){
            //FOREACH(itsample,_activejointvalues)
            //{
            //RAVELOG_DEBUG("REF at %f is %f\n",_time,*itsample);
            //}
            //}

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
        bool _forward;        
        bool _complete; // Trajectory complete flag
        dReal _softstart;

        //TODO: make enumerated run states

        /** Timestep for trajectory sampling */
        dReal _timestep;           

        dReal _time;
        dReal _runtime;

        std::vector<dReal> _ref_pos;   //-- Reference positions for the servos (in degrees)

        TrajectoryBaseConstPtr _traj; //Loaded trajectory from input command
        std::vector<dReal> _pose; // complete current timeslice of a loaded trajectory
        std::vector<dReal> _activejointvalues; // Set of joint values of trajectory active dof
        std::vector<int> _trajindices; // Set of joint values of trajectory active dof
        ConfigurationSpecification _spec; //COnfiguration spec for the trajectgory (not sure we need this)
        std::vector<dReal>::iterator _itdata; //Iterator for ConfigurationSpecification to extract data with.
        std::vector<dReal>::iterator _itref; //Iterator for ConfigurationSpecification to extract data with.

};

#endif
