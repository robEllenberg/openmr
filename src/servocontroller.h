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
#ifndef OPENRAVE_SERVOCONTROLLER_H
#define OPENRAVE_SERVOCONTROLLER_H

#include <math.h>
#include <fstream>

//-- Time vector
typedef std::vector<dReal> tvector;

class ServoController : public ControllerBase
{
    public:
        ServoController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
        __description = "Servo controller by Juan Gonzalez-Gomez and Rosen Diankov, overhauled by Robert Ellenberg";
        RegisterCommand("set",boost::bind(&ServoController::SetProperties,this,_1,_2),
                "Format: set property value(s)\n Use this command to set controller properties such as gains and individual servo reference positions. Note that this command is independent of the legacy commands");
        RegisterCommand("get",boost::bind(&ServoController::GetProperties,this,_1,_2),
                "Format: get property\n Use this command to get the current value of a controller property such as gains.");
        RegisterCommand("setpos",boost::bind(&ServoController::SetPos,this,_1,_2),
                "Format: setpos s1 s2...sN\n Set the reference position of all the robot joints. If the robot has N joints, there have to be N arguments");
        RegisterCommand("setpos1",boost::bind(&ServoController::SetPos1,this,_1,_2),
                "Format: setpos1 servo# pos\n Set the reference position of one joint. The first argument servo is the servo number, starting from 0. The argument pos is the reference position.");
        RegisterCommand("setgains",boost::bind(&ServoController::SetGains,this,_1,_2),
                "Format: setgains Kp [Ki] [Kd] [Kf] [Ka]. Set gains for the PID controller. Kp, Ki, and Kd are constant for all joints (may change in a future release. Kf is a decay constant for the integrator (0 = no decay, 1 = instant decay), and Ka is the first order filter coeficient for the error rate (1 = no filtering, Ka -> 0 gives less filtering).");
        RegisterCommand("getpos",boost::bind(&ServoController::GetPos,this,_1,_2),
                "Format: getpos. Get the position of ALL the servos (in degrees)");
        RegisterCommand("getpos1",boost::bind(&ServoController::GetPos1,this,_1,_2),
                "Format: getpos servo. Returns the current servo position (in degrees). The argument servo is the servo number, starting from 0");
        RegisterCommand("record_on",boost::bind(&ServoController::RecordOn,this,_1,_2),
                "Format: record_on . Start recording the servo positions and references to memory");
        RegisterCommand("record_off",boost::bind(&ServoController::RecordOff,this,_1,_2),
                "Format: record_off filename [startDOF stopDOF]. Stop recording and generate octave/matlab file of specified results. Can be run multiple times to export different servos. ");
        RegisterCommand("print",boost::bind(&ServoController::GetAllProperties,this,_1,_2),
                "Return controller properties as string.");

    }
        virtual ~ServoController() {}

        virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
        {
            _probot = robot;
            _dofindices = dofindices;
            _nControlTransformation = nControlTransformation;

            //-- Initialization of the odevelocity controller
#ifdef USE_CUSTOM_ODE_PLUGIN
            _pvelocitycontroller = RaveCreateController(GetEnv(),"odevelocity_rob"); 
#else
            _pvelocitycontroller = RaveCreateController(GetEnv(),"odevelocity"); 
#endif
            _pvelocitycontroller->Init(_probot,_dofindices, nControlTransformation);

            //-- Get the robot joints. Are needed in every simulation step for reading the
            //-- joint angles and maxvelocities
            //std::vector<KinBodyPtr> bodies;
            //GetEnv()->GetBodies(bodies);
            //FIXME: Currently joints are only updated on Init. Changing a robot's structure online may cause trouble.
            _joints = (boost::static_pointer_cast<KinBody>(_probot))->GetJoints();

            //-- Initialize the Recording mode
            _recording=false;

            //-- Initialize the vector for the recording mode
            _phi_tvec.resize(_dofindices.size());
            _ref_tvec.resize(_dofindices.size());

            //Updated to standard RAVE logging function
            RAVELOG_DEBUG("servocontroller initialized, controlling %d joints\n",_dofindices.size());

            Reset(0);

            return true;
        }

        virtual const std::vector<int>& GetControlDOFIndices() const { return _dofindices; }

        virtual int IsControlTransformation() const { return _nControlTransformation; }

        virtual void Reset(int options)
        {
            //-- Initially, the reference positions should be set to the joints position
            //-- in order for the servos to stay in the initial position
            _angle.resize(_dofindices.size());
            _velocity.resize(_dofindices.size());
            _ref_pos.resize(_dofindices.size());
            _parsed_pos.resize(_dofindices.size());
            _error.resize(_dofindices.size());
            _errSum.resize(_dofindices.size());
            _dError.resize(_dofindices.size());
            _KP.resize(_dofindices.size());
            _KI.resize(_dofindices.size());
            _KD.resize(_dofindices.size());

            //Fill all vectors with default values
            std::fill(_KP.begin(),_KP.end(),8.3);
            std::fill(_KI.begin(),_KI.end(),0);
            std::fill(_KD.begin(),_KD.end(),0);
            std::fill(_ref_pos.begin(),_ref_pos.end(),0);
            std::fill(_error.begin(),_error.end(),0);
            std::fill(_errSum.begin(),_errSum.end(),0);
            std::fill(_dError.begin(),_dError.end(),0);
            std::fill(_parsed_pos.begin(),_parsed_pos.end(),0);

            //-- Default value of the Proportional controller KP constant from old OpenMR
            // This should be backwards compatible with old code
            _Kf=.9998;
            _Ka=.1;
            _limitpad=.03;

        }

        void SetRadians(){
            _inradians = true;
        }

        inline dReal GetInputScale(){
            return _inradians ? 1.0 : PI/180 ;
        }

        virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans) 
        { 
            //Get all the joint limits from the robot
            std::vector<dReal> lower(_dofindices.size());
            std::vector<dReal> upper(_dofindices.size());
            _probot->GetDOFLimits(lower,upper);

            if ( values.size() < _ref_pos.size())
            {
                RAVELOG_WARN("Not enough values, %d < %d, ignoring...\n",values.size(),_ref_pos.size());
                return false;
            }

            dReal pos;
            for(size_t i = 0; i < _ref_pos.size(); ++i) {

                //CHANGE: All commands are in radians now
                pos=values[i]*GetInputScale();

                //TODO obviously this will not work for joints with a ROM smaller
                //than 2*_limitpad. Shouldn't be an issue, but future releases
                //will fix it.
                if ((lower[i]+_limitpad)>pos) _ref_pos[i]=lower[i]+_limitpad;
                else if ((upper[i]-_limitpad)<pos) _ref_pos[i]=upper[i]-_limitpad;
                else _ref_pos[i]=pos;
                //RAVELOG_DEBUG("Servo %d Position: %f\n",i,_ref_pos[i]);
            }

            return true;

        }

        virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
        {
            Reset(0);
            return false;
        }

        virtual void SimulationStep(dReal fTimeElapsed)
        {
            assert(fTimeElapsed > 0.0);

            const size_t dof=_probot->GetDOF();

            dReal error,derror,maxvel,rawcmd,satcmd;
            //RAVELOG_DEBUG("fTimeElapsed %f\n",fTimeElapsed);

            std::vector<dReal> lower(100,0);
            std::vector<dReal> upper(100,0);
            
            //Check all values by DOF to eliminate issues with multi-DOF joints
            _probot->GetDOFValues(_angle);
            _probot->GetDOFVelocityLimits(lower,upper,_dofindices);

            for (size_t i=0; i<_dofindices.size(); ++i) {

                //TODO: (low) Fix this to handle joint DOF varieties properly
                // Potential slowdown due to dynamic resizing of array?
                //TODO: Add error saturation to limit windup
                error = _ref_pos[i] - _angle[i];

                // find dError / dt and low-pass filter the data with hard-coded alpha
                derror = (error - _error[i])/fTimeElapsed*_Ka+_dError[i]*(1-_Ka);

                // Calculate decaying integration
                _errSum[i] = error*fTimeElapsed + _errSum[i]*_Kf;

                rawcmd = error*_KP[i] + derror*_KD[i] + _errSum[i]*_KI[i]; 

                if (rawcmd > upper[i]) satcmd = upper[i];
                else if (rawcmd < lower[i]) satcmd = lower[i];
                else satcmd=rawcmd;
                _velocity[i]=satcmd;

                // Update error history with new scratch value
                _error[i] = error;
                _dError[i] = derror;

            }

            //Check for record flag and copy DOF values into storage if necessary
            if (_recording) {
                for (size_t i=0; i<dof; i++) {
                    _phi_tvec[i].push_back(_angle[i]);
                    _ref_tvec[i].push_back(_ref_pos[i]);
                }
            }

            // Assign desired joint velocities
            _pvelocitycontroller->SetDesired(_velocity);

        }

        /**
         * Set a variety of controller properties via the text interface.
         * The "set" command via this function can control the servo's gains,
         * units, and other misc. parameters.
         */
        bool SetProperties(std::ostream& os, std::istream& is)
        {
            string cmd2;
            std::stringstream is2;
            while (is.good()){
                is >> cmd2;
                //Note: old gain-setting interface
                if ( cmd2 == "gains") {
                    //Pass stream through to setgains command
                    os << SetGains(os,is);  
                }
                else if (cmd2 == "gainvec" || cmd2 == "gainvector")
                {
                    is2 << is.rdbuf();
                    SetIndividualGains(os,is2);
                }
                else if (cmd2 == "filter")
                {
                    os << _GetFromStream(is,_Ka,0,1,"Derivative Filter Constant");
                }
                else if (cmd2 == "decay")
                {
                    os << _GetFromStream(is,_Kf,0,1,"Integrator Decay Constant");
                }
                else if (cmd2 == "radians" || cmd2 == "radian") _inradians=true;
                else if (cmd2 == "degrees" || cmd2 == "degree") _inradians=false;
            }
            //TODO: decide on return vs os for error reporting
            os << true;
            return true;
        }

        /**
         * Get a variety of controller properties via the text interface.
         * The "get" command via this function can report the servo's gains,
         * units, and other misc. parameters.
         */
        bool GetProperties(std::ostream& os, std::istream& is)
        {
            string cmd2;
            is >> cmd2;
            if (is.fail()) return false;
            //Note: old gain-setting interface
            else if (cmd2 == "filter")
            {
                os << _Ka;
            }
            else if (cmd2 == "decay")
            {
                os << _Kf;
            }
            else if (cmd2 == "units" ) 
                os << (_inradians ? "radians" : "degrees");
            else {
                RAVELOG_WARN("Command not recognized");
                return false;
            }
            return true;
        }

        /**
         * Set positions of all joints.
         * Read in joint angles in the current controller unit system, and assign
         * as the current position reference.
         */
        bool SetPos(std::ostream& os, std::istream& is)
        {
            //Get all the joint limits from the robot

            FOREACH(it,_parsed_pos){
                is >> *it;
                if( !is ) return false;
            }
            //Consolidated input validation to one function
            TransformConstPtr temp;
            os <<  SetDesired(_parsed_pos,temp);
            return true;
        }

        /**
         * Set the position of 1 joint.
         * Command format: "servo# angle", where the servo# is zero-indexed.
         * Make sure your units match the controller's units
         */
        bool SetPos1(std::ostream& os, std::istream& is)
        {
            int servo;
            dReal pos;
            is >> servo;
            is >> pos;

            os << SetServoReference(servo,pos);
            return true;
            //RAVELOG_DEBUG("Limits %f,%f, Input %f, Servo %d Position: %f\n",lower[0],upper[0],pos, servo,_ref_pos[servo]);
        }

        /** Common function to set a single servo reference */
        bool SetServoReference(int servo,dReal &pos)
        {
            std::vector<dReal> lower(3);
            std::vector<dReal> upper(3);
            _probot->GetJointFromDOFIndex(servo)->GetLimits(lower,upper);

            //-- Store the reference position in radians
            pos=pos*GetInputScale();
            //TODO: handle multi-dof joints
            if ((lower[0]+_limitpad)>pos) {
                _ref_pos[servo]=lower[0]+_limitpad;
                RAVELOG_WARN("Command position %f exceeds limit of %f\n",pos,_ref_pos[servo]);
            }
            else if ((upper[0]-_limitpad)<pos) {
                _ref_pos[servo]=upper[0]-_limitpad;
                RAVELOG_WARN("Command position %f exceeds limit of %f\n",pos,_ref_pos[servo]);
            }
            else _ref_pos[servo]=pos;

            return true;
        }

        /**
         * Set controller gains individually.
         * Set PID and filter gains from input string.
         * "set gainvec joint# kp ki kd joint# kp ki kd
         * Note that invalid or omitted values will be ignored and issue a warning.
         */
        bool SetIndividualGains(std::ostream& os, std::istream& is)
        {

            int curjoint=-1;
            dReal kp=-1.0,kd=-1.0,ki=-1.0;
            string name;

            while (!!is)
            {
                //Kludgy packet structure
                is >> curjoint;
                is >> kp;
                is >> ki;
                is >> kd;

                if (is.fail()) {
                    RAVELOG_WARN("Format error after joint %d",curjoint);
                    return false;
                }

                if (curjoint < 0 || curjoint > _probot->GetDOF()) {
                    RAVELOG_WARN("Input joint %d out of range",curjoint);
                    return false;
                }
                else if (curjoint == _probot->GetDOF())
                {
                    RAVELOG_DEBUG("Setting all joint gains...\n");
                    for (size_t i = 0; i < (size_t)_probot->GetDOF();++i)
                    {
                        //Assign all joints instead of just one
                        if ( kp>=0.0) _KP[i]=kp;
                        if ( ki>=0.0) _KI[i]=ki;
                        if ( kd>=0.0) _KD[i]=kd;
                    }
                }
                else{
                    if ( kp>=0.0) _KP[curjoint]=kp;
                    if ( ki>=0.0) _KI[curjoint]=ki;
                    if ( kd>=0.0) _KD[curjoint]=kd;
                }
            }
            return true;
        }


        /**
         * Set gains collectively (backwards compatible).
         */
        bool SetGains(std::ostream& os, std::istream& is)
        {

            dReal kp=-1.0;
            dReal ki=-1.0;
            dReal kd=-1.0;
            string name;

            is >> kp;
            is >> ki;
            is >> kd;
            RAVELOG_DEBUG("Received gains Kp=%f,Ki=%f,Kd=%f\n",kp,ki,kd);
            for (size_t i = 0; i < (size_t)_probot->GetDOF();++i)
            {
                //Assign all joints instead of just one
                if ( kp>=0) _KP[i]=kp;
                if ( ki>=0) _KI[i]=ki;
                if ( kd>=0) _KD[i]=kd;
            }
            //TODO: Assign filter coefs
            os << true;
            return true;
        }

        /**
         * Get positions of all servos in current units.
         * Based on the unit system, return a serialized list of all servo
         * positions in the current unit system.
         */
        bool GetPos(std::ostream& os, std::istream& is)
        {
            std::vector<dReal> angle;

            _probot->GetDOFValues(angle,_dofindices);
            for(size_t i = 0; i < _ref_pos.size(); ++i) {
                //-- Get the current joint angle of the i'th servo
                os << angle[0]/GetInputScale() << " ";
            }
            return true;
        }

        /**
         * Get the position of one servo.
         * The data string is formatted as: "servo# angle"
         */
        bool GetPos1(std::ostream& os, std::istream& is)
        {
            int servo;
            std::vector<dReal> angle;
            is >> servo;

            //-- Get the current joint angle
            std::vector<int> index (1,0);
            index[0]=servo;
            _probot->GetDOFValues(angle,index);
            os << angle[0]/GetInputScale() << " ";

            return true;
        }

        bool GetAllProperties(std::ostream& os, std::istream& is)
        {
            FOREACH(it,_dofindices)
            {
                os << "Gains, joint " << _probot->GetJointFromDOFIndex(*it)->GetName() <<
                    " (" << *it <<"): " << _KP[*it] << " " << _KI[*it] << " " << _KD[*it] << "\n";
            }
            os << "Filter Gains: "  << _Kf << " " << _Ka << "\n";
            os << "Units: " << (_inradians ? "radians" : "degrees") << "\n";
            return true;
        }

        /**
         * Start recording servo data to memory.
         * Begin logging all commanded and actual servo positions at each timestep
         * to RAM.  Avoid recording for long periods, as this may slow down the
         * simulation over time.
         */
        bool RecordOn(std::ostream& os, std::istream& is)
        {

            //-- Reset the data vectors
            for (size_t i=0; i<_dofindices.size(); i++) {
                _phi_tvec[i].resize(0);
                _ref_tvec[i].resize(0);
            }
            //Remove file definition here

            string file;
            if (is >> file){
                outFile.open(file.c_str());
            }

            _recording=true;
            RAVELOG_INFO("Enabled recording\n");

            return true;
        }

        /**
         * Disable recording and save to data file.
         * This command specifies the filename, and which of the saved joints to
         * write out to the file.
         */
        bool RecordOff(std::ostream& os, std::istream& is)
        {
            _recording=false;
            //Potential interference here, since recording could still be occuring.
            string file;
            size_t startDOF = 0;
            size_t stopDOF = _phi_tvec.size()-1;

            if ( !(is >> file) ) {
                //No file provided
                if (!outFile.is_open()){
                    RAVELOG_ERROR("No destination provided!\n");
                    os << 0;
                    return false;
                }
            }

            // If only 1 parameter is passed in, make both start and stop equal
            is >> startDOF >> stopDOF;

            RAVELOG_VERBOSE("Send data to %s\n",file.c_str());
            RAVELOG_VERBOSE("stopDOF: %d\n",stopDOF);
            RAVELOG_VERBOSE("startDOF: %d\n",startDOF);

            if (file == "string"){
                //Send data directly to output string
                _SerializeRecordedData(os,startDOF,stopDOF);
            }
            else  {
                //TODO: verify that outfile can be opened
                if (!outFile.is_open()) outFile.open(file.c_str());

                if (outFile.fail()) {
                    RAVELOG_ERROR("Invalid data file %s\n",file.c_str());
                    return false;
                }

                RAVELOG_INFO("Writing servo data %d to %d in data file: %s \n",startDOF,stopDOF,file.c_str());
                _WriteDataFile(startDOF,stopDOF);

                outFile.close();
            }

            RAVELOG_INFO("RECORD off");
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

    private:
        void fillVector(std::vector<dReal>& vec,dReal val,size_t len)
        {
            vec.resize(len);
            FOREACH(it, vec){
                *it=val;
            }
        }

        void writeGains()
        {
            FOREACH(it,_dofindices)
            {
                outFile << "Joint " << *it <<": " << _KP[*it] << " " << _KI[*it] << " " << _KD[*it] << " ";
            }
            outFile << " Filter Gains: "  << _Kf << " " << _Ka << "\n";
        }

        /**
         * Get a value from a string stream.
         * This function adds a few checks to the process of extracting input values, such as validity and bounds checking.
         * Obviously this slows things down a little, so it probably shouldn't be used for realtime functions.
         */
        bool _GetFromStream(std::istream& is, dReal &K, const dReal& min, const dReal& max, string name)
        {
            dReal k;
            if (is >> k) {
                if (k >= min && k <= max) {
                    K = k;
                    RAVELOG_VERBOSE("%s is now: %f\n",name.c_str(),K);
                    return true;
                }
                else RAVELOG_ERROR("%s %f is out of range, ignoring...\n",name.c_str(),k);
            } else RAVELOG_VERBOSE("%s not read",name.c_str());
            return false;
        }

        void _WriteDataFile()
        {
            //Export all servo data by default
            _WriteDataFile(0,_phi_tvec.size());
        }


        /**
         * Export servo data to a txt file by row.
         * The first column contains the name of the data field, and subsequent columns the data. 
         * Currently, there are no time-indexes available, but it will be exported in a future release.
         */
        void _WriteDataFile(size_t startDOF, size_t stopDOF)
        {

            size_t size = _phi_tvec[0].size();
            RAVELOG_INFO("Timesteps: %d\n",size);
            //Account for the fact that stopDOF is an index and not a quantity:
            stopDOF++;
            // Servo properties (gains)

            writeGains();

            //-- Servos angle
            for (size_t s=startDOF; s<stopDOF; s++) {
                outFile << _probot->GetJointFromDOFIndex(s)->GetName() << " " ;
                for (size_t t=0; t<size; t++) {
                    outFile << _phi_tvec[s][t]*180/PI << " ";
                }
                outFile << endl;
            }

            //-- Reference positions
            for (size_t s=startDOF; s<stopDOF; s++) {
                outFile << _probot->GetJointFromDOFIndex(s)->GetName() << "_REF " ;
                for (size_t t=0; t<size; t++) {
                    outFile << _ref_tvec[s][t]*180/PI << " ";
                }
                outFile << endl;
            }
        }

        //TODO: Make the recorded data its own class with an overloaded stream operator
        void _SerializeRecordedData(std::ostream& os, size_t startDOF, size_t stopDOF)
        {

            size_t size = _phi_tvec[0].size();
            RAVELOG_INFO("Timesteps: %d\n",size);
            //Account for the fact that stopDOF is an index and not a quantity:
            stopDOF++;
            // Servo properties (gains)

            //-- Servos angle
            for (size_t s=startDOF; s<stopDOF; s++) {
                os << _probot->GetJointFromDOFIndex(s)->GetName() << " " ;
                for (size_t t=0; t<size; t++) {
                    os << _phi_tvec[s][t]*180/PI << " ";
                }
                os << endl;
            }

            //-- Reference positions
            for (size_t s=startDOF; s<stopDOF; s++) {
                os << _probot->GetJointFromDOFIndex(s)->GetName() << "_REF " ;
                for (size_t t=0; t<size; t++) {
                    os << _ref_tvec[s][t]*180/PI << " ";
                }
                os << endl;
            }
        }

    protected:
        RobotBasePtr _probot;
        std::vector<int> _dofindices;
        int _nControlTransformation;

        ControllerBasePtr _pvelocitycontroller;
        std::vector<KinBody::JointPtr> _joints;
        std::vector<dReal> _ref_pos;  // Reference positions (in radians)
        std::vector<dReal> _parsed_pos;  // Raw reference positions read from inputs
        std::vector<dReal> _error;    // Current tracking error  
        std::vector<dReal> _dError;   // Tracking error rate
        std::vector<dReal> _errSum;   // tracking error sum (decaying)
        std::vector<dReal> _angle;
        std::vector<dReal> _velocity;

        /** Controller gain vectors */
        std::vector<dReal> _KP;                    
        std::vector<dReal> _KI;
        std::vector<dReal> _KD;

        /** Filter constants for integrator and differentiator */
        dReal _Kf;                    // -- "Forgetting" constant of integrator
        dReal _Ka;                    // -- first order filter for derivative

        /** Unit system flag (radians vs degrees) */
        dReal _inradians;
        // a global soft limit padding to prevent overshoot across joint limits
        // This is a bandaid fix...
        dReal _limitpad;
        dReal _time;

        //-- For recording....
        ofstream outFile;                 //-- Stream file for storing the servo positions
        bool _recording;                  //-- Recording mode
        std::vector<tvector> _phi_tvec;     //-- Servo's angles in time
        std::vector<tvector> _ref_tvec;     //-- Servo's reference positions in time

};

#endif
