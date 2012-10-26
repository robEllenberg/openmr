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
#include <fstream>

//-- Time vector
typedef std::vector<dReal> tvector;

class ServoController : public ControllerBase
{
    public:
        ServoController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
        //TODO: Add a way to set gains per joint to match physical behavior
        __description = "Servo controller by Juan Gonzalez-Gomez and Rosen Diankov";
        RegisterCommand("test",boost::bind(&ServoController::Test,this,_1,_2),
                "Command for testing and debugging");
        RegisterCommand("set",boost::bind(&ServoController::SetProperties,this,_1,_2),
                "Format: set property value(s)\n Use this command to set controller properties such as gains and individual servo reference positions. Note that this command is independent of the legacy commands");
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
        RegisterCommand("print",boost::bind(&ServoController::PrintProperties,this,_1,_2),
                "Return controller properties as string.");

    }
        virtual ~ServoController() {}

        virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
        {
            _probot = robot;
            _dofindices = dofindices;
            _nControlTransformation = nControlTransformation;

            //-- Initialization of the odevelocity controller
            _pvelocitycontroller = RaveCreateController(GetEnv(),"odevelocity"); 
            _pvelocitycontroller->Init(_probot,_dofindices, nControlTransformation);

            //-- Get the robot joints. Are needed in every simulation step for reading the
            //-- joint angles and maxvelocities
            std::vector<KinBodyPtr> bodies;
            GetEnv()->GetBodies(bodies);
            _joints = bodies[0]->GetJoints();

            //-- Initialize the Recording mode
            _recording=false;

            //-- Initialize the vector for the recording mode
            _phi_tvec.resize(_joints.size());
            _ref_tvec.resize(_joints.size());

            //Updated to standard RAVE logging function
            RAVELOG_DEBUG("servocontroller initialized\n");

            Reset(0);

            return true;
        }

        virtual const std::vector<int>& GetControlDOFIndices() const { return _dofindices; }

        virtual int IsControlTransformation() const { return _nControlTransformation; }

        virtual void Reset(int options)
        {
            //-- Initially, the reference positions should be set to the joints position
            //-- in order for the servos to stay in the initial position
            _ref_pos.resize(_probot->GetDOF());
            _parsed_pos.resize(_probot->GetDOF());
            _error.resize(_probot->GetDOF());
            _errSum.resize(_probot->GetDOF());
            _dError.resize(_probot->GetDOF());
            fillVector(_KP,8.3,_probot->GetDOF());
            fillVector(_KI,0,_probot->GetDOF());
            fillVector(_KD,0,_probot->GetDOF());

            std::vector<dReal> angle;
            for (size_t i=0; i<_joints.size(); i++) {
                //TODO: Use iterators?
                _joints[i]->GetValues(angle);
                _ref_pos[i]=angle[0];
                _error[i]=0.0;
                _errSum[i]=0.0;
                _dError[i]=0.0;
                _parsed_pos[i]=0.0;
                _KP[i]=8.3;
                _KI[i]=0.0;
                _KD[i]=0.0;
            }

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
            std::vector<dReal> lower(_ref_pos.size());
            std::vector<dReal> upper(_ref_pos.size());
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
                //than 2*_limitpad.  Shouldn't be an issue, but future releases
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
            //TODO: Why would this happen? should this be a more graceful failure?
            assert(fTimeElapsed > 0.0);

            const size_t dof=_probot->GetDOF();
            std::vector<dReal> angles(dof);
            std::vector<dReal> lasterror(dof);
            std::vector<dReal> cmdvelocities(dof);

            _probot->GetDOFValues(angles);

            dReal error,derror,maxvel,rawcmd,satcmd;
            //RAVELOG_DEBUG("fTimeElapsed %f\n",fTimeElapsed);

            for (size_t i=0; i<dof; i++) {

                error = _ref_pos[i] - angles[i];

                // find dError / dt and low-pass filter the data with hard-coded alpha
                derror = (error - _error[i])/fTimeElapsed*_Ka+_dError[i]*(1-_Ka);

                // Calculate decaying integration
                _errSum[i] = error*fTimeElapsed + _errSum[i]*_Kf;

                rawcmd = error*_KP[i] + derror*_KD[i] +  _errSum[i]*_KI[i]; 

                //-- Limit the cmdvelocities to its maximum
                maxvel = _joints[i]->GetMaxVel();
                if (rawcmd > maxvel) satcmd = maxvel;
                else if (rawcmd < -maxvel) satcmd = -maxvel;
                else satcmd=rawcmd;
                cmdvelocities[i]=satcmd;


                //TODO: windup protection, integral saturation, derivative filtering and saturation

                //-- Store the current sample (only in recording mode)

                // Update error history with new scratch value
                _error[i] = error-(rawcmd-satcmd)*fTimeElapsed/_KP[i];
                _dError[i] = derror;

            }

            //Check for record flag and copy DOF values into storage if necessary
            if (_recording) {
                for (size_t i=0; i<dof; i++) {
                    _phi_tvec[i].push_back(angles[i]);
                    _ref_tvec[i].push_back(_ref_pos[i]);
                }
            }

            // Assign desired joint velocities
            _pvelocitycontroller->SetDesired(cmdvelocities);

        }

        //-- Just a command test for debugging...
        bool Test(std::ostream& os, std::istream& is)
        {
            cout<<"Test..." << endl;

            return true;
        }

        /**
         * Set a variety of controller properties via the text interface.
         * The "set" command via this function can control the servo's gains,
         * units, and other misc. parameters.
         */
        bool SetProperties(std::ostream& os, std::istream& is)
        {
            dReal temp;
            string cmd2;
            bool flag;
            stringstream is2;
            while (is){
                is >> cmd2;
                //Note: old gain-setting interface
                if ( cmd2 == "gains") {
                    //Pass stream through to setgains command
                    is2 << "set" << cmd2 << " " << is.rdbuf();
                    return SetGains(os,is2);  
                }
                else if (cmd2 == "gainvec" || cmd2 == "gainvector")
                {
                    is2 <<  is.rdbuf();
                    return SetIndividualGains(os,is2);
                }
                else if (cmd2 == "radians" || cmd2 == "radian") _inradians=true;
                else if (cmd2 == "degrees" || cmd2 == "degree") _inradians=false;
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
            return SetDesired(_parsed_pos,temp);
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

            return SetServoReference(servo,pos);
            //RAVELOG_DEBUG("Limits %f,%f, Input %f, Servo %d Position: %f\n",lower[0],upper[0],pos, servo,_ref_pos[servo]);
        }

        /** Common function to set a single servo reference */
        bool SetServoReference(int servo,dReal &pos)
        {
            //TODO: possible poor performance here.
            std::vector<dReal> lower(1);
            std::vector<dReal> upper(1);
            _probot->GetJointFromDOFIndex(servo)->GetLimits(lower,upper);

            //-- Store the reference position in radians
            pos=pos*GetInputScale();
            if ((lower[0]+_limitpad)>pos) _ref_pos[servo]=lower[0]+_limitpad;
            else if ((upper[0]-_limitpad)<pos) _ref_pos[servo]=upper[0]-_limitpad;
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
                //Kuldgy packet structure
                is >> curjoint;
                is >> kp;
                is >> ki;
                is >> kd;
            
                if (curjoint < 0 || curjoint > _probot->GetDOF())
                    return false;
                else if (curjoint == _probot->GetDOF())
                {
                    RAVELOG_DEBUG("Setting all joint gains...\n");
                    for (size_t i = 0; i < _probot->GetDOF();++i)
                    {
                        //Assign all joints instead of just one
                        if ( kp>=0) _KP[i]=kp;
                        if ( ki>=0) _KI[i]=ki;
                        if ( kd>=0) _KD[i]=kd;
                    }
                }
                else{
                    if ( kp>=0) _KP[curjoint]=kp;
                    if ( ki>=0) _KI[curjoint]=ki;
                    if ( kd>=0) _KD[curjoint]=kd;
                }
            }
            return true;
        }


        /**
         * Set gains collectively (backwards compatible).
         */
        bool SetGains(std::ostream& os, std::istream& is)
        {

            // Since we may only want to set a few gains, don't freak out if the last few are not specified

            dReal kp=-1.0;
            dReal ki=-1.0;
            dReal kd=-1.0;
            string name;

            is >> kp;
            is >> ki;
            is >> kd;
            RAVELOG_DEBUG("Received gains Kp=%f,Ki=%f,Kd=%f\n",kp,ki,kd);
            for (size_t i = 0; i < _probot->GetDOF();++i)
            {
                //Assign all joints instead of just one
                if ( kp>=0) _KP[i]=kp;
                if ( ki>=0) _KI[i]=ki;
                if ( kd>=0) _KD[i]=kd;
            }
            //TODO: Assign filter coefs
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
            for(size_t i = 0; i < _ref_pos.size(); ++i) {

                //-- Get the current joint angle of the i'th servo
                _joints[i]->GetValues(angle);
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
            _joints[servo]->GetValues(angle);
            os << angle[0]/GetInputScale() << " ";

            return true;
        }

        bool PrintProperties(std::ostream& os, std::istream& is)
        {
            FOREACH(it,_dofindices)
            {
                os << "Gains, joint " << *it <<": " << _KP[*it] << " " << _KI[*it] << " " << _KD[*it] << "\n";
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
            for (size_t i=0; i<_joints.size(); i++) {
                _phi_tvec[i].resize(0);
                _ref_tvec[i].resize(0);
            }
            //Remove file definition here

            string file;
            if (is >> file){
                outFile.open(file.c_str());
            }

            _recording=true;

            return true;
        }

        /**
         * Disable recording and save to data file.
         * This command specifies the filename, and which of the saved joints to
         * write out to the file.
         */
        bool RecordOff(std::ostream& os, std::istream& is)
        {
            //-- Write the information in the output file
            //-- Open the file if the record_on command has not opened it already
            string file;
            if (!outFile.is_open() && is >> file) outFile.open(file.c_str());

            size_t startDOF = 0;
            size_t stopDOF = _phi_tvec.size()-1;

            // If only 1 parameter is passed in, make both start and stop equal
            is >> startDOF >> stopDOF;

            RAVELOG_VERBOSE("stopDOF: %d \n",stopDOF);
            RAVELOG_VERBOSE("startDOF: %d \n",startDOF);

            RAVELOG_INFO("Writing servo data %d to %d in data file: %s \n",startDOF,stopDOF,file.c_str());

            generate_octave_file(startDOF,stopDOF);

            //-- Close the file
            outFile.close();

            RAVELOG_INFO("RECORD off, max velocity: %f \n",_joints[0]->GetMaxVel());
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
        bool getFromStream(std::istream& is, dReal &K, const dReal& min, const dReal& max, char name[])
        {
            dReal k;
            if (is >> k) {
                if (k >= min && k <= max) {
                    K = k;
                    RAVELOG_VERBOSE("%s is now: %f\n",name,K);
                    return true;
                }
                else RAVELOG_ERROR("%s %f is out of range, ignoring...\n",name,k);
            } else RAVELOG_VERBOSE("%s not read",name);
            return false;
        }

        void generate_octave_file()
        {
            //Export all servo data by default
            generate_octave_file(0,_phi_tvec.size());
        }


        /**
         * Export servo data to a txt file by row.
         * The first column contains the name of the data field, and subsequent columns the data. 
         * Currently, there are no time-indexes available, but it will be exported in a future release.
         */
        void generate_octave_file(size_t startDOF, size_t stopDOF)
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
