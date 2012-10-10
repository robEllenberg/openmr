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
        RegisterCommand("setpos",boost::bind(&ServoController::SetPos,this,_1,_2),
                "Format: setpos s1 s2...sN\n Set the reference position of all the robot joints, in degrees, in the range [-90,90]. If the robot have N joints, there have to be N arguments");
        RegisterCommand("setpos1",boost::bind(&ServoController::SetPos1,this,_1,_2),
                "Format: setpos1 servo# pos\n Set the reference position of one joint. The first argument servo is the servo number, starting from 0. The argument pos is the reference position (in degrees) [-90,90] ");
        RegisterCommand("setgains",boost::bind(&ServoController::SetGains,this,_1,_2),
                "Format: setgains Kp [Ki] [Kd] [Kf] [Ka]. Set gains for the PID controller. Kp, Ki, and Kd are independent gains (may change in a future release. Kf is a decay constant for the integrator (0 = no decay, 1 = instant decay), and Ka is the first order filter coeficient for the error rate (1 = no filtering, Ka -> 0 gives less filtering).");
        RegisterCommand("getpos",boost::bind(&ServoController::GetPos,this,_1,_2),
                "Format: getpos. Get the position of ALL the servos (in degrees)");
        RegisterCommand("getpos1",boost::bind(&ServoController::GetPos1,this,_1,_2),
                "Format: getpos servo. Returns the current servo position (in degrees). The argument servo is the servo number, starting from 0");
        RegisterCommand("record_on",boost::bind(&ServoController::RecordOn,this,_1,_2),
                "Format: record_on . Start recording the servo positions and references to memory");
        RegisterCommand("record_off",boost::bind(&ServoController::RecordOff,this,_1,_2),
                "Format: record_off filename [startDOF stopDOF]. Stop recording and generate octave/matlab file of specified results. Can be run multiple times to export different servos. ");

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
        _error.resize(_probot->GetDOF());
        _errSum.resize(_probot->GetDOF());
        _dError.resize(_probot->GetDOF());
        std::vector<dReal> angle;
        for (size_t i=0; i<_joints.size(); i++) {
            //TODO: Use iterators?
            _joints[i]->GetValues(angle);
            _ref_pos[i]=angle[0];
            _error[i]=0.0;
            _errSum[i]=0.0;
            _dError[i]=0.0;
        }

        //-- Default value of the Proportional controller KP constant from old OpenMR
        // This should be backwards compatible with old code
        _KP=8.3;
        _KD=0;
        _KI=0;
        _Kf=.1;
        _Ka=.01;
        _limitpad=.03;

    }

    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans) { return false; }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        Reset(0);
        return false;
    }

    virtual void SimulationStep(dReal fTimeElapsed)
    {
        std::vector<dReal> angle;
        int dof=_probot->GetDOF();
        //std::vector<dReal> error(dof);
        std::vector<dReal> lasterror(dof);
        std::vector<dReal> velocity(dof);
        stringstream is;
        stringstream os;

        dReal error,derror;
        //RAVELOG_DEBUG("fTimeElapsed %f\n",fTimeElapsed);

        //Copy code from odecontroller.h to get all DOF velocities. For now,
        //gamble that velocities are "constant" enough that an environment lock
        //isn't necessary.
        //int dofindex = 0;
        //std::vector<OpenRAVE::dReal> valldofvelocities;
        //_probot->GetDOFVelocities(valldofvelocities);

        is << "setvelocity ";

        //-- K controller for all the joints
        for (size_t i=0; i<_joints.size(); i++) {
            
            //Kr is the opposit of Kf, which controls %remaining over time. kr=.9 -> e=.9 after 1 second
            // Assume fTimeElapsed << 1 sec.
            dReal kr = (1-_Kf*fTimeElapsed);
            //TODO: (low) Fix this to handle joint DOF varieties properly...mix of standards here?
            //-- Get the current joint angle
            _joints[i]->GetValues(angle);

            //-- Calculate the distance to the reference position (error)
            //-- and the desired velocity
            error = angle[0] - _ref_pos[i];

            //TODO: Why would this happen? should this be a more graceful failure?
            assert(fTimeElapsed > 0.0);

            // find dError / dt and low-pass filter the data with hard-coded alpha
            derror = (error - _error[i])/fTimeElapsed*_Ka+_dError[i]*(1-_Ka);

            // Calculate decaying integration
            _errSum[i] = error*fTimeElapsed + _errSum[i]*kr;

            velocity[i] = -error*_KP - derror*_KD - _errSum[i]*_KI; 

            //-- Limit the velocity to its maximum
            dReal Maxvel = _joints[i]->GetMaxVel();
            if (velocity[i] > Maxvel) velocity[i] = Maxvel;
            if (velocity[i] < -Maxvel) velocity[i] = -Maxvel;

            is << velocity[i] << " ";

            //-- Store the current sample (only in recording mode)
            if (_recording) {
              _phi_tvec[i].push_back(angle[0]);
              _ref_tvec[i].push_back(_ref_pos[i]);
            }

            // Update error history with new scratch value
            _error[i] = error;
            _dError[i] = derror;

        }

        //-- Set the joints velocities
        _pvelocitycontroller->SendCommand(os,is);
        
    }

    //-- Just a command test for debugging...
    bool Test(std::ostream& os, std::istream& is)
    {
      cout<<"Test..." << endl;

      return true;
    }

    /*****************************************************************/
    /* SetPos command. Set the reference position of ALL the joints  */
    /* The joint angles are in degreees                              */
    /*****************************************************************/
    bool SetPos(std::ostream& os, std::istream& is)
    {
        //Get all the joint limits from the robot

        std::vector<dReal> lower(_ref_pos.size());
        std::vector<dReal> upper(_ref_pos.size());
        _probot->GetDOFLimits(lower,upper);

        for(size_t i = 0; i < _ref_pos.size(); ++i) {
            dReal pos;
            is >> pos;

            if( !is )
                return false;

            //-- Store the reference positions in radians
            pos=pos*PI/180.0;

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

    /*********************************************************/
    //-- Set the position of 1 joint
    //-- The format is: servo angle (in degrees)
    //-- Servo number starts from 0
    /*********************************************************/
    bool SetPos1(std::ostream& os, std::istream& is)
    {
        int servo;
        dReal pos;
        is >> servo;
        is >> pos;

        //TODO: Make this work for multiple DOF joints
        std::vector<dReal> lower(1);
        std::vector<dReal> upper(1);
        _probot->GetJointFromDOFIndex(servo)->GetLimits(lower,upper);

        //-- Store the reference position in radians
        pos=pos*PI/180.0;
        if ((lower[0]+_limitpad)>pos) _ref_pos[servo]=lower[0]+_limitpad;
        else if ((upper[0]-_limitpad)<pos) _ref_pos[servo]=upper[0]-_limitpad;
        else _ref_pos[servo]=pos;

        //RAVELOG_DEBUG("Limits %f,%f, Input %f, Servo %d Position: %f\n",lower[0],upper[0],pos, servo,_ref_pos[servo]);

        return true;
    }

    
    /**
     * Set controller gains.
     * Kludgy input string of PD gains (set D to 0 and KP to 8.3 for default behavior)
     */
    bool SetGains(std::ostream& os, std::istream& is)
    {

        // Since we may only want to set a few gains, don't freak out if the last few are not specified
        getFromStream(is,_KP,0.0,10000.0,"Porportional Gain Kp");
        getFromStream(is,_KI,0.0,10000.0,"Integral Gain Ki");
        getFromStream(is,_KD,0.0,10000.0,"Derivative Gain Kd");
        getFromStream(is,_Kf,0.0,10000.0,"Integrator Decay Kf");
        getFromStream(is,_Ka,0.0,10000.0,"Differentiator Decay Ka");

        //This function doesn't "fail" exactly, so return true for now... 
        return true;
    }


    /*********************************************************/
    //-- Get the position of ALL the servos (in degrees)
    /*********************************************************/
    bool GetPos(std::ostream& os, std::istream& is)
    {
      std::vector<dReal> angle;
      for(size_t i = 0; i < _ref_pos.size(); ++i) {

        //-- Get the current joint angle of the i'th servo
       _joints[i]->GetValues(angle);
       os << angle[0]*180/PI << " ";
      }
      return true;
    }

    /*********************************************************/
    //-- Get the position of one servo
    //-- The format is: servo angle (in degrees)
    //-- Servo number starts from 0
    /*********************************************************/
    bool GetPos1(std::ostream& os, std::istream& is)
    {
      int servo;
      std::vector<dReal> angle;
      is >> servo;

       //-- Get the current joint angle
       _joints[servo]->GetValues(angle);
       os << angle[0]*180/PI << " ";

       //-- Just for debugging...
       //cout << "Angle: " << angle[0]*180/PI << endl;
      return true;
    }

    /**************************************************************/
    /* Start recording the servo position in the specified file.  */
    /* It will generate an octave file                            */
    /**************************************************************/
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

    /*******************************************************/
    /* Stop recording. The octave file will be generated   */
    /*******************************************************/
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

        RAVELOG_INFO("Writing servo data %d to %d in octave file: %s \n",startDOF,stopDOF,file.c_str());

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

        outFile << "Kp " << _KP << " Ki " << _KI << " Kd " << _KD << " Kf " << _Kf << " Ka " << _Ka << endl ;

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
    std::vector<dReal> _error;    // Current tracking error  
    std::vector<dReal> _dError;   // Tracking error rate
    std::vector<dReal> _errSum;   // tracking error sum (decaying)
    dReal _KP;                    //-- P controller KP constant
    dReal _KI;
    dReal _KD;
    dReal _Kf;                    // -- "Forgetting" constant of integrator
    dReal _Ka;                    // -- first order filter for derivative
    dReal _limitpad;             // a global soft limit padding to prevent overshoot across joint limits
                                 // This is a bandaid fix...

    //-- For recording....
    ofstream outFile;                 //-- Stream file for storing the servo positions
    bool _recording;                  //-- Recording mode
    std::vector<tvector> _phi_tvec;     //-- Servo's angles in time
    std::vector<tvector> _ref_tvec;     //-- Servo's reference positions in time

};

#endif
