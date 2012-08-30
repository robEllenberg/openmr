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
        RegisterCommand("Test",boost::bind(&ServoController::Test,this,_1,_2),
                "Command for testing and debugging");
        RegisterCommand("Setpos",boost::bind(&ServoController::SetPos,this,_1,_2),
                "Format: Setpos s1 [s2]. Set the reference position of all the robot joints, in degrees, in the range [-90,90]. If the robot have N joints, there have to be N arguments");
        RegisterCommand("Setpos1",boost::bind(&ServoController::SetPos1,this,_1,_2),
                "Format: Setpos1 servo pos. Set the reference position of one joint. The argument servo is the servo number, starting from 0. The argument pos is the reference position (in degrees) [-90,90] ");
        RegisterCommand("Setgains",boost::bind(&ServoController::SetGains,this,_1,_2),
                "Format: Setgains kp kd ki kf. Set the feedback control gains for the PID loop.");
        RegisterCommand("Getpos",boost::bind(&ServoController::GetPos,this,_1,_2),
                "Format: Getpos. Get the position of ALL the servos (in degrees)");
        RegisterCommand("Getpos1",boost::bind(&ServoController::GetPos1,this,_1,_2),
                "Format: Getpos servo. Returns the current servo position (in degrees, in the range [-90,90]. The argument servo is the servo number, starting from 0");
        RegisterCommand("Record_on",boost::bind(&ServoController::RecordOn,this,_1,_2),
                "Format: Record_on file. Start recording the servo position in the specified file. It will generate an octave/matlab file ");
        RegisterCommand("Record_off",boost::bind(&ServoController::RecordOff,this,_1,_2),
                "Format: Record_off. Stop recording. The octave/matlab file is generated ");

    }
    virtual ~ServoController() {}

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
        _dofindices = dofindices;
        _nControlTransformation = nControlTransformation;

        //-- Initilialization of the odevelocity controller
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
        std::vector<dReal> angle;
        for (size_t i=0; i<_joints.size(); i++) {
            _joints[i]->GetValues(angle);
            _ref_pos[i]=angle[0];
            _error[i]=0.0;
            _errSum[i]=0.0;
        }

        //-- Default value of the Proportional controller KP constant
        _KP=8.3;
        _KD=0;
        _KI=0;
        _Kf=.95;

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

            //TODO: (low) Fix this to handle joint DOF varieties properly...mix of standards here?
            //-- Get the current joint angle
            _joints[i]->GetValues(angle);

            //-- Calculate the distance to the reference position (error)
            //-- and the desired velocity
            //TODO: Incorporate angle diff function here to allow arbitrary input angles
            error = angle[0] - _ref_pos[i];

            //TODO: Why would this happen? should this be a more graceful failure?
            assert(fTimeElapsed > 0.0);

            // find dError / dt
            // Attempt to smooth out noise in time elapsed?
            derror = (error - _error[i])/fTimeElapsed;

            // Calculate decaying integration
            _errSum[i] = error*fTimeElapsed + _errSum[i]*(1-(1-_Kf)*fTimeElapsed);

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
      for(size_t i = 0; i < _ref_pos.size(); ++i) {
        dReal pos;
        is >> pos;

        if( !is )
          return false;

        //-- Store the reference positions in radians
        _ref_pos[i]=pos*PI/180;
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


      //-- TODO: Check errors!!!!!
      //-- Store the reference positions in radians
      _ref_pos[servo]=pos*PI/180;
           
      return true;
    }

    
    /**
     * Set controller gains.
     * Kludgy input string of PD gains (set D to 0 and KP to 8.3 for default behavior)
     */
    bool SetGains(std::ostream& os, std::istream& is)
    {
      dReal kp;
      dReal kd;
      dReal ki;
      dReal kf;

      is >> kp;
      is >> ki;
      is >> kd;
      is >> kf;

      if (kp >= 0.0) {
          _KP = kp;
          RAVELOG_VERBOSE("Kp Gain is now: %f\n",_KP);
      }
      else RAVELOG_ERROR("Kp Gain %f is out of range, ignoring...\n",kp);

      _KD = kd;
      RAVELOG_VERBOSE("Kd Gain is now: %f\n",_KD);

      _KI = ki;
      RAVELOG_VERBOSE("Ki Gain is now: %f\n",_KI);

      _Kf = kf;
      RAVELOG_VERBOSE("Kf Gain is now: %f\n",_Kf);

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

        //-- Get the current joint angle of the ith servo
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
      string file;
      is >> file;

       //-- Reset the data vectors
       for (size_t i=0; i<_joints.size(); i++) {
          _phi_tvec[i].resize(0);
          _ref_tvec[i].resize(0);
       }

       //-- Open the file
       outFile.open(file.c_str());

       //-- Seting the recording mode
        _recording=true;

        cout << "RECORD on:" << file << "\n";
 
      return true;
    }

    /*******************************************************/
    /* Stop recording. The octave file will be generated   */
    /*******************************************************/
    bool RecordOff(std::ostream& os, std::istream& is)
    {
      //-- Write the information in the output file
      generate_octave_file();

      //-- Close the file
      outFile.close();

      _recording=false;
      cout << "RECORD off\n";
      cout << "Max vel: " << _joints[0]->GetMaxVel() << endl;
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

  void generate_octave_file(void)
  {
    size_t size = _phi_tvec[0].size();
    cout << "Size: " << size << endl;

    //-- Servos angle
    for (size_t s=0; s<_phi_tvec.size(); s++) {
      outFile << "phi" << s <<"=[";
      for (size_t t=0; t<size; t++) {
        outFile << _phi_tvec[s][t]*180/PI << ",";
      }
      outFile << "];" << endl;
    }

    //-- Reference positions
    for (size_t s=0; s<_ref_tvec.size(); s++) {
      outFile << "ref" << s <<"=[";
      for (size_t t=0; t<size; t++) {
        outFile << _ref_tvec[s][t]*180/PI << ",";
      }
      outFile << "];" << endl;
    }

    //-- Time
    outFile << "t=[0:1:" << size-1 << "];" << endl;

    //-- Plot the servo angles
    outFile << "plot(";
    for (size_t s=0; s<_phi_tvec.size(); s++) {
      outFile << "t,phi" << s << ",'-'";

      //-- Add a ',' except for the last element
      if (s<_phi_tvec.size()-1)
        outFile << ",";
    }
    outFile << ");" << endl;

    //-- Plot the reference positions
    outFile << "hold on;";
    outFile << "plot(";
    for (size_t s=0; s<_ref_tvec.size(); s++) {
      outFile << "t,ref" << s << ",'-'";

      //-- Add a ',' except for the last element
      if (s<_ref_tvec.size()-1)
        outFile << ",";
    }
    outFile << ");" << endl;


    //-- Add the legends
    outFile << "legend(";
    for (size_t s=0; s<_phi_tvec.size(); s++) {
      outFile << "'Servo " << s << "'";

      //-- Add a ',' except for the last element
      if (s<_phi_tvec.size()-1)
        outFile << ",";
    }
    outFile << ");" << endl;


    outFile << "grid on;" << endl;
    outFile << "title('Servos angle')" << endl;
    outFile << "xlabel('Simulation time')" << endl;
    outFile << "ylabel('Angle (degrees)')" << endl;
    outFile << "axis([0," << size-1 << ",-90, 90])" << endl;
    outFile << "pause;" << endl;
  }

protected:
    RobotBasePtr _probot;
    std::vector<int> _dofindices;
    int _nControlTransformation;

    ControllerBasePtr _pvelocitycontroller;
    std::vector<KinBody::JointPtr> _joints;
    std::vector<dReal> _ref_pos;  //-- Reference positions (in radians)
    std::vector<dReal> _error;  //-- Reference positions (in radians)
    std::vector<dReal> _errSum;  //-- Reference positions (in radians)
    dReal _KP;                    //-- P controller KP constant
    dReal _KI;
    dReal _KD;
    dReal _Kf;                    // -- "Forgetting" constant of integrator

    //-- For recording....
    ofstream outFile;                 //-- Stream file for storing the servo positions
    bool _recording;                  //-- Recording mode
    std::vector<tvector> _phi_tvec;     //-- Servo's angles in time
    std::vector<tvector> _ref_tvec;     //-- Servo's reference positions in time

};

#endif
