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
        __description = "Servo controller by Juan Gonzalez-Gomez and Rosen Diankov";
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


        cout << "Servocontroller: INIT" << endl;

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
        std::vector<dReal> angle;
        for (size_t i=0; i<_joints.size(); i++) {
            _joints[i]->GetValues(angle);
            _ref_pos[i]=angle[0];
        }

        //-- Default value of the Proportional controller KP constant
        _KP=8.3;

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

            //-- Store the current sample (only in recording mode)
            if (_recording) {
              _phi_tvec[i].push_back(angle[0]);
              _ref_tvec[i].push_back(_ref_pos[i]);
            }
        }

        //-- Set the joints velocities
        _pvelocitycontroller->SendCommand(os,is);
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        //-- Set position command.
        //-- Set the position of all the joints
        // The joint angles are received in degrees
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
        //-- Set the position of 1 joint
        //-- The format is: servo angle (in degrees)
        //-- Servo number starts from 0
        else if ( cmd == "setpos1" ) {

            int servo;
            dReal pos;
            is >> servo;
            is >> pos;


            //-- TODO: Check errors!!!!!
            //-- Store the reference positions in radians
            _ref_pos[servo]=pos*PI/180;
            return true;
        }
        //-- Get the position of the servo
        else if ( cmd == "getpos1") {
          int servo;
          std::vector<dReal> angle;
          is >> servo;

          //-- Get the current joint angle
          _joints[servo]->GetValues(angle);
          os << angle[0]*180/PI << " ";

          //-- Just for debugging...
          //cout << "Angle: " << angle[0]*180/PI << endl;
        }

        else if ( cmd == "record_on" ) {
            string file;
            is >> file;

            //-- Open the file
            outFile.open(file.c_str());

            //-- Seting the recording mode
            _recording=true;

            cout << "RECORD on:" << file << "\n";
 
            return true;
        }
        else if ( cmd == "record_off" ) {

            //-- Write the information in the output file
            generate_octave_file();

            //-- Close the file
            outFile.close();

            _recording=false;

            cout << "RECORD off\n";
            cout << "Max vel: " << _joints[0]->GetMaxVel() << endl;
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


    //-- Add the legents
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
    dReal _KP;                    //-- P controller KP constant

    //-- For recording....
    ofstream outFile;                 //-- Stream file for storing the servo positions
    bool _recording;                  //-- Recording mode
    std::vector<tvector> _phi_tvec;     //-- Servo's angles in time
    std::vector<tvector> _ref_tvec;     //-- Servo's reference positions in time

   


};

#endif
