/*
Copyright (c) 2012, Robert W. Ellenberg
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may 
      be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef OPENRAVE_MIMIC_CONTROLLER_H
#define OPENRAVE_MIMIC_CONTROLLER_H

#include <math.h>
#include <fstream>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/utils.h>

static const dReal g_fEpsilonJointLimit = RavePow(g_fEpsilon,0.8);

class MimicController : public ControllerBase
{
public:
    MimicController(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv), cmdid(0), _bPause(false), _bIsDone(true), _bCheckCollision(false), _bThrowExceptions(false)
    {
        __description = ":Interface Author: Rosen Diankov\n\nIdeal controller used for planning and non-physics simulations. Forces exact robot positions.\n\n\
If \ref ControllerBase::SetPath is called and the trajectory finishes, then the controller will continue to set the trajectory's final joint values and transformation until one of three things happens:\n\n\
2. ControllerBase::SetDesired is called.\n\n\
3. ControllerBase::Reset is called resetting everything\n\n\
If SetDesired is called, only joint values will be set at every timestep leaving the transformation alone.\n";
        RegisterCommand("Pause",boost::bind(&MimicController::_Pause,this,_1,_2),
                        "pauses the controller from reacting to commands ");
        RegisterCommand("SetCheckCollisions",boost::bind(&MimicController::_SetCheckCollisions,this,_1,_2),
                        "If set, will check if the robot gets into a collision during movement");
        RegisterCommand("SetFollowRobot",boost::bind(&MimicController::_SetFollowRobot,this,_1,_2),
                        "Supply the name of a robot to follow its motions (should be the same type of robot!");
        _fCommandTime = 0;
        _nControlTransformation = 1;
    }
    virtual ~MimicController() {
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
        _psrcrobot.reset();
        if( flog.is_open() ) {
            flog.close();
        }
        if( !!_probot ) {
            string filename = RaveGetHomeDirectory() + string("/") + _probot->GetName() + string(".traj.xml");
            flog.open(filename.c_str());
            if( !flog ) {
                RAVELOG_WARN(str(boost::format("failed to open %s\n")%filename));
            }
            //flog << "<" << GetXMLId() << " robot=\"" << _probot->GetName() << "\"/>" << endl;
            _dofindices = dofindices;
            _nControlTransformation = nControlTransformation;
            _dofcircular.resize(0);
            FOREACH(it,_dofindices) {
                KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(*it);
                _dofcircular.push_back(pjoint->IsCircular(*it-pjoint->GetDOFIndex()));
            }
            _cblimits = _probot->RegisterChangeCallback(KinBody::Prop_JointLimits|KinBody::Prop_JointAccelerationVelocityTorqueLimits,boost::bind(&MimicController::_SetJointLimits,boost::bind(&utils::sptr_from<MimicController>, weak_controller())));
            _SetJointLimits();

            if( _dofindices.size() > 0 ) {
                _gjointvalues.reset(new ConfigurationSpecification::Group());
                _gjointvalues->offset = 0;
                _gjointvalues->dof = _dofindices.size();
                stringstream ss;
                ss << "joint_values " << _probot->GetName();
                FOREACHC(it, _dofindices) {
                    ss << " " << *it;
                }
                _gjointvalues->name = ss.str();
            }
            if( nControlTransformation ) {
                _gtransform.reset(new ConfigurationSpecification::Group());
                _gtransform->offset = _probot->GetDOF();
                _gtransform->dof = RaveGetAffineDOF(DOF_Transform);
                _gtransform->name = str(boost::format("affine_transform %s %d")%_probot->GetName()%DOF_Transform);
            }
        }
        _bPause = false;
        
        return true;
    }

    virtual void Reset(int options)
    {
        _vecdesired.resize(0);
        _bIsDone = true;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const {
        return _dofindices;
    }
    virtual int IsControlTransformation() const {
        return _nControlTransformation;
    }

    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
    {
        //KLUDGE: "Cheat", assumes that we might have a visualization robot
        //with fewer DOF than the original, but otherwise the same format. The
        //right way to do this iterating over each robot's joints to match up
        //the names, but that's much more expensive.
        if( values.size() < _dofindices.size() ) {
            throw openrave_exception(str(boost::format("wrong desired dimensions %d!=%d")%values.size()%_dofindices.size()),ORE_InvalidArguments);
        }
        if( !_bPause ) {
            EnvironmentMutex::scoped_lock lockenv(_probot->GetEnv()->GetMutex());
            _vecdesired = values;
            if( !!trans ) {
                _tdesired = *trans;
            }
            else if (!!_psrcrobot) {
                _tdesired = _psrcrobot->GetTransform();
            }
            else  {
                _tdesired = _probot->GetTransform();
            }
            _SetDOFValues(_vecdesired,_tdesired,0);
        }
        return true;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        return false;
    }

    virtual void SimulationStep(dReal fTimeElapsed)
    {
    }

    virtual bool IsDone() {
        return _bIsDone;
    }
    virtual dReal GetTime() const {
        return _fCommandTime;
    }
    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }


private:

    virtual bool _Pause(std::ostream& os, std::istream& is)
    {
        is >> _bPause;
        return !!is;
    }
    virtual bool _SetCheckCollisions(std::ostream& os, std::istream& is)
    {
        is >> _bCheckCollision;
        if( _bCheckCollision ) {
            _report.reset(new CollisionReport());
        }
        return !!is;
    }
    virtual bool _SetFollowRobot(std::ostream& os, std::istream& is)
    {
        string name;
        is >> name;
        _psrcrobot=_probot->GetEnv()->GetRobot(name);
        return !!is;
    }
    virtual bool _SetThrowExceptions(std::ostream& os, std::istream& is)
    {
        is >> _bThrowExceptions;
        return !!is;
    }

    inline boost::shared_ptr<MimicController> shared_controller() {
        return boost::dynamic_pointer_cast<MimicController>(shared_from_this());
    }
    inline boost::shared_ptr<MimicController const> shared_controller_const() const {
        return boost::dynamic_pointer_cast<MimicController const>(shared_from_this());
    }
    inline boost::weak_ptr<MimicController> weak_controller() {
        return shared_controller();
    }

    virtual void _SetJointLimits()
    {
        if( !!_probot ) {
            _probot->GetDOFLimits(_vlower[0],_vupper[0]);
            _probot->GetDOFVelocityLimits(_vupper[1]);
            _probot->GetDOFAccelerationLimits(_vupper[2]);
        }
    }


    virtual void _SetDOFValues(const std::vector<dReal>&values, dReal timeelapsed)
    {
        vector<dReal> prevvalues, curvalues;
        _probot->GetDOFValues(prevvalues);
        curvalues = prevvalues;
        int i =0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
        }
        _probot->SetDOFValues(curvalues,true);
    }

    virtual void _SetDOFValues(const std::vector<dReal>&values, const Transform &t, dReal timeelapsed)
    {
        BOOST_ASSERT(_nControlTransformation);
        vector<dReal> prevvalues, curvalues;
        _probot->GetDOFValues(prevvalues);
        curvalues = prevvalues;
        int i =0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
        }
        _probot->SetDOFValues(curvalues,t, true);
    }

    RobotBasePtr _probot;               ///< controlled body
    RobotBasePtr _psrcrobot;               ///< robot source of ref poses

    dReal _fCommandTime;

    std::vector<dReal> _vecdesired;         ///< desired values of the joints
    Transform _tdesired;

    std::vector<int> _dofindices;
    std::vector<uint8_t> _dofcircular;
    boost::array< std::vector<dReal>, 3> _vlower, _vupper; ///< position, velocity, acceleration limits
    int _nControlTransformation;
    ofstream flog;
    int cmdid;
    bool _bPause, _bIsDone, _bCheckCollision, _bThrowExceptions;
    CollisionReportPtr _report;
    UserDataPtr _cblimits;
    boost::shared_ptr<ConfigurationSpecification::Group> _gjointvalues, _gtransform;
    boost::mutex _mutex;
};

ControllerBasePtr CreateMimicController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new MimicController(penv,sinput));
}
#endif
