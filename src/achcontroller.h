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

#ifndef OPENRAVE_ACHCONTROLLER_H
#define OPENRAVE_ACHCONTROLLER_H

#include <math.h>
#include <fstream>
#include "ach-setup.h"
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/utils.h>

static const dReal g_fEpsilonJointLimit = RavePow(g_fEpsilon,0.8);

class ACHController : public ControllerBase
{
public:
    ACHController(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv), cmdid(0), _bPause(false), _bIsDone(true), _bCheckCollision(false), _bThrowExceptions(false), _bRecording(false)
    {
        __description = ":Interface Author: Rosen Diankov\n\nIdeal controller used for planning and non-physics simulations. Forces exact robot positions.\n\n\
If \ref ControllerBase::SetPath is called and the trajectory finishes, then the controller will continue to set the trajectory's final joint values and transformation until one of three things happens:\n\n\
1. ControllerBase::SetPath is called.\n\n\
2. ControllerBase::SetDesired is called.\n\n\
3. ControllerBase::Reset is called resetting everything\n\n\
If SetDesired is called, only joint values will be set at every timestep leaving the transformation alone.\n";
        RegisterCommand("Pause",boost::bind(&ACHController::_Pause,this,_1,_2),
                        "pauses the controller from reacting to commands ");
        RegisterCommand("SetCheckCollisions",boost::bind(&ACHController::_SetCheckCollisions,this,_1,_2),
                        "If set, will check if the robot gets into a collision during movement");
        RegisterCommand("SetThrowExceptions",boost::bind(&ACHController::_SetThrowExceptions,this,_1,_2),
                        "If set, will throw exceptions instead of print warnings. Format is:\n\n  [0/1]");
        RegisterCommand("SetRecord",boost::bind(&ACHController::SetRecord,this,_1,_2),
                        "Enable / disable recording. Format is:\n\n [T/F] [filename]");
        _fCommandTime = 0;
        _fSpeed = 1;
        _nControlTransformation = 0;
        _vsendtimes.resize(0);
    }
    virtual ~ACHController() {
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
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
            _cblimits = _probot->RegisterChangeCallback(KinBody::Prop_JointLimits|KinBody::Prop_JointAccelerationVelocityTorqueLimits,boost::bind(&ACHController::_SetJointLimits,boost::bind(&utils::sptr_from<ACHController>, weak_controller())));
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

        // Create joint map for use with controller
        _pjointmap=Hubo::MakeDirectJointMap(_probot);

        //Setup ACH stuff here:
        _ach=Hubo::setup_channels();

        Hubo::setup_memory<hubo_ref>(&H_ref,&(_ach.hubo_ref));
        Hubo::setup_memory<hubo_state>(&H_state,&(_ach.hubo_state));
        Hubo::setup_memory<hubo_param>(&H_param,&(_ach.hubo_param));

        clock_gettime(CLOCK_REALTIME,&_t);
        _vsendtimes.reserve(10*60*2000); //Pre-allocate X minutes worth of timestamps
        
        return true;
    }

    virtual void Reset(int options)
    {
        _ptraj.reset();
        _vecdesired.resize(0);
        _vsendtimes.resize(0); //Note that memory is still reserved..
        if( flog.is_open() ) {
            flog.close();
        }
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
        if( values.size() != _dofindices.size() ) {
            throw openrave_exception(str(boost::format("wrong desired dimensions %d!=%d")%values.size()%_dofindices.size()),ORE_InvalidArguments);
        }
        _fCommandTime = 0;
        _ptraj.reset();
        // do not set done to true here! let it be picked up by the simulation thread.
        // this will also let it have consistent mechanics as SetPath
        // (there's a race condition we're avoiding where a user calls SetDesired and then state savers revert the robot)
        if( !_bPause ) {
            EnvironmentMutex::scoped_lock lockenv(_probot->GetEnv()->GetMutex());
            _vecdesired = values;
            if( _nControlTransformation ) {
                if( !!trans ) {
                    _tdesired = *trans;
                }
                else {
                    _tdesired = _probot->GetTransform();
                }
                _SetDOFValues(_vecdesired,_tdesired,0);
            }
            else {
                _SetDOFValues(_vecdesired,0);
            }
            _bIsDone = false;     // set after _vecdesired has changed
        }
        return true;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        OPENRAVE_ASSERT_FORMAT0(!ptraj || GetEnv()==ptraj->GetEnv(), "trajectory needs to come from the same environment as the controller", ORE_InvalidArguments);
        boost::mutex::scoped_lock lock(_mutex);
        if( _bPause ) {
            RAVELOG_DEBUG("ACHController cannot start trajectories when paused\n");
            _ptraj.reset();
            _bIsDone = true;
            return false;
        }
        _fCommandTime = 0;
        _bIsDone = true;
        _vecdesired.resize(0);

        if( !!ptraj ) {
            _samplespec._vgroups.resize(0);
            _bTrajHasJoints = !!_gjointvalues && ptraj->GetConfigurationSpecification().FindCompatibleGroup(_gjointvalues->name,false) != ptraj->GetConfigurationSpecification()._vgroups.end();
            if( _bTrajHasJoints ) {
                _samplespec._vgroups.push_back(*_gjointvalues);
            }
            _bTrajHasTransform = !!_gtransform && ptraj->GetConfigurationSpecification().FindCompatibleGroup(_gtransform->name,false) != ptraj->GetConfigurationSpecification()._vgroups.end();
            if( _bTrajHasTransform ) {
                _samplespec._vgroups.push_back(*_gtransform);
            }
            _samplespec.ResetGroupOffsets();
            _vgrablinks.resize(0);
            _vgrabbodylinks.resize(0);
            int dof = _samplespec.GetDOF();
            FOREACHC(itgroup,ptraj->GetConfigurationSpecification()._vgroups) {
                if( itgroup->name.size()>=8 && itgroup->name.substr(0,8) == "grabbody") {
                    stringstream ss(itgroup->name);
                    std::vector<std::string> tokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
                    if( itgroup->dof == 1 && tokens.size() >= 4 ) {
                        if( tokens.at(2) == _probot->GetName() ) {
                            KinBodyPtr pbody = GetEnv()->GetKinBody(tokens.at(1));
                            if( !!pbody ) {
                                _samplespec._vgroups.push_back(*itgroup);
                                _samplespec._vgroups.back().offset = dof;
                                _vgrabbodylinks.push_back(GrabBody(dof,boost::lexical_cast<int>(tokens.at(3)), pbody));
                            }
                            dof += _samplespec._vgroups.back().dof;
                        }
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("robot %s invalid grabbody tokens: %s")%_probot->GetName()%ss.str()));
                    }
                }
                else if( itgroup->name.size()>=4 && itgroup->name.substr(0,4) == "grab") {
                    stringstream ss(itgroup->name);
                    std::vector<std::string> tokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
                    if( tokens.size() >= 2 && tokens[1] == _probot->GetName() ) {
                        _samplespec._vgroups.push_back(*itgroup);
                        _samplespec._vgroups.back().offset = dof;
                        for(int idof = 0; idof < _samplespec._vgroups.back().dof; ++idof) {
                            _vgrablinks.push_back(make_pair(dof+idof,boost::lexical_cast<int>(tokens.at(2+idof))));
                        }
                        dof += _samplespec._vgroups.back().dof;
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("robot %s invalid grab tokens: %s")%_probot->GetName()%ss.str()));
                    }
                }
            }
            BOOST_ASSERT(_samplespec.IsValid());

            // see if at least one point can be sampled, this make it easier to debug bad trajectories
            vector<dReal> v;
            ptraj->Sample(v,0,_samplespec);
            if( _bTrajHasTransform ) {
                Transform t;
                _samplespec.ExtractTransform(t,v.begin(),_probot);
            }

            if( !!flog ) {
                ptraj->serialize(flog);
            }

            _ptraj = RaveCreateTrajectory(GetEnv(),ptraj->GetXMLId());
            _ptraj->Clone(ptraj,0);
            _bIsDone = false;
        }
        //Reset initial time based on RT clock
        //clock_gettime(CLOCK_REALTIME,&_t);
        //TODO: test if this is overkill?

        return true;
    }

    virtual void SimulationStep(dReal fTimeElapsed)
    {
        if( _bPause ) {
            return;
        }
        boost::mutex::scoped_lock lock(_mutex);
        TrajectoryBaseConstPtr ptraj = _ptraj; // because of multi-threading setting issues
        if( !!ptraj ) {
            vector<dReal> sampledata;
            ptraj->Sample(sampledata,_fCommandTime,_samplespec);

            // already sampled, so change the command times before before setting values
            // incase the below functions fail
            if( _fCommandTime > ptraj->GetDuration() ) {
                _fCommandTime = ptraj->GetDuration();
                _bIsDone = true;
            }
            else {
                _fCommandTime += _fSpeed * fTimeElapsed;
            }

            // first process all grab info
            list<KinBodyPtr> listrelease;
            list<pair<KinBodyPtr, KinBody::LinkPtr> > listgrab;
            FOREACH(itgrabinfo,_vgrablinks) {
                int bodyid = int(std::floor(sampledata.at(itgrabinfo->first)+0.5));
                if( bodyid != 0 ) {
                    KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(abs(bodyid));
                    if( !pbody ) {
                        RAVELOG_WARN(str(boost::format("failed to find body id %d")%bodyid));
                        continue;
                    }
                    if( bodyid < 0 ) {
                        if( !!_probot->IsGrabbing(pbody) ) {
                            listrelease.push_back(pbody);
                        }
                    }
                    else {
                        KinBody::LinkPtr pgrabbinglink = _probot->IsGrabbing(pbody);
                        if( !!pgrabbinglink ) {
                            if( pgrabbinglink->GetIndex() != itgrabinfo->second ) {
                                listrelease.push_back(pbody);
                                listgrab.push_back(make_pair(pbody,_probot->GetLinks().at(itgrabinfo->second)));
                            }
                        }
                        else {
                            listgrab.push_back(make_pair(pbody,_probot->GetLinks().at(itgrabinfo->second)));
                        }
                    }
                }
            }
            FOREACH(itgrabinfo,_vgrabbodylinks) {
                int dograb = int(std::floor(sampledata.at(itgrabinfo->offset)+0.5));
                if( dograb <= 0 ) {
                    if( !!_probot->IsGrabbing(itgrabinfo->pbody) ) {
                        listrelease.push_back(itgrabinfo->pbody);
                    }
                }
                else {
                    KinBody::LinkPtr pgrabbinglink = _probot->IsGrabbing(itgrabinfo->pbody);
                    if( !!pgrabbinglink ) {
                        if( pgrabbinglink->GetIndex() != itgrabinfo->robotlinkindex ) {
                            listrelease.push_back(itgrabinfo->pbody);
                            listgrab.push_back(make_pair(itgrabinfo->pbody,_probot->GetLinks().at(itgrabinfo->robotlinkindex)));
                        }
                    }
                    else {
                        listgrab.push_back(make_pair(itgrabinfo->pbody,_probot->GetLinks().at(itgrabinfo->robotlinkindex)));
                    }
                }
            }

            vector<dReal> vdofvalues;
            if( _bTrajHasJoints && _dofindices.size() > 0 ) {
                vdofvalues.resize(_dofindices.size());
                _samplespec.ExtractJointValues(vdofvalues.begin(),sampledata.begin(), _probot, _dofindices, 0);
            }

            Transform t;
            if( _bTrajHasTransform && _nControlTransformation ) {
                _samplespec.ExtractTransform(t,sampledata.begin(),_probot);
                if( vdofvalues.size() > 0 ) {
                    _SetDOFValues(vdofvalues,t, _fCommandTime > 0 ? fTimeElapsed : 0);
                }
                else {
                    _probot->SetTransform(t);
                }
            }
            else if( vdofvalues.size() > 0 ) {
                _SetDOFValues(vdofvalues, _fCommandTime > 0 ? fTimeElapsed : 0);
            }

            // always release after setting dof values
            FOREACH(itbody,listrelease) {
                _probot->Release(*itbody);
            }
            FOREACH(it,listgrab) {
                _probot->Grab(it->first,it->second);
            }
        }

        if( _vecdesired.size() > 0 ) {
            if( _nControlTransformation ) {
                _SetDOFValues(_vecdesired,_tdesired, 0);
            }
            else {
                _SetDOFValues(_vecdesired, 0);
            }
            _bIsDone = true;
        }
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

    /**
     * Start recording controller data to memory.
     */
    bool SetRecord(std::ostream& os, std::istream& is)
    {
        bool bWasRecording=_bRecording;
        is >> _bRecording;
        if (_bRecording)  RAVELOG_INFO("Recording enabled\n");
        string file;
        if (is >> file){
            _outFile.open(file.c_str());
        }
        if (bWasRecording && !_bRecording){
            RAVELOG_INFO("Recording disabled, writing\n");
            _WriteTimeDataToFile();
            _outFile.close();
        }
        return true;
    }

private:

    /**Export time data by row. */
    void _WriteTimeDataToFile()
    {
        //Simply iterate over file and dump data
        FOREACH(it,_vsendtimes){
            _outFile << *it << endl;
        }
    }

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
    virtual bool _SetThrowExceptions(std::ostream& os, std::istream& is)
    {
        is >> _bThrowExceptions;
        return !!is;
    }

    inline boost::shared_ptr<ACHController> shared_controller() {
        return boost::dynamic_pointer_cast<ACHController>(shared_from_this());
    }
    inline boost::shared_ptr<ACHController const> shared_controller_const() const {
        return boost::dynamic_pointer_cast<ACHController const>(shared_from_this());
    }
    inline boost::weak_ptr<ACHController> weak_controller() {
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

    void _SendACHReferences( vector<dReal> values)
    {
        // Get latest ACH message (why? this assumes that we are the only controller updating the ref,
        // and if we're not, then this is a totally unsafe way to update (need mutexes or something).
        size_t fs;
        ach_status_t r = (ach_status_t) ach_get( &(_ach.hubo_ref), &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
        //TODO: Eliminate these log messages? 
        if(r != ACH_OK) {
            //RAVELOG_VERBOSE("Ref r = %s\n",ach_result_to_string(r));
        }
        else   assert( sizeof(H_ref) == fs ); 

        r = (ach_status_t) ach_get( &(_ach.hubo_state), &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );

        if(r != ACH_OK) {
            //RAVELOG_VERBOSE("State r = %s\n",ach_result_to_string(r));
        }
        else   assert( sizeof(H_state) == fs ); 

        string name;
        //TODO: Test that all controllable joints map correctly
        for (size_t i = 0;i<_dofindices.size();++i){
            size_t dof = _dofindices[i];
            if ((*_pjointmap)[dof] >= 0) H_ref.ref[(*_pjointmap)[dof]]=values[dof];
        }
        struct timespec tnew;
        clock_gettime( CLOCK_REALTIME, &tnew);
        ach_put( &(_ach.hubo_ref), &H_ref, sizeof(H_ref));
        //crude way to force high significant digits)
        /*RAVELOG_DEBUG("%12.12f %12.12f \n",
                (double)Hubo::ts_diff(&tnew,&_t)/(double)NSEC_PER_SEC,
                H_ref.ref[Hubo::name2jmc["LHP"]]);
                */
        if (_bRecording) _vsendtimes.push_back(Hubo::ts_diff(&tnew,&_t));

    }

    virtual void _SetDOFValues(const std::vector<dReal>&values, dReal timeelapsed)
    {
        vector<dReal> prevvalues, curvalues, curvel;
        _probot->GetDOFValues(prevvalues);
        curvalues = prevvalues;
        _probot->GetDOFVelocities(curvel);
        Vector linearvel, angularvel;
        _probot->GetLinks().at(0)->GetVelocity(linearvel,angularvel);
        int i = 0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
            curvel.at(*it) = 0;
        }
        _CheckLimits(prevvalues, curvalues, timeelapsed);
        _probot->SetDOFValues(curvalues,true);
        _probot->SetDOFVelocities(curvel,linearvel,angularvel);
        //TODO: use this as safety for self-collisions
        _CheckConfiguration();
        //TODO: Don't send to ACH if trajectory is complete?
        _SendACHReferences(curvalues); 
    }
    virtual void _SetDOFValues(const std::vector<dReal>&values, const Transform &t, dReal timeelapsed)
    {
        BOOST_ASSERT(_nControlTransformation);
        vector<dReal> prevvalues, curvalues, curvel;
        _probot->GetDOFValues(prevvalues);
        curvalues = prevvalues;
        _probot->GetDOFVelocities(curvel);
        int i = 0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
            curvel.at(*it) = 0;
        }
        _CheckLimits(prevvalues, curvalues, timeelapsed);
        _probot->SetDOFValues(curvalues,t, true);
        _probot->SetDOFVelocities(curvel,Vector(),Vector());
        _CheckConfiguration();
        _SendACHReferences(curvalues); 
    }


    void _CheckLimits(std::vector<dReal>& prevvalues, std::vector<dReal>&curvalues, dReal timeelapsed)
    {
        for(size_t i = 0; i < _vlower[0].size(); ++i) {
            if( !_dofcircular[i] ) {
                if( curvalues.at(i) < _vlower[0][i]-g_fEpsilonJointLimit ) {
                    _ReportError(str(boost::format("robot %s dof %d is violating lower limit %e < %e, time=%f")%_probot->GetName()%i%_vlower[0][i]%curvalues[i]%_fCommandTime));
                }
                if( curvalues.at(i) > _vupper[0][i]+g_fEpsilonJointLimit ) {
                    _ReportError(str(boost::format("robot %s dof %d is violating upper limit %e > %e, time=%f")%_probot->GetName()%i%_vupper[0][i]%curvalues[i]%_fCommandTime));
                }
            }
        }
        if( timeelapsed > 0 ) {
            vector<dReal> vdiff = curvalues;
            _probot->SubtractDOFValues(vdiff,prevvalues);
            for(size_t i = 0; i < _vupper[1].size(); ++i) {
                dReal maxallowed = timeelapsed * _vupper[1][i]+g_fEpsilonJointLimit;
                if( RaveFabs(vdiff.at(i)) > maxallowed ) {
                    _ReportError(str(boost::format("robot %s dof %d is violating max velocity displacement %f > %f, time=%f")%_probot->GetName()%i%RaveFabs(vdiff.at(i))%maxallowed%_fCommandTime));
                }
            }
        }
    }

    void _CheckConfiguration()
    {
        if( _bCheckCollision ) {
            if( GetEnv()->CheckCollision(KinBodyConstPtr(_probot),_report) ) {
                _ReportError(str(boost::format("collsion in trajectory: %s, time=%f\n")%_report->__str__()%_fCommandTime));
            }
            if( _probot->CheckSelfCollision(_report) ) {
                _ReportError(str(boost::format("self collsion in trajectory: %s, time=%f\n")%_report->__str__()%_fCommandTime));
            }
        }
    }

    void _ReportError(const std::string& s)
    {
        if( !!_ptraj ) {
            string filename = str(boost::format("%s/failedtrajectory%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%1000));
            ofstream f(filename.c_str());
            f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
            _ptraj->serialize(f);
            RAVELOG_DEBUG(str(boost::format("trajectory dumped to %s")%filename));
        }
        if( _bThrowExceptions ) {
            throw openrave_exception(s,ORE_Assert);
        }
        else {
            RAVELOG_WARN(s);
        }
    }

    RobotBasePtr _probot;               ///< controlled body
    dReal _fSpeed;                    ///< how fast the robot should go
    TrajectoryBasePtr _ptraj;         ///< computed trajectory robot needs to follow in chunks of _pbody->GetDOF()
    bool _bTrajHasJoints, _bTrajHasTransform;
    std::vector< pair<int, int> > _vgrablinks; /// (data offset, link index) pairs
    struct GrabBody
    {
        GrabBody() : offset(0), robotlinkindex(0) {
        }
        GrabBody(int offset, int robotlinkindex, KinBodyPtr pbody) : offset(offset), robotlinkindex(robotlinkindex), pbody(pbody) {
        }
        int offset;
        int robotlinkindex;
        KinBodyPtr pbody;
    };
    std::vector<GrabBody> _vgrabbodylinks;
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
    ConfigurationSpecification _samplespec;
    boost::shared_ptr<ConfigurationSpecification::Group> _gjointvalues, _gtransform;
    boost::mutex _mutex;

    Hubo::hubo_channels _ach;
    struct hubo_ref H_ref;
    struct hubo_state H_state;
    struct hubo_param H_param;

    struct timespec _t;
    //TODO: Create a joint log
    std::vector<uint64_t> _vsendtimes;
    ofstream _outFile;
    bool  _bRecording;

    Hubo::DirectJointMapPtr _pjointmap;
};

ControllerBasePtr CreateACHController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new ACHController(penv,sinput));
}
#endif
