/*
  MEX wrapper for the MuJoCo HAPTIX C/C++ API (mjhaptix_user.dll/lib)

  Written by Emo Todorov

  Copyright (C) 2017 Roboti LLC
  
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
  
      http://www.apache.org/licenses/LICENSE-2.0
  
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/


#include "mex.h"
#include "haptix.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

static hxRobotInfo hxinfo;
static bool initialized = false;


//------------------------ copy data between MATLAB and Mujoco --------------------------

// check size of numeric field from MATLAB
static void checkNumeric(const mxArray* mx, const char* name, int sz0, int sz1)
{
    char msg[100];

    if( !mx )
    {
        sprintf(msg, "%s: missing numeric argument", name);
        mexErrMsgTxt(msg);
    }

    if( mxGetNumberOfDimensions(mx)!=2 )
    {
        sprintf(msg, "%s: numeric argument has %d dimensions, should be 2", 
            name, mxGetNumberOfDimensions(mx));
        mexErrMsgTxt(msg);
    }

    if( mxGetClassID(mx)!=mxDOUBLE_CLASS )
    {
        sprintf(msg, "%s: expected class DOUBLE", name);
        mexErrMsgTxt(msg);
    }

    const mwSize* sz = mxGetDimensions(mx);
    if( (sz[0]!=sz0 || sz[1]!=sz1) && (sz[0]!=sz1 || sz[1]!=sz0) )
    {
        sprintf(msg, "%s: expected %d-by-%d or %d-by-%d, got %d-by-%d", 
            name, sz0, sz1, sz1, sz0, sz[0], sz[1]);
        mexErrMsgTxt(msg);
    }
}


// MATLAB => Mujoco: structure field
template <typename T>
void mx2mjc(T* mj, const mxArray* arg, const int nr, const int nc, const char* name)
{
    // get field and check
    const mxArray* mx = mxGetField(arg, 0, name);
    if( !mx )
    {
        // skip empty array
        if( nr==0 || nc==0 )
            return;

        char msg[200];
        sprintf(msg, "missing field '%s'", name);
        mexErrMsgTxt(msg);
    }
    checkNumeric(mx, name, nr, nc);

    // copy data
    double* mat = mxGetPr(mx);
    for( int r=0; r<nr; r++ )
        for( int c=0; c<nc; c++ )
            mj[c+r*nc] = (T)mat[r+c*nr];
}


// MATLAB => Mujoco: structure field, 2D array
template <typename T, const int nc>
void mx2mjc2(T mj[][nc], const mxArray* arg, const int nr, const char* name)
{
    // get field and check
    const mxArray* mx = mxGetField(arg, 0, name);
    if( !mx )
    {
        // skip empty array
        if( nr==0 || nc==0 )
            return;

        char msg[200];
        sprintf(msg, "missing field '%s'", name);
        mexErrMsgTxt(msg);
    }
    checkNumeric(mx, name, nr, nc);

    // copy data
    double* mat = mxGetPr(mx);
    for( int r=0; r<nr; r++ )
        for( int c=0; c<nc; c++ )
            mj[r][c] = (T)mat[r+c*nr];
}


// Mujoco => MATLAB: structure field
template <typename T>
void mjc2mx(mxArray* out, const T* mj, const int nr, const int nc, const char* name)
{
    // return if no data; empty matrix assigned by default
    if( !nr || !nc )
        return;

    // check field name
    if( mxGetFieldNumber(out, name)==-1 )
    {
        mexPrintf("field name '%s' unrecognized\n", name);
        mexErrMsgTxt("error copying data from MuJoCo to Matlab");
    }

    // create field
    mxSetField(out, 0, name, mxCreateDoubleMatrix(nr, nc, mxREAL));

    // copy data
    double* mat = mxGetPr(mxGetField(out, 0, name));
    for( int r=0; r<nr; r++ )
        for( int c=0; c<nc; c++ )
            mat[r+c*nr] = (double)mj[c+r*nc];
}


// Mujoco => MATLAB: structure field, 2D array
template <typename T, const int nc>
void mjc2mx2(mxArray* out, const T mj[][nc], const int nr, const char* name)
{
    // return if no data; empty matrix assigned by default
    if( !nr )
        return;

    // check field name
    if( mxGetFieldNumber(out, name)==-1 )
    {
        mexPrintf("field name '%s' unrecognized\n", name);
        mexErrMsgTxt("error copying data from MuJoCo to Matlab");
    }

    // create field
    mxSetField(out, 0, name, mxCreateDoubleMatrix(nr, nc, mxREAL));

    // copy data
    double* mat = mxGetPr(mxGetField(out, 0, name));
    for( int r=0; r<nr; r++ )
        for( int c=0; c<nc; c++ )
            mat[r+c*nr] = (double)mj[r][c];
}



//----------------------- Simple API utility functions ----------------------------------

// wrapper for hx_robot_info
mxArray* hxrobotinfo(void)
{
    // call simulator, make sure call succeeds
    if( hx_robot_info(&hxinfo) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 7;
    const char* name[size] = {
        "motor_count",
        "joint_count",
        "contact_sensor_count",
        "imu_count",
        "motor_limit",
        "joint_limit",
        "update_rate"
    };

    // create option structure
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &hxinfo.motor_count,    1, 1, "motor_count");
    mjc2mx(out, &hxinfo.joint_count,    1, 1, "joint_count");
    mjc2mx(out, &hxinfo.contact_sensor_count,1,1, "contact_sensor_count");
    mjc2mx(out, &hxinfo.imu_count,      1, 1, "imu_count");
    mjc2mx2(out, hxinfo.motor_limit,    hxinfo.motor_count, "motor_limit");
    mjc2mx2(out, hxinfo.joint_limit,    hxinfo.joint_count, "joint_limit");
    mjc2mx(out, &hxinfo.update_rate,    1, 1, "update_rate");

    return out;
}


// wrapper for hx_update
mxArray* hxupdate(const mxArray* pin)
{
    hxSensor hxsensor;
    hxCommand hxcommand;

    // copy command data
    int nu = hxinfo.motor_count;
    mx2mjc(hxcommand.ref_pos, pin, nu, 1, "ref_pos");
    mx2mjc(hxcommand.ref_vel, pin, nu, 1, "ref_vel");
    mx2mjc(hxcommand.gain_pos, pin, nu, 1, "gain_pos");
    mx2mjc(hxcommand.gain_vel, pin, nu, 1, "gain_vel");

    // copy enable flags
    mx2mjc(&hxcommand.ref_pos_enabled, pin, 1, 1, "ref_pos_enabled");
    mx2mjc(&hxcommand.ref_vel_enabled, pin, 1, 1, "ref_vel_enabled");
    mx2mjc(&hxcommand.gain_pos_enabled, pin, 1, 1, "gain_pos_enabled");
    mx2mjc(&hxcommand.gain_vel_enabled, pin, 1, 1, "gain_vel_enabled");

    // call simulator, make sure call succeeds
    if( hx_update(&hxcommand, &hxsensor) )
        mexErrMsgTxt(hx_last_result());

    // create sensor structure
    const int size = 10;
    const char* name[size] = {
        "time_stamp",
        "motor_pos",
        "motor_vel",
        "motor_torque",
        "joint_pos",
        "joint_vel",
        "contact",
        "imu_linear_acc",
        "imu_angular_vel",
        "imu_orientation"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mxSetField(out, 0, "time_stamp", 
        mxCreateDoubleScalar(hx_double_time(&hxsensor.time_stamp)));
    mjc2mx(out, hxsensor.motor_pos,     nu, 1, "motor_pos");
    mjc2mx(out, hxsensor.motor_vel,     nu, 1, "motor_vel");
    mjc2mx(out, hxsensor.motor_torque,  nu, 1, "motor_torque");
    mjc2mx(out, hxsensor.joint_pos,     hxinfo.joint_count, 1, "joint_pos");
    mjc2mx(out, hxsensor.joint_vel,     hxinfo.joint_count, 1, "joint_vel");
    mjc2mx(out, hxsensor.contact,       hxinfo.contact_sensor_count, 1, "contact");
    mjc2mx2(out, hxsensor.imu_linear_acc, hxinfo.imu_count, "imu_linear_acc");
    mjc2mx2(out, hxsensor.imu_angular_vel, hxinfo.imu_count, "imu_angular_vel");
    mjc2mx2(out, hxsensor.imu_orientation, hxinfo.imu_count, "imu_orientation");
    return out;
}


// wrapper for hx_read_sensors
mxArray* hxreadsensors(void)
{
    int nu = hxinfo.motor_count;
    hxSensor hxsensor;

    // call simulator, make sure call succeeds
    if( hx_read_sensors(&hxsensor) )
        mexErrMsgTxt(hx_last_result());

    // create sensor structure
    const int size = 10;
    const char* name[size] = {
        "time_stamp",
        "motor_pos",
        "motor_vel",
        "motor_torque",
        "joint_pos",
        "joint_vel",
        "contact",
        "imu_linear_acc",
        "imu_angular_vel",
        "imu_orientation"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mxSetField(out, 0, "time_stamp", 
        mxCreateDoubleScalar(hx_double_time(&hxsensor.time_stamp)));
    mjc2mx(out, hxsensor.motor_pos,     nu, 1, "motor_pos");
    mjc2mx(out, hxsensor.motor_vel,     nu, 1, "motor_vel");
    mjc2mx(out, hxsensor.motor_torque,  nu, 1, "motor_torque");
    mjc2mx(out, hxsensor.joint_pos,     hxinfo.joint_count, 1, "joint_pos");
    mjc2mx(out, hxsensor.joint_vel,     hxinfo.joint_count, 1, "joint_vel");
    mjc2mx(out, hxsensor.contact,       hxinfo.contact_sensor_count, 1, "contact");
    mjc2mx2(out, hxsensor.imu_linear_acc, hxinfo.imu_count, "imu_linear_acc");
    mjc2mx2(out, hxsensor.imu_angular_vel, hxinfo.imu_count, "imu_angular_vel");
    mjc2mx2(out, hxsensor.imu_orientation, hxinfo.imu_count, "imu_orientation");
    return out;
}



//----------------------- Native API utility functions: GET -----------------------------

// wrapper for mj_get_state
mxArray* getstate(void)
{
    mjState mjstate;

    // call simulator, make sure call succeeds
    if( mj_get_state(&mjstate) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 7;
    const char* name[size] = {
        "nq",
        "nv",
        "na",
        "time",
        "qpos",
        "qvel",
        "act"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjstate.nq,    1, 1, "nq");
    mjc2mx(out, &mjstate.nv,    1, 1, "nv");
    mjc2mx(out, &mjstate.na,    1, 1, "na");
    mjc2mx(out, &mjstate.time,  1, 1, "time");
    mjc2mx(out, mjstate.qpos,   mjstate.nq, 1, "qpos");
    mjc2mx(out, mjstate.qvel,   mjstate.nv, 1, "qvel");
    mjc2mx(out, mjstate.act,    mjstate.na, 1, "act");

    return out;
}


// wrapper for mj_get_control
mxArray* getcontrol(void)
{
    mjControl mjcontrol;

    // call simulator, make sure call succeeds
    if( mj_get_control(&mjcontrol) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 3;
    const char* name[size] = {
        "nu",
        "time",
        "ctrl"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjcontrol.nu,   1, 1, "nu");
    mjc2mx(out, &mjcontrol.time, 1, 1, "time");
    mjc2mx(out, mjcontrol.ctrl,  mjcontrol.nu, 1, "ctrl");

    return out;
}


// wrapper for mj_get_applied
mxArray* getapplied(void)
{
    mjApplied mjapplied;

    // call simulator, make sure call succeeds
    if( mj_get_applied(&mjapplied) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 5;
    const char* name[size] = {
        "nv",
        "nbody",
        "time",
        "qfrc_applied",
        "xfrc_applied"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjapplied.nv,           1, 1, "nv");
    mjc2mx(out, &mjapplied.nbody,        1, 1, "nbody");
    mjc2mx(out, &mjapplied.time,         1, 1, "time");
    mjc2mx(out, mjapplied.qfrc_applied,  mjapplied.nv, 1, "qfrc_applied");
    mjc2mx2(out, mjapplied.xfrc_applied, mjapplied.nbody, "xfrc_applied");

    return out;
}


// wrapper for mj_get_onebody
mxArray* getonebody(int id)
{
    mjOneBody mjonebody;
    mjonebody.bodyid = id;

    // call simulator, make sure call succeeds
    if( mj_get_onebody(&mjonebody) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 12;
    const char* name[size] = {
        "bodyid",
        "isfloating",
        "time",
        "linacc",
        "angacc",
        "contactforce",
        "pos",
        "quat",
        "linvel",
        "angvel",
        "force",
        "torque"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjonebody.bodyid,       1, 1, "bodyid");
    mjc2mx(out, &mjonebody.isfloating,   1, 1, "isfloating");
    mjc2mx(out, &mjonebody.time,         1, 1, "time");
    mjc2mx(out, mjonebody.linacc,        3, 1, "linacc");
    mjc2mx(out, mjonebody.angacc,        3, 1, "angacc");
    mjc2mx(out, mjonebody.contactforce,  3, 1, "contactforce");
    mjc2mx(out, mjonebody.pos,           3, 1, "pos");
    mjc2mx(out, mjonebody.quat,          4, 1, "quat");
    mjc2mx(out, mjonebody.linvel,        3, 1, "linvel");
    mjc2mx(out, mjonebody.angvel,        3, 1, "angvel");
    mjc2mx(out, mjonebody.force,         3, 1, "force");
    mjc2mx(out, mjonebody.torque,        3, 1, "torque");

    return out;
}


// wrapper for mj_get_mocap
mxArray* getmocap(void)
{
    mjMocap mjmocap;

    // call simulator, make sure call succeeds
    if( mj_get_mocap(&mjmocap) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 4;
    const char* name[size] = {
        "nmocap",
        "time",
        "pos",
        "quat"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjmocap.nmocap, 1, 1, "nmocap");
    mjc2mx(out, &mjmocap.time,   1, 1, "time");
    mjc2mx2(out, mjmocap.pos,    mjmocap.nmocap, "pos");
    mjc2mx2(out, mjmocap.quat,   mjmocap.nmocap, "quat");

    return out;
}


// wrapper for mj_get_dynamics
mxArray* getdynamics(void)
{
    mjDynamics mjdynamics;

    // call simulator, make sure call succeeds
    if( mj_get_dynamics(&mjdynamics) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 5;
    const char* name[size] = {
        "nv",
        "na",
        "time",
        "qacc",
        "actdot"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjdynamics.nv,   1, 1, "nv");
    mjc2mx(out, &mjdynamics.na,   1, 1, "na");
    mjc2mx(out, &mjdynamics.time, 1, 1, "time");
    mjc2mx(out, mjdynamics.qacc,  mjdynamics.nv, 1, "qacc");
    mjc2mx(out, mjdynamics.actdot, mjdynamics.na, 1, "actdot");

    return out;
}


// wrapper for mj_get_sensor
mxArray* getsensor(void)
{
    mjSensor mjsensor;

    // call simulator, make sure call succeeds
    if( mj_get_sensor(&mjsensor) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 3;
    const char* name[size] = {
        "nsensordata",
        "time",
        "sensordata"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjsensor.nsensordata,  1, 1, "nsensordata");
    mjc2mx(out, &mjsensor.time,         1, 1, "time");
    mjc2mx(out, mjsensor.sensordata,    mjsensor.nsensordata, 1, "sensordata");

    return out;
}


// wrapper for mj_get_body
mxArray* getbody(void)
{
    mjBody mjbody;

    // call simulator, make sure call succeeds
    if( mj_get_body(&mjbody) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 4;
    const char* name[size] = {
        "nbody",
        "time",
        "pos",
        "mat"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjbody.nbody, 1, 1, "nbody");
    mjc2mx(out, &mjbody.time,  1, 1, "time");
    mjc2mx2(out, mjbody.pos,   mjbody.nbody, "pos");
    mjc2mx2(out, mjbody.mat,   mjbody.nbody, "mat");

    return out;
}


// wrapper for mj_get_geom
mxArray* getgeom(void)
{
    mjGeom mjgeom;

    // call simulator, make sure call succeeds
    if( mj_get_geom(&mjgeom) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 4;
    const char* name[size] = {
        "ngeom",
        "time",
        "pos",
        "mat"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjgeom.ngeom, 1, 1, "ngeom");
    mjc2mx(out, &mjgeom.time,  1, 1, "time");
    mjc2mx2(out, mjgeom.pos,   mjgeom.ngeom, "pos");
    mjc2mx2(out, mjgeom.mat,   mjgeom.ngeom, "mat");

    return out;
}


// wrapper for mj_get_site
mxArray* getsite(void)
{
    mjSite mjsite;

    // call simulator, make sure call succeeds
    if( mj_get_site(&mjsite) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 4;
    const char* name[size] = {
        "nsite",
        "time",
        "pos",
        "mat"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjsite.nsite, 1, 1, "nsite");
    mjc2mx(out, &mjsite.time,  1, 1, "time");
    mjc2mx2(out, mjsite.pos,   mjsite.nsite, "pos");
    mjc2mx2(out, mjsite.mat,   mjsite.nsite, "mat");

    return out;
}


// wrapper for mj_get_tendon
mxArray* gettendon(void)
{
    mjTendon mjtendon;

    // call simulator, make sure call succeeds
    if( mj_get_tendon(&mjtendon) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 4;
    const char* name[size] = {
        "ntendon",
        "time",
        "length",
        "velocity"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjtendon.ntendon, 1, 1, "ntendon");
    mjc2mx(out, &mjtendon.time,    1, 1, "time");
    mjc2mx(out, mjtendon.length,   mjtendon.ntendon, 1, "length");
    mjc2mx(out, mjtendon.velocity, mjtendon.ntendon, 1, "velocity");

    return out;
}


// wrapper for mj_get_actuator
mxArray* getactuator(void)
{
    mjActuator mjactuator;

    // call simulator, make sure call succeeds
    if( mj_get_actuator(&mjactuator) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 5;
    const char* name[size] = {
        "nu",
        "time",
        "length",
        "velocity",
        "force"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjactuator.nu,      1, 1, "nu");
    mjc2mx(out, &mjactuator.time,    1, 1, "time");
    mjc2mx(out, mjactuator.length,   mjactuator.nu, 1, "length");
    mjc2mx(out, mjactuator.velocity, mjactuator.nu, 1, "velocity");

    return out;
}


// wrapper for mj_get_force
mxArray* getforce(void)
{
    mjForce mjforce;

    // call simulator, make sure call succeeds
    if( mj_get_force(&mjforce) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 4;
    const char* name[size] = {
        "nv",
        "time",
        "nonconstraint",
        "constraint"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjforce.nv,        1, 1, "nv");
    mjc2mx(out, &mjforce.time,      1, 1, "time");
    mjc2mx(out, mjforce.nonconstraint, mjforce.nv, 1, "nonconstraint");
    mjc2mx(out, mjforce.constraint, mjforce.nv, 1, "constraint");

    return out;
}


// wrapper for mj_get_contact
mxArray* getcontact(void)
{
    mjContact mjcontact;

    // call simulator, make sure call succeeds
    if( mj_get_contact(&mjcontact) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 8;
    const char* name[size] = {
        "ncon",
        "time",
        "pos",
        "frame",
        "dist",
        "force",
        "geom1",
        "geom2"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjcontact.ncon,  1, 1, "ncon");
    mjc2mx(out, &mjcontact.time,  1, 1, "time");
    mjc2mx2(out, mjcontact.pos,   mjcontact.ncon,    "pos");
    mjc2mx2(out, mjcontact.frame, mjcontact.ncon,    "frame");
    mjc2mx(out, mjcontact.dist,   mjcontact.ncon, 1, "dist");
    mjc2mx2(out, mjcontact.force, mjcontact.ncon,    "force");
    mjc2mx(out, mjcontact.geom1,  mjcontact.ncon, 1, "geom1");
    mjc2mx(out, mjcontact.geom2,  mjcontact.ncon, 1, "geom2");

    return out;
}


//----------------------- Native API utility functions: set -----------------------------

// wrapper for mj_set_state
void setstate(const mxArray* pin)
{
    mjState mjstate;
    memset(&mjstate, 0, sizeof(mjState));

    // copy data
    mx2mjc(&mjstate.nq, pin, 1, 1, "nq");
    mx2mjc(&mjstate.nv, pin, 1, 1, "nv");
    mx2mjc(&mjstate.na, pin, 1, 1, "na");
    mx2mjc(mjstate.qpos, pin, mjstate.nq, 1, "qpos");
    mx2mjc(mjstate.qvel, pin, mjstate.nv, 1, "qvel");
    mx2mjc(mjstate.act,  pin, mjstate.na, 1, "act");

    // call simulator, make sure call succeeds
    if( mj_set_state(&mjstate) )
        mexErrMsgTxt(hx_last_result());
}


// wrapper for mj_set_control
void setcontrol(const mxArray* pin)
{
    mjControl mjcontrol;
    memset(&mjcontrol, 0, sizeof(mjControl));

    // copy data
    mx2mjc(&mjcontrol.nu, pin, 1, 1, "nu");
    mx2mjc(mjcontrol.ctrl, pin, mjcontrol.nu, 1, "ctrl");

    // call simulator, make sure call succeeds
    if( mj_set_control(&mjcontrol) )
        mexErrMsgTxt(hx_last_result());
}


// wrapper for mj_set_applied
void setapplied(const mxArray* pin)
{
    mjApplied mjapplied;
    memset(&mjapplied, 0, sizeof(mjApplied));

    // copy data
    mx2mjc(&mjapplied.nv, pin, 1, 1, "nv");
    mx2mjc(&mjapplied.nbody, pin, 1, 1, "nbody");
    mx2mjc(mjapplied.qfrc_applied, pin, mjapplied.nv, 1, "qfrc_applied");
    mx2mjc2(mjapplied.xfrc_applied, pin, mjapplied.nbody, "xfrc_applied");

    // call simulator, make sure call succeeds
    if( mj_set_applied(&mjapplied) )
        mexErrMsgTxt(hx_last_result());
}


// wrapper for mj_set_onebody
void setonebody(const mxArray* pin)
{
    mjOneBody mjonebody;
    memset(&mjonebody, 0, sizeof(mjOneBody));

    // copy data
    mx2mjc(&mjonebody.bodyid, pin, 1, 1, "bodyid");
    mx2mjc(mjonebody.pos,    pin, 3, 1, "pos");
    mx2mjc(mjonebody.quat,   pin, 4, 1, "quat");
    mx2mjc(mjonebody.linvel, pin, 3, 1, "linvel");
    mx2mjc(mjonebody.angvel, pin, 3, 1, "angvel");
    mx2mjc(mjonebody.force,  pin, 3, 1, "force");
    mx2mjc(mjonebody.torque, pin, 3, 1, "torque");

    // call simulator, make sure call succeeds
    if( mj_set_onebody(&mjonebody) )
        mexErrMsgTxt(hx_last_result());
}


// wrapper for mj_set_mocap
void setmocap(const mxArray* pin)
{
    mjMocap mjmocap;
    memset(&mjmocap, 0, sizeof(mjMocap));

    // copy data
    mx2mjc(&mjmocap.nmocap, pin, 1, 1, "nmocap");
    mx2mjc2(mjmocap.pos,  pin, mjmocap.nmocap, "pos");
    mx2mjc2(mjmocap.quat, pin, mjmocap.nmocap, "quat");

    // call simulator, make sure call succeeds
    if( mj_set_mocap(&mjmocap) )
        mexErrMsgTxt(hx_last_result());
}



//----------------------- Native API utility functions: command, info -------------------

// wrapper for mj_info
mxArray* info(void)
{
    mjInfo mjinfo;

    // call simulator, make sure call succeeds
    if( mj_info(&mjinfo) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 37;
    const char* name[size] = {
        "nq",
        "nv",
        "na",
        "njnt",
        "nbody",
        "ngeom",
        "nsite",
        "ntendon",
        "nu",
        "neq",
        "nkey",
        "nmocap",
        "nsensor",
        "nsensordata",
        "nmat",
        "timestep",
        "apirate",
        "sensor_type",
        "sensor_datatype",
        "sensor_objtype",
        "sensor_objid",
        "sensor_dim",
        "sensor_adr",
        "sensor_noise",
        "jnt_type",
        "jnt_bodyid",
        "jnt_qposadr",
        "jnt_dofadr",
        "jnt_range",
        "geom_type",
        "geom_bodyid",
        "eq_type",
        "eq_obj1id",
        "eq_obj2id",
        "actuator_trntype",
        "actuator_trnid",
        "actuator_ctrlrange"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjinfo.nq,          1, 1, "nq");
    mjc2mx(out, &mjinfo.nv,          1, 1, "nv");
    mjc2mx(out, &mjinfo.na,          1, 1, "na");
    mjc2mx(out, &mjinfo.njnt,        1, 1, "njnt");
    mjc2mx(out, &mjinfo.nbody,       1, 1, "nbody");
    mjc2mx(out, &mjinfo.ngeom,       1, 1, "ngeom");
    mjc2mx(out, &mjinfo.nsite,       1, 1, "nsite");
    mjc2mx(out, &mjinfo.ntendon,     1, 1, "ntendon");
    mjc2mx(out, &mjinfo.nu,          1, 1, "nu");
    mjc2mx(out, &mjinfo.neq,         1, 1, "neq");
    mjc2mx(out, &mjinfo.nkey,        1, 1, "nkey");
    mjc2mx(out, &mjinfo.nmocap,      1, 1, "nmocap");
    mjc2mx(out, &mjinfo.nsensor,     1, 1, "nsensor");
    mjc2mx(out, &mjinfo.nsensordata, 1, 1, "nsensordata");
    mjc2mx(out, &mjinfo.nmat,        1, 1, "nmat");

    mjc2mx(out, &mjinfo.timestep,    1, 1, "timestep");
    mjc2mx(out, &mjinfo.apirate,     1, 1, "apirate");

    mjc2mx(out, mjinfo.sensor_type,  mjinfo.nsensor, 1, "sensor_type");
    mjc2mx(out, mjinfo.sensor_datatype, mjinfo.nsensor, 1, "sensor_datatype");
    mjc2mx(out, mjinfo.sensor_objtype,  mjinfo.nsensor, 1, "sensor_objtype");
    mjc2mx(out, mjinfo.sensor_objid, mjinfo.nsensor, 1, "sensor_objid");
    mjc2mx(out, mjinfo.sensor_dim,   mjinfo.nsensor, 1, "sensor_dim");
    mjc2mx(out, mjinfo.sensor_adr,   mjinfo.nsensor, 1, "sensor_adr");
    mjc2mx(out, mjinfo.sensor_noise, mjinfo.nsensor, 1, "sensor_noise");

    mjc2mx(out, mjinfo.jnt_type,     mjinfo.njnt, 1, "jnt_type");
    mjc2mx(out, mjinfo.jnt_bodyid,   mjinfo.njnt, 1, "jnt_bodyid");
    mjc2mx(out, mjinfo.jnt_qposadr,  mjinfo.njnt, 1, "jnt_qposadr");
    mjc2mx(out, mjinfo.jnt_dofadr,   mjinfo.njnt, 1, "jnt_dofadr");
    mjc2mx2(out, mjinfo.jnt_range,   mjinfo.njnt,    "jnt_range");

    mjc2mx(out, mjinfo.geom_type,    mjinfo.ngeom, 1, "geom_type");
    mjc2mx(out, mjinfo.geom_bodyid,  mjinfo.ngeom, 1, "geom_bodyid");

    mjc2mx(out, mjinfo.eq_type,      mjinfo.neq, 1, "eq_type");
    mjc2mx(out, mjinfo.eq_obj1id,    mjinfo.neq, 1, "eq_obj1id");
    mjc2mx(out, mjinfo.eq_obj2id,    mjinfo.neq, 1, "eq_obj2id");

    mjc2mx(out, mjinfo.actuator_trntype, mjinfo.nu, 1, "actuator_trntype");
    mjc2mx2(out, mjinfo.actuator_trnid,  mjinfo.nu,    "actuator_trnid");
    mjc2mx2(out, mjinfo.actuator_ctrlrange, mjinfo.nu, "actuator_ctrlrange");

    return out;
}



// wrapper for mj_update
mxArray* update(const mxArray* pin)
{
    mjControl mjcontrol;
    mjSensor mjsensor;

    // prepare control
    mjcontrol.time = 0;
    mx2mjc(&mjcontrol.nu, pin, 1, 1, "nu");
    mx2mjc(mjcontrol.ctrl, pin, mjcontrol.nu, 1, "ctrl");

    // call simulator, make sure call succeeds
    if( mj_update(&mjcontrol, &mjsensor) )
        mexErrMsgTxt(hx_last_result());

    // create Matlab structure
    const int size = 3;
    const char* name[size] = {
        "nsensordata",
        "time",
        "sensordata"
    };
    mxArray* out = mxCreateStructMatrix(1, 1, size, name);

    // fill structure with data
    mjc2mx(out, &mjsensor.nsensordata,  1, 1, "nsensordata");
    mjc2mx(out, &mjsensor.time,         1, 1, "time");
    mjc2mx(out, mjsensor.sensordata,    mjsensor.nsensordata, 1, "sensordata");

    return out;
}



//------------------------- main mex API ------------------------------------------------

const char _help[] =
"\nUSAGE:  [output] = mjhx(command, [input])\n\n"
"  OUTPUT         COMMAND               INPUT\n\n"
"                 'hx_connect'          [host name or ip address]\n"
"                 'hx_close'\n"
"  hxRobotInfo    'hx_robot_info'\n"
"  hxSensor       'hx_update'           hxCommand\n"
"  hxSensor       'hx_read_sensors'\n\n"
"  mjState        'get_state'\n"
"  mjControl      'get_control'\n"
"  mjApplied      'get_applied'\n"
"  mjOneBody      'get_onebody'         bodyid\n"
"  mjMocap        'get_mocap'\n"
"  mjDynamics     'get_dynamics'\n"
"  mjSensor       'get_sensor'\n"
"  mjBody         'get_body'\n"
"  mjGeom         'get_geom'\n"
"  mjSite         'get_site'\n"
"  mjTendon       'get_tendon'\n"
"  mjActuator     'get_actuator'\n"
"  mjForce        'get_force'\n"
"  mjContact      'get_contact'\n"
"                 'set_state'           mjState\n"
"                 'set_control'         mjControl\n"
"                 'set_applied'         mjApplied\n"
"                 'set_onebody'         mjOneBody\n"
"                 'set_mocap'           mjMocap\n"
"  float[4]       'get_rgba'            type, id\n"
"                 'set_rgba'            type, id, rgba\n"
"                 'set_geomsize'        geomid, size[3]\n"
"                 'connect'             [host name or ip address]\n"
"                 'close'\n"
"  int            'connected'\n"
"  mjInfo         'info'\n"
"                 'step'\n"
"  mjSensor       'update'              mjControl, waitflag\n"
"                 'reset'               [keyframe]\n"
"                 'equality'            eqid, state\n"
"                 'message'             [message]\n"
"  int            'name2id'             type, name\n"
"  string         'id2name'             type, id\n";


// exit function
void exitFunction(void)
{
    // close socket
    hx_close();

    // unlock mex so MATLAB can remove it from memory
    mexUnlock();
    initialized = false;
}


// entry point
void mexFunction(int nout, mxArray* pout[], int nin, const mxArray* pin[])
{
    char command[200], text[200], text1[200];
    float rgba[4], geomsize[3];
    double* ptr;
    int i, res = 0;

    // register exit function only once
    if( !initialized )
    {
        mexAtExit(exitFunction);
        mexLock();
        initialized = true;
    }

    // no inputs: print help, return
    if( !nin )
    {
        mexPrintf("%s\n", _help);
        return;
    }

    // get command string
    if( mxGetClassID(pin[0])!=mxCHAR_CLASS )
        mexErrMsgTxt("first argument must be command string");
    mxGetString(pin[0], command, 200);

    //-------------------------------------------- SIMPLE API

    // hx_connect or connect
    if( !strcmp(command, "hx_connect") || !strcmp(command, "connect") )
    {
        // no host name: NULL
        if( nin==1 )
            res = hx_connect(0, 0);
        else
        {
            if( nin!=2 || mxGetClassID(pin[1])!=mxCHAR_CLASS )
                mexErrMsgTxt("string argument expected");
            else
            {
                mxGetString(pin[1], text, 200);
                res = hx_connect(text, 0);
            }
        }
    }

    // hx_close or close
    else if( !strcmp(command,"hx_close") || !strcmp(command,"close") )
        exitFunction();

    // hx_robot_info
    else if( !strcmp(command, "hx_robot_info") )
    {
        res = hx_robot_info(&hxinfo);
        if( res==hxOK )
            pout[0] = hxrobotinfo();
    }

    // hx_update
    else if( !strcmp(command, "hx_update") )
    {
        // command argument required
        if( nin!=2 || mxGetClassID(pin[1])!=mxSTRUCT_CLASS)
            mexErrMsgTxt("command argument expected");

        pout[0] = hxupdate(pin[1]);
    }

    // hx_read_sensors
    else if( !strcmp(command, "hx_read_sensors") )
        pout[0] = hxreadsensors();

    //-------------------------------------------- NATIVE API: get

    // get_state
    else if( !strcmp(command, "get_state") )
        pout[0] = getstate();

    // get_control
    else if( !strcmp(command, "get_control") )
        pout[0] = getcontrol();

    // get_applied
    else if( !strcmp(command, "get_applied") )
        pout[0] = getapplied();

    // get_onebody
    else if( !strcmp(command, "get_onebody") )
    {
        // bodyid argument expected
        if( nin!=2 || mxGetClassID(pin[1])!=mxDOUBLE_CLASS )
            mexErrMsgTxt("numeric argument expected");
        else
           pout[0] = getonebody((int)mxGetScalar(pin[1]));
    }

    // get_mocap
    else if( !strcmp(command, "get_mocap") )
        pout[0] = getmocap();

    // get_dynamics
    else if( !strcmp(command, "get_dynamics") )
        pout[0] = getdynamics();

    // get_sensor
    else if( !strcmp(command, "get_sensor") )
        pout[0] = getsensor();

    // get_body
    else if( !strcmp(command, "get_body") )
        pout[0] = getbody();

    // get_geom
    else if( !strcmp(command, "get_geom") )
        pout[0] = getgeom();

    // get_site
    else if( !strcmp(command, "get_site") )
        pout[0] = getsite();

    // get_tendon
    else if( !strcmp(command, "get_tendon") )
        pout[0] = gettendon();

    // get_actuator
    else if( !strcmp(command, "get_actuator") )
        pout[0] = getactuator();

    // get_force
    else if( !strcmp(command, "get_force") )
        pout[0] = getforce();

    // get_contact
    else if( !strcmp(command, "get_contact") )
        pout[0] = getcontact();

    //-------------------------------------------- NATIVE API: set

    // set_state
    else if( !strcmp(command, "set_state") )
    {
        // state argument required
        if( nin!=2 || mxGetClassID(pin[1])!=mxSTRUCT_CLASS )
            mexErrMsgTxt("mjState argument expected");

        setstate(pin[1]);
    }

    // set_control
    else if( !strcmp(command, "set_control") )
    {
        // control argument required
        if( nin!=2 || mxGetClassID(pin[1])!=mxSTRUCT_CLASS )
            mexErrMsgTxt("mjControl argument expected");

        setcontrol(pin[1]);
    }

    // set_applied
    else if( !strcmp(command, "set_applied") )
    {
        // control argument required
        if( nin!=2 || mxGetClassID(pin[1])!=mxSTRUCT_CLASS )
            mexErrMsgTxt("mjApplied argument expected");

        setapplied(pin[1]);
    }

    // set_onebody
    else if( !strcmp(command, "set_onebody") )
    {
        // control argument required
        if( nin!=2 || mxGetClassID(pin[1])!=mxSTRUCT_CLASS )
            mexErrMsgTxt("mjOneBody argument expected");

        setonebody(pin[1]);
    }

    // set_mocap
    else if( !strcmp(command, "set_mocap") )
    {
        // mocap argument required
        if( nin!=2 || mxGetClassID(pin[1])!=mxSTRUCT_CLASS )
            mexErrMsgTxt("mjMocap argument expected");

        setmocap(pin[1]);
    }

    // set_geomsize
    else if( !strcmp(command, "set_geomsize") )
    {
        // check arguments
        if( nin!=3 || 
            mxGetClassID(pin[1])!=mxDOUBLE_CLASS || 
            mxGetClassID(pin[2])!=mxDOUBLE_CLASS )
            mexErrMsgTxt("expected arguments: geomid, size[3]");
        checkNumeric(pin[2], "GEOMSIZE", 3, 1);

        // convert size double to float
        ptr = mxGetPr(pin[2]);
        for( i=0; i<3; i++ )
            geomsize[i] = (float)ptr[i];

        res = mj_set_geomsize((int)mxGetScalar(pin[1]), geomsize);
    }

    //-------------------------------------------- NATIVE API: rgba

    // get_rgba
    else if( !strcmp(command, "get_rgba") )
    {
        // check arguments
        if( nin!=3 || 
            mxGetClassID(pin[1])!=mxCHAR_CLASS || 
            mxGetClassID(pin[2])!=mxDOUBLE_CLASS )
            mexErrMsgTxt("expected arguments: type, id]");

        // get string type
        mxGetString(pin[1], text, 200);

        // get rgba
        res = mj_get_rgba(text, (int)mxGetScalar(pin[2]), rgba);

        // create output
        pout[0] = mxCreateDoubleMatrix(1, 4, mxREAL);

        // convert float to double
        ptr = mxGetPr(pout[0]);
        for( i=0; i<4; i++ )
            ptr[i] = (double)rgba[i];
    }

    // set_rgba
    else if( !strcmp(command, "set_rgba") )
    {
        // check arguments
        if( nin!=4 || 
            mxGetClassID(pin[1])!=mxCHAR_CLASS || 
            mxGetClassID(pin[2])!=mxDOUBLE_CLASS || 
            mxGetClassID(pin[3])!=mxDOUBLE_CLASS )
            mexErrMsgTxt("expected arguments: type, id, rgba[4]");
        checkNumeric(pin[3], "RGBA", 4, 1);

        // convert rgba double to float
        ptr = mxGetPr(pin[3]);
        for( i=0; i<4; i++ )
            rgba[i] = (float)ptr[i];

        // get string type
        mxGetString(pin[1], text, 200);

        res = mj_set_rgba(text, (int)mxGetScalar(pin[2]), rgba);
    }

    //-------------------------------------------- NATIVE API: command and info

    // connected
    else if( !strcmp(command, "connected") )
        pout[0] = mxCreateDoubleScalar(mj_connected());

    // info
    else if(!strcmp(command, "info"))
        pout[0] = info();

    // step
    else if( !strcmp(command, "step") )
        res = mj_step();

    // update
    else if( !strcmp(command, "update") )
    {
        // check arguments
        if( nin!=2 || mxGetClassID(pin[1])!=mxSTRUCT_CLASS )
            mexErrMsgTxt("expected mjControl argument");

        pout[0] = update(pin[1]);
    }

    // reset
    else if( !strcmp(command, "reset") )
    {
        // no argument: -1
        if( nin==1 )
            res = mj_reset(-1);
        else
        {
            if( nin!=2 || mxGetClassID(pin[1])!=mxDOUBLE_CLASS )
                mexErrMsgTxt("numeric argument expected");
            else
                res = mj_reset((int)mxGetScalar(pin[1]));
        }
    }

    // equality
    else if( !strcmp(command, "equality") )
    {
        // check arguments
        if( nin!=3 || 
            mxGetClassID(pin[1])!=mxDOUBLE_CLASS || 
            mxGetClassID(pin[2])!=mxDOUBLE_CLASS )
            mexErrMsgTxt("expected two arguments: eqid, state");

        res = mj_equality((int)mxGetScalar(pin[1]), (int)mxGetScalar(pin[2]));
    }

    // message
    else if( !strcmp(command, "message") )
    {
        // no argument: empty message
        if( nin==1 )
            res = mj_message(0);
        else
        {
            if( nin!=2 || mxGetClassID(pin[1])!=mxCHAR_CLASS)
                mexErrMsgTxt("string argument expected");
            else
            {
                mxGetString(pin[1], text, 200);
                res = mj_message(text);
            }
        }
    }

    // name2id
    else if( !strcmp(command, "name2id") )
    {
        // check arguments
        if( nin!=3 || 
            mxGetClassID(pin[1])!=mxCHAR_CLASS || 
            mxGetClassID(pin[2])!=mxCHAR_CLASS )
            mexErrMsgTxt("expected two arguments: type, name");

        mxGetString(pin[1], text, 200);
        mxGetString(pin[2], text1, 200);
        pout[0] = mxCreateDoubleScalar(mj_name2id(text, text1));
    }

    // id2name
    else if( !strcmp(command, "id2name") )
    {
        // check arguments
        if( nin!=3 || 
            mxGetClassID(pin[1])!=mxCHAR_CLASS || 
            mxGetClassID(pin[2])!=mxDOUBLE_CLASS )
            mexErrMsgTxt("expected two arguments: type, id");

        mxGetString(pin[1], text, 200);
        pout[0] = mxCreateString(mj_id2name(text, (int)mxGetScalar(pin[2])));
    }

    // unknown command
    else
        mexErrMsgTxt("unknown command");

    // return error message text
    if( res )
        mexErrMsgTxt(hx_last_result());
}
