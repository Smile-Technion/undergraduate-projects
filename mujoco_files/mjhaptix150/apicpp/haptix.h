/*
  C/C++ API for socket-based programmatic access to MuJoCo HAPTIX

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

#pragma once


// DLL export/import
#if defined(MJ_STATIC)
    #define MJAPI
#elif defined(MJ_EXPORT)
    #define MJAPI __declspec(dllexport)
#else
    #define MJAPI __declspec(dllimport)
#endif


// this is a C API
#if defined(__cplusplus)
extern "C" {
#endif


/****************************************************************************************

PART 1:  Simple API  (prefix 'hx')

    This API is designed in collaboration with DARPA and OSRF, and is common to
    MuJoCo and Gazebo. It limits access to the types of data normally available
    on robotic devices, so that it can be implemented on a physical device later.

    This API is tailored to the needs of the DARPA HAPTIX program, and makes
    certain assumptions about the model structure (see online documentation).

****************************************************************************************/


//-------------------------------- constants --------------------------------------------

// predefined sizes for array allocation
#define hxMAXMOTOR              32
#define hxMAXJOINT              32
#define hxMAXCONTACTSENSOR      32
#define hxMAXIMU                32


// API return codes
typedef enum
{
    hxOK = 0,                               // success
    hxERROR                                 // error; call hx_last_result for description
} hxResult;


//-------------------------------- data structures --------------------------------------

// simulation time (part of hxSensor)
struct _hxTime
{
    int sec;                                // seconds
    int nsec;                               // nanoseconds
};
typedef struct _hxTime hxTime;


// description of the robot model being simulated
struct _hxRobotInfo
{
    // array sizes
    int motor_count;                        // number of motors
    int joint_count;                        // number of hinge joints
    int contact_sensor_count;               // number of contact sensors
    int imu_count;                          // number of IMUs

    // model parameters
    float motor_limit[hxMAXMOTOR][2];       // minimum and maximum motor positions (motor units)
    float joint_limit[hxMAXJOINT][2];       // minimum and maximum joint angles (rad)
    float update_rate;                      // update rate supported by the robot (Hz)
};
typedef struct _hxRobotInfo hxRobotInfo;


// sensor data returned from simulator
struct _hxSensor
{
    // simulation time
    hxTime time_stamp;                      // simulation time at which the data was collected

    // motor data
    float motor_pos[hxMAXMOTOR];            // motor position (motor-specific units)
    float motor_vel[hxMAXMOTOR];            // motor velocity
    float motor_torque[hxMAXMOTOR];         // motor torque

    // joint data
    float joint_pos[hxMAXJOINT];            // joint position (rad)
    float joint_vel[hxMAXJOINT];            // joint velocity (rad/s)

    // contact data
    float contact[hxMAXCONTACTSENSOR];      // sum of contact normal forces within the sensor zone (N)

    // inertial measurement unit (IMU) data
    float imu_linear_acc[hxMAXIMU][3];      // linear acceleration in IMU frame, includes gravity (m/s^2)
    float imu_angular_vel[hxMAXIMU][3];     // angular velocity in IMU frame (rad/s)
    float imu_orientation[hxMAXIMU][4];     // IMU orientation quaternion; reserved for future use
};
typedef struct _hxSensor hxSensor;


// motor commands sent to simulator
struct _hxCommand
{
    // servo controller reference values
    float ref_pos[hxMAXMOTOR];              // reference position
    int ref_pos_enabled;                    // should ref_pos be updated in this call
    float ref_vel[hxMAXMOTOR];              // reference velocity
    int ref_vel_enabled;                    // should ref_vel be updated in this call

    // servo controller gains
    float gain_pos[hxMAXMOTOR];             // position feedback gain
    int gain_pos_enabled;                   // should gain_pos be updated in this call
    float gain_vel[hxMAXMOTOR];             // velocity feedback gain
    int gain_vel_enabled;                   // should gain_vel be updated in this call
};
typedef struct _hxCommand hxCommand;


//--------------------------- API functions ---------------------------------------------

// connect to specified host (name or IP address, NULL: local host); port is ignored
MJAPI hxResult hx_connect(const char* host, int port);

// close connection to simulator
MJAPI hxResult hx_close(void);

// get info about robot model presently loaded in simulator
MJAPI hxResult hx_robot_info(hxRobotInfo* robotinfo);

// update motor commands and read sensor data: blocking call, takes 1000/update_rate ms
MJAPI hxResult hx_update(const hxCommand* command, hxSensor* sensor);

// equivalent to calling hx_update with all XXX_enabled fields in hxCommand set to 0
MJAPI hxResult hx_read_sensors(hxSensor* sensor);

// text description of last hxResult returned by any API function call
MJAPI const char* hx_last_result(void);

// convert time structure to double
MJAPI double hx_double_time(const hxTime* time);



/****************************************************************************************

PART 2:  Native API  (prefix 'mj')

    This API provides more complete access to the simulator and does not make
    assumptions about the model structure, except for the maximum array sizes.
    It aims to maximize the benefits of using the simulator.

    The simple and native APIs can be mixed.
    
    Unlike the simple API where the user is expected to know the sizes of the
    variable-size arrays (by calling hx_robot_info), here the sizes of the
    variable-size arrays are included in the structures containing the arrays.

****************************************************************************************/


//-------------------------------- constants --------------------------------------------

// predefined size for array allocation
#define mjMAXSZ 1000


// API return codes
typedef enum
{
    mjCOM_OK            = 0,            // success

    // server-to-client errors
    mjCOM_BADSIZE       = -1,           // data has invalid size
    mjCOM_BADINDEX      = -2,           // object has invalid index
    mjCOM_BADTYPE       = -3,           // invalid object type
    mjCOM_BADCOMMAND    = -4,           // unknown command
    mjCOM_NOMODEL       = -5,           // model has not been loaded
    mjCOM_CANNOTSEND    = -6,           // could not send data
    mjCOM_CANNOTRECV    = -7,           // could not receive data
    mjCOM_TIMEOUT       = -8,           // receive timeout

    // client-side errors
    mjCOM_NOCONNECTION  = -9,           // connection not established
    mjCOM_CONNECTED     = -10,          // already connected
} mjtResult;


// type of geometric shape
typedef enum
{
    mjGEOM_PLANE = 0,                   // plane
    mjGEOM_HFIELD,                      // height field
    mjGEOM_SPHERE,                      // sphere
    mjGEOM_CAPSULE,                     // capsule
    mjGEOM_ELLIPSOID,                   // ellipsoid
    mjGEOM_CYLINDER,                    // cylinder
    mjGEOM_BOX,                         // box
    mjGEOM_MESH                         // mesh
} mjtGeom;


// type of sensor
typedef enum _mjtSensor             
{
    // common robotic sensors, attached to a site
    mjSENS_TOUCH        = 0,            // scalar contact normal forces summed over sensor zone
    mjSENS_ACCELEROMETER,               // 3D linear acceleration, in local frame
    mjSENS_VELOCIMETER,                 // 3D linear velocity, in local frame
    mjSENS_GYRO,                        // 3D angular velocity, in local frame
    mjSENS_FORCE,                       // 3D force between site's body and its parent body
    mjSENS_TORQUE,                      // 3D torque between site's body and its parent body
    mjSENS_MAGNETOMETER,                // 3D magnetometer
    mjSENS_RANGEFINDER,                 // scalar distance to nearest geom or site along z-axis

    // sensors related to scalar joints, tendons, actuators
    mjSENS_JOINTPOS,                    // scalar joint position (hinge and slide only)
    mjSENS_JOINTVEL,                    // scalar joint velocity (hinge and slide only)
    mjSENS_TENDONPOS,                   // scalar tendon position
    mjSENS_TENDONVEL,                   // scalar tendon velocity
    mjSENS_ACTUATORPOS,                 // scalar actuator position
    mjSENS_ACTUATORVEL,                 // scalar actuator velocity
    mjSENS_ACTUATORFRC,                 // scalar actuator force

    // sensors related to ball joints
    mjSENS_BALLQUAT,                    // 4D ball joint quaterion
    mjSENS_BALLANGVEL,                  // 3D ball joint angular velocity

    // sensors attached to an object with spatial frame: (x)body, geom, site, camera
    mjSENS_FRAMEPOS,                    // 3D position
    mjSENS_FRAMEQUAT,                   // 4D unit quaternion orientation
    mjSENS_FRAMEXAXIS,                  // 3D unit vector: x-axis of object's frame
    mjSENS_FRAMEYAXIS,                  // 3D unit vector: y-axis of object's frame
    mjSENS_FRAMEZAXIS,                  // 3D unit vector: z-axis of object's frame
    mjSENS_FRAMELINVEL,                 // 3D linear velocity
    mjSENS_FRAMEANGVEL,                 // 3D angular velocity
    mjSENS_FRAMELINACC,                 // 3D linear acceleration
    mjSENS_FRAMEANGACC,                 // 3D angular acceleration

    // sensors related to kinematic subtrees; attached to a body (which is the subtree root)
    mjSENS_SUBTREECOM,                  // 3D center of mass of subtree
    mjSENS_SUBTREELINVEL,               // 3D linear velocity of subtree
    mjSENS_SUBTREEANGMOM,               // 3D angular momentum of subtree

    // user-defined sensor
    mjSENS_USER                         // sensor data provided by mjcb_sensor callback
} mjtSensor;


// type of joint
typedef enum
{
    mjJNT_FREE = 0,                     // "joint" defining floating body
    mjJNT_BALL,                         // ball joint
    mjJNT_SLIDE,                        // sliding/prismatic joint
    mjJNT_HINGE                         // hinge joint
} mjtJoint;


// type of actuator transmission
typedef enum                            
{
    mjTRN_JOINT = 0,                    // force on joint
    mjTRN_JOINTINPARENT,                // force on joint, expressed in parent frame
    mjTRN_SLIDERCRANK,                  // force via slider-crank linkage
    mjTRN_TENDON,                       // force on tendon
    mjTRN_SITE                          // force on site
} mjtTrn;


// type of equality constraint
typedef enum
{
    mjEQ_CONNECT = 0,                   // connect two bodies at a point (ball joint)
    mjEQ_WELD,                          // fix relative position and orientation of two bodies
    mjEQ_JOINT,                         // couple the values of two scalar joints with cubic
    mjEQ_TENDON,                        // couple the lengths of two tendons with cubic
    mjEQ_DISTANCE                       // fix the contact distance between two geoms
} mjtEq;


//------------------------- Quantities that can be SET and GET --------------------------

// state of the dynamical system in generalized coordinates
struct _mjState
{
    int nq;                             // number of generalized positions
    int nv;                             // number of generalized velocities
    int na;                             // number of actuator activations
    float time;                         // simulation time
    float qpos[mjMAXSZ];                // generalized positions
    float qvel[mjMAXSZ];                // generalized velocities
    float act[mjMAXSZ];                 // actuator activations
};
typedef struct _mjState mjState;


// control signals
struct _mjControl
{
    int nu;                             // number of actuators
    float time;                         // simulation time
    float ctrl[mjMAXSZ];                // control signals
};
typedef struct _mjControl mjControl;


// applied forces
struct _mjApplied
{
    int nv;                             // number of generalized velocities
    int nbody;                          // number of bodies
    float time;                         // simulation time
    float qfrc_applied[mjMAXSZ];        // generalized forces
    float xfrc_applied[mjMAXSZ][6];     // Cartesian forces and torques applied to bodies
};
typedef struct _mjApplied mjApplied;


// detailed information about one body
struct _mjOneBody
{
    int bodyid;                         // body id, provided by user

    // get only
    int isfloating;                     // 1 if body is floating, 0 otherwise
    float time;                         // simulation time
    float linacc[3];                    // linear acceleration
    float angacc[3];                    // angular acceleration
    float contactforce[3];              // net force from all contacts on this body

    // get for all bodies; set for floating bodies only
    //  (setting the state of non-floating bodies would require inverse kinematics)
    float pos[3];                       // position
    float quat[4];                      // orientation quaternion
    float linvel[3];                    // linear velocity
    float angvel[3];                    // angular velocity

    // get and set for all bodies 
    //  (modular access to the same data as provided by mjApplied.xfrc_applied)
    float force[3];                     // Cartesian force applied to body CoM
    float torque[3];                    // Cartesian torque applied to body
};
typedef struct _mjOneBody mjOneBody;


// Cartesian positions and orientations of mocap bodies (treated as constant by simulator)
struct _mjMocap
{
    int nmocap;                         // number of mocap bodies
    float time;                         // simulation time
    float pos[mjMAXSZ][3];              // positions
    float quat[mjMAXSZ][4];             // quaternion orientations
};
typedef struct _mjMocap mjMocap;


//------------------------- Quantities that can be GET only -----------------------------

// main output of forward dynamics; used internally to integrate the state
struct _mjDynamics
{
    int nv;                             // number of generalized velocities
    int na;                             // number of actuator activations
    float time;                         // simulation time
    float qacc[mjMAXSZ];                // generalized accelerations
    float actdot[mjMAXSZ];              // time-derivatives of actuator activations
};
typedef struct _mjDynamics mjDynamics;


// sensor data; use the sensor desciptors in mjInfo to decode
struct _mjSensor
{
    int nsensordata;                    // size of sensor data array
    float time;                         // simulation time
    float sensordata[mjMAXSZ];          // sensor data array
};
typedef struct _mjSensor mjSensor;


// body positions and orientations in Cartesian coordinates (from forward kinematics)
struct _mjBody
{
    int nbody;                          // number of bodies
    float time;                         // simulation time
    float pos[mjMAXSZ][3];              // positions
    float mat[mjMAXSZ][9];              // frame orientations
};
typedef struct _mjBody mjBody;


// geom positions and orientations in Cartesian coordinates
struct _mjGeom
{
    int ngeom;                          // number of geoms
    float time;                         // simulation time
    float pos[mjMAXSZ][3];              // positions
    float mat[mjMAXSZ][9];              // frame orientations
};
typedef struct _mjGeom mjGeom;


// site positions and orientations in Cartesian coordinates
struct _mjSite
{
    int nsite;                          // number of sites
    float time;                         // simulation time
    float pos[mjMAXSZ][3];              // positions
    float mat[mjMAXSZ][9];              // frame orientations
};
typedef struct _mjSite mjSite;


// tendon lengths and velocities
struct _mjTendon
{
    int ntendon;                        // number of tendons
    float time;                         // simulation time
    float length[mjMAXSZ];              // tendon lengths
    float velocity[mjMAXSZ];            // tendon velocities    
};
typedef struct _mjTendon mjTendon;


// actuator lengths, velocities, and (scalar) forces in actuator space
struct _mjActuator
{
    int nu;                             // number of actuators
    float time;                         // simulation time
    float length[mjMAXSZ];              // actuator lengths
    float velocity[mjMAXSZ];            // actuator velocities
    float force[mjMAXSZ];               // actuator forces
};
typedef struct _mjActuator mjActuator;


// generalized forces acting on the system, resulting in dynamics:
//   M(qpos)*qacc = nonconstraint + constraint
struct _mjForce
{
    int nv;                             // number of generalized velocities/forces
    float time;                         // simulation time
    float nonconstraint[mjMAXSZ];       // sum of all non-constraint forces
    float constraint[mjMAXSZ];          // constraint forces (including contacts)
};
typedef struct _mjForce mjForce;


// information about all detected contacts
struct _mjContact
{
    int ncon;                           // number of detected contacts
    float time;                         // simulation time
    float dist[mjMAXSZ];                // contact normal distance
    float pos[mjMAXSZ][3];              // contact position in world frame
    float frame[mjMAXSZ][9];            // contact frame relative to world frame (0-2: normal)  
    float force[mjMAXSZ][3];            // contact force in contact frame   
    int geom1[mjMAXSZ];                 // id of 1st contacting geom    
    int geom2[mjMAXSZ];                 // id of 2nd contacting geom (force: 1st -> 2nd)
};
typedef struct _mjContact mjContact;


// static information about the model
struct _mjInfo
{
    // sizes
    int nq;                             // number of generalized positions
    int nv;                             // number of generalized velocities
    int na;                             // number of actuator activations
    int njnt;                           // number of joints
    int nbody;                          // number of bodies
    int ngeom;                          // number of geoms
    int nsite;                          // number of sites
    int ntendon;                        // number of tendons
    int nu;                             // number of actuators/controls
    int neq;                            // number of equality constraints
    int nkey;                           // number of keyframes
    int nmocap;                         // number of mocap bodies
    int nsensor;                        // number of sensors
    int nsensordata;                    // number of elements in sensor data array
    int nmat;                           // number of materials

    // timing parameters
    float timestep;                     // simulation timestep
    float apirate;                      // API update rate (same as hxRobotInfo.update_rate)

    // sensor descriptors
    int sensor_type[mjMAXSZ];           // sensor type (mjtSensor)
    int sensor_datatype[mjMAXSZ];       // type of sensorized object
    int sensor_objtype[mjMAXSZ];        // type of sensorized object
    int sensor_objid[mjMAXSZ];          // id of sensorized object
    int sensor_dim[mjMAXSZ];            // number of (scalar) sensor outputs
    int sensor_adr[mjMAXSZ];            // address in sensor data array
    float sensor_noise[mjMAXSZ];        // noise standard deviation

    // joint properties
    int jnt_type[mjMAXSZ];              // joint type (mjtJoint)
    int jnt_bodyid[mjMAXSZ];            // id of body to which joint belongs
    int jnt_qposadr[mjMAXSZ];           // address of joint position data in qpos
    int jnt_dofadr[mjMAXSZ];            // address of joint velocity data in qvel
    float jnt_range[mjMAXSZ][2];        // joint range; (0,0): no limits

    // geom properties
    int geom_type[mjMAXSZ];             // geom type (mjtGeom)
    int geom_bodyid[mjMAXSZ];           // id of body to which geom is attached

    // equality constraint properties
    int eq_type[mjMAXSZ];               // equality constraint type (mjtEq)
    int eq_obj1id[mjMAXSZ];             // id of constrained object
    int eq_obj2id[mjMAXSZ];             // id of 2nd constrained object; -1 if not applicable

    // actuator properties
    int actuator_trntype[mjMAXSZ];      // transmission type (mjtTrn)
    int actuator_trnid[mjMAXSZ][2];     // transmission target id
    float actuator_ctrlrange[mjMAXSZ][2]; // actuator control range; (0,0): no limits
};
typedef struct _mjInfo mjInfo;


//--------------------------- API get/set functions -------------------------------------

// get dynamic data from simulator
MJAPI mjtResult mj_get_state    (mjState* state);
MJAPI mjtResult mj_get_control  (mjControl* control);
MJAPI mjtResult mj_get_applied  (mjApplied* applied);
MJAPI mjtResult mj_get_onebody  (mjOneBody* onebody);
MJAPI mjtResult mj_get_mocap    (mjMocap* mocap);
MJAPI mjtResult mj_get_dynamics (mjDynamics* dynamics);
MJAPI mjtResult mj_get_sensor   (mjSensor* sensor);
MJAPI mjtResult mj_get_body     (mjBody* body);
MJAPI mjtResult mj_get_geom     (mjGeom* geom);
MJAPI mjtResult mj_get_site     (mjSite* site);
MJAPI mjtResult mj_get_tendon   (mjTendon* tendon);
MJAPI mjtResult mj_get_actuator (mjActuator* actuator);
MJAPI mjtResult mj_get_force    (mjForce* force);
MJAPI mjtResult mj_get_contact  (mjContact* contact);

// set dynamic data in simulator
MJAPI mjtResult mj_set_state    (const mjState* state);
MJAPI mjtResult mj_set_control  (const mjControl* control);
MJAPI mjtResult mj_set_applied  (const mjApplied* applied);
MJAPI mjtResult mj_set_onebody  (const mjOneBody* onebody);
MJAPI mjtResult mj_set_mocap    (const mjMocap* mocap);
MJAPI mjtResult mj_set_geomsize (int id, const float* size);

// get and set rgba static data in simulator
//  valid object types: geom, site, tendon, material
MJAPI mjtResult mj_get_rgba     (const char* type, int id, float* rgba);
MJAPI mjtResult mj_set_rgba     (const char* type, int id, const float* rgba);


//--------------------------- API command and information functions ---------------------

// connect to simulator
MJAPI mjtResult mj_connect(const char* host);

// close connection to simulator
MJAPI mjtResult mj_close(void);

// return last result code
MJAPI mjtResult mj_result(void);

// return 1 if connected to simulator, 0 otherwise
MJAPI int mj_connected(void);

// get static properties of current model
MJAPI mjtResult mj_info(mjInfo* info);

// advance simulation if paused, no effect if running
MJAPI mjtResult mj_step(void);

// set control, step if paused or wait for 1/apirate if running, get sensor data
MJAPI mjtResult mj_update(const mjControl* control, mjSensor* sensor);

// reset simulation to specified key frame; -1: reset to model reference configuration
MJAPI mjtResult mj_reset(int keyframe);

// modify state of specified equality constraint (1: enable, 0: disable)
MJAPI mjtResult mj_equality(int eqid, int state);

// show text message in simulator; NULL: clear currently shown message
MJAPI mjtResult mj_message(const char* message);

// return id of object with specified type and name; -1: not found; -2: error
//  valid object types: body, geom, site, joint, tendon, sensor, actuator, equality
MJAPI int mj_name2id(const char* type, const char* name);

// return name of object with specified type and id; NULL: error
MJAPI const char* mj_id2name(const char* type, int id);

// install user error handler
MJAPI void mj_handler(void(*handler)(int));


#if defined(__cplusplus)
}
#endif
