#include "haptix.h"
#include "string.h"
#include "math.h"

#include "stdio.h"

hxRobotInfo info;
hxCommand command;
hxSensor sensor;

mjApplied applied;

int main(int argc, char* argv[])
{
    int i, grasp = 0;
    double signal, interp, lastchange = 0;

    // optional argument: IP address or host name of remote simulator machine
    if( argc==2 )
        hx_connect(argv[1], 0);
    else
        hx_connect(0, 0);

    // clear hxCommand, prepare to update ref_pos field only
    memset(&command, 0, sizeof(hxCommand));
    command.ref_pos_enabled = 1;

    // get hxRobotInfo and hxSensor, show message
    hx_robot_info(&info);
    hx_read_sensors(&sensor);
    mj_message("grasp reflex");

    // update loop: 2 min
    while( sensor.time_stamp.sec<120 )
    {
        // grasp: gradually move ref_pos towards position 2/3 in-between motor limits
        //  first three motors control the wrist, so leave their ref_pos = 0
        if( grasp )
        {
            // interpolation factor for closing in 0.5 sec
            interp = 2.0*(hx_double_time(&sensor.time_stamp)-lastchange);
            if( interp>1 )
                interp = 1;

            // set interpolated ref_pos
            for( i=3; i<info.motor_count; i++ )
                command.ref_pos[i] = (float)(interp/3.0*
                    (2.0*info.motor_limit[i][1] + info.motor_limit[i][0]));
        }

        // release: ref_pos = 0
        else
            for( i=3; i<info.motor_count; i++ )
                command.ref_pos[i] = 0;

        // send motor commands and get new sensor data
        hx_update(&command, &sensor);

        // allow grasp/release change 2 sec after last change
        if( hx_double_time(&sensor.time_stamp)>=lastchange+2.0 )
        {
            // grasp if contact and not currently grasping
            if( !grasp )
            {
                // sum readings from all contact sensors
                signal = 0;
                for( i=0; i<info.contact_sensor_count; i++ )
                    signal += sensor.contact[i];

                // grasp if net contact force is over 0.1N
                if( signal>0.1 )
                {
                    grasp = 1;
                    lastchange = hx_double_time(&sensor.time_stamp);
                }
            }

            // release if rotation and currently grasping
            else if( grasp )
            {
                // sum readings from all imu angluar velocities
                signal = 0;
                for( i=0; i<info.imu_count; i++ )
                    signal += fabs(sensor.imu_angular_vel[i][0]) +
                              fabs(sensor.imu_angular_vel[i][1]) +
                              fabs(sensor.imu_angular_vel[i][2]);

                // release if net angular velocity is over 50 rad/s
                if( signal>50.0 )
                {
                    grasp = 0;
                    lastchange = hx_double_time(&sensor.time_stamp);
                }
            }
        }
    }

    // clear message, close connection
    mj_message(0);
    hx_close();
    return 0;
}