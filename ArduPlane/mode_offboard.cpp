#include "mode.h"
#include "Plane.h"

void ModeOffboard::run()
{
    reset_controllers();
}
void ModeOffboard::handle_offboard_request(uint8_t group_mlx, const float control_value[8])
{
    if (group_mlx == 0){
        SRV_Channel::Aux_servo_function_t servo_function_map[8] = {
            SRV_Channel::k_aileron,
            SRV_Channel::k_elevator,
            SRV_Channel::k_rudder,
            SRV_Channel::k_throttle,
            SRV_Channel::k_flap,
            SRV_Channel::k_dspoilerLeft1,
            SRV_Channel::k_airbrake,
            SRV_Channel::k_landing_gear_control
        };

        // Set the actuator controls
        for (uint8_t i = 0; i < 8; i++) {
            float control = control_value[i];
            SRV_Channel::Aux_servo_function_t servo_function = servo_function_map[i];
            SRV_Channels::set_output_norm(servo_function, control);
        }
    }
}
