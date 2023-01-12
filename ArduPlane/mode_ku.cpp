#include "mode.h"
#include "Plane.h"

void ModeKU::update()
{
    return;
}
bool ModeKU::handle_KU_request(int32_t param1, int32_t param2, int32_t param3, int32_t param4, int32_t param5)
{
    if (param5 == 0) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, param1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, param2);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, param3);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, param4);
    } else if (param5 == 1) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, param1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, param2);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor3, param3);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor4, param4);
    } else if (param5 == 2) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor5, param1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor6, param2);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor7, param3);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor8, param4);
    }
    plane.servos_output();
    return true;
}
