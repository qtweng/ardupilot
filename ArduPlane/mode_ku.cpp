#include "mode.h"
#include "Plane.h"

void ModeKU::update()
{
    return;
}
bool ModeKU::handle_KU_request(int8_t option, int32_t param0, int32_t param1, int32_t param2, int32_t param3)
{
    if (option == 0) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, param0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, param1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, param2);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, param3);
    } else if (option == 1) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, param0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, param1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor3, param2);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor4, param3);
    } else if (option == 2) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor5, param0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor6, param1);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor7, param2);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor8, param3);
    }
    return true;
}
