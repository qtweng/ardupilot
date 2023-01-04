#include "mode.h"
#include "Plane.h"

void ModeKU::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
    return
}
bool ModeKU::handle_KU_request(bool option, int32_t param0, int32_t param1, int32_t param2, int32_t param3)
{
    if (option == 0) {
        aileron = param0;
        elevator = param1;
        throttle = param2;
        rudder = param3;
    } else {
        motor1 = param0;
        motor2 = param1;
        motor3 = param2;
        motor4 = param3;
    }
    return true;
}
