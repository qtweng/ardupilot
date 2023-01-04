#include "mode.h"
#include "Plane.h"

void ModeKU::_enter()
{
    plane.aileron_ku = 0;
    plane.elevator_ku = 0;
    plane.throttle_ku = 0;
    plane.rudder_ku = 0;
    plane.motor1 = 0;
    plane.motor2 = 0;
    plane.motor3 = 0;
    plane.motor4 = 0;
    return true;
}
void ModeKU::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.aileron_ku);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.elevator_ku);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.throttle_ku);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, plane.rudder_ku);
    return;
}
bool ModeKU::handle_KU_request(bool option, int32_t param0, int32_t param1, int32_t param2, int32_t param3)
{
    if (option == 0) {
        plane.aileron_ku = param0;
        plane.elevator_ku = param1;
        plane.throttle_ku = param2;
        plane.rudder_ku = param3;
    } else {
        plane.motor1 = param0;
        plane.motor2 = param1;
        plane.motor3 = param2;
        plane.motor4 = param3;
    }
    return true;
}
