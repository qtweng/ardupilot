#include <AP_HAL/AP_HAL.h>
#include "LN_Control.h"
    
extern const AP_HAL::HAL& hal;

int32_t LN_Control::nav_roll_cd(void) const
{
    // return the roll angle in centi-degrees
    return static_cast<int>(phicmd*18000/3.14159);
}

void LN_Control::init(float alt_init, float latL1, float latLN, int latN, const struct Location &home, const struct Location &wp) 
{
    alt0 = alt_init;

    L1lat = latL1;
    LNlat = latLN;
    Nlat = latN;

    path = 9;
    
    wptLOSerr = Eigen::VectorXf::Ones(Nlat) * INFINITY;
}
// update L1 control for waypoint navigation
bool LN_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP)
{

    // L1 switching logic
    float swdist = 50;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "wptLOSerr: %f", wptLOSerr[9]);
    for(int i = 9; i >= 0; --i){
        if(wptLOSerr[i] < swdist && i == Nlat-1 && path == 0){
            Vector3f prev_ned;
            if (!prev_WP.get_vector_from_origin_NEU(prev_ned)) {
                prev_ned = Vector3f(0, 0, 0);
            }
            Vector3f next_ned;
            if (!next_WP.get_vector_from_origin_NEU(next_ned)) {
                next_ned = Vector3f(0, 0, 0);
            }
            // Just got close enough to next waypoint so generate new line parameterization
            path = i;
            lineparam_n = wptparameterization(prev_ned*0.0328084, next_ned*0.0328084);
            return true;
        }
        if(wptLOSerr[i] < swdist and i < path){
            // Got close enough to next waypoint to move i-th line to new line
            path = i;
        }
        if(wptLOSerr[i] < swdist and i == 0){
            // Far enough through waypoint that there is no longer 2 different line params
            lineparam = lineparam_n;
        }
    }
    GDNC_lat_LN();
    return false;
}

Eigen::Matrix<float,2,3> LN_Control::wptparameterization(const Vector3f &prev_WP, const Vector3f &next_WP)
{
    // convert waypoints from Location to vectors
    Eigen::Vector3f next;
    next << next_WP[0], next_WP[1], next_WP[2];
    Eigen::Vector3f prev;
    prev << prev_WP[0], prev_WP[1], prev_WP[2];

    // prevent divide-by-zero on init
    Eigen::Vector3f v = (next - prev)/((next - prev).norm() + 0.0001);
    Eigen::Vector3f p0 = next;
    p0(2) = 0;

    Eigen::Matrix<float,2,3> line;
    line << p0(0), p0(1), p0(2),
            v(0), v(1), v(2);
    return line;
}
Eigen::Vector3f LN_Control::projectpoint(Eigen::Matrix<float,2,3> line, Eigen::Vector2f q)
{
    Eigen::Vector2f v;
    v << line.row(1).col(0), line.row(1).col(1);
    Eigen::Vector2f p;
    p << line.row(0).col(0), line.row(0).col(1);
    Eigen::Vector2f pq = q - p;
    auto dotpq_v = pq.dot(v);
    auto dotv_v = v.dot(v);
    // prevent divide-by-zero on init
    auto t = dotpq_v/(dotv_v + 0.0001);
    Eigen::Vector2f projq = p + t*v;

    Eigen::Vector3f projection;
    projection << projq[0], projq[1], 0;
    return projection;
}
void LN_Control::GDNC_lat_LN(){
    Vector3f uav_ned;
    if (!_ahrs.get_relative_position_NED_home(uav_ned)) {
        uav_ned = Vector3f(0, 0, 0);
    }
    uav_ned = uav_ned*3.28084;
    auto pN     = uav_ned[0];
    auto pE     = uav_ned[1];
    auto Vgps   = _ahrs.groundspeed_vector()*3.28084;
    auto pNdot  = Vgps[0];
    auto pEdot  = Vgps[1];
    Eigen::VectorXf LN     = Eigen::VectorXf::LinSpaced(Nlat, LNlat, L1lat);
    Eigen::VectorXf phicmds = Eigen::VectorXf::Zero(Nlat);

    // aircraft inertial position
    Eigen::Vector3f posAirNE;
    posAirNE << pN, pE, 0;
    // aircraft inertial velocity vector
    Eigen::Vector3f velAirNE;
    velAirNE << pNdot, pEdot, 0;


    for(int i = 0; i < Nlat; ++i){
        // in switching mode, use previous line parameters before path cutoff

        // waypoint vector
        Eigen::Vector3f vecWptNE;
        // waypoint target
        Eigen::Vector3f posWpt2NE;
        if(i < path){
            vecWptNE << lineparam.row(1).col(0), lineparam.row(1).col(1), 0;
            posWpt2NE << lineparam.row(0).col(0), lineparam.row(0).col(1), lineparam.row(0).col(2);
        }
        else{ 
        // if i is higher than path, it is looking at the waypoint ahead
            vecWptNE << lineparam_n.row(1).col(0), lineparam_n.row(1).col(1), 0;
            posWpt2NE << lineparam_n.row(0).col(0), lineparam_n.row(0).col(1), lineparam_n.row(0).col(2);
        }
        Eigen::Matrix<float,2,3> WptVec;
        WptVec <<   posWpt2NE(0), posWpt2NE(1), posWpt2NE(2),
                    vecWptNE(0), vecWptNE(1), vecWptNE(2);
        Eigen::Vector2f q;
        q << posAirNE[0], posAirNE[1];
        // projection of aircraft onto current
        Eigen::Vector3f posnNE = projectpoint(WptVec, q);
        // vector between intersection point and aircraft
        Eigen::Vector3f vecnNE = posnNE - posAirNE;
        auto lennNE = vecnNE.norm();
        auto lennL1 = safe_sqrt(sq(LN(i) - lennNE) + 0.0001);
        auto L1lataug = (LN(i) + lennNE + lennL1)/2;
        auto lenmNE   = safe_sqrt(abs(sq(L1lataug) - (sq(vecnNE(0)) + sq(vecnNE(1)) + sq(vecnNE(2))) + 0.0001));
        
        // L1 point on path

        Eigen::Vector3f posL1NE = posnNE + vecWptNE*lenmNE;
        // Waypoint switching range
        wptLOSerr(i) = (posWpt2NE - posL1NE).norm();

        // vector from aircraft to L1 point
        Eigen::Vector3f vecL1NE = posL1NE - posAirNE;

        // calculate phicmds according to L1
        Eigen::Vector3f sinetaupperNE = velAirNE.cross(vecL1NE);
        // rolldir = sign(sinetaupperNE(2))
        auto rolldir  = (0 < sinetaupperNE(2)) - (sinetaupperNE(2) < 0);
        auto sinetaNE = sinetaupperNE.norm()/(velAirNE.norm()*vecL1NE.norm());
        auto alat     = 2*(velAirNE).array().square().sum()*sinetaNE/(vecL1NE.norm());
        phicmds(i) = 0.27*i*rolldir*atanf(alat/32.2);
    }

    phicmd = phicmds.mean();
    phicmd = 0.5*(phicmd + -25/57.3 + safe_sqrt(sq(phicmd - -25/57.3) + 0.0001));
    phicmd = 0.5*(phicmd + 25/57.3 - safe_sqrt(sq(phicmd - 25/57.3) + 0.0001));
}
