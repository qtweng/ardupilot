#pragma once

//To include Eigen https://discuss.ardupilot.org/t/add-new-library-to-the-waf-build-system/62812/7
#include "Eigen/Core"
#include "Eigen/Dense"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_TECS/AP_TECS.h>

class LN_Control{
    public:
        LN_Control(AP_AHRS &ahrs)
            : _ahrs(ahrs)
        {}
        /* Do not allow copies */
        CLASS_NO_COPY(LN_Control);


        void init(float alt_init, float latL1, float latLN, int latN, const struct Location &home, const struct Location &wp);

        int32_t nav_roll_cd(void) const;
        bool update_waypoint(const struct Location &prev_WP, const struct Location &next_WP);

        Eigen::Matrix<float,2,3> wptparameterization(const struct Location &prev_WP, const struct Location &next_WP);
        Eigen::Vector3f projectpoint(Eigen::Matrix<float,2,3> lineparam, Eigen::Vector2f q);
        void GDNC_lat_LN(const struct Location &prev_WP, const struct Location &next_WP);
        void GDNC_lon_LN(const struct Location &prev_WP, const struct Location &next_WP);

    private:
        // reference to the AHRS object
        AP_AHRS &_ahrs;

        // pointer to the SpdHgtControl object
        const AP_TECS *_tecs;

        float alt0;

        float L1lat;
        float LNlat;
        int Nlat;

        int path;

        Eigen::Matrix<float,2,3> lineparam;
        Eigen::Matrix<float,2,3> lineparam_n;
        Eigen::VectorXf wptLOSerr;

        float phicmd;
};