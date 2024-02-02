#pragma once

//To include Eigen https://discuss.ardupilot.org/t/add-new-library-to-the-waf-build-system/62812/7
#include <stdio.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_TECS/AP_TECS.h>

class LN_Control{
    public:
        LN_Control(AP_AHRS &ahrs)
            : _ahrs(ahrs)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }
        /* Do not allow copies */
        CLASS_NO_COPY(LN_Control);


        void init();

        int32_t nav_roll_cd(void) const;
        bool update_waypoint(const struct Location &prev_WP, const struct Location &next_WP);

        Eigen::Matrix<float,2,3> wptparameterization(const Vector3f &prev_WP, const Vector3f &next_WP);
        Eigen::Vector3f projectpoint(Eigen::Matrix<float,2,3> lineparam, Eigen::Vector2f q);
        void GDNC_lat_LN();
        void GDNC_lon_LN(const struct Location &prev_WP, const struct Location &next_WP);

        // this supports the LN_Control* user settable parameters
        static const struct AP_Param::GroupInfo var_info[];

    private:
        // reference to the AHRS object
        AP_AHRS &_ahrs;

        // pointer to the SpdHgtControl object
        const AP_TECS *_tecs;

        AP_Float _alt0;

        AP_Float _LNlat;
        AP_Float _L1lat;
        AP_Int8 _Nlat;
        float LNlat;
        float L1lat;
        int Nlat;

        AP_Float _swdist;
        
        int path;

        Eigen::Matrix<float,2,3> lineparam;
        Eigen::Matrix<float,2,3> lineparam_n;

        Eigen::VectorXf wptLOSerr;
        Eigen::VectorXf LN;
        Eigen::VectorXf phicmds;

        float phicmd;
};
