/**
 * @file pid.hpp
 * @author Yauheni Kalosha (kevmn07@gmail.com)
 * @brief Header for Base PID class
 * @version 0.1
 * @date 2023-02-15
 * 
 * SPDX_licencs_Identifier: GPL-3.0-or-later
 * @copyright Copyright (c) 2023 Yauheni Kalosha 
 */
#ifndef _PID_H
#define _PID_H

#include <cstdint>

#define DT_MIN_PID 10 // Minimal PID time slice expressed in us

/// @brief Basic float-point PID controller, Independent Gain mode only
class base_pid {

    // Variables used to store intermediate calculation results
    uint64_t tmp_dt;
    float tmp_co;
    float tmp_err;
    float d_iterm;

    protected :
        float* pv;     // Process variable Input
        float* sp;     // Setpoint Input
        float* tb;     // Tieback Input, it directly drives the Controlthis Output in Manual mode
        float* co;     // Control Output

        float kp;       // Proportional Gain     
        float ki;       // Integral Gain, redused to usec by dividing by 1.0e+6
        float kd;       // Differential Gain, redused to usec by multiplying by 1.0e+6
        float db;       // Deadband
        float pvll;     // Process variable low limit
        float pvhl;     // Process variable high limit
        float spll;     // Setpoint low limit
        float sphl;     // Setpoint high limit
        float coll;     // Control output low limit
        float cohl;     // Control output high limit
        uint64_t dtmin; // Minimum time interval between adjacent PID calculations expressed in us
        bool  db_on;    // Deadband On/Off
        bool  man_on;   // Manual Mode On/Off

        uint64_t lts;   // The last calculation timestamp
        bool lman_on;   // The last run Manual Mode On/Off        
        float Iterm;    // Integral term
        float lerr;      // The last calculated Error (sp - pv)

    public:
        /// @brief Constructors
        base_pid();
        base_pid(float* ppv, float* psp, float* pco, float* ptie = nullptr);
        base_pid(float* ppv, float* psp, float* pco, float* ptie,
                float kpv, float kiv, float kdv, float dbv,
                float pvllv = -__FLT_MAX__, float pvhlv = __FLT_MAX__,
                float spllv = -__FLT_MAX__, float sphlv = __FLT_MAX__,
                float collv = -__FLT_MAX__, float cohlv = __FLT_MAX__,
                bool db_onv = false, bool man_on = true, uint64_t dtminv = DT_MIN_PID);

        /// @brief Get Process variable limits
        void get_pv_limits(float& ll, float& hl);

        /// @brief Get Process variable limits 
        int set_pv_limits(float& ll, float& hl);

        /// @brief Get Setpoint limits
        void get_sp_limits(float& ll, float& hl);

        /// @brief Set Setpoint limits
        int set_sp_limits(float& ll, float& hl);

        /// @brief Get Control Outputs limits
        void get_co_limits(float& ll, float& hl);

        /// @brief Set Control Outputs limits
        int set_cp_limits(float& ll, float& hl);

        /// @brief Get Gain parameters
        void get_gain_param(float& kpv, float& kiv, float& kdv);

        /// @brief Set Gain parameters
        int set_gain_param(float& kpv, float& kiv, float& kdv);

        /// @brief Get Deadband parameters
        void get_db_param(float& dbv, bool& db_onv);

        /// @brief Set Deadband parameters
        int set_db_param(float& dbv, bool& db_onv);

        /// @brief Get Manual mode parameter
        void get_man_param(bool& man_onv);

        /// @brief Set Manual mode parameter
        void set_man_param(bool& man_onv);

        /// @brief Get Time Slice parameter
        void get_dtmin_param(uint64_t& dtminv);

        /// @brief Set Time Slice parameter, 1 usec or more
        int set_dtmin_param(uint64_t& dtminv);

        /// @brief Process Basic float-point PID controller calclation 
        int run_pid(uint64_t tstamp);
    };

#endif /* _PID_H */