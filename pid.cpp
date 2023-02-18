/**
 * @file pid.cpp
 * @author Yauheni Kalosha (kevmn07@gmail.com)
 * @brief Base PID class, assesment
 * @version 0.1
 * @date 2023-02-15
 * 
 * SPDX_licencs_Identifier: GPL-3.0-or-later
 * @copyright Copyright (c) 2023 Yauheni Kalosha 
 * 
 */

#include "pid.hpp"

    /// @brief The Default constructor creates a Basic float-point PID controller
    ///        that unable to run without farther configuration
    base_pid::base_pid() :
        pv{nullptr},        // Process variable Input
        sp{nullptr},        // Setpoint Input
        tb{nullptr},        // Tieback Input, it directly drives the Controlthis Output in Manual mode
        co{nullptr},        // Control Output

        kp{0},              // Proportional Gain     
        ki{0},              // Integral Gain, redused to usec by multiplying by 1.0e-6
        kd{0},              // Differential Gain, redused to usec by multiplying by 1.0e+6
        db{0},              // Deadband
        pvll{-__FLT_MAX__}, // Process variable low limit
        pvhl{__FLT_MAX__},  // Process variable high limit
        spll{-__FLT_MAX__}, // Setpoint low limit
        sphl{__FLT_MAX__},  // Setpoint high limit
        coll{-__FLT_MAX__}, // Control output low limit
        cohl{__FLT_MAX__},  // Control output high limit
        dtmin{DT_MIN_PID},  // Minimum time interval between adjacent PID calculations expressed in usec
        db_on{false},       // Deadband On/Off
        man_on{true},       // Manual Mode On/Off

        lts{0},             // The last calculation timestamp
        lman_on{true},      // The last run Manual Mode On/Off  
        Iterm{0},           // Integral term
        lerr{0},             // The last calculated Error (sp - pv)
        tmp_co{0}           // The last calculated Control Output
        {};

    /// @brief The Simplified Constructor creates a basic float-point PID controller with minimal 
    ///        parameter set and Manual mode enabled
    /// @param ppv  Process variable pointer
    /// @param psp  Setpoint pointer
    /// @param pco  Control output pointer
    /// @param ptie Tieback variable pointe, Default value is nullptr 
    base_pid::base_pid(float* ppv, float* psp, float* pco, float* ptie) :
        pv{ppv},            // Process variable Input
        sp{psp},            // Setpoint Input
        tb{ptie},           // Tieback Input, it directly drives the Controlthis Output in Manual mode
        co{pco},            // Control Output
        kp{0},              // Proportional Gain     
        ki{0},              // Integral Gain, redused to usec by multiplying by 1.0e-6
        kd{0},              // Differential Gain, redused to usec by multiplying by 1.0e+6
        db{0},              // Deadband
        pvll{-__FLT_MAX__}, // Process variable low limit
        pvhl{__FLT_MAX__},  // Process variable high limit
        spll{-__FLT_MAX__}, // Setpoint low limit
        sphl{__FLT_MAX__},  // Setpoint high limit
        coll{-__FLT_MAX__}, // Control output low limit
        cohl{__FLT_MAX__},  // Control output high limit
        dtmin{DT_MIN_PID},  // Minimum time interval between adjacent PID calculations expressed in usec
        db_on{false},       // Deadband On/Off
        man_on{true},       // Manual Mode On/Off

        lts{0},             // The last calculation timestamp
        lman_on{true},      // The last run Manual Mode On/Off  
        Iterm{0},           // Integral term
        lerr{0},            // The last calculated Error (sp - pv)
        tmp_co{0}           // The last calculated Control Output
        {};

    /// @brief Constructor creates a basic float-point PID controller with full parameter set
    /// @param ppv    Process variable Input
    /// @param psp    Setpoint Input
    /// @param pco    Control Output
    /// @param ptie   Tieback Input, it directly drives the Controlthis Output in Manual mode
    /// @param kpv    Proportional Gain  
    /// @param kiv    Integral Gain, redused to usec by dividing by 1.0e+6
    /// @param kdv    Differential Gain, redused to usec by multiplying by 1.0e+6
    /// @param dbv    Deadband
    /// @param pvllv  Process variable low limit
    /// @param pvhlv  Process variable high limit
    /// @param spllv  Setpoint low limit
    /// @param sphlv  Setpoint high limit
    /// @param collv  Control output low limit
    /// @param cohlv  Control output high limit
    /// @param db_onv Deadband On/Off
    /// @param man_on Manual Mode On/Off
    /// @param dtminv Minimum time interval between adjacent PID calculations expressed in usec
    base_pid::base_pid(float* ppv, float* psp, float* pco, float* ptie,
                float kpv, float kiv, float kdv, float dbv,
                float pvllv, float pvhlv,
                float spllv, float sphlv,
                float collv, float cohlv,
                bool db_onv, bool man_onv, uint64_t dtminv) :
        pv{ppv},            // Process variable Input
        sp{psp},            // Setpoint Input
        tb{ptie},           // Tieback Input, it directly drives the Controlthis Output in Manual mode
        co{pco},            // Control Output
        kp{kpv},            // Proportional Gain     
        ki{kiv*(float)1.0e-6},     // Integral Gain, redused to usec by multiplying by 1.0e-6
        kd{kdv*(float)1.0e+6},     // Differential Gain, redused to usec by multiplying by 1.0e+6
        db{dbv},            // Deadband
        pvll{pvllv},        // Process variable low limit
        pvhl{pvhlv},        // Process variable high limit
        spll{spllv},        // Setpoint low limit
        sphl{sphlv},        // Setpoint high limit
        coll{collv},        // Control output low limit
        cohl{cohlv},        // Control output high limit
        dtmin{dtminv},      // Minimum time interval between adjacent PID calculations expressed in usec
        db_on{db_onv},      // Deadband On/Off
        man_on{man_onv},    // Manual Mode On/Off

        lts{0},             // The last calculation timestamp
        lman_on{true},      // The last run Manual Mode On/Off  
        Iterm{0},           // Integral term
        lerr{0},            // The last calculated Error (sp - pv)
        tmp_co{0}           // The last calculated Control Output                
        {};

        /// @brief Get Process variable limits
        /// @param ll  - Referense to the Low Level limiter of the Process variable value 
        /// @param hl  - Referense to the High Level limiter of the Process variable value
    void base_pid::get_pv_limits(float& ll, float& hl) {
        ll = pvll;
        hl = pvhl;
    };

        /// @brief Get Process variable limits 
        /// @param ll  - Referense to the Low Level limiter of the Process variable value 
        /// @param hl  - Referense to the High Level limiter of the Process variable value
        /// @return 0  - O'k
        ///         -1 - Error
    int base_pid::set_pv_limits(float& ll, float& hl) {
        // Verify limits order
        if (hl < ll) {
            pvll = hl;
            pvhl = ll;
            return -1;
        }
        pvll = ll;
        pvhl = hl;
        return 0;
    };


        /// @brief Get Setpoint limits
        /// @param ll  - Referense to the Low Level limiter of the Setpoint variable value 
        /// @param hl  - Referense to the High Level limiter of the Setpoint variable value
    void base_pid::get_sp_limits(float& ll, float& hl) {
            ll = spll;
            hl = sphl;
    };

        /// @brief Set Setpoint limits
        /// @param ll  - Referense to the Low Level limiter of the Process variable value 
        /// @param hl  - Referense to the High Level limiter of the Process variable value
        /// @return 0  - O'k
        ///         -1 - Error
    int base_pid::set_sp_limits(float& ll, float& hl) {
        // Verify limits order
        if (hl < ll) {
            spll = hl;
            sphl = ll;
            return -1;
        }
        spll = ll;
        sphl = hl;
        return 0;
    };

        /// @brief Get Control Outputs limits
        /// @param ll  - Referense to the Low Level limiter of the Setpoint variable value 
        /// @param hl  - Referense to the High Level limiter of the Setpoint variable value
    void base_pid::get_co_limits(float& ll, float& hl) {
        ll = coll;
        hl = cohl;
    };
    
        /// @brief Set Control Outputs limits
        /// @param ll  - Referense to the Low Level limiter of the Control Output variable value 
        /// @param hl  - Referense to the High Level limiter of the Control output variable value
        /// @return 0  - O'k
        ///         -1 - Error
    int base_pid::set_cp_limits(float& ll, float& hl) {
        // Verify limits order
        if (hl < ll) {
            coll = hl;
            cohl = ll;
            return -1;
        }
        coll = ll;
        cohl = hl;
        return 0;
    };
    
        /// @brief Get Gain parameters
        /// @param kpv - Referense to the Proportional Gain variable value 
        /// @param kiv - Referense to the Integer Gain variable value
        /// @param kdv - Referense to the Differential Gain variable value multiplified by 1.0e+6
    void base_pid::get_gain_param(float& kpv, float& kiv, float& kdv) {
        kpv = kp;
        kiv = ki * 1.0e+6 ;
        kdv = kd * 1.0e-6;
    };

        /// @brief Set Gain parameters
        /// @param kpv - Referense to the Proportional Gain variable value 
        /// @param kiv  - Referense to the Integer Gain variable value
        /// @param kdv  - Referense to the Differential Gain variable value
        /// @return 0  - O'k
        ///         -1 - Error
    int base_pid::set_gain_param(float& kpv, float& kiv, float& kdv) {
        // Check values 
        if ( kpv < -__FLT_MAX__ || kpv > __FLT_MAX__ ||
             kiv < -__FLT_MAX__ || kiv > __FLT_MAX__ || 
             kdv < -(__FLT_MAX__ * 1.0e-6) || kpv > (__FLT_MAX__ * 1.0e-6)) {
                return -1;
        }
        kp = kpv;
        ki = kiv * 1.0e-6;
        kd = kdv * 1.0e+6;
        return 0;
    };

        /// @brief Get Deadband parameters
        /// @param dbv - Referense to the Deadband variable value 
        /// @param db_onv - Referense to the Deadband mode switch
    void base_pid::get_db_param(float& dbv, bool& db_onv) {
        dbv = db;
        db_onv = db_on;
    };

        /// @brief Set Deadband parameters
        /// @param dbv - Referense to the Deadband variable value 
        /// @param db_onv - Referense to the Deadband mode switch
        /// @return 0  - O'k
        ///         -1 - Error
    int base_pid::set_db_param(float& dbv, bool& db_onv) {
        if (dbv < -__FLT_MAX__ || dbv > __FLT_MAX__) {
            return -1;
        }
        db = dbv;
        db_on = db_onv;
        return 0;
    }

        /// @brief Get Manual mode parameter
        /// @param man_onv - Referense to the Manual mode switch
    void base_pid::get_man_param(bool& man_onv) {
        man_onv = man_on;
    };

        /// @brief Set Manual mode parameter
        /// @param man_onv - Referense to the Manual mode switch
    void base_pid::set_man_param(bool& man_onv) {
        man_on = man_onv;
    };

        /// @brief Get Time Slice parameter
        /// @param dtminv - Referense to the Time Slice parameter expressed in usec
    void base_pid::get_dtmin_param(uint64_t& dtminv) {
        dtminv = dtmin;
    };

        /// @brief Set Time Slice parameter, 1 usec or more
        /// @param dtminv - Referense to the Time Slice parameter
        /// @return 0  - O'k
        ///         -1 - Error
    int base_pid::set_dtmin_param(uint64_t& dtminv) {
        dtmin = dtminv;
        if (dtmin == 0) {
            dtmin++;
            return -1;
        }
        return 0;
    };

        /// @brief Process Basic float-point PID controller calclation 
        /// @param tstamp - Time, when the calculation is performed
        /// @return 0  - O'k
        ///         -1 - Error
    int base_pid::run_pid(uint64_t tstamp) {
        // Check process variables
        if (pv == nullptr || sp == nullptr || co == nullptr) {
            return -1;
        }

        // Check dtmin param
        if (dtmin == 0) {
            dtmin == 1;
            return -1;
        }

        // Only update CO if no minimal time slice elapsed
        tmp_dt = tstamp - lts;
        if (tmp_dt < dtmin) {
            *co = tmp_co; 
            return 0;
        } 

        // Update lts
        lts = tstamp;

        // Tieback drives CO if Manual mode is enabled, but CO limits still apply.
        if (man_on) {
            // Check Tieback connection 
            tmp_co = (tb == nullptr) ? 0 : *tb;
            tmp_co = (tmp_co < coll) ? coll : (tmp_co > cohl) ? cohl : tmp_co;
            *co = tmp_co;
            lman_on = true;     // For future bumpless switching back
            return 0;
        }

        // Run bumpless if we come from Manual mode
        if (lman_on) {
            lman_on = false;
            // Set Iterm to the last co value
            Iterm = tmp_co; 
        }

        // Now we are ready to calculate the new co value
        tmp_err = *sp - *pv;

        // Skip further calculations if Deadband is Enabled
        // and we are in the Deadband region 
        if (db_on && tmp_err < db) {
            lerr = tmp_err;
            *co = tmp_co;
            return 0;
        }

        // Add Proportional kick
        tmp_co = kp * tmp_err;

        // Add Dterm and update lerr
        tmp_co += kd * (tmp_err - lerr) / (float)tmp_dt;
        lerr = tmp_err;

        // Process Iterm
        // Reset Iterm if ki == 0
        if (ki == 0) {
            Iterm = 0;
        }
        else {
            // Add the last Iterm, calculate Iterm delta,
            // and check results against the limits (anti-windup)
            d_iterm = ki * tmp_err * (float)tmp_dt;
            tmp_co += Iterm;
            if (!((tmp_co > cohl && d_iterm > 0) ||
                (tmp_co < coll && d_iterm < 0))) {
                tmp_co += d_iterm;
                Iterm  += d_iterm; 
            }
        }
        // Check results against limits and set Control output
        tmp_co = (tmp_co < coll) ? coll : (tmp_co > cohl) ? cohl : tmp_co;
        *co = tmp_co;
        return 0;
    };
