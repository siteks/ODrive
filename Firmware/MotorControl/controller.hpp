#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class Controller {
public:
    // Note: these should be sorted from lowest level of control to
    // highest level of control, to allow "<" style comparisons.
    enum ControlMode_t{
        CTRL_MODE_VOLTAGE_CONTROL = 0,
        CTRL_MODE_CURRENT_CONTROL = 1,
        CTRL_MODE_VELOCITY_CONTROL = 2,
        CTRL_MODE_POSITION_CONTROL = 3,
        CTRL_MODE_TRAJECTORY_CONTROL = 4
    };

    struct Config_t {
        ControlMode_t control_mode = CTRL_MODE_POSITION_CONTROL;  //see: Motor_control_mode_t
        float pos_gain = 20.0f;  // [(counts/s) / counts]
        float vel_gain = 5.0f / 10000.0f;  // [A/(counts/s)]
        float pos_integrator_gain = 0.0f;
        // float vel_gain = 5.0f / 200.0f, // [A/(rad/s)] <sensorless example>
        float vel_integrator_gain = 10.0f / 10000.0f;  // [A/(counts/s * s)]
        float vel_limit = 20000.0f;           // [counts/s]
        bool use_anticogging = false;
        // The default task needs more stack for this to work, expanded from 256 to 1024
        // line 177 freertos.c
        float fft[256];
    };
    float cogging_map[4096];
    static const int N = 4096;
    static const int Nc = 128;
    float coeffs[N];

    Controller(Config_t& config);
    void reset();

    void set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward);
    void set_vel_setpoint(float vel_setpoint, float current_feed_forward);
    void set_current_setpoint(float current_setpoint);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);

    bool update(float pos_estimate, float vel_estimate, float* current_setpoint);

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    // TODO: anticogging overhaul:
    // - expose selected (all?) variables on protocol
    // - make calibration user experience similar to motor & encoder calibration
    // - use python tools to Fourier transform and write back the smoothed map or Fourier coefficients
    // - make the calibration persistent

    typedef struct {
        int index;
        //float cogging_map[4096];
        //bool use_anticogging;
        bool calib_anticogging;
        float calib_pos_threshold;
        float calib_vel_threshold;
    } Anticogging_t;
    Anticogging_t anticogging_ = {
        .index = 0,
        .calib_anticogging = false,
        .calib_pos_threshold = 5.0f,
        .calib_vel_threshold = 1000.0f,
    };
    bool valid_cogging_map = false;
    float read_fft_map(uint32_t index)
    {
        return config_.fft[index];
    }
    void write_fft_map(uint32_t index, float val)
    {
        config_.fft[index] = val;
    }
    void calc_cogging_map();
    float read_cogging_map(uint32_t index)
    {
        return cogging_map[index];
    }
    void write_cogging_map(uint32_t index, float val)
    {
        cogging_map[index] = val;
    }
    void clear_cogging_map()
    {
        valid_cogging_map = false;
        for (int i = 0; i < 4096; i++)
            cogging_map[i] = 0;
    }


    // variables exposed on protocol
    float pos_setpoint_ = 0.0f;
    float vel_setpoint_ = 0.0f;
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_current_ = 0.0f;  // [A]
    float current_setpoint_ = 0.0f;        // [A]
    float pos_integrator_current = 0.0f;
    uint32_t cogging_map_ptr = 0; // protocol doesn't recognise int, needs to be uint32_t
    

    uint32_t traj_start_loop_count_ = 0;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("pos_setpoint", &pos_setpoint_),
            make_protocol_property("vel_setpoint", &vel_setpoint_),
            make_protocol_property("vel_integrator_current", &vel_integrator_current_),
            make_protocol_property("current_setpoint", &current_setpoint_),
            make_protocol_object("config",
                make_protocol_property("control_mode", &config_.control_mode),
                make_protocol_property("pos_gain", &config_.pos_gain),
                make_protocol_property("vel_gain", &config_.vel_gain),
                make_protocol_property("pos_integrator_gain", &config_.pos_integrator_gain),
                make_protocol_property("vel_integrator_gain", &config_.vel_integrator_gain),
                make_protocol_property("vel_limit", &config_.vel_limit),
                make_protocol_property("use_anticogging", &config_.use_anticogging)
            ),
            make_protocol_object("anticogging",
                make_protocol_property("calib_anticogging", &anticogging_.calib_anticogging),
                make_protocol_property("calib_pos_threshold", &anticogging_.calib_pos_threshold),
                make_protocol_property("calib_vel_threshold", &anticogging_.calib_vel_threshold)
            ),
            make_protocol_property("valid_cogging_map", &valid_cogging_map),
            make_protocol_function("read_fft_map", *this, &Controller::read_fft_map, "index"),
            make_protocol_function("write_fft_map", *this, &Controller::write_fft_map, "index", "val"),
            make_protocol_function("calc_cogging_map", *this, &Controller::calc_cogging_map),
            make_protocol_function("read_cogging_map", *this, &Controller::read_cogging_map, "index"),
            make_protocol_function("write_cogging_map", *this, &Controller::write_cogging_map, "index", "val"),
            make_protocol_function("clear_cogging_map", *this, &Controller::clear_cogging_map),
            make_protocol_function("set_pos_setpoint", *this, &Controller::set_pos_setpoint,
                "pos_setpoint", "vel_feed_forward", "current_feed_forward"),
            make_protocol_function("set_vel_setpoint", *this, &Controller::set_vel_setpoint,
                "vel_setpoint", "current_feed_forward"),
            make_protocol_function("set_current_setpoint", *this, &Controller::set_current_setpoint,
                "current_setpoint"),
            make_protocol_function("move_to_pos", *this, &Controller::move_to_pos, "goal_point"),
            make_protocol_function("start_anticogging_calibration", *this, &Controller::start_anticogging_calibration)
        );
    }
};

#endif // __CONTROLLER_HPP
