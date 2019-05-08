
#include "odrive_main.h"


Encoder::Encoder(const EncoderHardwareConfig_t& hw_config,
                Config_t& config) :
        hw_config_(hw_config),
        config_(config)
{
    update_pll_gains();
#if 1
    if (config.pre_calibrated) {
#else
    if (config.pre_calibrated && (config.mode == Encoder::MODE_HALL || config.mode == Encoder::MODE_SINCOS)) {
#endif
        is_ready_ = true;
    }
}

static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}


// The TLE5012E1000 seemed ideal as a simple magnetic incremental encoder, except that
// it has a default setting for direction change hysteresis of 0.703 degrees, discovered
// only after building some boards and trying it out with a motor. This is enough
// to cause oscillation. It can be changed using the serial SPI'ish interface but this is
// not currently supported in the code. Below is a nasty hack to make it work, reusing the 
// current A, B, Z pins as clk, csq, and data. On the positive side, this will give absolute 
// encoder positions


#define GPIO_set_dir_input(bank, pin)       \
{GPIO_InitTypeDef x =                   \
    {   .Pin = pin,                     \
        .Mode = GPIO_MODE_INPUT,        \
        .Pull = GPIO_NOPULL             \
    };                                  \
HAL_GPIO_Init(bank, &x);}
#define GPIO_set_dir_output(bank, pin, val)      \
{GPIO_InitTypeDef x =                   \
    {   .Pin = pin,                     \
        .Mode = GPIO_MODE_OUTPUT_PP,    \
        .Pull = GPIO_NOPULL             \
    };                                  \
HAL_GPIO_WritePin(bank, pin, val);      \
HAL_GPIO_Init(bank, &x);}



static void send_enc_word(const EncoderHardwareConfig_t &hw_config, int data)
{
    for(int i = 0; i < 16; i++, data<<=1)
    {
        // Max toggle rate tested gave ~90ns high and low widths, datasheet needs >40ns
        // so no delays should be fine here
        HAL_GPIO_WritePin(hw_config.sck_port, hw_config.sck_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(hw_config.data_port, hw_config.data_pin, (data & 0x8000) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hw_config.sck_port, hw_config.sck_pin, GPIO_PIN_RESET);
    }
}

static uint16_t read_enc_register(const EncoderHardwareConfig_t &hw_config, int addr)
{

    // We can read multiple registers but just do single for now. If words is zero, 
    // one register is read but there is no safety word. Any number  greater then zero
    // results in that many words being read, with an additional safety word following.
    //
    // The data pin must turn around from output to input
    //              read        unlock        update      addr          words
    uint16_t val = (1 << 15) | (0xc << 11) | (0 << 10) | (addr << 4) | (0 << 0);

    GPIO_set_dir_output(hw_config.data_port, hw_config.data_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw_config.csq_port, hw_config.csq_pin, GPIO_PIN_RESET);
    delay_us(1);    // tcss 105ns
    send_enc_word(hw_config, val);
    GPIO_set_dir_input(hw_config.data_port, hw_config.data_pin);

    uint16_t data = 0;
    for(int i = 0; i < 16; i++)
    {
        HAL_GPIO_WritePin(hw_config.sck_port, hw_config.sck_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(hw_config.sck_port, hw_config.sck_pin, GPIO_PIN_RESET);
        data = (data << 1) | (HAL_GPIO_ReadPin(hw_config.data_port, hw_config.data_pin) ? 1 : 0);
    }
    HAL_GPIO_WritePin(hw_config.csq_port, hw_config.csq_pin, GPIO_PIN_SET);
    return data;
}

static void write_enc_register(const EncoderHardwareConfig_t &hw_config, int addr, uint16_t data)
{
    //              read        unlock        update      addr          words
    uint16_t val = (0 << 15) | (0xa << 11) | (0 << 10) | (addr << 4) | (0 << 0);

    GPIO_set_dir_output(hw_config.data_port, hw_config.data_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw_config.csq_port, hw_config.csq_pin, GPIO_PIN_RESET);
    delay_us(1);    // tcss 105ns
    send_enc_word(hw_config, val);
    send_enc_word(hw_config, data);
    GPIO_set_dir_input(hw_config.data_port, hw_config.data_pin);
    HAL_GPIO_WritePin(hw_config.csq_port, hw_config.csq_pin, GPIO_PIN_SET);
}

static void set_hysteresis(const EncoderHardwareConfig_t &hw_config, int h)
{
    uint16_t data = read_enc_register(hw_config, 0xd);
    data = (data & 0xfffc) | h;
    write_enc_register(hw_config, 0xd, data);
}

static int16_t get_enc_position(const EncoderHardwareConfig_t &hw_config)
{
    int16_t a = read_enc_register(hw_config, 2) & 0x7fff; 
    return a;
}

void Encoder::setup() {
#ifndef ENCODER_TYPE_TLE5012
    HAL_TIM_Encoder_Start(hw_config_.timer, TIM_CHANNEL_ALL);
    GPIO_subscribe(hw_config_.index_port, hw_config_.index_pin, GPIO_NOPULL,
            enc_index_cb_wrapper, this);
    set_idx_subscribe();
#else
    //if (hw_config_.sck_pin == M1_ENC_Z_Pin) return;

    GPIO_set_dir_output(hw_config_.csq_port, hw_config_.csq_pin, GPIO_PIN_SET);
    GPIO_set_dir_output(hw_config_.sck_port, hw_config_.sck_pin, GPIO_PIN_RESET);
    GPIO_set_dir_output(hw_config_.data_port, hw_config_.data_pin, GPIO_PIN_RESET);

    set_hysteresis(hw_config_, 0);
    delay_us(10);
    set_hysteresis(hw_config_, 0);
    delay_us(10);

    if (config_.pre_calibrated)
        is_ready_ = true;


#endif
}

void Encoder::set_error(Error_t error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_ENCODER_FAILED;
}

bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
}

//--------------------
// Hardware Dependent
//--------------------

// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
// (maybe by attaching the interrupt on start search, synergistic with following)
void Encoder::enc_index_cb() {
    if (config_.use_index) {
        set_circular_count(0, false);
        if (config_.zero_count_on_find_idx)
            set_linear_count(0); // Avoid position control transient after search
        if (config_.pre_calibrated) {
            is_ready_ = true;
        } else {
            // We can't use the update_offset facility in set_circular_count because
            // we also set the linear count before there is a chance to update. Therefore:
            // Invalidate offset calibration that may have happened before idx search
            is_ready_ = false;
        }
        index_found_ = true;
    }
#ifndef ENCODER_TYPE_TLE5012
    // Disable interrupt
    GPIO_unsubscribe(hw_config_.index_port, hw_config_.index_pin);
#endif
}
void Encoder::set_idx_subscribe(bool override_enable) {
#ifndef ENCODER_TYPE_TLE5012
    if (config_.use_index && (override_enable || !config_.find_idx_on_lockin_only)) {
        GPIO_subscribe(hw_config_.index_port, hw_config_.index_pin, GPIO_PULLDOWN,
                enc_index_cb_wrapper, this);
    } else if (!config_.use_index || config_.find_idx_on_lockin_only) {
        GPIO_unsubscribe(hw_config_.index_port, hw_config_.index_pin);
    }
#endif
}

void Encoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

void Encoder::check_pre_calibrated() {
    if (!is_ready_)
        config_.pre_calibrated = false;
    if (config_.mode == MODE_INCREMENTAL && !index_found_)
        config_.pre_calibrated = false;
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    shadow_count_ = count;
    pos_estimate_ = (float)count;
    //Write hardware last
    //hw_config_.timer->Instance->CNT = raw_count_multiplier * count;

    cpu_exit_critical(prim);
}

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count, bool update_offset) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    if (update_offset) {
        config_.offset += count - count_in_cpr_;
        config_.offset = mod(config_.offset, config_.cpr);
    }

    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr_ = (float)count_in_cpr_;

    cpu_exit_critical(prim);
}

bool Encoder::run_index_search() {
    config_.use_index = true;
    index_found_ = false;
    if (!config_.idx_search_unidirectional && axis_->motor_.config_.direction == 0) {
        axis_->motor_.config_.direction = 1;
    }

    bool orig_finish_on_enc_idx = axis_->config_.lockin.finish_on_enc_idx;
    axis_->config_.lockin.finish_on_enc_idx = true;
    bool status = axis_->run_lockin_spin();
    axis_->config_.lockin.finish_on_enc_idx = orig_finish_on_enc_idx;
    return status;
}

bool Encoder::run_direction_find() {
    int32_t init_enc_val = shadow_count_;
    bool orig_finish_on_distance = axis_->config_.lockin.finish_on_distance;
    axis_->config_.lockin.finish_on_distance = true;
    axis_->motor_.config_.direction = 1; // Must test spin forwards for direction detect logic
    bool status = axis_->run_lockin_spin();
    axis_->config_.lockin.finish_on_distance = orig_finish_on_distance;

    if (status) {
        // Check response and direction
        if (shadow_count_ > init_enc_val + 8) {
            // motor same dir as encoder
            axis_->motor_.config_.direction = 1;
        } else if (shadow_count_ < init_enc_val - 8) {
            // motor opposite dir as encoder
            axis_->motor_.config_.direction = -1;
        } else {
            axis_->motor_.config_.direction = 0;
        }
    }

    return status;
}

// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
// TODO: Do the scan with current, not voltage!
bool Encoder::run_offset_calibration() {
    static const float start_lock_duration = 1.0f;
    static const int num_steps = (int)(config_.calib_scan_distance / config_.calib_scan_omega * (float)current_meas_hz);

    // Require index found if enabled
    if (config_.use_index && !index_found_) {
        set_error(ERROR_INDEX_NOT_FOUND_YET);
        return false;
    }

    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    shadow_count_ = count_in_cpr_;

    float voltage_magnitude;
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_HIGH_CURRENT)
        voltage_magnitude = axis_->motor_.config_.calibration_current * axis_->motor_.config_.phase_resistance;
    else if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL)
        voltage_magnitude = axis_->motor_.config_.calibration_current;
    else
        return false;

    // go to motor zero phase for start_lock_duration to get ready to scan
    int i = 0;
    axis_->run_control_loop([&](){
        if (!axis_->motor_.enqueue_voltage_timings(voltage_magnitude, 0.0f))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);
        return ++i < start_lock_duration * current_meas_hz;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    int32_t init_enc_val = shadow_count_;
    int64_t encvaluesum = 0;

    // scan forward
    i = 0;
    axis_->run_control_loop([&](){
        float phase = wrap_pm_pi(config_.calib_scan_distance * (float)i / (float)num_steps - config_.calib_scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * our_arm_cos_f32(phase);
        float v_beta = voltage_magnitude * our_arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);

        encvaluesum += shadow_count_;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    // Check response and direction
    if (shadow_count_ > init_enc_val + 8) {
        // motor same dir as encoder
        axis_->motor_.config_.direction = 1;
    } else if (shadow_count_ < init_enc_val - 8) {
        // motor opposite dir as encoder
        axis_->motor_.config_.direction = -1;
    } else {
        // Encoder response error
        set_error(ERROR_NO_RESPONSE);
        return false;
    }

    //TODO avoid recomputing elec_rad_per_enc every time
    // Check CPR
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = config_.calib_scan_distance / elec_rad_per_enc;
    calib_scan_response_ = fabsf(shadow_count_-init_enc_val);
    if(fabsf(calib_scan_response_ - expected_encoder_delta)/expected_encoder_delta > config_.calib_range)
    {
        set_error(ERROR_CPR_OUT_OF_RANGE);
        return false;
    }

    // scan backwards
    i = 0;
    axis_->run_control_loop([&](){
        float phase = wrap_pm_pi(-config_.calib_scan_distance * (float)i / (float)num_steps + config_.calib_scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * our_arm_cos_f32(phase);
        float v_beta = voltage_magnitude * our_arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);

        encvaluesum += shadow_count_;
        
        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    config_.offset = encvaluesum / (num_steps * 2);
    int32_t residual = encvaluesum - ((int64_t)config_.offset * (int64_t)(num_steps * 2));
    config_.offset_float = (float)residual / (float)(num_steps * 2) + 0.5f; // add 0.5 to center-align state to phase

    is_ready_ = true;
    return true;
}

static bool decode_hall(uint8_t hall_state, int32_t* hall_cnt) {
    switch (hall_state) {
        case 0b001: *hall_cnt = 0; return true;
        case 0b011: *hall_cnt = 1; return true;
        case 0b010: *hall_cnt = 2; return true;
        case 0b110: *hall_cnt = 3; return true;
        case 0b100: *hall_cnt = 4; return true;
        case 0b101: *hall_cnt = 5; return true;
        default: return false;
    }
}


void Encoder::sample_now() {
    switch (config_.mode) {
        case MODE_INCREMENTAL: {
            tim_cnt_sample_ = (int16_t)hw_config_.timer->Instance->CNT;
        } break;

        case MODE_HALL: {
            // do nothing: samples already captured in general GPIO capture
        } break;

        case MODE_SINCOS: {
            sincos_sample_s_ = (get_adc_voltage(GPIO_3_GPIO_Port, GPIO_3_Pin) / 3.3f) - 0.5f;
            sincos_sample_c_ = (get_adc_voltage(GPIO_4_GPIO_Port, GPIO_4_Pin) / 3.3f) - 0.5f;
        } break;

        case MODE_ABSOLUTE: {
            // Do nothing
        } break;
        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
        } break;
    }
}

bool Encoder::update() {
    // update internal encoder state.
    // Calculate encoder pll gains
    float pll_kp = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    int32_t delta_enc = 0;
    switch (config_.mode) {
        case MODE_INCREMENTAL: {
            //TODO: use count_in_cpr_ instead as shadow_count_ can overflow
            //or use 64 bit
            int16_t delta_enc_16 = (int16_t)tim_cnt_sample_ - (int16_t)shadow_count_;
            delta_enc = (int32_t)delta_enc_16; //sign extend
        } break;

        case MODE_HALL: {
            int32_t hall_cnt;
            if (decode_hall(hall_state_, &hall_cnt)) {
                delta_enc = hall_cnt - count_in_cpr_;
                delta_enc = mod(delta_enc, 6);
                if (delta_enc > 3)
                    delta_enc -= 6;
            } else {
                if (!config_.ignore_illegal_hall_state) {
                    set_error(ERROR_ILLEGAL_HALL_STATE);
                    return false;
                }
            }
        } break;

        case MODE_SINCOS: {
            float phase = fast_atan2(sincos_sample_s_, sincos_sample_c_);
            int fake_count = (int)(1000.0f * phase);
            //CPR = 6283 = 2pi * 1k

            delta_enc = fake_count - count_in_cpr_;
            delta_enc = mod(delta_enc, 6283);
            if (delta_enc > 6283/2)
                delta_enc -= 6283;
        } break;
        
        case MODE_ABSOLUTE: {
            int16_t pos = get_enc_position(hw_config_);
            int32_t diff = (int32_t)pos - (int32_t)last_pos;
            last_pos = pos;
            if (diff < -16384) 
                diff += 32768;
            else if (diff > 16384) 
                diff -= 32768;
            delta_enc = (int32_t)diff;
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
           return false;
        } break;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_ += current_meas_period * vel_estimate_;
    pos_cpr_      += current_meas_period * vel_estimate_;
    // discrete phase detector
    float delta_pos     = (float)(shadow_count_ - (int32_t)floorf(pos_estimate_));
    float delta_pos_cpr = (float)(count_in_cpr_ - (int32_t)floorf(pos_cpr_));
    delta_pos_cpr = wrap_pm(delta_pos_cpr, 0.5f * (float)(config_.cpr));
    // pll feedback
    pos_estimate_ += current_meas_period * pll_kp_ * delta_pos;
    pos_cpr_      += current_meas_period * pll_kp_ * delta_pos_cpr;
    pos_cpr_ = fmodf_pos(pos_cpr_, (float)(config_.cpr));
    vel_estimate_      += current_meas_period * pll_ki_ * delta_pos_cpr;
    bool snap_to_zero_vel = false;
    if (fabsf(vel_estimate_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_ = 0.0f; //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel || !config_.enable_phase_interpolation) {
        interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.offset_float);
    // ph = fmodf(ph, 2*M_PI);
    phase_ = wrap_pm_pi(ph);

    const float phys_rad_per_enc = 2 * M_PI / (float)config_.cpr;
    float pph = phys_rad_per_enc * (interpolated_enc - config_.offset_float);
    phys_phase_ = wrap_pm_pi(pph);

    return true;
}
