/********************************************************************************
                        Start include dependencies from Atmel Studio headers
********************************************************************************/
#include <asf.h>
/********************************************************************************
                        End include dependencies from Atmel Studio headers
********************************************************************************/



/********************************************************************************
                        Start include dependencies from Periphboard/
********************************************************************************/
#include "../PeriphBoard/system_clock.h"
#include "../PeriphBoard/global_ports.h"
#include "../PeriphBoard/keypad.h"
#include "../PeriphBoard/ssd.h"
#include "../PeriphBoard/ssd_interrupt.h"
#include "../PeriphBoard/adc_dac.h"
#include "../PeriphBoard/utilities.h"
#include "../PeriphBoard/motor.h"
#include "../PeriphBoard/voltage_divider.h"

    // Dependencies for other timers
#include "../PeriphBoard/clock_manager_utilities.h"
/********************************************************************************
                        End include dependencies from Periphboard/
********************************************************************************/



/********************************************************************************
                        Start Macro switches
                        - Enable/Disable these to change the
                            behaviour of the program
********************************************************************************/
#define ENC_IMPL_1
#define RISING_DETECT
#define USE_PID_CONTROLLER
/********************************************************************************
                        End Macro switches
********************************************************************************/



/********************************************************************************
                        Start ADC/DAC-related macros
                        - Set up constants used to configure the ADC and DAC
********************************************************************************/
#define ADC_PIN     11
#define AIN_PIN     0x13

#define ADC_MAX_RESOLUTION 4095
/********************************************************************************
                        End ADC/DAC-related macros
********************************************************************************/



/********************************************************************************
                        Start Motor-related macros
                        - Set up constants used to configure the motor
                        - Also set up constants used for configuring the EIC
                            and mapping it to certain pins
********************************************************************************/
#define MOT_PERIOD 200
#define STOP_SPEED 100      // Duty cycle for 0 speed
#define MAX_POS_COUNT 1600

#define ENCODE_PHASEA_PIN 28
#define ENCODE_PHASEB_PIN 14
#define ENCODE_PHASEA_EIC 8
#define ENCODE_PHASEB_EIC 14
/********************************************************************************
                        End Motor-related macros
********************************************************************************/



/********************************************************************************
                        Start Display and control-related macros
        Keys            |       Display Mode 1      |       Display Mode 2
       --------------------------------------------------------------------------
        A               |       Position            |       Position error
        B               |         RPM               |          RPM error
        C               |      User input           |      Raw counter value
********************************************************************************/
    // Values to check when actually displaying
#define SHOW_POS_STATE              0u
#define SHOW_RPM_STATE              1u
#define SHOW_USR_STATE              2u
#define SHOW_POS_ERR_STATE          10u
#define SHOW_RPM_ERR_STATE          11u
#define SHOW_RAW_STATE              22u


#define SHOW_NEXT_DISPLAY_STATE     10u // A function key to give keys A, B, and C
                                        //  multiple modes
#define MAX_DISPLAY_STATE_MODES     20u
/********************************************************************************
                        End Display and control-related macros
********************************************************************************/



/********************************************************************************
                        Start miscellaneous macros
********************************************************************************/
#define MAX_RPM 4200
#define RPM_FACTOR_NUM 1875
#define RPM_FACTOR_DEN   64
#define MULT_RPM(COUNT) ((COUNT * RPM_FACTOR_NUM) / RPM_FACTOR_DEN)
/********************************************************************************
                        End miscellaneous macros
********************************************************************************/



/********************************************************************************
                Start program control state related macros and variables
                        - use bit flags to determine what kind of
                            control state the program is in.
                            For example, controlling speed using
                            the potentiometer.
                        - bit 0: poteniometer control (0) or keypad control (1)
                        - bit 1: speed control (0) or position control (1)
                        - bit 2: Determines whether (1) or not (0) program should
                            target user input value (In future clean up, remove
                            this flag from the program)
                        - bit 3: Indicates direction of the spinning motor
********************************************************************************/
static BOOLEAN__ ctrl_flags = 0x0;
#define KEY_CTRL        0x1
#define POS_CTRL        0x2
#define TARGET_USER     0x4
#define MOT_DIR         0x8
#define STOP_MOT        0x10
#define MOVE_TO_POT     0x20
/********************************************************************************
                End program control state related macros and variables
********************************************************************************/



/********************************************************************************
                        Start Keypad related functions
********************************************************************************/
void keypad_handler(UINT8 row_to_check);
/********************************************************************************
                        End Keypad related functions
********************************************************************************/



/********************************************************************************
                        Start EIC related functions
********************************************************************************/
    // Set up both the ports and EIC for encoder
void disable_encoder_eic(void);
void enable_encoder_eic(void);
void enable_encoder_eic_clock(void);
void configure_encoder_eic(void);   // Hard code interrupt choices first, generalize later
void configure_encoder_ports(void); // Set up ports used for encoder
/********************************************************************************
                        End EIC related functions
********************************************************************************/



/********************************************************************************
                        Start Speed/Position control related functions
                        (Uses TC2)
                        (spctrl ~ Speed-Position Control)
********************************************************************************/
void enable_spctrl_tc_clocks(void);
void enable_spctrl_timer(void);
void disable_spctrl_timer(void);
void configure_spctrl_timer(void);
BOOLEAN__ spctrl_interrupted(void);
void clear_spctrl_interrupt(void);
static inline void spctrl_handler(void);
void position_ctrl(void);
void speed_ctrl(void);
/********************************************************************************
                        End Speed/Position control related functions
                        (Uses TC2)
                        (spctrl ~ Speed-Position Control)
********************************************************************************/



/********************************************************************************
                        Start Global variables
********************************************************************************/
static PortGroup* bankA_ptr = NULL, *bankB_ptr = NULL;    // Mostly for configuring pins for EIC
static Eic* eic_ptr = NULL;
static TcCount8* spctrl_timer_ptr = NULL;

    // Encoder variables
static INT32
    counter = 0u,       // Raw counter for encoder readings
/*
    error = 0u, state = 0u,
*/
    display_state = 1   // Determine what the display should show
    ;
static UINT8 display_second_mode = 0;   // Addition factor to help determine
                                        //  display mode

static INT8 speed_up = 0;               // For keypad control when manually
                                        //  speeding up or down

    // Keypad variables
static UINT8 row, col;

static INT32 user_input = 0;  // Store a number the user types in
static INT32 motor_cur_pos = 800;       // Track the current position of the motor
                                        //  Moves from 0 to 1600. Position 0 is
                                        //  defined as 800.
static INT32 motor_cur_rpm = 0;         // Track the current speed of the motor.

static UINT32 adc_read_raw = 0;
static UINT32 adc_read_duty = 0;
static UINT32 adc_read_pos = 0;
static UINT32 duty_from_adc = 0;
/********************************************************************************
                        End Global variables
********************************************************************************/



int main (void)
{
    Simple_Clk_Init();
    delay_init();

    configure_global_ports();
    configure_ssd_ports();
    configure_keypad_ports();

    configure_display_timer();
    enable_display_timer();

    configure_spctrl_timer();
    enable_spctrl_timer();

    configure_voltage_divider_ports();
    // 12-bit resolution
    configure_adc_custom(
        0x2,    // Select a V_DD_AN/2 (1.65) reference
        0x0,    // Now collect 1 sample at a time.
            // Total sampling time length = (SAMPLEN+1)*(Clk_ADC/2)
        0x0,    // Set sampling time to 1 adc clock cycle?
        0x1,    // Relative to main clock, have adc clock run 4 times slower
        0x0,    // For averaging more than 2 samples, change RESSEL (0x1 for 16-bit)
        0xF,    // Since reference is 1/2, set gain to 1/2 to keep largest
                // input voltage range (expected input will be 0 - 3.3V)
        0x18,   // Not using the negative for differential, so ground it.
        AIN_PIN // Map the adc to analog pin AIN_PIN
        );
    map_to_adc_odd(ADC_PIN);
    configure_motor_pwm_custom(
        0x1,                    // Set presynchronizer to prescaled clock
        0x0,                    // Prescale clock by 1
        0x2,                    // Select the Normal PWM waveform
        MOT_PERIOD,
        MOT_PERIOD/2            // Start with 50% duty cycle
        );


    configure_encoder_eic();
    enable_encoder_eic();

    return 0;
}

void keypad_handler(UINT8 row_to_check){
    static UINT32 col_temp = 0;

        // Read from keypad and process input
    check_key(&row, &col, row_to_check);

    if(!(ctrl_flags & KEY_CTRL)){  // Read from adc if in potentiometer control
        adc_read_raw = read_adc();
        adc_read_duty = map32(
            adc_read_raw,
            0, ADC_MAX_RESOLUTION,
            motor_duty_min(), motor_duty_max()
            );
        adc_read_pos = map32(
            adc_read_raw,
            0, ADC_MAX_RESOLUTION,
            0, MAX_POS_COUNT
            );
        duty_from_adc = adc_read_raw * 180 / ADC_MAX_RESOLUTION + 10;
    }

        // Switch modes. Uses multi-key combinations.
    if(row == 0 && col == 0xB){ // Exit motor mode
        ctrl_flags |= STOP_MOT;
        return;
    } else if(row == 2 && col == 0xD){  // Change to pot control
        ctrl_flags &= ~KEY_CTRL;
        ctrl_flags |= MOVE_TO_POT;
    } else if(row == 2 && col == 0xB){  // Change to key control
        if(!(ctrl_flags & POS_CTRL))    ctrl_flags |= STOP_MOT;
        ctrl_flags |= KEY_CTRL;
    } else if(row == 3 && col == 0x9){  // Change to speed control
        if(!(ctrl_flags & KEY_CTRL))    ctrl_flags |= MOVE_TO_POT;
        else                            ctrl_flags |= STOP_MOT;
        ctrl_flags &= ~POS_CTRL;
    } else if(row == 3 && col == 0x6){  // Change to position control
        ctrl_flags |= STOP_MOT;
        ctrl_flags |= POS_CTRL;
    } else if(row == 1 && col == 6){    // Change display to show the
                                        //  second set of values
        display_second_mode = SHOW_NEXT_DISPLAY_STATE;
    } else if(row == 1 && col == 9){    // Change display to show the
                                        //  first set of values
        display_second_mode = 0u;
    }

        // Single key combinations
    if(col != 0x10){
        static UINT8 row_col = 20;
        col_temp = find_lsob(col);
        row_col = row*10 + col_temp;
        switch(row_col){
            case 30:        // A
                display_state = SHOW_POS_STATE + display_second_mode;
                break;
            case 20:        // B
                display_state = SHOW_RPM_STATE + display_second_mode;
                break;
            case 10:        // C
                display_state = SHOW_USR_STATE + display_second_mode;
                break;
            case 00:        // D
                if(
                    ctrl_flags & KEY_CTRL && 
                    ctrl_flags & POS_CTRL       // Temporary until user can
                                                //  enter in rpm manually
                ){           // Enter user input
                    ctrl_flags |= TARGET_USER;
                }
                break;
            case 01:        // '#' ~ delete the last user input digit
                if(ctrl_flags & POS_CTRL){
                    user_input /= 10;
                    ctrl_flags &= ~TARGET_USER;
                    ctrl_flags |= STOP_MOT;
                }
                break;
            case 03:        // * ~ Clear user input
                if(ctrl_flags & POS_CTRL){
                    user_input = 0;
                    ctrl_flags &= ~TARGET_USER;
                    ctrl_flags |= STOP_MOT;
                }
                break;
                            // Below are for number inputs
            case 02:
                if(ctrl_flags & TARGET_USER && ctrl_flags & POS_CTRL){
                    ctrl_flags &= ~TARGET_USER;
                    ctrl_flags |= STOP_MOT;
                }
                if(!(ctrl_flags & KEY_CTRL) || (user_input >= 1000)) break;
                user_input *= 10;
                break;
            default:
                if(ctrl_flags & TARGET_USER && ctrl_flags & POS_CTRL){
                    ctrl_flags &= ~TARGET_USER;
                    ctrl_flags |= STOP_MOT;
                }
                if(!(ctrl_flags & KEY_CTRL) || (user_input >= 1000)) break;
                user_input = user_input*10 + (3u-row)*3u + (4u-col_temp);
                break;
        }
    }

    if((row == 0) && (col == 0x2) && motor_duty_cur() < motor_duty_max()){
        speed_up = 1;
        ctrl_flags |= TARGET_USER;
    }else if((row == 0) && (col == 0x8) && motor_duty_cur() > motor_duty_min()){
        speed_up = -1;
        ctrl_flags |= TARGET_USER;
    }else speed_up = 0;
}

void disable_encoder_eic(void){
    eic_ptr->CTRL.reg &= ~(1 << 1u);
    while(eic_ptr->STATUS.reg & (1 << 7u));
}

void enable_encoder_eic(void){
    while(eic_ptr->STATUS.reg & (1 << 7u));
    eic_ptr->CTRL.reg |= (1 << 1u);
}

void enable_encoder_eic_clock(void){
    configure_global_ports();

    // PM_APBAMASK is in the pm_bit_pos position
    power_manager->APBAMASK.reg |= (1 << 6);

    gen_clk->CLKCTRL.reg =  //  Setup in the CLKCTRL register
          (0x0<<8)  //  General clock source
        | 0x03;       //  Clock selection ID(see table 14-2)
    gen_clk->CLKCTRL.reg |= 0x1u << 14;    // enable it.
}

void configure_encoder_eic(void){
    configure_global_ports();
    eic_ptr = (Eic*)(EIC);

    configure_encoder_ports();
    enable_encoder_eic_clock();

    // Disable first
    disable_encoder_eic();

    // pin PA28 maps to EIC[ENCODE_PHASEA_EIC]
    // pin PB14 maps to EIC[ENCODE_PHASEB_EIC]
        // Use rising edge detection
    eic_ptr->CONFIG[1].reg |=
#ifdef RISING_DETECT
          (0x1 << 0u)   // For EIC[ENCODE_PHASEA_EIC]
        | (0x1 << 24u)  // For EIC[ENCODE_PHASEB_EIC]
#else
          (0x3 << 0u)   // For EIC[ENCODE_PHASEA_EIC]
        | (0x3 << 24u)  // For EIC[ENCODE_PHASEB_EIC]
#endif
        | (0x1 << 3u)   // EIC[ENCODE_PHASEA_EIC] Filter
        | (0x1 << 27u)  // EIC[ENCODE_PHASEB_EIC] Filter
        ;
    eic_ptr->EVCTRL.reg |=          // Enable generation of interrupts
        (1 << ENCODE_PHASEA_EIC)
        | (1 << ENCODE_PHASEB_EIC)
        ;
    NVIC->ISER[0] |= 1 << 4u;       // Enable interrupts for EIC
    NVIC->IP[1] &= ~(0x3 << 6u);    // Set highest priority (00)
    eic_ptr->INTENSET.reg |=        // Enable interrupts
        (1 << ENCODE_PHASEA_EIC) |
        (1 << ENCODE_PHASEB_EIC)
        ;

    eic_ptr->INTFLAG.reg |= (1 << ENCODE_PHASEA_EIC) | (1 << ENCODE_PHASEB_EIC);
}

void configure_encoder_ports(void){
    configure_global_ports();
    bankA_ptr = bankA;
    bankB_ptr = bankB;

    // Configure pins ENCODE_PHASEA_PIN and ENCODE_PHASEB_PIN
    //  and select EIC function
    bankA_ptr->PINCFG[ENCODE_PHASEA_PIN].reg |= 0x1 | (1 << 2u);
    bankB_ptr->PINCFG[ENCODE_PHASEB_PIN].reg |= 0x1 | (1 << 2u);
    bankA_ptr->PMUX[ENCODE_PHASEA_PIN/2u].reg |= 0x0;
    bankB_ptr->PMUX[ENCODE_PHASEB_PIN/2u].reg |= 0x0;
}

void TC7_Handler(void){
    static UINT8 dig = 0, hold = 0u;
    static const UINT8 hold_lim = 25;
    static INT32 temp = 0;      // General temporary variable
        // Share display interrupt with other functionality
    if(display_interrupted()){
        /**************************************************
            Use sample and hold technique to manually
            slow down how often the display is updated
            with the counter value.
        **************************************************/
        motor_cur_rpm = MULT_RPM(counter);
        if(hold == hold_lim){
                              // Simple update display mode
            switch(display_state){
                case SHOW_POS_STATE:
                    update_display(motor_cur_pos);
                    break;
                case SHOW_RPM_STATE:
                    if(counter >= 0){
                        update_display(motor_cur_rpm);
                    } else {
                        update_display(-motor_cur_rpm);
                    }
                    break;
                case SHOW_POS_ERR_STATE:
                    if(motor_cur_pos < temp){
                        update_display(temp - motor_cur_pos);
                    } else {
                        update_display(motor_cur_pos - temp);
                    }
                    break;
                case SHOW_RPM_ERR_STATE:
                    temp = motor_duty_cur();
                    if(temp < STOP_SPEED){
                        temp = (motor_cur_rpm - (motor_duty_cur()-STOP_SPEED)*MAX_RPM/MOT_PERIOD);
                    } else {
                        temp = (motor_cur_rpm - (STOP_SPEED-motor_duty_cur())*MAX_RPM/MOT_PERIOD);
                    }
                    if(temp < 0){
                        update_display(-temp);
                    } else {
                        update_display(temp);
                    }
                    break;
                case SHOW_USR_STATE:
                    update_display(user_input);
                    break;
                case SHOW_RAW_STATE:
                    update_display(counter);
                    break;
                default: break;
            }
        }
        counter = 0;        // Always reset counter regardless
        hold = (hold+1u)%(hold_lim+1u);
            // Display to one segment and check associated keypad row
        display_handler(dig);
        if(ctrl_flags & MOT_DIR)    bankB_ptr->OUT.reg |= (1 << 9u);
        else                        bankB_ptr->OUT.reg &= ~(1 << 9u);
        keypad_handler(dig);
        dig = (dig+1u)%4u;
        clear_display_interrupt();
    }
}

void EIC_Handler(void){            // Upon interrupt for one phase, check the value for the other phase
                                   //  to determine direction.
    if(eic_ptr->INTFLAG.reg & (1 << 8u)){
        if(bankB->IN.reg & (1 << 14u)){
            --counter;
            --motor_cur_pos;
            ctrl_flags |= MOT_DIR;
        } else {
            ++counter;
            ++motor_cur_pos;
            ctrl_flags &= ~MOT_DIR;
        }
        eic_ptr->INTFLAG.reg |= (1 << 8u);
    } else if(eic_ptr->INTFLAG.reg & (1 << 14u)){
        if(bankA->IN.reg & (1 << 28u)){
            ++counter;
            ++motor_cur_pos;
            ctrl_flags &= ~MOT_DIR;
        } else {
            --counter;
            --motor_cur_pos;
            ctrl_flags |= MOT_DIR;
        }
        eic_ptr->INTFLAG.reg |= (1 << 14u);
    }
    motor_cur_pos %= MAX_POS_COUNT;
}

/********************************************************************************
                        Start Speed/Position control related functions
                        (Uses TC2)
                        (spctrl ~ Speed-Position Control)
********************************************************************************/
void enable_spctrl_tc_clocks(void){
    enable_clock(
        10u,    // PM_APBCMASK is in the 10 position
        0x14    // ID for TC2 is 0x14  (see table 14-2)
        );
}

void enable_spctrl_timer(void){
    while(spctrl_timer_ptr->STATUS.reg & (1 << 7u));    // Synchronize before proceeding
    spctrl_timer_ptr->CTRLA.reg |= 1 << 1u;    // Re-enable the timer
}

void disable_spctrl_timer(void){
    spctrl_timer_ptr->CTRLA.reg &= ~(1 << 1u);    // Disable the timer
    while(spctrl_timer_ptr->STATUS.reg & (1 << 7u));    // Synchronize before proceeding
}

void configure_spctrl_timer(void){
    spctrl_timer_ptr = timer2_8; // Use the one of the count structures within the union

    enable_spctrl_tc_clocks();
    disable_spctrl_timer();

        // Set up timer 2 settings
    spctrl_timer_ptr->CTRLA.reg |=
          (0x1 << 12u)  // Set presynchronizer to prescaled clock
        | (0x5 << 8u)   // Prescale clock by 64
        | (0x1 << 2u)   // Start in 8-bit mode
        | (0x2 << 5u)   // Select the Normal PWM waveform generator
        ;
    spctrl_timer_ptr->PER.reg = 80;
    spctrl_timer_ptr->CC[0].reg = 40;

        // Set up timer 2 interrupt
    NVIC->ISER[0] |= 1 << 15u;
    NVIC->IP[3] |= (0x1 << 24u);
    spctrl_timer_ptr->INTENSET.reg |= 1 << 4u;
    spctrl_timer_ptr->INTFLAG.reg |= 0x1 << 4u;
}

BOOLEAN__ spctrl_interrupted(void){
    return spctrl_timer_ptr->INTFLAG.reg &= 0x1;
}

void clear_spctrl_interrupt(void){
    spctrl_timer_ptr->INTFLAG.reg |= 0x1;
}

static inline void spctrl_handler(void){
    if(!(ctrl_flags & KEY_CTRL))    ctrl_flags |= TARGET_USER;

    static const UINT32 stop_duty = MOT_PERIOD/2;
    if(ctrl_flags & STOP_MOT){
        if(step_toward8_basic(stop_duty, 5)) ctrl_flags &= ~STOP_MOT;
        return;
    } else if(ctrl_flags & MOVE_TO_POT){
        if(step_toward8_basic(adc_read_duty, 5)) ctrl_flags &= ~MOVE_TO_POT;
        return;
    }

    if(ctrl_flags & TARGET_USER){
        if(ctrl_flags & POS_CTRL)   position_ctrl();
        else                        speed_ctrl();
    }
}

void position_ctrl(void){
#ifdef USE_PID_CONTROLLER
    static const UINT8 error_margin = 1;
    static float err = 0, err_prev = 0, err_future = 0, err_accum = 0;
    static const float Kp = 0.1f, Ki = 0.01f, Kd = 0.0005f;
    static const float Kp2 = 0.05f, Ki2 = 0.25f, Kd2 = 0.5f;
    static const float dt = 0.00064;
    static float move_speed = STOP_SPEED;
    static float move_speed_prev = STOP_SPEED;
    static float move_speed_err_margin = 1000;
#else
    static const UINT8 error_margin = 3;
    static const UINT8 speed_margin = 10;
    static const UINT8 move_speed = STOP_SPEED + speed_margin;
    static const UINT8 move_opp_speed = STOP_SPEED - speed_margin;
#endif

    user_input %= MAX_POS_COUNT;

    if(!(ctrl_flags & KEY_CTRL))    // If control is from potentiometer
        user_input = adc_read_pos;

#ifdef USE_PID_CONTROLLER
    if(
        (motor_cur_pos < user_input - error_margin) ||
        (motor_cur_pos > user_input + error_margin)
    ){
                // Move motor
/*      // Attempt to automatically reset the PID controller
        //  when any of the terms become unstable
        move_speed_prev = move_speed_prev - move_speed;
        if(
            move_speed_prev > move_speed_err_margin ||
            move_speed_prev < -move_speed_err_margin
        ){
            err = 0;
            err_prev = 0;
            err_accum = 0;
            err_future = 0;
            move_speed = STOP_SPEED;
            move_speed_prev = STOP_SPEED;
            ctrl_flags |= STOP_MOT;
            return;
        }
*/
        motor_set_duty(move_speed);
//        move_speed_prev = move_speed;

            // Update error values
        err = motor_cur_pos - user_input;
        err_future = (err - err_prev)/dt;
        err_accum += err*dt;

            // Multiply by PID constants and alter motor duty cycle
        if(ctrl_flags & KEY_CTRL){
            move_speed =
                  STOP_SPEED
                + Kp2*err            // P
                + Ki2*err_accum      // I
                + Kd2*err_future     // D
                ;
        } else {
            move_speed =
                  STOP_SPEED
                + Kp*err            // P
                + Ki*err_accum      // I
                + Kd*err_future     // D
                ;
        }

        err_prev = err;

        return;
    }
#else
    if(
        (motor_cur_pos < user_input - error_margin) ||
        (motor_cur_pos > user_input + error_margin)
    ){
        if(motor_cur_pos > user_input)  motor_set_duty(move_speed);
        else                            motor_set_duty(move_opp_speed);
        return;
    }
#endif
    motor_set_duty(STOP_SPEED);
}

void speed_ctrl(void){
/*      // Start of converting position PID to speed PID
    static const UINT8 STOP_SPEED = MOT_PERIOD/2;
#ifdef USE_PID_CONTROLLER
    static const UINT8 error_margin = 100;

    static UINT8 duty_cur = 0;
    static float err = 0, err_prev = 0, err_future = 0, err_accum = 0;
    static const float Kp = 0.1f, Ki = 0.01f, Kd = 0.0005f;
    static const float Kp2 = 0.05f, Ki2 = 0.25f, Kd2 = 0.5f;
    static const float dt = 0.00064;
    static float move_speed = STOP_SPEED;
    static float move_speed_prev = STOP_SPEED;
    static float move_speed_err_margin = 1000;

    if(ctrl_flags & KEY_CTRL){   // Use keypad
        motor_step_duty(speed_up);
    } else {                    // Use potentiometer
                // Move motor
        motor_set_duty(duty_from_adc);

            // Update error values
        duty_cur = motor_duty_cur();
        if(duty_cur < STOP_SPEED)
            err = motor_cur_rpm - (motor_duty_cur()-STOP_SPEED)*MAX_RPM;
        else if(duty_cur > STOP_SPEED){
            err = motor_cur_rpm - (STOP_SPEED-motor_duty_cur())*MAX_RPM;
        }
        err_future = (err - err_prev)/dt;
        err_accum += err*dt;

            // Multiply by PID constants and alter motor duty cycle
        if(ctrl_flags & KEY_CTRL){
            move_speed =
                  STOP_SPEED
                + Kp2*err            // P
                + Ki2*err_accum      // I
                + Kd2*err_future     // D
                ;
        } else {
            move_speed =
                  STOP_SPEED
                + Kp*err            // P
                + Ki*err_accum      // I
                + Kd*err_future     // D
                ;
        }

        err_prev = err;
    }
#else
    static UINT32 read_hold_temp = 0;
    if(ctrl_flags & KEY_CTRL){   // Use keypad
        motor_step_duty(speed_up);
    } else {                    // Use potentiometer
        read_hold_temp = adc_read_raw;
        read_hold_temp = read_hold_temp * 180 / ADC_MAX_RESOLUTION + 10;
        motor_set_duty(read_hold_temp);
    }
#endif
*/
    static UINT32 read_hold_temp = 0;
    if(ctrl_flags & KEY_CTRL){   // Use keypad
        motor_step_duty(speed_up);
    } else {                    // Use potentiometer
        read_hold_temp = adc_read_raw;
        read_hold_temp = read_hold_temp * 180 / ADC_MAX_RESOLUTION + 10;
        motor_set_duty(read_hold_temp);
    }
}

void TC2_Handler(void){
    if(spctrl_interrupted()){
        spctrl_handler();
    }
    clear_spctrl_interrupt();
}
/********************************************************************************
                        End Speed/Position control related functions
                        (Uses TC2)
********************************************************************************/