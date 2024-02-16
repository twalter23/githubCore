/*
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

//-------------OWNTECH DRIVERS-------------------
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "DataAPI.h"
//------------ZEPHYR DRIVERS----------------------
#include "zephyr/zephyr.h"
#include "zephyr/console/console.h"

#define RECORD_SIZE 128 // Number of point to record

#define LEG1_CAPA_DGND PC6
#define LEG2_CAPA_DGND PB7
#define DRIVER_LEG1_ON PC12
#define DRIVER_LEG2_ON PB13


//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_application_task();   // Code to be executed in the background task
void loop_communication_task();   // Code to be executed in the background task
void loop_control_task();     // Code to be executed in real time in the critical task

//--------------USER VARIABLES DECLARATIONS----------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

uint8_t received_serial_char;
char bufferstr[255];

/* Measure variables */

static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high;
static float32_t V_high;

static float32_t serial_refresh_rate_ms = 500;

static float32_t delta_V1;
static float32_t V1_max = 0.0;
static float32_t V1_min = 0.0;

static float32_t delta_V2;
static float32_t V2_max = 0.0;
static float32_t V2_min = 0.0;

int8_t AppTask_num, CommTask_num;

static float32_t acquisition_moment = 0.06;

static float meas_data; // temp storage meas value (ctrl task)

float32_t starting_duty_cycle = 0.1;

static const uint16_t NumberOfStepsInRamp = 10000;
static const float32_t duty_cycle_step = 0.05; //[-] duty cycle step (comm task)
static const float32_t duty_ramp_step = duty_cycle_step/NumberOfStepsInRamp;

static float32_t duty_cycle_target = starting_duty_cycle; //[-] duty cycle (comm task)
static float32_t applied_duty_cycle= 0.05;

float32_t slope;

/* Variables used for recording */
typedef struct Record_master {
    float32_t I1_low_value;
    float32_t I2_low_value;
    float32_t V1_low_value;
    float32_t V2_low_value;
    float32_t Vhigh_value;
    float32_t Ihigh_value;
    float32_t V1_low_value_no_cap;
    float32_t V2_low_value_no_cap;
} record_t;

record_t record_array[RECORD_SIZE];

static uint32_t counter = 0;
static uint32_t print_counter = 0;

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    STEPMODE_1,
    STEPMODE_2,
    POWERMODE
};


uint8_t mode = IDLEMODE;



//---------------SETUP FUNCTIONS----------------------------------

void setup_routine()
{
    data.enableTwistDefaultChannels();
    spin.version.setBoardVersion(SPIN_v_0_9);
    twist.initAllBuck(); // initialize in buck mode

    spin.gpio.configurePin(LEG1_CAPA_DGND, OUTPUT);
    spin.gpio.configurePin(LEG2_CAPA_DGND, OUTPUT);
    spin.gpio.configurePin(DRIVER_LEG1_ON, OUTPUT);
    spin.gpio.configurePin(DRIVER_LEG2_ON, OUTPUT);

    float32_t GV1 = 0.044301359147286994;
    float32_t OV1 = -89.8291125470221;
    float32_t GV2 = 0.043891466731813246;
    float32_t OV2 = -89.01321095039089;
    float32_t GVH = 0.029777494874229947;
    float32_t OVH = 0.12805533844297656;

    float32_t GI1 = 0.005510045850270965;
    float32_t OI1 = -11.298753103344417;
    float32_t GI2 = 0.005569903739753797;
    float32_t OI2 = -11.47851441455354;
    float32_t GIH = 0.0052774398156665;
    float32_t OIH = -10.864400298536168;

    data.setParameters(V1_LOW, GV1, OV1);
    data.setParameters(V2_LOW, GV2, OV2);
    data.setParameters(V_HIGH, GVH, OVH);

    data.setParameters(I1_LOW, GI1, OI1);
    data.setParameters(I2_LOW, GI2, OI2);
    data.setParameters(I_HIGH, GIH, OIH);

    spin.gpio.setPin(LEG1_CAPA_DGND);
    spin.gpio.setPin(LEG2_CAPA_DGND);

    AppTask_num = task.createBackground(loop_application_task);
    CommTask_num = task.createBackground(loop_communication_task);
    task.createCritical(&loop_control_task, control_task_period);

    task.startBackground(AppTask_num);
    task.startBackground(CommTask_num);
    task.startCritical();
}

//---------------LOOP FUNCTIONS----------------------------------

void loop_communication_task()
{
    received_serial_char = console_getchar();
    switch (received_serial_char)
    {
    case 'h':
        //----------SERIAL INTERFACE MENU-----------------------
        // printk(" ________________________________________\n");
        // printk("|     ------- MENU ---------             |\n");
        // printk("|     press i : idle mode                |\n");
        // printk("|     press s : serial mode              |\n");
        // printk("|     press p : power mode               |\n");
        // printk("|     press u : duty cycle UP            |\n");
        // printk("|     press d : duty cycle DOWN          |\n");
        // printk("|________________________________________|\n\n");
        //------------------------------------------------------
        break;
    case 'i':
        printk("idle mode\n");
        mode = IDLEMODE;
        break;
    case 'p':
        printk("power mode\n");
        mode = POWERMODE;
        counter = 0;
        break;
    case 'a':
        printk("step mode 1\n");
        mode = STEPMODE_1;
        counter = 0;
        spin.gpio.resetPin(LEG1_CAPA_DGND);
        spin.gpio.setPin(LEG2_CAPA_DGND);
        break;
    case 's':
        printk("step mode 2\n");
        mode = STEPMODE_2;
        counter = 0;
        spin.gpio.setPin(LEG1_CAPA_DGND);
        spin.gpio.resetPin(LEG2_CAPA_DGND);
        break;
    case 'w':
        spin.gpio.setPin(LEG1_CAPA_DGND);
        spin.gpio.setPin(LEG2_CAPA_DGND);
        break;
    case 'x':
        spin.gpio.resetPin(LEG1_CAPA_DGND);
        spin.gpio.resetPin(LEG2_CAPA_DGND);
        break;
    case 'o':
        spin.gpio.resetPin(DRIVER_LEG1_ON);
        spin.gpio.resetPin(DRIVER_LEG1_ON);
        break;
    case 'f':
        spin.gpio.resetPin(DRIVER_LEG1_ON);
        spin.gpio.resetPin(DRIVER_LEG1_ON);
        break;
    case 'u':
        duty_cycle_target = duty_cycle_target + duty_cycle_step;
        break;
    case 'd':
        duty_cycle_target = duty_cycle_target - duty_cycle_step;
        break;
    default:
        break;
    }
}

void loop_application_task()
{
    if (mode == IDLEMODE)
    {
        spin.led.turnOff();
    }
    else if (mode == POWERMODE)
    {
        spin.led.turnOn();
            printk("%.3f:", applied_duty_cycle);
            printk("%.3f:", record_array[print_counter].I1_low_value);
            printk("%.3f:", record_array[print_counter].I2_low_value);
            printk("%.3f:", record_array[print_counter].Ihigh_value);
            printk("%.3f:", record_array[print_counter].V1_low_value);
            printk("%.3f:", record_array[print_counter].V2_low_value);
            printk("%.3f:", record_array[print_counter].Vhigh_value);
            printk("%d:\n", print_counter);
            print_counter++;
            if(print_counter > 30) print_counter = 0;

    }
    else if (mode == STEPMODE_1 || mode == STEPMODE_2)
    {
        spin.led.toggle();
        if(counter >= RECORD_SIZE){
            printk("%f:", record_array[print_counter].I1_low_value);
            printk("%f:", record_array[print_counter].I2_low_value);
            printk("%f:", record_array[print_counter].Ihigh_value);
            printk("%f:", record_array[print_counter].V1_low_value);
            printk("%f:", record_array[print_counter].V2_low_value);
            printk("%f:", record_array[print_counter].Vhigh_value);
            printk("%d:", print_counter);
            print_counter++;
            if(print_counter > 30) print_counter = 0;
        }
    }
    task.suspendBackgroundMs(serial_refresh_rate_ms);
}

void loop_control_task()
{
    meas_data = data.getLatest(V1_LOW);
    if (meas_data != NO_VALUE)
        V1_low_value = meas_data;

    meas_data = data.getLatest(V2_LOW);
    if (meas_data != NO_VALUE)
        V2_low_value = meas_data;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != NO_VALUE)
        I1_low_value = meas_data;

    meas_data = data.getLatest(I1_LOW);
    if (meas_data != NO_VALUE)
        I2_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data != NO_VALUE)
        V_high = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != NO_VALUE)
        I_high = meas_data;


    if (mode==IDLEMODE){
            pwm_enable = false;
            twist.stopLeg(LEG1);
            twist.stopLeg(LEG2);
    }
    else if (mode == POWERMODE ||  mode == STEPMODE_1 || mode == STEPMODE_2)
    {
        
        if(applied_duty_cycle==duty_cycle_target)
        {
            slope = 0; //we reached duty
        }

        if(applied_duty_cycle>duty_cycle_target)
        {
            slope = -1; //negative slope
        }

        if(applied_duty_cycle<duty_cycle_target)
        {
            slope = 1; //nagative slope
        }

        applied_duty_cycle = applied_duty_cycle + (slope*duty_ramp_step);

        // spin.setInterleavedDutyCycle(duty_cycle);
        if(mode == STEPMODE_1) twist.setLegDutyCycle(LEG1,applied_duty_cycle);
        if(mode == STEPMODE_2) twist.setLegDutyCycle(LEG2,applied_duty_cycle);
        if(mode == POWERMODE) twist.setAllDutyCycle(applied_duty_cycle);
        if (!pwm_enable)
        {
            pwm_enable = true;
            if(mode == STEPMODE_1) twist.startLeg(LEG1);
            if(mode == STEPMODE_2) twist.startLeg(LEG2);
            if(mode == POWERMODE)  twist.startAll();

            counter = 0;
            V1_max  = 0;
            V2_max  = 0;
        }

        if(mode == STEPMODE_1){
            if(counter<RECORD_SIZE/2) spin.gpio.resetPin(LEG1_CAPA_DGND);
            record_array[counter].I1_low_value = I1_low_value;
            record_array[counter].V1_low_value = V1_low_value;
        }
        if(mode == STEPMODE_2) {
            record_array[counter].V2_low_value = V2_low_value;
            record_array[counter].I2_low_value = I2_low_value;
        }

        record_array[counter].Ihigh_value = I_high;
        record_array[counter].Vhigh_value = V_high;

        if(V1_low_value>V1_max) V1_max = V1_low_value;
        if(V2_low_value>V2_max) V2_max = V2_low_value;

        if(counter < RECORD_SIZE) counter++;
    }
}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}
