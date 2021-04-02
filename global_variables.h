/*
 * global_variables.h
 *
 *  Created on: 3.15.2019
 *      Author: zitao liao
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

#include "global_define.h"

//manual debug variabls
int16 dummy_read = 0;
int16 debug_flag = 1;
float theta_debug = 0;

//control flow variable
int16 SFO_status = 0;
Uint16 sdata;     // Send data buffer
Uint16 rdata;     // Receive data buffer
Uint16 rdata_point;  // Keep track of where we are
Uint16 i;       // in the data stream to check received data


float theta_test = 0;
float i_buf = 0;
int16 i_buf_count = 0;
int16 i_dc_count = 0;
float i_dc = 0;
// FCML control variables
volatile float32 main_duty = DEFAULT_DUTY;
volatile float32 main_duty_debug = DEFAULT_DUTY;
volatile int16 sine_index = 0;
volatile int16 sine_step_vc1 = 0;
int16 deadtime = 2;
int16 deadtime1 = 2;
int16 num_points= 2500; 	// number of points in a complete 60 sine wave, should be fs/60
float step = 0.0004;     // step = 1/num_points
float32 d_dc = 0.1;

// voltage measurements
int16 Vout = 0;
int16 Vrec = 0 ;
int16 Vac_pos = 0;
int16 Vac_neg = 0;
int16 Vac = 0;

// pll loop variables
// notch filter
float32 notch_in = 0;
float32 notch_in1 = 0; //x[n-1]
float32 notch_in2 = 0;
float32 notch_out = 0;
float32 notch_out1 = 0;
float32 notch_out2 = 0;

//150k
float32 notch_b0 = 0.994998549663076;
float32 notch_b1 = -1.989896540894321;
float32 notch_b2 = 0.994998549663076;
float32 notch_a0 = 0.989997099326151;
float32 notch_a1 = -1.989896540894321;


//75k
//float32 notch_b0 = 0.990046630379859;
//float32 notch_b1 = -1.979693038985706;
//float32 notch_b2 = 0.990046630379859;
//float32 notch_a0 = 0.980093260759718;
//float32 notch_a1 = -1.979693038985706;



// PI controller
float32 Kp_pll = 3;
float32 Ki_pll = 1.437966465082929e-05;
float32 x_sum_pll = 0;
float32 pll_PI_out = 0;
float32 theta = 0;
float32 theta_pre = 0;
int16 phase_locked = 0;
int16 duty_test = 0;

int16 pll_on = 0;

//Vout, Vrec moving average data
int16 Vout_sample[MOV_AVE_SIZE];
int16 Vout_pointer=0;
int32 Vout_sum=0;
float Vout_ave=0;

float ibuf_sample[MOV_AVE_SIZE];
float ibuf_sum=0;
float ibuf_ave=0;
float ibuf_ave_cal=0;




// current measurements
int16 IL_count = 0;
int16 IL_bias = 0;
int16 Idc_bias_count = 0;
int16 Iout_bias = 0;
int32 IL_bias_sum = 0;
int32 Iout_bias_sum = 0;


//for c1 = 75uf
volatile float32 i_Kdebug = 1;

float32 feedforward_duty = DEFAULT_DUTY;


// unfolder control term
int16 period_count = 0;
int16 pre_period_count = 0;
int16 pos_gating = 0;
int16 neg_gating = 0;
int16 pos_gating_count = 10;
int16 neg_gating_count = 10;
int16 ac_dir = 0;

int16 boost_wait = 100;


// start-up and zero_crossing control
volatile int16 startup_in_progress = 1;
volatile int16 update_ref = 0;
int16 gain_boost_high = 50;
int16 gain_boost_count = 0;
int16 gain_boost=1.5;
float32 IL_err_sum_trans_step = 0;

float complete_ff = 0;

float theta_ff = 0;

float32 duty_limit =0;


//--------buffer variables------------------------------//
//buffer sensing, vab and vc2 in ADC count
int16 Vc2_count = 0;
int16 Vab_count = 0;

// ADC voltage measurements (in volts)
float Vbus_V = 0;
float Vbus_V_ave = 0;
float Vc2_V = 0;
float Vc1_V = 0; // vc1_v = vbus_v - vab_v
int vcb_pos = 0;
int vcb_neg = 0;

int16 Vc2_bias_count = 0;
int16 Vbus_bias_count = 0;

float Vbus_adc_fullvolt_to_count_ratio = 0;
float Vbus_adc_count_to_fullvolt_ratio = 0;



//vc1 notch filter parameters
// Notch filter coefficients are float32 types because high precision is required.
//notch q = 5
//@120Hz
float32 b0_notch = 0.999528983028614;
float32 b1_notch = -1.999035769948092;
float32 b2_notch = 0.999528983028614;
float32 a1_notch = -1.999035769948092;
float32 a2_notch = 0.999057966057229;

//q = 20, fs = 150khz
//float32 b0_notch = 0.999874352082578;
//float32 b1_notch = -1.999723441205725;
//float32 b2_notch = 0.999874352082578;
//float32 a1_notch = -1.999723441205725;
//float32 a2_notch = 0.999748704165156;

// Notch filter sample variables

float Vc1_notch_V = 0;
float Vc1_bpf_V = 0;
float Vc1_bpf_V_old = 0;



float kp_PI = 2E-4;
float ki_PI = 1E-4;
float Vc2_integral_limit = 1;
float a2_I = 0;
float b1_I = 0;
float b2_I = 0;
float Vc2_avg_V = 0;

float theta_c1 = 0;
float theta_c1_pre = 0;
float vc1_shape = 0;
int16 vc1_pll_enable =1;
float pll_compensation =0;



//full bridge control
float M_control = 0;//beta * dvc1/dt
//vc1 differentiator
// Differentiator parameters and coefficients





float isqrt = 0;
float isqrt_hold = 0;
float vc1_v_ff = 0;
int input_ff = 1;
float i_offset = 0.06; //150mA offset because of the function generator
float32 km = 0;
float s_d = 0;
float32 k_pr = 50;
float32 k_pr1 = 50;
float32 k_m = 0.04;
float32 k_t = 0;
float32 k_shift = 0;
float fb_en = 3e-4;
//pi loop for voltage regulation


float32 vcb_ref_sine = 0;
float32 vcb_err =0;

float32 vcb_fb = 0;
float32 vcb_sum_lim = 0.01;
float32 pr_lim = 0.1;


//-----------------120 hz parameters----------------//
float32 b0_120 = 3.333201172306336e-05;
float32 b1_120 = 0;
float32 b2_120 = -3.333201172306336e-05;
float32 a1_120 = -1.999908070791048;
float32 a0_120 = 0.999933335976554;

//pr
//float32 b0_120 = 3.333301167376564e-06;
//float32 b1_120 = 0;
//float32 b2_120 = -3.333301167376564e-06;
//float32 a1_120 = -1.999968067454212;
//float32 a0_120 = 0.999993333397666;


float32 in_120 = 0;
float32 in1_120 = 0; //x[n-1]
float32 in2_120 = 0;
float32 out_120 = 0;
float32 out1_120 = 0;
float32 out2_120 = 0;

//-----------------120 hz parameters----------------//





//-----------------240 hz parameters----------------//

float32 b0_240 = 3.333138013043675e-05;
float32 b1_240 = 0;
float32 b2_240 = -3.333138013043675e-05;
float32 a1_240 = -1.999832278412671;
float32 a0_240 = 0.999933337239739;

//pr
//float32 b0_240 = 3.333238004324364e-06;
//float32 b1_240 = 0;
//float32 b2_240 = -3.333238004324364e-06;
//float32 a1_240 = -1.999892271665245;
//float32 a0_240 = 0.999993333523992;

float32 in_240 = 0;
float32 in1_240 = 0; //x[n-1]
float32 in2_240 = 0;
float32 out_240 = 0;
float32 out1_240 = 0;
float32 out2_240 = 0;
//-----------------240 hz parameters----------------//


//-----------------360 hz parameters----------------//

float32 b0_360 = 3.333032752924740e-05;
float32 b1_360 = 0;
float32 b2_360 = -3.333032752924740e-05;
float32 a1_360 = -1.999705964164747;
float32 a0_360 = 0.999933339344942;

//float32 b0_360 = 3.333132737890008e-06;
//float32 b1_360 = 0;
//float32 b2_360 = -3.333132737890008e-06;
//float32 a1_360 = -1.999765951733484;
//float32 a0_360 = 0.999993333734524;


float32 in_360 = 0;
float32 in1_360 = 0; //x[n-1]
float32 in2_360 = 0;
float32 out_360 = 0;
float32 out1_360 = 0;
float32 out2_360 = 0;

//-----------------360 hz parameters----------------//


// moving average for harmonics
float h120_sum = 0;
float h240_sum = 0;
float h360_sum = 0;

float h120_ave = 0;
float h240_ave = 0;
float h360_ave = 0;


float h120_sample[MOV_AVE_SIZE];
float h240_sample[MOV_AVE_SIZE];
float h360_sample[MOV_AVE_SIZE];


// pi for harmonics

float h120_i = 0;
float h240_i = 0;
float h360_i = 0;

float h120_err = 0;
float h240_err = 0;
float h360_err = 0;


float kp_harm = 1e-9;
float ki_harm = 1e-9;

float tot_mag = 0;
float h180_mag = 0;
float h300_mag = 0;

float ref_harmonic = 0.3;
float ref_120hz = 1;


float kp_fund = 1e-8;
float ki_fund = 1e-8;

float k_harm_comp= 1.5;
float k_scale = 1;

float h120_pre = 0;
float h120_hold = 0;

float err_lim = 1;
float harm_err_lim = 0.5;
//tuning for mlcc cap
int16 Vout_rp =0;
float dir_ac = 0;
int dir_dc = 1;
int dir_count = 0;
int16 dac_num = 1;
//float i_dc_ripple = 0;
float i_dc_ripple_sum = 0;
float tot_mag_man = 0.01;

#endif /* GLOBAL_VARIABLES_H_ */
