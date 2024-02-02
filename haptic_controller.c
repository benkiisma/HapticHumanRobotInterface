#include "haptic_controller.h"
#include "communication.h"
#include "drivers/adc.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/callback_timers.h"
#include "lib/utils.h"
#include "torque_regulator.h"
#include "drivers/adc.h"

#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 1000 // Default control loop period [us].

volatile uint32_t  hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].

#define SIZE 500
volatile float32_t voltage = 0;
volatile float32_t prev_voltage = 0;
volatile float32_t highpassed = 0;
volatile float32_t prev_highpassed = 0;
volatile float32_t bandpassed = 0;
volatile float32_t prev_bandpassed = 0;
volatile float32_t envelope = 0;

volatile float32_t dt;
volatile float32_t activate = 0;

volatile float32_t maxEMG = -1000;
volatile float32_t minEMG = 1000;
volatile float32_t objPos = 0;
volatile float32_t prev_pos[5];
volatile float32_t integral_error = 0;
volatile float32_t prev_error = 0;



void hapt_Update(void);

/**
  * @brief Initializes the haptic controller.
  */
void hapt_Init(void)
{
    hapt_timestamp = 0;
    hapt_motorTorque = 0.0f;

    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // Share some variables with the computer.
    /*comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod,
                           cbt_SetHapticControllerPeriod);
    comm_monitorFloat("motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
    comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);
    comm_monitorFloat("timestep", (float32_t*)&hapt_timestamp, READONLY);*/
    comm_monitorFloat("encoder_paddle_pos [deg]", (float32_t*)&hapt_encoderPaddleAngle, READONLY);
    comm_monitorFloat("EMG voltage", (float32_t*)&voltage, READONLY);
    //comm_monitorFloat("EMG highpassed", (float32_t*)&highpassed, READONLY);
    //comm_monitorFloat("EMG lowpassed", (float32_t*)&bandpassed, READONLY);
    comm_monitorFloat("Envelope", (float32_t*)&envelope, READONLY);
    comm_monitorFloat("max", (float32_t*)&maxEMG, READONLY);
    comm_monitorFloat("min", (float32_t*)&minEMG, READONLY);
    comm_monitorFloat("Activate", (float32_t*)&activate, READWRITE);
    comm_monitorFloat("objective Position", (float32_t*)&objPos, READONLY);
}

float32_t low_pass(float32_t prev_filtered, float32_t dt, float32_t current_voltage, float32_t frequency){
	float32_t RC = 1.0/(frequency*2*3.14);
	float32_t alpha = dt/(RC+dt);
	return prev_filtered + (alpha * (current_voltage - prev_filtered));
}

float32_t high_pass(float32_t prev_filtered, float32_t dt, float32_t current_voltage, float32_t frequency, float32_t prev_voltage){
	float32_t RC = 1.0/(frequency*2*3.14);
	float32_t alpha = RC/(RC + dt);
	return alpha * (prev_filtered + current_voltage - prev_voltage);
}

/**
  * @brief Updates the haptic controller state.
  */
void hapt_Update()
{

    float32_t motorShaftAngle; // [deg].

    // Compute the dt (uncomment if you need it).
     dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f; // [s].

    // Increment the timestamp.
    hapt_timestamp += cbt_GetHapticControllerPeriod();
    
    // Get the Hall sensor voltage.
    hapt_hallVoltage = hall_GetVoltage();

    // Get the encoder position.
    motorShaftAngle = enc_GetPosition();
    hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;

    // retrieve EMG value, store it in array

    voltage = adc_GetChannelVoltage(ADC_CHANNEL_6);
	highpassed = high_pass(prev_highpassed, dt, voltage, 20, prev_voltage);
	bandpassed = low_pass(prev_bandpassed, dt, highpassed, 400);

	if(bandpassed<0){
		envelope = ((SIZE-1)*envelope - bandpassed)/SIZE;
	}
	else
	{
		envelope = ((SIZE-1)*envelope + bandpassed)/SIZE;
	}

	prev_voltage = voltage;
	prev_highpassed = highpassed;
	prev_bandpassed = bandpassed;

	if (hapt_timestamp >= 1000000.0f * 5){
		if(envelope>maxEMG){
			maxEMG = envelope;
		}
		else if(envelope<minEMG){
			minEMG = envelope;
		}
	}

	float32_t motor_torque = 0;

	// ############## Torque control ##############
	if(activate == 1){
		float32_t torque_max = 0.006;
		motor_torque = ((envelope - minEMG)/(maxEMG-minEMG)-0.2);
		if(motor_torque<0)motor_torque=0;
		motor_torque /= 0.8;
		motor_torque *= torque_max;
	}

	// ############## Position control ##############
	if(activate == 2){

		float32_t mean_pos = 0;
		for(int i=0;i<4;i++){
			mean_pos += prev_pos[i+1];
			prev_pos[i] = prev_pos[i+1];
		}
		prev_pos[4] = hapt_encoderPaddleAngle;
		mean_pos = (mean_pos + prev_pos[4])/5;

		objPos = (envelope - minEMG)/(maxEMG-minEMG) * 60 - 30; // objective position for PID in degrees

		/*float32_t kp = 0.0002;
		float32_t ki = 0.000015;
		float32_t kd = 0.0000003;*/
		float32_t kp = 0.0002;
		float32_t ki = 0.000001;
		float32_t kd = 0;

		float32_t error = objPos - mean_pos;
		integral_error += error;
		float32_t speed = (error - prev_error) / dt;

		motor_torque = kp * error + ki * integral_error + kd * (0 - speed);

		prev_error = error;
	}

	// ############## New position control ##############
	if(activate==3){
		float32_t mean_pos = 0;
		for(int i=0;i<4;i++){
			mean_pos += prev_pos[i+1];
			prev_pos[i] = prev_pos[i+1];
		}
		prev_pos[4] = hapt_encoderPaddleAngle;
		mean_pos = (mean_pos + prev_pos[4])/5;

		float32_t ratio = (envelope - minEMG)/(maxEMG-minEMG);
		if (ratio<0.3){
			objPos -= 0.005;
		}
		else if (ratio>0.7){
			objPos += 0.005;
		}
		if (objPos > 30){
			objPos = 30;
		}
		if (objPos < -30){
			objPos = -30;
		}

		float32_t kp = 0.0002;
		float32_t ki = 0.000001;
		float32_t kd = 0;

		float32_t error = objPos - mean_pos;
		integral_error += error;
		float32_t speed = (error - prev_error) / dt;

		motor_torque = kp * error + ki * integral_error + kd * (0 - speed);

		prev_error = error;
	}

	torq_SetTorque(motor_torque);
}


