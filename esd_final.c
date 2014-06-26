#include "kernel.h"
#include "kernel_id.h"
#include "com_cfg.h"
#include "ecrobot_interface.h"
#include <stdlib.h>

#define RIGHT_MOTOR_PORT NXT_PORT_A
#define LEFT_MOTOR_PORT NXT_PORT_C
#define TOUCH_SENSOR_PORT NXT_PORT_S4
#define LIGHT_SENSOR_PORT NXT_PORT_S1

#define MIN_SPEED 59
#define MAX_SPEED 69

#define EMERGENCY_SPEED 75

#define GO_BACK_ROTATIONS 1.5
#define GO_OUTSIDE_ROTATIONS 1.5
#define TURN_DEGREES 200

DeclareCounter(SysTimerCnt);
DeclareEvent(TouchSensorOnEvent);
DeclareEvent(LightSensorReadingEvent);
DeclareEvent(EmergencyEvent);
DeclareTask(LineFollowerTask);
DeclareTask(EmergencyTask);
DeclareTask(MotorControlTask);
DeclareTask(TouchSensorTask);
DeclareTask(LightSensorTask);
DeclareTask(LCDTask);

void user_1ms_isr_type2(void);
void ecrobot_device_terminate(void);
void ecrobot_device_initialize(void);
void disp(int row, char* str, int val);
void send_encoder_request();
void send_motor_command(U32 motor, int speed, int brake);

enum EMERGENCY_STATES {GO_BACK, NEEDS_TURN, TURNING, GO_AWAY}; 

int touch_sensor = 0;
int light_sensor = 0;

int left_brake   = 0;
int right_brake  = 0;

int current_left_motor_speed  = 0;
int current_right_motor_speed = 0;

int initial_brightness = 0;

int current_left_motor_count = 0;
int current_right_motor_count = 0;

/* nxtOSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
	StatusType ercd;

	/* Increment OSEK Alarm Counter */
	ercd = SignalCounter(SysTimerCnt);
	if (ercd != E_OK)
	{
		ShutdownOS(ercd);
	}
}

void ecrobot_device_initialize(void)
{
	ecrobot_set_light_sensor_active(LIGHT_SENSOR_PORT);
}

void ecrobot_device_terminate(void)
{
	ecrobot_set_light_sensor_inactive(LIGHT_SENSOR_PORT);
}

TASK(LineFollowerTask)
{
	StartCOM(COMAPP);

	systick_wait_ms(300);

	initial_brightness = light_sensor;
	int new_brightness = initial_brightness;
	int before_brightness = initial_brightness;

	while(1) {

		WaitEvent(LightSensorReadingEvent | EmergencyEvent);

		EventMaskType mask = 0;
		GetEvent(LineFollowerTask, &mask);

		if (EmergencyEvent & mask) {
			ClearEvent(EmergencyEvent);
			TerminateTask();
		} else if (LightSensorReadingEvent & mask) {
			if ( new_brightness < (initial_brightness * 0.77) ) {

				if ( before_brightness >= new_brightness ) {
					// Turn to the right
					send_motor_command(LEFT_MOTOR_PORT, MIN_SPEED, 0);
					send_motor_command(RIGHT_MOTOR_PORT, MAX_SPEED, 0);
				}
				before_brightness = new_brightness;
			} else {

				// Turn to the left
				send_motor_command(LEFT_MOTOR_PORT, MAX_SPEED, 0);
				send_motor_command(RIGHT_MOTOR_PORT, MIN_SPEED, 0);
			}
			new_brightness = light_sensor;
			ClearEvent(LightSensorReadingEvent);
		}

	}
	TerminateTask();
}

TASK(TouchSensorTask)
{
	static UINT8 old_touch_status = 0;

	touch_sensor = ecrobot_get_touch_sensor(TOUCH_SENSOR_PORT);
	
	if (old_touch_status == 0 && touch_sensor == 1) {
		SetEvent(EmergencyTask, TouchSensorOnEvent);
	}

	old_touch_status = touch_sensor;

	TerminateTask();
}

TASK(LightSensorTask)
{
	light_sensor = ecrobot_get_light_sensor(LIGHT_SENSOR_PORT);
	SetEvent(LineFollowerTask, LightSensorReadingEvent);
	TerminateTask();
}

TASK(LCDTask)
{
	display_clear(0);
	disp(0, "Left speed ", current_left_motor_speed);
	disp(1, "Left brake ", left_brake);
	disp(2, "Right speed ", current_right_motor_speed);
	disp(3, "Right brake ", right_brake);
	disp(4, "Light ", light_sensor);
	disp(5, "Touch ", touch_sensor);
	disp(6, "Inital light ", initial_brightness);
	display_update();
	TerminateTask();
}

TASK(EmergencyTask)
{
	WaitEvent(TouchSensorOnEvent);
	ClearEvent(TouchSensorOnEvent);

	SetEvent(LineFollowerTask, EmergencyEvent);

	send_encoder_request();

	send_motor_command(LEFT_MOTOR_PORT, 0, 0);
	send_motor_command(RIGHT_MOTOR_PORT, 0, 0);

	int initial_left_count  = current_left_motor_count;
	int initial_right_count = current_right_motor_count;

	int terminate = 0;

	int state = GO_BACK;

	while(!terminate) {

		send_encoder_request();

		switch(state) {

			case GO_BACK:
			{
				send_motor_command(LEFT_MOTOR_PORT, -EMERGENCY_SPEED, 0);
				send_motor_command(RIGHT_MOTOR_PORT, -EMERGENCY_SPEED, 0);

				if (abs(current_right_motor_count - initial_right_count) / 360.0 >= GO_BACK_ROTATIONS) {
					state = NEEDS_TURN;
				}
				break;
			}
			case NEEDS_TURN:
			{
				send_motor_command(LEFT_MOTOR_PORT, EMERGENCY_SPEED, 0);
				state = TURNING;

				initial_left_count  = current_right_motor_count;
				initial_right_count = current_right_motor_count;
				break;
			}
			case TURNING:
			{
				if (abs(current_right_motor_count - initial_right_count) >= TURN_DEGREES) {
					state = GO_AWAY;
					initial_right_count = current_right_motor_count;
				}
				break;
			}
			case GO_AWAY:
			{
				send_motor_command(LEFT_MOTOR_PORT, EMERGENCY_SPEED, 0);
				send_motor_command(RIGHT_MOTOR_PORT, EMERGENCY_SPEED, 0);

				if (abs(current_right_motor_count - initial_right_count) / 360.0 >= GO_OUTSIDE_ROTATIONS) {
					terminate = 1;
				}
				break;
			}
		}
	}

	send_motor_command(LEFT_MOTOR_PORT, 0, 0);
	send_motor_command(RIGHT_MOTOR_PORT, 0, 0);

	TerminateTask();
}

TASK(MotorControlTask)
{
	MOTOR_CTR_MSG message;

	RecieveMessage(MotorCtrMsgRcv,&message);

	if (message.encoder_request)
	{
		current_left_motor_count  = nxt_motor_get_count(LEFT_MOTOR_PORT);
		current_right_motor_count = nxt_motor_get_count(RIGHT_MOTOR_PORT);
		TerminateTask();
	}

	nxt_motor_set_speed(message.motor, message.speed, message.brake);

	if (message.motor == RIGHT_MOTOR_PORT) {
		current_right_motor_speed = message.speed;
		right_brake = message.brake;
	} else if (message.motor == LEFT_MOTOR_PORT) {
		current_left_motor_speed  = message.speed;
		left_brake  = message.brake;
	}
	TerminateTask();
}

void send_encoder_request()
{
	MOTOR_CTR_MSG message;
	message.encoder_request = 1;
	SendMessage(MotorCtrMsgSnd, &message);
}

void send_motor_command(U32 motor, int speed, int brake){
	MOTOR_CTR_MSG message;
	message.motor = motor;
	message.speed = speed;
	message.brake = brake;
	message.encoder_request = 0;
	SendMessage(MotorCtrMsgSnd, &message);
}

void disp(int row, char *str, int val)
{ 
	display_goto_xy(0, row);
	display_string(str);
	display_int(val, 0);
}