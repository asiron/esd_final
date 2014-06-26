#include "kernel.h"
#include "kernel_id.h"
#include "com_cfg.h"
#include "ecrobot_interface.h"
#include <stdlib.h>

#define RIGHT_MOTOR_PORT NXT_PORT_A
#define LEFT_MOTOR_PORT NXT_PORT_C
#define SONAR_SENSOR_PORT NXT_PORT_S4

#define MIN_SPEED 1
#define MAX_SPEED 100


DeclareCounter(SysTimerCnt);

DeclareAlarm(SonarSensorTaskTrigger);
DeclareAlarm(MotorSensorTaskTrigger);
DeclareAlarm(LCDTaskTrigger);

DeclareTask(SonarSensorTask);
DeclareTask(DataValidationTask);
DeclareTask(DataProcessingTask);
DeclareTask(OutputProcessingTask);
DeclareTask(MonitorTask);
DeclareTask(MotorSensorTask);
DeclareTask(LCDTask);

void user_1ms_isr_type2(void);
void ecrobot_device_terminate(void);
void ecrobot_device_initialize(void);
void disp(int row, char* str, int val);
void send_motor_command(U32 motor, int speed, int brake);

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
	ecrobot_init_sonar_sensor(SONAR_SENSOR_PORT);
}

void ecrobot_device_terminate(void)
{
	ecrobot_term_sonar_sensor(SONAR_SENSOR_PORT);
}


TASK(DataAcquisitionTask)
{
	if (!shutdown)
	{
		/* code */
	}
	TerminateTask();
}

TASK(SonarSensorTask)
{
	S32 sonar_sensor_value = ecrobot_get_sonar_sensor(LIGHT_SENSOR_PORT);

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