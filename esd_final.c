#include "kernel.h"
#include "kernel_id.h"
#include "com_cfg.h"
#include "ecrobot_interface.h"
#include <stdlib.h>
#include <string.h>

#define RIGHT_MOTOR_PORT NXT_PORT_A
#define LEFT_MOTOR_PORT NXT_PORT_C
#define SONAR_SENSOR_PORT NXT_PORT_S4

#define MIN_SPEED 1
#define MAX_SPEED 100

#define A_COEFF (-100.0/18.0)
#define B_COEFF (2800.0/18.0)

#define UPPER_SONAR_BOUND 28
#define LOWER_SONAR_BOUND 10

#define LOWER_SPEED_BOUND 50
#define LOWER_REV_BOUND 50
#define LOWER_ACC_BOUND 10

#define NUMBER_OF_EXECUTIONS 20

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

void send_motor_sensor_message(MessageIdentifier target, UINT32 left_motor_count , UINT32 right_motor_count);
void send_motor_command(MessageIdentifier target, int left_motor, int right_motor, int left_break, int right_break);
void send_sonar_sensor_message(MessageIdentifier target, INT32 value, UINT32 id);
void send_validation_result(MessageIdentifier target, UINT8 result, UINT32 id);

void disp(int row, char* str, int val);
void disp_string(int row, char* str);

// Global values for debugging
int current_left_motor_speed  = 0;
int current_right_motor_speed = 0;
UINT32 current_left_motor_count = 0;
UINT32 current_right_motor_count = 0;
int current_left_brake  = 0;
int current_right_brake = 0;
INT32 current_sonar_sensor = 0;

int vali_id = 0;
int sensor_id = 0;

int acq_passes = 0;
int validation_passes = 0;
int processing_passes = 0;

int valid_failed_counter = 0;

int difference = 0;

int monitor_counter = 0;

char debug_message[32];

int count_difference_left  = 0;
int count_difference_right = 0;

int speed_difference_left  = 0;
int speed_difference_right = 0;

int cancel_alarm_counter = 0;

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
	StartCOM(COMAPP);
}

void ecrobot_device_terminate(void)
{
	ecrobot_term_sonar_sensor(SONAR_SENSOR_PORT);
}

TASK(SonarSensorTask)
{
	acq_passes++;

	static UINT32 unique_id = 0;

	INT32 sonar_sensor_value = ecrobot_get_sonar_sensor(SONAR_SENSOR_PORT);
	current_sonar_sensor = sonar_sensor_value;

	send_sonar_sensor_message(SonarSensorValueMsgSendForDataValidationTask, sonar_sensor_value, unique_id);
	send_sonar_sensor_message(SonarSensorValueMsgSendForDataProcessingTask, sonar_sensor_value, unique_id);

	sensor_id = unique_id;

	unique_id++;

	TerminateTask();
}

TASK(DataProcessingTask)
{
	processing_passes++;

	SONAR_SENSOR_MSG sensor_message;
	RecieveMessage(SonarSensorValueMsgReceiveForDataProcessingTask, &sensor_message);

	int speed = A_COEFF * sensor_message.value + B_COEFF ;

	DATA_VALIDATION_RESULT_MSG validation_message;
	RecieveMessage(DataValidationResultMsgReceive, &validation_message);

	difference = sensor_message.id - validation_message.id ;

	if ((sensor_message.id == validation_message.id) &&
		(validation_message.result == 1))
	{
		send_motor_command(MotorCommandMsgSendForOutputProcessingTask, speed, speed, 0, 0);
		strcpy(debug_message, "");
	} else {
		valid_failed_counter++;
		strcpy(debug_message, "Vali failed");
	}

	TerminateTask();
}

TASK(DataValidationTask)
{
	validation_passes++;

	SONAR_SENSOR_MSG sensor_message;

	RecieveMessage(SonarSensorValueMsgReceiveForDataValidationTask, &sensor_message);

	UINT8 result = 1;

	if ((sensor_message.value < LOWER_SONAR_BOUND) ||
		(sensor_message.value > UPPER_SONAR_BOUND))
	{
		result = 0;
	}

	vali_id = sensor_message.id;

	send_validation_result(DataValidationResultMsgSend, result, sensor_message.id);
	TerminateTask();
}

TASK(OutputProcessingTask)
{
	MOTOR_COMMAND_MSG message;

	RecieveMessage(MotorCommandMsgReceiveForOutputProcessingTask, &message);

	current_left_motor_speed  = message.left_motor;
	current_right_motor_speed = message.right_motor;

	nxt_motor_set_speed(LEFT_MOTOR_PORT, message.left_motor, message.left_motor_break);
	nxt_motor_set_speed(RIGHT_MOTOR_PORT, message.right_motor, message.right_motor_break);

	SendMessage(MotorCommandMsgSendForMonitorTask, &message);

	TerminateTask();
}

TASK(MonitorTask)
{

	static int before_n_executions_left_count = 0;
	static int after_n_executions_left_count  = 0;

	static int before_n_executions_right_count = 0;
	static int after_n_executions_right_count  = 0;

	static int before_n_executions_left_speed  = 0;
	static int before_n_executions_right_speed = 0;

	static int after_n_executions_left_speed  = 0;
	static int after_n_executions_right_speed = 0;

	static int execution_number = 0;

	MOTOR_SENSOR_MSG motor_sensor_message;
	RecieveMessage(MotorSensorValuesMsgReceive, &motor_sensor_message);
	
	MOTOR_COMMAND_MSG motor_command_message;
	RecieveMessage(MotorCommandMsgReceiveForMonitorTask, &motor_command_message);

	UINT8 left_motor_speed  = motor_command_message.left_motor;
	UINT8 right_motor_speed = motor_command_message.right_motor;

	execution_number++;
	execution_number %= NUMBER_OF_EXECUTIONS;

	if (execution_number == 0)
	{
		// Speed difference

		before_n_executions_left_speed  = after_n_executions_left_speed;
		before_n_executions_right_speed = after_n_executions_right_speed;

		after_n_executions_left_speed  = motor_command_message.left_motor;
		after_n_executions_right_speed = motor_command_message.right_motor;

		speed_difference_left  = abs(after_n_executions_left_speed  - before_n_executions_left_speed);
		speed_difference_right = abs(after_n_executions_right_speed - before_n_executions_right_speed);

		// Count difference

		before_n_executions_right_count = after_n_executions_right_count;
		before_n_executions_left_count  = after_n_executions_left_count;

		after_n_executions_left_count  = motor_sensor_message.left_motor_count;
		after_n_executions_right_count = motor_sensor_message.right_motor_count;

		count_difference_right = after_n_executions_right_count - before_n_executions_right_count;
		count_difference_left  = after_n_executions_left_count  - before_n_executions_left_count;	

		if ( ((left_motor_speed  > LOWER_SPEED_BOUND) && (count_difference_left  < LOWER_REV_BOUND) && (speed_difference_left  < LOWER_ACC_BOUND)) ||
			 ((right_motor_speed > LOWER_SPEED_BOUND) && (count_difference_right < LOWER_REV_BOUND) && (speed_difference_right < LOWER_ACC_BOUND)) )
		{
			//cancel_alarm_counter++;
			CancelAlarm(SonarSensorTaskTrigger);
		}
	}

	TerminateTask();
}

TASK(MotorSensorTask)
{
	UINT32 left_motor_count  = nxt_motor_get_count(NXT_PORT_C);
	UINT32 right_motor_count = nxt_motor_get_count(NXT_PORT_A);
	current_left_motor_count  = left_motor_count;
	current_right_motor_count = right_motor_count;

	send_motor_sensor_message(MotorSensorValuesMsgSend, left_motor_count, right_motor_count);

	TerminateTask();
}

TASK(LCDTask)
{
	display_clear(0);

	//disp(0, "Left speed ", current_left_motor_speed);
	//disp(1, "Left brake ", current_left_brake);
	//disp(2, "Right speed ", current_right_motor_speed);
	//disp(3, "Right brake ", current_right_brake);

	// Difference between unique id's
	//disp(0, "Diff", difference);

	disp(0, "L Count", count_difference_left);
	disp(1, "R Count", count_difference_right);

	disp(2, "L speed ", current_left_motor_speed);
	disp(3, "R speed ", current_right_motor_speed);

	// disp(1, "Acq pass", acq_passes);
	// disp(2, "Valid pass", validation_passes);
	// disp(3, "Process pass", processing_passes);

	disp(4, "Sonar ", current_sonar_sensor);
	
	disp(5, "L SDiff", speed_difference_left);
	disp(6, "R SDiff", speed_difference_right);

	//disp(5, "L count ", current_left_motor_count);
	//disp(6, "R count ", current_right_motor_count);

	// disp(5, "Sensor ID ", sensor_id);
	// disp(6, "Valid ID ", vali_id);

	//disp(7, "Vali count ", valid_failed_counter);
	disp(7, "Cancel ", cancel_alarm_counter);
//	disp(6, "monitor cnt", monitor_counter);


//	disp_string(7, debug_message);
	display_update();
	TerminateTask();
}

void send_motor_sensor_message(MessageIdentifier target, UINT32 left_motor_count , UINT32 right_motor_count)
{
	MOTOR_SENSOR_MSG message;
	message.left_motor_count  = left_motor_count;
	message.right_motor_count = right_motor_count;
	SendMessage(target, &message);
}

void send_motor_command(MessageIdentifier target, int left_motor, int right_motor, int left_break, int right_break)
{
	MOTOR_COMMAND_MSG message;
	message.left_motor  = left_motor;
	message.right_motor = right_motor;
	message.left_motor_break  = left_break;
	message.right_motor_break = right_break; 
	SendMessage(target, &message);
}

void send_sonar_sensor_message(MessageIdentifier target, INT32 value, UINT32 id)
{
	SONAR_SENSOR_MSG message;
	message.id    = id;
	message.value = value;
	SendMessage(target, &message);
}

void send_validation_result(MessageIdentifier target, UINT8 result, UINT32 id)
{
	DATA_VALIDATION_RESULT_MSG message;
	message.id = id;
	message.result = result;
	SendMessage(target, &message);
}

void disp_string(int row, char* str)
{
	display_goto_xy(0, row);
	display_string(str);
}

void disp(int row, char *str, int val)
{ 
	display_goto_xy(0, row);
	display_string(str);
	display_int(val, 0);
}