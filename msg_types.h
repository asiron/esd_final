typedef struct sonar_sensor_reading_msg {
	INT32 value;
	UINT32 id;
} SONAR_SENSOR_MSG ;

typedef struct motor_command_msg {
	UINT8 left_motor;
	UINT8 right_motor;
	UINT8 left_motor_break;
	UINT8 right_motor_break;
} MOTOR_COMMAND_MSG ;

typedef struct data_validation_result_msg {
	UINT8 result;
	UINT32 id;
} DATA_VALIDATION_RESULT_MSG ;

typedef struct motor_sensor_msg {
	int left_motor_count;
	int right_motor_count;
} MOTOR_SENSOR_MSG ;