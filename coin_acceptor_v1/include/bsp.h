#ifndef BSP_H
#define BSP_H
#define SOL_PATH_BLOCK		5
#define SOL_EM_PULSER		7
#define SOL_ACCPT_COIN		8
#define SOL_EM_SENS			A0

#define OPT_SENS_ENTRANCE	9
#define OPT_SENS_DIAM		10
#define BTN_1				2
#define BUZZER_PIN			12
#define LED_RED_PIN			13
#define LED_GRN_PIN 		6

// EEPROM ADDRESSES
#define VALID_CONTENTS_FLG			0xAB
#define ADDR_VALID_CONTENTS			0
#define COIN_DATA_OFFSET			1

#define ADDR_COIN_1_EM_AVG			1	
#define ADDR_COIN_1_BLOCK_TM_AVG	2
#define ADDR_COIN_1_EM_SD			3
#define ADDR_COIN_1_BLOCK_TM_SD		4

#define ADDR_COIN_2_EM_AVG			5
#define ADDR_COIN_2_BLOCK_TM_AVG	6
#define ADDR_COIN_2_EM_SD			7
#define ADDR_COIN_2_BLOCK_TM_SD		8

#define ADDR_COIN_3_EM_AVG			9
#define ADDR_COIN_3_BLOCK_TM_AVG	10
#define ADDR_COIN_3_EM_SD			11
#define ADDR_COIN_3_BLOCK_TM_SD		12

#define TOKEN_DATA_LEN				13
// Constants
#define EM_PULSE_PERIOD			5
#define COIN_NUM				3
#define SOL_OPEN_TIME			250
#define EM_VAL_MAX				1023	// Maximm adc value on solenoid input
#define EM_MEASURE_TIME			250		// Time taken measuring em
#define READ_EM_DELAY			5		// 
#define OS_READ_DIAM_TO			2000000
#define OP_TIMEOUT				3000	// Operation timeout
#define OS_ENTRANCE_POLL_TIME	4000000	// Polling timeout for entrance optical sensor
#define OS_READ_DIAM_TIME		1500000	//
#define BTN_1_POLL_DELAY		10
#define BTN_1_HOLD_TIME_TO_PROG	10000
#define BTN_1_HOLD_TIME_TO_EJECT 1000
#define BTN_1_INTEGRAL_TO_ACTIVATE_PROG ((BTN_1_HOLD_TIME_TO_PROG/2))
#define PROG_STATE_BTN_PRESS_TO	6000	// Timeout for user to toggle button indicating the coin to be programmed in programming mode
#define UNPRSSD_BTN_STATE		HIGH	// Unpressed btn state
#define COIN_ITERATIONS_CTR		10
#define COIN_PROG_TO			15000	// Timeout if in programming mode no coins are inserted
// Machine states
#define STATE_POLL				1
#define STATE_READ_EM			2
#define STATE_READ_OS_DIAM		3
#define STATE_ACCEPT_REJECT		4
#define STATE_PROGRAMMING		5
#define STATE_CHECK_BTN_1		6
#define STATE_ABORT_EJECT		7
#define STATE_ERROR				15
// Debugging 
#define DEBUG_BAUD				9600
#define DEBUG
#ifdef DEBUG 
#define PRINT(x)				Serial.print(x)
#define PRINT_LN(x)				Serial.println(x)
#define POLL_SERIAL_INPUT()		Serial.available()
#define READ_SERIAL_INPUT()		Serial.read()
#define INIT_SERIAL(x)				Serial.begin(x)
#else
#define PRINT(x)				void
#define PRINT_LN(x)				void
#define POLL_SERIAL_INPUT()		void
#define READ_SERIAL_INPUT()		void
#endif
#endif
