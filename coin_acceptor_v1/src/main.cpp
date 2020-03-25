#include <Arduino.h>
#include <EEPROM.h>
#include "bsp.h"
#include "token_props.h"

token_t tokens[COIN_NUM];
// variables
bool OS_door = false;
int em_val = EM_VAL_MAX, lo_em_val = EM_VAL_MAX;
unsigned long elapsed_time = 0, start_time = 0;
unsigned long OS_time_block = 0;
uint8_t state = STATE_POLL;

void ring_alarm(int num_buzz = 5, int period = 500);
void init_io();
bool assess_values(unsigned long tme, int em);
bool btn_debounce(uint8_t btn, uint8_t dbnc_tm = 2);
void prog_state();
void em_read();
void blink(uint8_t pin, uint8_t num_times, int prd);
bool init_token_values();
uint16_t calc_std_dev(uint16_t *values, int len, uint16_t avg);
void setup()
{
  INIT_SERIAL(DEBUG_BAUD);
  // Initialise IO
  init_io();

  // Initialize values from eeprom
  init_token_values();
}

void loop()
{
  // State machine
  switch (state)
  {
  case STATE_POLL:
    digitalWrite(LED_GRN_PIN, HIGH);
    if (pulseIn(OPT_SENS_ENTRANCE, HIGH, OS_ENTRANCE_POLL_TIME) > 0)
    {
      PRINT_LN("\nCoin Inserted");
      state = STATE_READ_EM;
    }
    else
    {
      state = STATE_CHECK_BTN_1;
    }
    digitalWrite(LED_GRN_PIN, LOW);
    break;
  case STATE_READ_EM:
  {
    em_read();
    PRINT("\nEM_LOW_VAL = ");
    PRINT(lo_em_val);
    digitalWrite(SOL_PATH_BLOCK, HIGH);
    state = STATE_READ_OS_DIAM;
  }
  break;
  case STATE_READ_OS_DIAM:
    OS_time_block = pulseIn(OPT_SENS_DIAM, HIGH, OS_READ_DIAM_TO)/1000;
    if (OS_time_block > 0)
    {
      PRINT("\tOS_block_time = ");
      PRINT_LN(OS_time_block);
      state = STATE_ACCEPT_REJECT;
    }
    else
    {
      // Ring alarm, eror
      state = STATE_ERROR;
    }
    digitalWrite(SOL_PATH_BLOCK, LOW);
    break;
  case STATE_ACCEPT_REJECT:
    if (assess_values(OS_time_block, lo_em_val))
    {
      digitalWrite(SOL_ACCPT_COIN, HIGH);
      delay(1000);
      digitalWrite(SOL_ACCPT_COIN, LOW);
    }
    // Reset values
    lo_em_val = EM_VAL_MAX;
    OS_time_block = 0;
    state = STATE_POLL;
    break;

  case STATE_CHECK_BTN_1:
  {
    unsigned long int tm = millis();
    int long_press = 0;
    while (!btn_debounce(BTN_1))
    {
      PRINT_LN("Button prog pressed");
      if (millis() - tm > (unsigned long)BTN_1_HOLD_TIME_TO_EJECT)
      {
        long_press = 1;
        blink(LED_RED_PIN, 2, 500);
      }
      if (millis() - tm > (unsigned long)BTN_1_HOLD_TIME_TO_PROG)
      {
        long_press = 2;
      }
    }
    switch (long_press)
    {
    case 1:
      PRINT_LN("ABORT and EJETC");
      state = STATE_ABORT_EJECT;
      break;
    case 2:
      PRINT_LN("Programming state selected");
      state = STATE_PROGRAMMING;
      break;
    default:
      state = STATE_POLL;
      break;
    }
  }

  break;

  case STATE_PROGRAMMING:
    prog_state();
    state = STATE_POLL;
    break;
  case STATE_ABORT_EJECT:
    digitalWrite(SOL_PATH_BLOCK, HIGH);
    delay(500);
    digitalWrite(SOL_PATH_BLOCK, LOW);
    state = STATE_POLL;
    break;
  case STATE_ERROR:
    ring_alarm();
    state = STATE_POLL;
    break;
  }
}

// Initialization
void init_io()
{
  // Outputs
  pinMode(SOL_PATH_BLOCK, OUTPUT);
  pinMode(SOL_EM_PULSER, OUTPUT);
  pinMode(SOL_ACCPT_COIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_GRN_PIN, OUTPUT);
  digitalWrite(SOL_ACCPT_COIN, LOW);
  digitalWrite(LED_GRN_PIN, LOW);
  //Inputs
  //pinMode(SOL_EM_SENS,      INPUT); //is an analog input
  pinMode(OPT_SENS_ENTRANCE, INPUT_PULLUP);
  pinMode(OPT_SENS_DIAM, INPUT_PULLUP);
  pinMode(BTN_1, INPUT_PULLUP);
}

// Alarms
void ring_alarm(int num_buzz, int period)
{

  // Open block solenoids
  while (num_buzz--)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(SOL_PATH_BLOCK, HIGH);
    delay(period);
    digitalWrite(SOL_PATH_BLOCK, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    delay(period);
  }
}

bool assess_values(unsigned long tme, int em)
{
  PRINT("ASSESSING tme = "); PRINT(tme); PRINT("\tem="); PRINT_LN(em);
  unsigned long temp = 0;
  for (int i = 0; i < COIN_NUM; i++)
  {
    temp = abs((int)tokens[i].avg_diam - (int)tme);
    PRINT("Temp 1 = "); PRINT_LN(temp);
    if (temp <= 2*tokens[i].SD_daim)
    {
      temp = abs((int)tokens[i].avg_em - (int)em);
      PRINT("Temp 2 = "); PRINT_LN(temp);
      if (temp <= tokens[i].SD_em)
      {
        PRINT("**** Token --> "); PRINT(i+1); PRINT_LN(" inserted!! ****");
        return true;
      }
    }
    
  }
  return false;
}

  void prog_state()
  {

    blink(LED_RED_PIN, 2, 250);
    delay(1000);
    blink(LED_RED_PIN, 2, 250);

    // Count number of button presses
    unsigned long int tm = millis();
    int prev_state = !UNPRSSD_BTN_STATE;
    int curr_state;
    int btn_press_ctr = 0;
    while (millis() - tm < PROG_STATE_BTN_PRESS_TO)
    {
      curr_state = btn_debounce(BTN_1);
      if (curr_state == (!UNPRSSD_BTN_STATE) && prev_state == UNPRSSD_BTN_STATE)
      {
        btn_press_ctr++;
      }
      prev_state = curr_state;
      delay(5);
    }

    // Check if btn presses are less than max coin num
    if (btn_press_ctr > COIN_NUM)
    {
      ring_alarm();
      return;
    }
    PRINT("PROGRAMMING TOKEN No. --> ");
    PRINT_LN(btn_press_ctr);
    blink(LED_RED_PIN, btn_press_ctr, 500);
    int coins_ctr = 0;
    tm = millis();
    unsigned long long total_OS_time_block = 0;
    unsigned long total_em_val = 0;

    uint16_t coins_em_values[COIN_ITERATIONS_CTR];
    uint16_t OS_time_block_values[COIN_ITERATIONS_CTR];
    while (coins_ctr < COIN_ITERATIONS_CTR && millis() - tm < COIN_PROG_TO)
    {
      digitalWrite(LED_RED_PIN, HIGH);
      if (pulseIn(OPT_SENS_ENTRANCE, HIGH, 5000000) > 0)
      {
        digitalWrite(LED_RED_PIN, LOW);
        // 1. Read em sensor low val
        em_read();
        if (!lo_em_val)
        {
          ring_alarm();
          // 2. Open bocking solenoid
        }
        digitalWrite(SOL_PATH_BLOCK, HIGH);
        // 3. Read OS diameter
        OS_time_block = pulseIn(OPT_SENS_DIAM, HIGH, 1000000) / 1000;
        //\ alarm if nothing was received ( block)
        if (!OS_time_block)
        {
          ring_alarm();
          return;
        }
        // 4. Close blocking solenoid
        digitalWrite(SOL_PATH_BLOCK, LOW);

        // 5. Add to total, save value for calculations
        total_OS_time_block += OS_time_block;
        total_em_val += lo_em_val;

        coins_em_values[coins_ctr] = lo_em_val;
        OS_time_block_values[coins_ctr] = OS_time_block;

        PRINT("EM= ");
        PRINT(lo_em_val);
        PRINT("\tDIAM= ");
        PRINT_LN(OS_time_block);

        // Update time and counter
        tm = millis();
        coins_ctr++;
        digitalWrite(LED_RED_PIN, LOW);
        delay(250);
        blink(LED_RED_PIN, coins_ctr, 250);
      }
    }

    // Get average
    if (coins_ctr > 4)
    {
      em_val = total_em_val / coins_ctr;
      OS_time_block = total_OS_time_block / coins_ctr;
    }
    else
    {
      ring_alarm();
      return;
    }

    // Calculate standard deviation
    tokens[btn_press_ctr - 1].SD_daim = calc_std_dev(OS_time_block_values, coins_ctr, OS_time_block);
    if (tokens[btn_press_ctr - 1].SD_daim < 1)
      tokens[btn_press_ctr - 1].SD_daim = 1;

    tokens[btn_press_ctr - 1].SD_em = calc_std_dev(coins_em_values, coins_ctr, em_val);
    if (tokens[btn_press_ctr - 1].SD_em < 1)
      tokens[btn_press_ctr - 1].SD_em = 1;

    tokens[btn_press_ctr - 1].avg_em = em_val;
    tokens[btn_press_ctr - 1].avg_diam = OS_time_block;

    // Store value
    int addr = 0;
    if (btn_press_ctr == 1)
      addr = ADDR_COIN_1_EM_AVG;
    else if (btn_press_ctr == 2)
      addr = ADDR_COIN_2_EM_AVG;
    else if (btn_press_ctr == 3)
      addr = ADDR_COIN_3_EM_AVG;

    uint8_t buff;
    /*
  // Save EM reading
  temp_u16 = tokens[btn_press_ctr-1].avg_em;
  buff = (uint8_t)(temp_u16 & 0x00ff);
  EEPROM.write(addr++, buff);
  buff = (temp_u16 >> 8);
  EEPROM.write(addr++, buff);

  // Save OS tim block
  temp_u16 = tokens[btn_press_ctr-1].avg_diam;
  buff = (uint8_t)(temp_u16 & 0x00ff);
  EEPROM.write(addr++, buff);
  buff = (uint8_t)(temp_u16 >> 8);
  EEPROM.write(addr++, buff);

  // EM SD
  temp_u16 = tokens[btn_press_ctr-1].SD_em;
  buff = (uint8_t)(temp_u16 & 0x00ff);
  EEPROM.write(addr++, buff);
  buff = (uint8_t)(temp_u16 >> 8);
  EEPROM.write(addr++, buff);

  // Save OS SD
  temp_u16 = tokens[btn_press_ctr-1].SD_daim;
  buff = (uint8_t)(temp_u16 & 0x00ff);
  EEPROM.write(addr++, buff);
  buff = (uint8_t)(temp_u16 >> 8);
  EEPROM.write(addr++, buff);
  */

    // Save EM reading
    buff = (uint8_t)tokens[btn_press_ctr - 1].avg_em;
    EEPROM.write(addr++, buff);

    // Save OS tim block
    buff = (uint8_t)tokens[btn_press_ctr - 1].avg_diam;
    EEPROM.write(addr++, buff);

    // EM SD
    buff = (uint8_t)tokens[btn_press_ctr - 1].SD_em;
    EEPROM.write(addr++, buff);

    // Save OS SD
    buff = (uint8_t)tokens[btn_press_ctr - 1].SD_daim;
    EEPROM.write(addr++, buff);

    // EM avlid
    EEPROM.write(ADDR_VALID_CONTENTS, VALID_CONTENTS_FLG);

    // Print feedback
    PRINT("DIAM_AVG = ");
    PRINT((int)tokens[btn_press_ctr - 1].avg_diam);
    PRINT("\tDIAM_SD = ");
    PRINT((int)tokens[btn_press_ctr - 1].SD_daim);
    PRINT("\t\tEM_AVG = ");
    PRINT((int)tokens[btn_press_ctr - 1].avg_em);
    PRINT("\tEM_SD = ");
    PRINT_LN((int)tokens[btn_press_ctr - 1].SD_em);

    blink(LED_RED_PIN, btn_press_ctr, 500);
  }

  void em_read()
  {
    bool ps = HIGH;
    delay(500);
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(SOL_EM_PULSER, HIGH);
      DELAY_EMP(EM_PULSE_PERIOD);
      digitalWrite(SOL_EM_PULSER, LOW);
      DELAY_EMP(EM_PULSE_PERIOD);
    }
    unsigned long int ttl_val = 0;
    unsigned long int ctr = 0;
    start_time = millis();
    while (millis() - start_time < EM_MEASURE_TIME)
    {
      digitalWrite(SOL_EM_PULSER, ps);
      DELAY_EMP(EM_PULSE_PERIOD);
      ttl_val += analogRead(SOL_EM_SENS);
      ctr++;
      //if (em_val < lo_em_val)
      //lo_em_val = em_val;
      ps = !ps;
    }
    digitalWrite(SOL_EM_PULSER, LOW);
    lo_em_val = (int)(ttl_val / ctr);
  }

  // Checks if button was pressed (low) state
  bool btn_debounce(uint8_t btn, uint8_t dbnc_tm)
  {
    bool prev_state = (bool)digitalRead(btn);
    delay(dbnc_tm);
    if ((bool)digitalRead(btn) == prev_state)
      return prev_state;
    return UNPRSSD_BTN_STATE;
  }

  void blink(uint8_t pin, uint8_t num_times, int prd)
  {
    for (int i = 0; i < num_times; i++)
    {
      digitalWrite(pin, HIGH);
      delay(prd);
      digitalWrite(pin, LOW);
      delay(prd);
    }
  }

  bool init_token_values()
  {
    uint8_t buff[TOKEN_DATA_LEN];
    int addr = ADDR_VALID_CONTENTS;
    for (int i = 0; i < TOKEN_DATA_LEN; i++)
      buff[i] = EEPROM.read(addr++);

    if (buff[ADDR_VALID_CONTENTS] != VALID_CONTENTS_FLG)
    {
      ring_alarm();

      blink(LED_RED_PIN, 2, 500);
      blink(LED_GRN_PIN, 2, 500);
      return false;
    }
    addr = COIN_DATA_OFFSET;
    PRINT_LN();
    for (int i = 0; i < COIN_NUM; i++)
    {
      /*tokens[i].avg_em = (buff[addr++] && 0x00ff);
    tokens[i].avg_em |= ((uint16_t)buff[addr++] << 8);
    tokens[i].avg_diam = (buff[addr++] && 0x00ff);
    tokens[i].avg_diam |= ((uint16_t)buff[addr++] << 8);
    tokens[i].SD_em = (buff[addr++] && 0x00ff);
    tokens[i].SD_em |= ((uint16_t)buff[addr++] << 8);
    tokens[i].SD_daim = (buff[addr++] && 0x00ff);
    tokens[i].SD_daim |= ((uint16_t)buff[addr++] << 8);*/
      tokens[i].coin_num = i;
      tokens[i].avg_em = buff[addr++];
      tokens[i].avg_diam = buff[addr++];
      tokens[i].SD_em = buff[addr++];
      tokens[i].SD_daim = buff[addr++];

      PRINT("TOKEN ");
      PRINT(i);
      PRINT("\tAVG_EM = ");
      PRINT(tokens[i].avg_em);
      PRINT("\tEM_SD = ");
      PRINT(tokens[i].SD_em);
      PRINT("\tAVG_DIAM = ");
      PRINT(tokens[i].avg_diam);
      PRINT("\tDIAM_SD = ");
      PRINT_LN(tokens[i].SD_daim);
    }

    return true;
  }

  uint16_t calc_std_dev(uint16_t * values, int len, uint16_t avg)
  {

    uint32_t std_dev = 0;
    for (int i = 0; i < len; i++)
    {
      std_dev += abs(values[i] - avg) * (values[i] - avg);
    }
    std_dev /= (uint32_t)len;
    std_dev = sqrt(std_dev);

    return (uint16_t)std_dev;
  }
