
CON

  RC_0 = 25
  RC_1 = 26
  RC_2 = 24
  RC_3 = 27  'R/C input channel assignments (pin values are specified in the RC_Receiver object)
  RC_4 = 0
  RC_5 = 1
  RC_6 = 2
  RC_7 = 3 
  

  PING = RC_7


  'Output pins to corresponding motors
  MOTOR_FL = 15
  MOTOR_FR = 14
  MOTOR_BR = 13
  MOTOR_BL = 12

  MOTOR_AUX1 = 17 
  MOTOR_AUX2 = 18

  EXP_RX = 19
  EXP_TX = 20


  CS_ALT = 10 
  CS_AG  = 6
  SDO    = 7
  SDI    = 8
  SCL    = 9
  CS_M   = 5
  LED_PIN = 4

  BUZZER_1 = 11
  BUZZER_2 = 11



  RC_0_MASK = 1<<RC_0
  RC_1_MASK = 1<<RC_1
  RC_2_MASK = 1<<RC_2
  RC_3_MASK = 1<<RC_3
  RC_4_MASK = 1<<RC_4
  RC_5_MASK = 1<<RC_5
  RC_6_MASK = 1<<RC_6
  RC_7_MASK = 1<<RC_7

  RC_MASK = RC_0_MASK | RC_1_MASK + RC_2_MASK | RC_3_MASK | RC_4_MASK | RC_5_MASK + RC_6_MASK | RC_7_MASK  


pub dummy
  'just here to make this object compile, because all objects need at least one function