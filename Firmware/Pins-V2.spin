
CON

  RC_0 = 26
  RC_1 = 27
  RC_2 = 0
  RC_3 = 1   'R/C input channel assignments (pin values are specified in the RC_Receiver object)
  RC_4 = 2
  RC_5 = 3
  RC_6 = 4
  RC_7 = 5 
  

  PING = RC_7


  'Output pins to corresponding motors
  MOTOR_FL = 15
  MOTOR_FR = 16
  MOTOR_BR = 17
  MOTOR_BL = 18


  '        V2    V1
  CS_ALT = 9    '8
  CS_AG  = 11   '11
  SDO    = 13   '12
  SDI    = 14   '13
  SCL    = 12   '14
  CS_M   = 10   '15
  LED_PIN = 8   '19

  BUZZER_1 = 6
  BUZZER_2 = 7



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