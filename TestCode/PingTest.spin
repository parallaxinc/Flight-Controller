CON

  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

  PING_Pin = 5                                          ' I/O Pin For PING)))


VAR

  long  range

    
OBJ

  LCD  : "fullduplexserial.spin"
  ping : "ping"

  
PUB Start

  LCD.Start(31, 30, 0, 115200 )

  repeat
    ping.fire(PING_Pin)
    waitcnt(2_000_000 + cnt )
    range := ping.Millimeters(PING_Pin)                 ' Get Range In Millimeters
    LCD.dec( range )
    lcd.tx( 13 )
    waitcnt( 2_000_000 + cnt )

  


  lcd.rx
  LCD.tx(0)
  LCD.str(string("PING))) Demo", 13, 13, "Inches      -", 13, "Centimeters -"))

  repeat                                                ' Repeat Forever
    LCD.tx(2)
    LCD.tx(15)
    LCD.tx(2)                                           ' Position Cursor
    
    range := ping.Inches(PING_Pin)                      ' Get Range In Inches
    LCD.dec(range)                                      ' Print Inches
    LCD.str(string(".0 "))                              ' Pad For Clarity
    LCD.tx(2)
    LCD.tx(14)
    LCD.tx(3)                                           ' Position Cursor
    range := ping.Millimeters(PING_Pin)                 ' Get Range In Millimeters
    LCD.dec(range / 10)                                 ' Print Whole Part
    LCD.tx(".")                                         ' Print Decimal Point
    LCD.dec(range // 10)                             ' Print Fractional Part

    waitcnt(clkfreq / 10 + cnt)                         ' Pause 1/10 Second