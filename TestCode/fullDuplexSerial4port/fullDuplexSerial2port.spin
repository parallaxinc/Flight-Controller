''******************************************************************
''*  Based on                                                      *
''*  Full-Duplex Serial Driver v1.1                                *
''*  Author: Chip Gracey                                           *
''*  Copyright (c) 2006 Parallax, Inc.                             *
''*  See end of file for terms of use.                             *
''*                                                                *
''*  Juergen Buchmueller 2010-04-21                                *
''*   Modified to support 2 serial ports only                      *
''*   Increased RX buffer size to 256 bytes each                   *
''*  Tim Moore 2008 - 64 byte buffers x 4 ports                    *
''*   Modified to support 4 serial ports                           *
''*   It should run 1 port faster than FullDuplexSerial or run     *
''*   up to 4 ports                                                *
''*   Merged 64 byte rx buffer change                              *
''*   Merged Debug_PC (Jon Williams)                               *
''*   Uses DAT rather than VAR so can be used in multiple objects  *
''*   If you want multiple objects using this driver, you must     *
''*   copy the driver to a new file and make sure version long is  *
''*   unique in each version
''*   Added txflush                                                *
''*   Optimization perf                                            *
''*    1 port up to 750kbps                                        *
''*    2 port up to 230kbps                                        *
''*    3 port up to 140kbps                                        *
''*    4 port up to 100kbps                                        *
''*   Tested 4 ports to 115Kbps with 6MHz crystal                  *
''*   These are approx theoretical worse case you may get faster   *
''*   if port is active but idle                                   *
''*   Added RTS/CTS flow control                                   *
''*                                                                *
''*   There is no perf penalty supporting 4 serial ports when they *
''*   are not enabled                                              *
''*   There is no perf penalty supporting CTS and RTS              *
''*   Enabling CTS on any port costs 4 clocks per port             *
''*   Enabling RTS on any port costs 32 clocks per port            *
''*   Main Rx+Tx loop is ~256 clocks per port (without CTS/RTS)    *
''*   compared with FullDuplexSerial at ~356 clocks                *
''*                                                                *
''*   There is a cost to read/write a byte in the transmit/        *
''*   receive routines. The transmit cost is greater than the      *
''*   receive cost so between propellors you can run at max baud   *
''*   rate. If receiving from another device, the sending device   *
''*   needs a delay between each byte once you are above ~470kbps  *
''*   with 1 port enabled                                          *
''*                                                                *
''*   Size:                                                        *
''*     Cog Initialzation code 1 x 8 + 4 x 25                      *
''*     Cog Receive code 4 x 30 words                              *
''*     Cog Transmit code 4 x 26 words                             *
''*     Spin/Cog circular buffer indexes 4 x 4 words               *
''*       Used in both spin and Cog and read/written in both       *
''*       directions                                               *
''*     Spin/Cog per port info 4 x 8 words                         *
''*       Passed from Spin to Cog on cog initialization            *
''*     Spin per port info 4 x 1 byte                              *
''*       Used by Spin                                             *
''*     Spin/Cog rx/tx buffer hub address 4 x 4 words              *
''*       Passed from Spin to Cog on cog initialization            *
''*     Spin/Cog rx/tx index hub address 4 x 4 words               *
''*       Passed from Spin to Cog on cog initialization            *
''*     Spin per port rx buffer 4 x 64 byte                        *
''*       Read by Spin, written by cog                             *
''*     Cog per port rx state 4 x 4 words (overlayed on rx buffer) *
''*       Used by Cog                                              *
''*     Spin per port tx buffer 4 x 16 byte                        *
''*       Written by Spin, read by Cog                             *
''*     Cog per port tx state 4 x 4 words (overlayed on tx buffer) *
''*       Used by Cog                                              *
''*     Cog constants 4 words                                      *
''*   A significant amount of space (4 x 16 words) is used for     *
''*   pre-calculated information: hub addresses, per port          *
''*   configuration. This speeds up the tx/rx routines at the cost *
''*   of this space.                                               *
''*                                                                *
''*   Note: There are 8 longs remaining in the cog's memory,       *
''*   expect to do some work to add features :).                   *
''*                                                                *
''*   7/1/08: Fixed bug of not receiving with only 1 port enabled  *
''*           Fixed bug of rts not working on ports 0, 2 and 3     *
''*  7/22/08: Missed a jmpret call in port 1 and 3 tx              *
''*           Fixed a bug in port 3 tx not increasing tx ptr       *
''*  7/24/08: Added version variable to change if need multiple    *
''*           copies of the driver                                 *
''*                                                                *
''******************************************************************

CON
  FF                            = 12                    ' form feed
  CR                            = 13                    ' carriage return

  NOMODE                        = %000000
  INVERTRX                      = %000001
  INVERTTX                      = %000010
  OCTX                          = %000100
  NOECHO                        = %001000
  INVERTCTS                     = %010000
  INVERTRTS                     = %100000

  PINNOTUSED                    = -1                    'tx/tx/cts/rts pin is not used
  
  DEFAULTTHRESHOLD              = 0                     'rts buffer threshold

  BAUD1200                      = 1200
  BAUD2400                      = 2400
  BAUD4800                      = 4800
  BAUD9600                      = 9600
  BAUD19200                     = 19200
  BAUD38400                     = 38400
  BAUD57600                     = 57600
  BAUD115200                    = 115200
      
PUB Init
''Always call init before adding ports
  Stop

PUB AddPort(port,rxpin,txpin,ctspin,rtspin,rtsthreshold,mode,baudrate)
'' Call AddPort to define each port
'' port 0-3 port index of which serial port
'' rx/tx/cts/rtspin pin number                          XXX#PINNOTUSED if not used
'' rtsthreshold - buffer threshold before rts is used   XXX#DEFAULTTHRSHOLD means use default
'' mode bit 0 = invert rx                               XXX#INVERTRX
'' mode bit 1 = invert tx                               XXX#INVERTTX
'' mode bit 2 = open-drain/source tx                    XXX#OCTX
'' mode bit 3 = ignore tx echo on rx                    XXX#NOECHO
'' mode bit 4 = invert cts                              XXX#INVERTCTS
'' mode bit 5 = invert rts                              XXX#INVERTRTS
'' baudrate
  if cog OR (port > 1)
    abort
  if rxpin <> -1
    long[@rxmask0][port] := |< rxpin
  if txpin <> -1
    long[@txmask0][port] := |< txpin
  if ctspin <> -1
    long[@ctsmask0][port] := |< ctspin
  if rtspin <> -1
    long[@rtsmask0][port] := |< rtspin
    if (rtsthreshold > 0) AND (rtsthreshold < 256)
      long[@rtssize0][port] := rtsthreshold
    else
      long[@rtssize0][port] := 192                      'default rts threshold 3/4 of buffer
  long[@rxtx_mode0][port] := mode
  if mode & INVERTRX
    byte[@rxchar0][port] := $ff
  long[@bit_ticks0][port] := (clkfreq / baudrate)
  long[@bit4_ticks0][port] := long[@bit_ticks0][port] >> 2

PUB Start : okay
'' Call start to start cog
'' Start serial driver - starts a cog
'' returns false if no cog available
''
  rxbuff_head_ptr0 := rxbuff_ptr0 := @rx_buffer0
  rxbuff_head_ptr1 := rxbuff_ptr1 := @rx_buffer1
  txbuff_tail_ptr0 := txbuff_ptr0 := @tx_buffer0
  txbuff_tail_ptr1 := txbuff_ptr1 := @tx_buffer1
  rx_head_ptr0 := @rx_head0
  rx_head_ptr1 := @rx_head1
  rx_tail_ptr0 := @rx_tail0
  rx_tail_ptr1 := @rx_tail1
  tx_head_ptr0 := @tx_head0
  tx_head_ptr1 := @tx_head1
  tx_tail_ptr0 := @tx_tail0
  tx_tail_ptr1 := @tx_tail1
  bit_ticks_ptr0 := @new_bit_ticks0
  bit_ticks_ptr1 := @new_bit_ticks1
  new_bit_ticks0 := long[@bit_ticks0][0]
  new_bit_ticks1 := long[@bit_ticks0][1]
  okay := cog := cognew(@entry, @rx_head0) + 1

PUB Stop
'' Stop serial driver - frees a cog
  if cog
    cogstop(cog~ - 1)
  longfill(@startfill, 0, (@endfill-@startfill)/4)      'initialize head/tails,port info and hub buffer pointers

PUB getCogID : result
  return cog -1
'
PUB rxflush(port)
' Flush receive buffer
  repeat while rxcheck(port) => 0

PUB rxavail(port) : truefalse

'' Check if byte(s) available
'' returns true (-1) if bytes available
  if long[@rx_tail0][port] <> long[@rx_head0][port]
    truefalse:=-1
  else
    truefalse:=0


PUB rxcheck(port) : rxbyte
'' Check if byte received (never waits)
'' returns -1 if no byte received, $00..$FF if byte
'  if port > 3
'    abort
  rxbyte--
  if long[@rx_tail0][port] <> long[@rx_head0][port]
    rxbyte := byte[@rxchar0][port] ^ byte[@rx_buffer0][(port<<6)+long[@rx_tail0][port]]
    long[@rx_tail0][port] := (long[@rx_tail0][port] + 1) & $ff
{
PUB rxtime(port,ms) : rxbyte | t
'' Wait ms milliseconds for a byte to be received
'' returns -1 if no byte received, $00..$FF if byte
  t := cnt
  repeat until (rxbyte := rxcheck(port)) => 0 or (cnt - t) / (clkfreq / 1000) > ms
}
PUB rx(port) : rxbyte
'' Receive byte (may wait for byte)
'' returns $00..$FF
  repeat while (rxbyte := rxcheck(port)) < 0

'PUB Lookahead(port) : rxbyte | tailvalue' returns a long with the next 4 bytes but doesn't read them (the rx_byte is actually a long)
'   tailvalue := long[@rx_tail][port] ' tail value 0 to 63
'   repeat 4 ' collect 4 bytes and turn them into a long
''     rxbyte := (rxbyte <<8) ' shift byte over by 1
'     rxbyte := (rxbyte <<8) + byte[@rxchar0][port] ^ byte[@rx_buffer0][(port<<6)+tailvalue] ' get value and add to current value
'     tailvalue := (tailvalue +1) & $FF ' add one and cycle if >255

PUB Lookbehind(port) : rxbyte | headvalue ' takes the head and looks at the last 4 characters to come into the port
   headvalue := (long[@rx_head0][port] -4) & $FF' head value minus 4 and 0 to 255
   repeat 4 ' collect 4 bytes
     rxbyte := (rxbyte <<8) + byte[@rxchar0][port] ^ byte[@rx_buffer0][(port<<6)+headvalue]
     headvalue := (headvalue +1) & $FF ' add one and cycle if >255

PUB tx(port,txbyte)
'' Send byte (may wait for room in buffer)
'  if port > 1
'    abort
  repeat until (long[@tx_tail0][port] <> (long[@tx_head0][port] + 1) & $F)
  byte[@tx_buffer0][(port<<4)+long[@tx_head0][port]] := txbyte
  long[@tx_head0][port] := (long[@tx_head0][port] + 1) & $F

  if long[@rxtx_mode0][port] & NOECHO
    rx(port)
{
PUB txflush(port)
  repeat until (long[@tx_tail][port] == long[@tx_head][port])
}
PUB str(port,stringptr)
'' Send string                    
  repeat strsize(stringptr)
    tx(port,byte[stringptr++])

PUB strln(port,strAddr)
'' Print a zero-terminated string
  str(port,strAddr)
  tx(port,CR)  

PUB dec(port,value)
'' Print a decimal number
  decl(port,value,10,0)

PUB decf(port,value, width)
'' Prints signed decimal value in space-padded, fixed-width field
  decl(port,value,width,1)

PUB decx(port,value, digits)
'' Prints zero-padded, signed-decimal string
'' -- if value is negative, field width is digits+1
  decl(port,value,digits,2)

PUB decl(port,value,digits,flag) | i
  digits := 1 #> digits <# 10
  if value < 0
    -value
    tx(port,"-")

  i := 1_000_000_000
  if flag & 3
    if digits < 10                                      ' less than 10 digits?
      repeat (10 - digits)                              '   yes, adjust divisor
        i /= 10

  repeat digits
    if value => i
      tx(port,value / i + "0")
      value //= i
      result~~
    elseif (i == 1) OR result OR (flag & 2)
      tx(port,"0")
    elseif flag & 1
      tx(port," ")
    i /= 10

PUB hex(port,value, digits)
'' Print a hexadecimal number
  value <<= (8 - digits) << 2
  repeat digits
    tx(port,lookupz((value <-= 4) & $F : "0".."9", "A".."F"))
{
PUB ihex(port,value, digits)
'' Print an indicated hexadecimal number
  tx(port,"$")
  hex(port,value,digits)

PUB bin(port,value, digits)
'' Print a binary number
  value <<= 32 - digits
  repeat digits
    tx(port,(value <-= 1) & 1 + "0")

PUB padchar(port,count, txbyte)
  repeat count
     tx(port,txbyte)

PUB ibin(port,value, digits)
'' Print an indicated binary number
  tx(port,"%")
  bin(port,value,digits)
}
PUB putc(port,txbyte)
'' Send a byte to the terminal
  tx(port,txbyte)
{
PUB newline(port)
  putc(port,CR)

PUB cls(port)
  putc(port,FF)
}
PUB getc(port)
'' Get a character
'' -- will not block if nothing in uart buffer
   return rxcheck(port)
'  return rx

PUB get_entry
  return @entry

PUB get_tx_head
  return tx_head_ptr0

PUB get_tx_tail
  return tx_tail_ptr0

PUB get_tx_buffer
  return txbuff_ptr0

PUB get_rx_head
  return rx_head_ptr0

PUB get_rx_tail
  return rx_tail_ptr0

PUB get_rx_buffer
  return rxbuff_ptr0

PUB get_bit_ticks0
  return bit_ticks_ptr0

PUB get_tx_head1
  return tx_head_ptr1

PUB get_tx_tail1
  return tx_tail_ptr1

PUB get_tx_buffer1
  return txbuff_ptr1

PUB get_rx_head1
  return rx_head_ptr1

PUB get_rx_tail1
  return rx_tail_ptr1

PUB get_rx_buffer1
  return rxbuff_ptr1

PUB get_bit_ticks1
  return bit_ticks_ptr1

DAT
'***********************************
'* Assembly language serial driver *
'***********************************
'
                        org     0
'
' Entry
'                   
'To maximize the speed of rx and tx processing, all the mode checks are no longer inline
'The initialization code checks the modes and modifies the rx/tx code for that mode
'e.g. the if condition for rx checking for a start bit will be inverted if mode INVERTRX
'is it, similar for other mode flags
'The code is also patched depending on whether a cts or rts pin are supplied. The normal
' routines support cts/rts processing. If the cts/rts mask is 0, then the code is patched
'to remove the addtional code. This means I/O modes and CTS/RTS handling adds no extra code
'in the rx/tx routines which not required.
'Similar with the co-routine variables. If a rx or tx pin is not configured the co-routine
'variable for the routine that handles that pin is modified so the routine is never called
'We start with port 3 and work down to ports because we will be updating the co-routine pointers
'and the order matters. e.g. we can update txcode3 and then update rxcode3 based on txcode3
'port 3
entry
rxcode0 if_never        mov     rxcode0,#receive0     'statically set these variables
txcode0 if_never        mov     txcode0,#transmit0
rxcode1 if_never        mov     rxcode1,#receive1
txcode1 if_never        mov     txcode1,#transmit1

'port 1
                        test    rxtx_mode1,#OCTX wz   'init tx pin according to mode
                        test    rxtx_mode1,#INVERTTX wc
        if_z_ne_c       or      outa,txmask1
        if_z            or      dira,txmask1
        if_z_eq_c       or      txout1,domuxnc        'patch muxc to muxnc
        if_nz           movd    txout1,#dira          'change destination from outa to dira
                        test    rxtx_mode1,#INVERTRX wz 'wait for start bit on rx pin
        if_nz           xor     start1,doifc2ifnc     'if_c jmp to if_nc
                        or      ctsmask1,#0     wz
        if_nz           test    rxtx_mode1,#INVERTCTS wc
        if_nz_and_nc    or      ctsi1,doif_z_or_nc    'if_nc jmp
        if_nz_and_c     or      ctsi1,doif_z_or_c     'if_c jmp
        if_z            mov     txcts1,transmit1      'copy the jmpret over the cts test
        if_z            movs    ctsi1,#txcts1         'patch the jmps to transmit to txcts0  
        if_z            add     txcode1,#1            'change co-routine entry to skip first jmpret
                                                      'patch rx routine depending on whether rts is used
                                                      'and if it is inverted
                        or      rtsmask1,#0     wz
        if_nz           test    rxtx_mode1,#INVERTRTS wc
        if_nz_and_nc    or      rts1,domuxnc          'patch muxc to muxnc
        if_z            mov     norts1,rec1i          'patch to a jmp #receive1
        if_z            movs    start1,#receive1      'skip all rts processing                  
                        or      txmask1,#0      wz
'        if_z            mov     txcode1,rxcode2       'use variable in case it has been changed
                        or      rxmask1,#0      wz
'        if_z            mov     rxcode1,txcode1       'use variable in case it has been changed
'port 0
                        test    rxtx_mode0,#OCTX wz    'init tx pin according to mode
                        test    rxtx_mode0,#INVERTTX wc
        if_z_ne_c       or      outa,txmask0
        if_z            or      dira,txmask0
                                                      'patch tx routine depending on invert and oc
                                                      'if invert change muxc to muxnc
                                                      'if oc change out1 to dira
        if_z_eq_c       or      txout0,domuxnc        'patch muxc to muxnc
        if_nz           movd    txout0,#dira          'change destination from outa to dira
                                                      'patch rx wait for start bit depending on invert
                        test    rxtx_mode0,#INVERTRX wz 'wait for start bit on rx pin
        if_nz           xor     start0,doifc2ifnc     'if_c jmp to if_nc
                                                      'patch tx routine depending on whether cts is used
                                                      'and if it is inverted
                        or      ctsmask0,#0     wz    'cts pin? z not set if in use
        if_nz           test    rxtx_mode0,#INVERTCTS wc 'c set if inverted
        if_nz_and_nc    or      ctsi0,doif_z_or_nc    'if_nc jmp
        if_nz_and_c     or      ctsi0,doif_z_or_c     'if_c jmp
        if_z            mov     txcts0,transmit0      'copy the jmpret over the cts test
        if_z            movs    ctsi0,#txcts0         'patch the jmps to transmit to txcts0  
        if_z            add     txcode0,#1            'change co-routine entry to skip first jmpret
                                                      'patch rx routine depending on whether rts is used
                                                      'and if it is inverted
                        or      rtsmask0,#0     wz    'rts pin, z not set if in use
        if_nz           test    rxtx_mode0,#INVERTRTS wc
        if_nz_and_nc    or      rts0,domuxnc          'patch muxc to muxnc
        if_z            mov     norts0,rec0i          'patch to a jmp #receive
        if_z            movs    start0,#receive0      'skip all rts processing if not used
                                                      'patch all of tx routine out if not used                  
                        or      txmask0,#0      wz
'        if_z            mov     txcode,rxcode1        'use variable in case it has been changed
                                                      'patch all of rx routine out if not used                  
                        or      rxmask0,#0      wz
'        if_z            mov     rxcode,txcode         'use variable in case it has been changed
'
' Receive
'
receive0                jmpret  rxcode0,txcode0       'run a chunk of transmit code, then return
                                                      'patched to a jmp if pin not used                        
                        rdlong  bit_ticks0, bit_ticks_ptr0
                        mov     bit4_ticks0, bit_ticks0
                        shr     bit4_ticks0, #2
                        test    rxmask0,ina      wc
start0  if_c            jmp     #norts0               'go check rts if no start bit
                                                      'will be patched to jmp #receive if no rts  

                        mov     rxbits0,#9            'ready to receive byte
                        mov     rxcnt0,bit4_ticks0    '1/4 bits
                        add     rxcnt0,cnt

:bit0                   add     rxcnt0,bit_ticks0     '1 bit period
                        
:wait0                  jmpret  rxcode0,txcode0       'run a chuck of transmit code, then return

                        mov     t1,rxcnt0             'check if bit receive period done
                        sub     t1,cnt
                        cmps    t1,#0           wc
        if_nc           jmp     #:wait0

                        test    rxmask0,ina     wc    'receive bit on rx pin
                        rcr     rxdata0,#1
                        djnz    rxbits0,#:bit0        'get remaining bits

                        jmpret  rxcode0,txcode0       'run a chunk of transmit code, then return
                        
                        shr     rxdata0,#32-9         'justify and trim received byte

                        wrbyte  rxdata0,rxbuff_head_ptr0'{7-22}
                        add     rx_head0,#1
                        and     rx_head0,#$FF          '256 byte buffer
                        wrlong  rx_head0,rx_head_ptr0  '{8}
                        mov     rxbuff_head_ptr0,rxbuff_ptr0 'calculate next byte head_ptr
                        add     rxbuff_head_ptr0,rx_head0
norts0
                        rdlong  rx_tail0,rx_tail_ptr0 '{7-22 or 8} will be patched to jmp #r3 if no rts
                        mov     t1,rx_head0
                        sub     t1,rx_tail0           'calculate number bytes in buffer
                        and     t1,#$FF               'fix wrap
                        cmps    t1,rtssize0     wc    'is it more than the threshold
rts0                    muxc    outa,rtsmask0         'set rts correctly

rec0i                   jmp     #receive0             'byte done, receive next byte
'
' Receive1
'
receive1                jmpret  rxcode1,txcode1       'run a chunk of transmit code, then return
                        
                        rdlong  bit_ticks1, bit_ticks_ptr1
                        mov     bit4_ticks1, bit_ticks1
                        shr     bit4_ticks1, #2
                        test    rxmask1,ina     wc
start1  if_c            jmp     #norts1               'go check rts if no start bit

                        mov     rxbits1,#9            'ready to receive byte
                        mov     rxcnt1,bit4_ticks1    '1/4 bits
                        add     rxcnt1,cnt                          

:bit1                   add     rxcnt1,bit_ticks1     '1 bit period
                        
:wait1                  jmpret  rxcode1,txcode1       'run a chuck of transmit code, then return

                        mov     t1,rxcnt1             'check if bit receive period done
                        sub     t1,cnt
                        cmps    t1,#0           wc
        if_nc           jmp     #:wait1

                        test    rxmask1,ina     wc    'receive bit on rx pin
                        rcr     rxdata1,#1
                        djnz    rxbits1,#:bit1

                        jmpret  rxcode1,txcode1       'run a chunk of transmit code, then return
                        shr     rxdata1,#32-9         'justify and trim received byte

                        wrbyte  rxdata1,rxbuff_head_ptr1 '7-22
                        add     rx_head1,#1
                        and     rx_head1,#$FF         '256 byte buffers
                        wrlong  rx_head1,rx_head_ptr1
                        mov     rxbuff_head_ptr1,rxbuff_ptr1 'calculate next byte head_ptr
                        add     rxbuff_head_ptr1,rx_head1
norts1
                        rdlong  rx_tail1,rx_tail_ptr1    '7-22 or 8 will be patched to jmp #r3 if no rts
                        mov     t1,rx_head1
                        sub     t1,rx_tail1
                        and     t1,#$FF
                        cmps    t1,rtssize1     wc
rts1                    muxc    outa,rtsmask1

rec1i                   jmp     #receive1             'byte done, receive next byte
'
'
' Transmit
'
transmit0               jmpret  txcode0,rxcode1       'run a chunk of receive code, then return
                                                      'patched to a jmp if pin not used                        
                        
txcts0                  test    ctsmask0,ina    wc    'if flow-controlled dont send
                        rdlong  t1,tx_head_ptr0       '{7-22} - head[0]
                        cmp     t1,tx_tail0     wz    'tail[0]
ctsi0   if_z            jmp     #transmit0            'may be patched to if_z_or_c or if_z_or_nc

                        rdbyte  txdata0,txbuff_tail_ptr0'{8}
                        add     tx_tail0,#1
                        and     tx_tail0,#$0F    wz
                        wrlong  tx_tail0,tx_tail_ptr0  '{8}
        if_z            mov     txbuff_tail_ptr0,txbuff_ptr0 'reset tail_ptr if we wrapped
        if_nz           add     txbuff_tail_ptr0,#1    'otherwise add 1
                        
                        jmpret  txcode0,rxcode1

                        shl     txdata0,#2
                        or      txdata0,txbitor      'ready byte to transmit
                        mov     txbits0,#11
                        mov     txcnt0,cnt

txbit0                  shr     txdata0,#1       wc
txout0                  muxc    outa,txmask0          'maybe patched to muxnc dira,txmask
                        add     txcnt0,bit_ticks0     'ready next cnt

:wait                   jmpret  txcode0,rxcode1       'run a chunk of receive code, then return

                        mov     t1,txcnt0             'check if bit transmit period done
                        sub     t1,cnt
                        cmps    t1,#0           wc
        if_nc           jmp     #:wait

                        djnz    txbits0,#txbit0       'another bit to transmit?
txjmp0                  jmp     ctsi0                 'byte done, transmit next byte
'
' Transmit1
'
transmit1               jmpret  txcode1,rxcode0       'run a chunk of receive code, then return
                        
txcts1                  test    ctsmask1,ina    wc    'if flow-controlled dont send
                        rdlong  t1,tx_head_ptr1
                        cmp     t1,tx_tail1     wz
ctsi1   if_z            jmp     #transmit1            'may be patched to if_z_or_c or if_z_or_nc

                        rdbyte  txdata1,txbuff_tail_ptr1
                        add     tx_tail1,#1
                        and     tx_tail1,#$0F   wz
                        wrlong  tx_tail1,tx_tail_ptr1
        if_z            mov     txbuff_tail_ptr1,txbuff_ptr1 'reset tail_ptr if we wrapped
        if_nz           add     txbuff_tail_ptr1,#1   'otherwise add 1

                        jmpret  txcode1,rxcode0       'run a chunk of receive code, then return
                        
                        shl     txdata1,#2
                        or      txdata1,txbitor       'ready byte to transmit
                        mov     txbits1,#11
                        mov     txcnt1,cnt

txbit1                  shr     txdata1,#1      wc
txout1                  muxc    outa,txmask1          'maybe patched to muxnc dira,txmask
                        add     txcnt1,bit_ticks1     'ready next cnt

:wait1                  jmpret  txcode1,rxcode0       'run a chunk of receive code, then return

                        mov     t1,txcnt1             'check if bit transmit period done
                        sub     t1,cnt
                        cmps    t1,#0           wc
        if_nc           jmp     #:wait1

                        djnz    txbits1,#txbit1       'another bit to transmit?
txjmp1                  jmp     ctsi1                 'byte done, transmit next byte
'
'These are used by SPIN and assembler, using DAT rather than VAR
'so all COGs share this instance of the object
'
startfill
  cog                   long      0                   'cog flag/id

  'Dont Change the order of any of these initialized variables without modifying
  'the code to match - both spin and assembler
  'Dont make any of the initialized variables, uninitialized, only the initialized
  'variables are duplicated in hub memory
  rx_head0              long      0                   '16 longs for circular buffer head/tails
  rx_head1              long      0                           
  rx_tail0              long      0
  rx_tail1              long      0 
  tx_head0              long      0
  tx_head1              long      0
  tx_tail0              long      0
  tx_tail1              long      0

  'This set of variables were initialized to the correct values in Spin and loaded into this cog
  'when it started
  rxmask0               long      0                   '33 longs for per port info
  rxmask1               long      0
  txmask0               long      0
  txmask1               long      0
  ctsmask0              long      0
  ctsmask1              long      0
  rtsmask0              long      0
  rtsmask1              long      0
  rxtx_mode0            long      0
  rxtx_mode1            long      0
  bit4_ticks0           long      0
  bit4_ticks1           long      0
  bit_ticks0            long      0
  bit_ticks1            long      0
  rtssize0              long      0
  rtssize1              long      0
  rxchar0               byte      0
  rxchar1               byte      0

  'This set of variables were initialized to the correct values in Spin and loaded into this cog
  'when it started. They contain the hub memory address of the rx/txbuffers
  rxbuff_ptr0           long      0                   '32 longs for per port buffer hub ptr
  rxbuff_ptr1           long      0
  txbuff_ptr0           long      0
  txbuff_ptr1           long      0
  rxbuff_head_ptr0      long      0
  rxbuff_head_ptr1      long      0
  txbuff_tail_ptr0      long      0
  txbuff_tail_ptr1      long      0

  rx_head_ptr0          long      0
  rx_head_ptr1          long      0
  rx_tail_ptr0          long      0
  rx_tail_ptr1          long      0
  tx_head_ptr0          long      0
  tx_head_ptr1          long      0
  tx_tail_ptr0          long      0
  tx_tail_ptr1          long      0
endfill       'used to calculate size of variables for longfill with 0
'
'note we are overlaying cog variables on top of these hub arrays, we need the space
'
  rx_buffer0                                          'overlay rx variables over rxbuffer
  rxdata0               long      0                   'transmit and receive buffers
  rxbits0               long      0
  rxcnt0                long      0
  rxdata1               long      0
  rxbits1               long      0
  rxcnt1                long      0
  t1                    long      0
                        long      0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0

  rx_buffer1            long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0
                        long      0,0,0,0,0,0,0,0

  tx_buffer0                                          'overlay tx variables over txbuffer
  txdata0               long      0
  txbits0               long      0
  txcnt0                long      0
                        long      0

  tx_buffer1                                          'overlay tx variables over txbuffer
  txdata1               long      0
  txbits1               long      0
  txcnt1                long      0
                        long      0

  bit_ticks_ptr0        long      0-0
  bit_ticks_ptr1        long      0-0
'values to patch the code
  doifc2ifnc            long      $003c0000           'patch condition if_c to if_nc using xor
  doif_z_or_c           long      $00380000           'patch condition if_z to if_z_or_c using or
  doif_z_or_nc          long      $002c0000           'patch condition if_z to if_z_or_nc using or
  domuxnc               long      $04000000           'patch muxc to muxnc using or
'
  txbitor               long      $0401               'bits to or for transmitting

'If you want multiple copys of this driver in a project, then copy the file to multiple files and change
'version in each to be unique
  version               long      1
'
        FIT             $1F0
new_bit_ticks0          long      0-0
new_bit_ticks1          long      0-0
''      MIT LICENSE
{{
'  Permission is hereby granted, free of charge, to any person obtaining
'  a copy of this software and associated documentation files
'  (the "Software"), to deal in the Software without restriction,
'  including without limitation the rights to use, copy, modify, merge,
'  publish, distribute, sublicense, and/or sell copies of the Software,
'  and to permit persons to whom the Software is furnished to do so,
'  subject to the following conditions:
'
'  The above copyright notice and this permission notice shall be included
'  in all copies or substantial portions of the Software.
'
'  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
'  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
'  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
'  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
'  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
'  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
'  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
}}
