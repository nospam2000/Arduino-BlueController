# Makefile for ATmegaBOOT
# E.Lins, 18.7.2005
# $Id$
#
# Instructions
#
# To make bootloader .hex file:
# make diecimila
# make lilypad
# make ng
# etc...
#
# To burn bootloader .hex file:
# make diecimila_isp
# make lilypad_isp
# make ng_isp
# etc...

# program name should not be changed...
PROGRAM    = optiboot

# Uncomment this next lines to build within the arduino environment,
# using the arduino-included avrgcc toolset (mac and pc)
# ENV ?= arduino
# OS ?= macosx
# OS ?= windows


# enter the parameters for the avrdude isp tool
ISPTOOL	   = stk500v2
ISPPORT	   = usb
ISPSPEED   = -b 115200

MCU_TARGET = atmega168
LDSECTION  = --section-start=.text=0x3e00

ifeq ($(ENV), arduino)
# For Arduino, we assume that we're connected to the optiboot directory
# included with the arduino distribution, which means that the full set
# of avr-tools are "right up there" in standard places.
#TOOLROOT = ../../../tools
GCCROOT = $(TOOLROOT)/avr/bin/
AVRDUDE_CONF = -C$(TOOLROOT)/avr/etc/avrdude.conf
else
GCCROOT =
AVRDUDE_CONF =
endif

ifeq ($(OS), windows)
# On windows, SOME of the tool paths will need to have backslashes instead
# of forward slashes (because they use windows cmd.exe for execution instead
# of a unix/mingw shell?)
fixpath = $(subst /,\,$1)
else
fixpath = $(1)
endif

# the efuse should really be 0xf8; since, however, only the lower
# three bits of that byte are used on the atmega168, avrdude gets
# confused if you specify 1's for the higher bits, see:
# http://tinker.it/now/2007/02/24/the-tale-of-avrdude-atmega168-and-extended-bits-fuses/
#
# similarly, the lock bits should be 0xff instead of 0x3f (to
# unlock the bootloader section) and 0xcf instead of 0x0f (to
# lock it), but since the high two bits of the lock byte are
# unused, avrdude would get confused.

ISPFUSES    = $(GCCROOT)avrdude $(AVRDUDE_CONF) -c $(ISPTOOL) \
              -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
              -e -u -U lock:w:0x3f:m -U efuse:w:0x$(EFUSE):m \
              -U hfuse:w:0x$(HFUSE):m -U lfuse:w:0x$(LFUSE):m
ISPFLASH    = $(GCCROOT)avrdude $(AVRDUDE_CONF) -c $(ISPTOOL) \
              -p $(MCU_TARGET) -P $(ISPPORT) $(ISPSPEED) \
              -U flash:w:$(PROGRAM)_$(TARGET).hex -U lock:w:0x0f:m

STK500 = "C:\Program Files\Atmel\AVR Tools\STK500\Stk500.exe"
STK500-1 = $(STK500) -e -d$(MCU_TARGET) -pf -vf -if$(PROGRAM)_$(TARGET).hex \
-lFF -LFF -f$(HFUSE)$(LFUSE) -EF8 -ms -q -cUSB -I200kHz -s -wt
STK500-2 = $(STK500) -d$(MCU_TARGET) -ms -q -lCF -LCF -cUSB -I200kHz -s -wt

OBJ        = $(PROGRAM).o
OPTIMIZE = -Os -fno-inline-small-functions -fno-split-wide-types -mshort-calls

DEFS       = 
LIBS       =

CC         = $(GCCROOT)avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) $(DEFS)
override LDFLAGS       = -Wl,$(LDSECTION) -Wl,--relax -nostartfiles -Wl,--gc-sections -nostdlib

OBJCOPY        = $(GCCROOT)avr-objcopy
OBJDUMP        = $(call fixpath,$(GCCROOT)avr-objdump)

# Test platforms
# Virtual boot block test
virboot328: TARGET = atmega328
virboot328: MCU_TARGET = atmega328p
virboot328: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200' '-DVIRTUAL_BOOT'
virboot328: AVR_FREQ = 16000000L
virboot328: LDSECTION  = --section-start=.text=0x7e00
virboot328: $(PROGRAM)_atmega328.hex
virboot328: $(PROGRAM)_atmega328.lst

# 20MHz clocked platforms
#
# These are capable of 230400 baud, or 115200 baud on PC (Arduino Avrdude issue)
#

pro20: TARGET = pro_20mhz
pro20: MCU_TARGET = atmega168
pro20: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
pro20: AVR_FREQ = 20000000L
pro20: $(PROGRAM)_pro_20mhz.hex
pro20: $(PROGRAM)_pro_20mhz.lst
pro20_isp: pro20
pro20_isp: TARGET = pro_20mhz
# 2.7V brownout
pro20_isp: HFUSE = DD
# Full swing xtal (20MHz) 258CK/14CK+4.1ms
pro20_isp: LFUSE = C6
# 512 byte boot
pro20_isp: EFUSE = 04
pro20_isp: isp

# 16MHz clocked platforms
#
# These are capable of 230400 baud, or 115200 baud on PC (Arduino Avrdude issue)
#

pro16: TARGET = pro_16MHz
pro16: MCU_TARGET = atmega168
pro16: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
pro16: AVR_FREQ = 16000000L
pro16: $(PROGRAM)_pro_16MHz.hex
pro16: $(PROGRAM)_pro_16MHz.lst
pro16_isp: pro16
pro16_isp: TARGET = pro_16MHz
# 2.7V brownout
pro16_isp: HFUSE = DD
# Full swing xtal (20MHz) 258CK/14CK+4.1ms
pro16_isp: LFUSE = C6
# 512 byte boot
pro16_isp: EFUSE = 04
pro16_isp: isp

# Diecimila and NG use identical bootloaders
diecimila: TARGET = diecimila
diecimila: MCU_TARGET = atmega168
diecimila: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
diecimila: AVR_FREQ = 16000000L 
diecimila: $(PROGRAM)_diecimila.hex
diecimila: $(PROGRAM)_diecimila.lst
diecimila_isp: diecimila
diecimila_isp: TARGET = diecimila
# 2.7V brownout
diecimila_isp: HFUSE = DD
# Low power xtal (16MHz) 16KCK/14CK+65ms
diecimila_isp: LFUSE = FF
# 512 byte boot
diecimila_isp: EFUSE = 04
diecimila_isp: isp

atmega328: TARGET = atmega328
atmega328: MCU_TARGET = atmega328p
atmega328: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
atmega328: AVR_FREQ = 16000000L
atmega328: LDSECTION  = --section-start=.text=0x7e00
atmega328: $(PROGRAM)_atmega328.hex
atmega328: $(PROGRAM)_atmega328.lst
atmega328_isp: atmega328
atmega328_isp: TARGET = atmega328
atmega328_isp: MCU_TARGET = atmega328p
# 512 byte boot
atmega328_isp: HFUSE = DE
# Low power xtal (16MHz) 16KCK/14CK+65ms
atmega328_isp: LFUSE = FF
# 2.7V brownout
atmega328_isp: EFUSE = 05
atmega328_isp: isp

# Sanguino has a minimum boot size of 1024 bytes, so enable extra functions
sanguino: TARGET = atmega644p
sanguino: MCU_TARGET = atmega644p
sanguino: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200' '-DBIGBOOT'
sanguino: AVR_FREQ = 16000000L
sanguino: LDSECTION  = --section-start=.text=0xfc00
sanguino: $(PROGRAM)_atmega644p.hex
sanguino: $(PROGRAM)_atmega644p.lst
sanguino_isp: sanguino
sanguino_isp: TARGET = atmega644p
sanguino_isp: MCU_TARGET = atmega644p
# 1024 byte boot
sanguino_isp: HFUSE = DE
# Low power xtal (16MHz) 16KCK/14CK+65ms
sanguino_isp: LFUSE = FF
# 2.7V brownout
sanguino_isp: EFUSE = 05
sanguino_isp: isp

# Mega has a minimum boot size of 1024 bytes, so enable extra functions
mega: TARGET = atmega1280
mega: MCU_TARGET = atmega1280
mega: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200' '-DBIGBOOT'
mega: AVR_FREQ = 16000000L
mega: LDSECTION  = --section-start=.text=0x1fc00
mega: $(PROGRAM)_atmega1280.hex
mega: $(PROGRAM)_atmega1280.lst
mega_isp: mega
mega_isp: TARGET = atmega1280
mega_isp: MCU_TARGET = atmega1280
# 1024 byte boot
mega_isp: HFUSE = DE
# Low power xtal (16MHz) 16KCK/14CK+65ms
mega_isp: LFUSE = FF
# 2.7V brownout
mega_isp: EFUSE = 05
mega_isp: isp

# ATmega8
#
atmega8: TARGET = atmega8
atmega8: MCU_TARGET = atmega8
atmega8: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
atmega8: AVR_FREQ = 16000000L 
atmega8: LDSECTION  = --section-start=.text=0x1e00
atmega8: $(PROGRAM)_atmega8.hex
atmega8: $(PROGRAM)_atmega8.lst
atmega8_isp: atmega8
atmega8_isp: TARGET = atmega8
atmega8_isp: MCU_TARGET = atmega8
# 2.7V brownout
atmega8_isp: HFUSE = DC
# Low power xtal (16MHz) 16KCK/14CK+65ms
atmega8_isp: LFUSE = BF
atmega8_isp: isp

# ATmega88
atmega88: TARGET = atmega88
atmega88: MCU_TARGET = atmega88
atmega88: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
atmega88: AVR_FREQ = 16000000L 
atmega88: $(PROGRAM)_atmega88.hex
atmega88: $(PROGRAM)_atmega88.lst
atmega88_isp: atmega88
atmega88_isp: TARGET = atmega88
atmega88_isp: MCU_TARGET = atmega88
# 2.7V brownout
atmega88_isp: HFUSE = DD
# Low power xtal (16MHz) 16KCK/14CK+65ms
atemga88_isp: LFUSE = FF
# 512 byte boot
atmega88_isp: EFUSE = 04
atmega88_isp: isp

# 8MHz clocked platforms
#
# These are capable of 115200 baud
#

lilypad: TARGET = lilypad
lilypad: MCU_TARGET = atmega168
lilypad: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
lilypad: AVR_FREQ = 8000000L
lilypad: $(PROGRAM)_lilypad.hex
lilypad: $(PROGRAM)_lilypad.lst
lilypad_isp: lilypad
lilypad_isp: TARGET = lilypad
# 2.7V brownout
lilypad_isp: HFUSE = DD
# Internal 8MHz osc (8MHz) Slow rising power
lilypad_isp: LFUSE = E2
# 512 byte boot
lilypad_isp: EFUSE = 02
lilypad_isp: isp

lilypad_resonator: TARGET = lilypad_resonator
lilypad_resonator: MCU_TARGET = atmega168
lilypad_resonator: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
lilypad_resonator: AVR_FREQ = 8000000L
lilypad_resonator: $(PROGRAM)_lilypad_resonator.hex
lilypad_resonator: $(PROGRAM)_lilypad_resonator.lst
lilypad_resonator_isp: lilypad_resonator
lilypad_resonator_isp: TARGET = lilypad_resonator
# 2.7V brownout
lilypad_resonator_isp: HFUSE = DD
# Full swing xtal (20MHz) 258CK/14CK+4.1ms
lilypad_resonator_isp: LFUSE = C6
# 512 byte boot
lilypad_resonator_isp: EFUSE = 02
lilypad_resonator_isp: isp

pro8: TARGET = pro_8MHz
pro8: MCU_TARGET = atmega168
pro8: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
pro8: AVR_FREQ = 8000000L
pro8: $(PROGRAM)_pro_8MHz.hex
pro8: $(PROGRAM)_pro_8MHz.lst
pro8_isp: pro8
pro8_isp: TARGET = pro_8MHz
# 2.7V brownout
pro8_isp: HFUSE = DD
# Full swing xtal (20MHz) 258CK/14CK+4.1ms
pro8_isp: LFUSE = C6
# 512 byte boot
pro8_isp: EFUSE = 02
pro8_isp: isp

atmega328_pro8: TARGET = atmega328_pro_8MHz
atmega328_pro8: MCU_TARGET = atmega328p
atmega328_pro8: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
atmega328_pro8: AVR_FREQ = 8000000L
atmega328_pro8: LDSECTION  = --section-start=.text=0x7e00
atmega328_pro8: $(PROGRAM)_atmega328_pro_8MHz.hex
atmega328_pro8: $(PROGRAM)_atmega328_pro_8MHz.lst
atmega328_pro8_isp: atmega328_pro8
atmega328_pro8_isp: TARGET = atmega328_pro_8MHz
atmega328_pro8_isp: MCU_TARGET = atmega328p
# 512 byte boot
atmega328_pro8_isp: HFUSE = DE
# Low power xtal (16MHz) 16KCK/14CK+65ms
atmega328_pro8_isp: LFUSE = FF
# 2.7V brownout
atmega328_pro8_isp: EFUSE = 05
atmega328_pro8_isp: isp


bluec88_115200: TARGET = bluec88_8MHz_115200bps
bluec88_115200: MCU_TARGET = atmega88
# bluec88_115200: CFLAGS += '-DBAUD_RATE=115200' '-DBLUECONTROLLER' '-DNRWWSECTIONFAVORSIZE'
bluec88_115200: CFLAGS += '-DBAUD_RATE=115200' '-DBLUECONTROLLER'
bluec88_115200: AVR_FREQ = 8000000L
bluec88_115200: LDSECTION  = --section-start=.text=0x1e00
bluec88_115200: $(PROGRAM)_bluec88_8MHz_115200bps.hex
bluec88_115200: $(PROGRAM)_bluec88_8MHz_115200bps.lst
bluec88_115200_isp: bluec88_115200
bluec88_115200_isp: TARGET = bluec88_8MHz_115200bps
bluec88_115200_isp: HFUSE = D6 # BOD=1,8V (BODLEVEL0, EESAVE, SPIEN)
bluec88_115200_isp: LFUSE = E2 # internal osc, 8MHz (SUT0, CKSEL:0,2,3)
bluec88_115200_isp: EFUSE = FC # 512 byte bootloader (BOOTSZ0, BOOTRST) 
bluec88_115200_isp: isp


bluec88_19200: TARGET = bluec88_8MHz_19200bps
bluec88_19200: MCU_TARGET = atmega88
# bluec88_19200: CFLAGS += '-DBAUD_RATE=19200' '-DBLUECONTROLLER' '-DNRWWSECTIONFAVORSIZE'
bluec88_19200: CFLAGS += '-DBAUD_RATE=19200' '-DBLUECONTROLLER'
bluec88_19200: AVR_FREQ = 8000000L
bluec88_19200: LDSECTION  = --section-start=.text=0x1e00
bluec88_19200: $(PROGRAM)_bluec88_8MHz_19200bps.hex
bluec88_19200: $(PROGRAM)_bluec88_8MHz_19200bps.lst
bluec88_19200_isp: bluec88_19200
bluec88_19200_isp: TARGET = bluec88_8MHz_19200bps
bluec88_19200_isp: HFUSE = D6 # BOD=1,8V (BODLEVEL0, EESAVE, SPIEN)
bluec88_19200_isp: LFUSE = E2 # internal osc, 8MHz (SUT0, CKSEL:0,2,3)
bluec88_19200_isp: EFUSE = FC # 512 byte bootloader (BOOTSZ0, BOOTRST) 
bluec88_19200_isp: isp


bluec328_115200: TARGET = bluec328_8MHz_115200bps
bluec328_115200: MCU_TARGET = atmega328p
#bluec328_115200: CFLAGS += '-DBAUD_RATE=115200' '-DBLUECONTROLLER' '-DNRWWSECTIONFAVORSIZE'
bluec328_115200: CFLAGS += '-DBAUD_RATE=115200' '-DBLUECONTROLLER'
#bluec328_115200: CFLAGS += '-DOSCCAL_VALUE=148' # the value 148 is for my "BlueControllerX" board
#bluec328_115200: CFLAGS += '-DOSCCAL_VALUE=139' # the value 139 is for my "BlueController RoboTX" board
#bluec328_115200: CFLAGS += '-DOSCCAL_VALUE=74' # the value 74 is for my "BlueController" board
bluec328_115200: AVR_FREQ = 8000000L
bluec328_115200: LDSECTION  = --section-start=.text=0x7e00
bluec328_115200: $(PROGRAM)_bluec328_8MHz_115200bps.hex
bluec328_115200: $(PROGRAM)_bluec328_8MHz_115200bps.lst
bluec328_115200_isp: bluec328_115200
bluec328_115200_isp: TARGET = bluec328_8MHz_115200bps
bluec328_115200_isp: HFUSE = D6 # 512 byte bootloader (SPIEN, EESAVE, BOOTRST)
bluec328_115200_isp: LFUSE = E2 # internal osc, 8MHz (SUT0, CKSEL:0,2,3)
bluec328_115200_isp: EFUSE = FE # BOD=1,8V (BODLEVEL0)
bluec328_115200_isp: isp


bluec328_19200: TARGET = bluec328_8MHz_19200bps
bluec328_19200: MCU_TARGET = atmega328p
#bluec328_19200: CFLAGS += '-DBAUD_RATE=115200' '-DBLUECONTROLLER' '-DNRWWSECTIONFAVORSIZE'
bluec328_19200: CFLAGS += '-DBAUD_RATE=19200' '-DBLUECONTROLLER'
bluec328_19200: AVR_FREQ = 8000000L
bluec328_19200: LDSECTION  = --section-start=.text=0x7e00
bluec328_19200: $(PROGRAM)_bluec328_8MHz_19200bps.hex
bluec328_19200: $(PROGRAM)_bluec328_8MHz_19200bps.lst
bluec328_19200_isp: bluec328_19200
bluec328_19200_isp: TARGET = bluec328_8MHz_19200bps
bluec328_19200_isp: HFUSE = D6 # 512 byte bootloader (SPIEN, EESAVE, BOOTRST)
bluec328_19200_isp: LFUSE = E2 # internal osc, 8MHz (SUT0, CKSEL:0,2,3)
bluec328_19200_isp: EFUSE = FE # BOD=1,8V (BODLEVEL0)
bluec328_19200_isp: isp

# experimental
arduinobt328_115200: TARGET = arduinobt328_115200
arduinobt328_115200: MCU_TARGET = atmega328p
#arduinobt328_115200: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
arduinobt328_115200: CFLAGS += '-DBAUD_RATE=115200' '-DARDUINO_BT' '-DBLUECONTROLLER'
arduinobt328_115200: AVR_FREQ = 16000000L 
arduinobt328_115200: $(PROGRAM)_arduinobt328_115200bps.hex
arduinobt328_115200: $(PROGRAM)_arduinobt328_115200bps.lst
arduinobt328_115200_isp: arduinobt328_115200
arduinobt328_115200_isp: TARGET = arduinobt328_115200
# 512 byte boot
arduinobt328_115200_isp: HFUSE = DE
# Low power xtal (16MHz) 16KCK/14CK+65ms
arduinobt328_115200_isp: LFUSE = FF
# 2.7V brownout
arduinobt328_115200_isp: EFUSE = 05
arduinobt328_115200_isp: isp

arduinobt168: TARGET = arduinobt168
arduinobt168: MCU_TARGET = atmega168
#arduinobt168: CFLAGS += '-DLED_START_FLASHES=3' '-DBAUD_RATE=115200'
arduinobt168: CFLAGS += '-DBAUD_RATE=115200' '-DARDUINO_BT' '-DBLUECONTROLLER'
arduinobt168: AVR_FREQ = 16000000L 
arduinobt168: $(PROGRAM)_arduinobt168.hex
arduinobt168: $(PROGRAM)_arduinobt168.lst
arduinobt168_isp: arduinobt168
arduinobt168_isp: TARGET = arduinobt168
# 2.7V brownout
arduinobt168_isp: HFUSE = D5
# Low power xtal (16MHz) 16KCK/14CK+65ms
arduinobt168_isp: LFUSE = FF
# 512 byte boot
arduinobt168_isp: EFUSE = 04
arduinobt168_isp: isp

# 1MHz clocked platforms
#
# These are capable of 9600 baud
#

luminet: TARGET = luminet
luminet: MCU_TARGET = attiny84
luminet: CFLAGS += '-DLED_START_FLASHES=3' '-DSOFT_UART' '-DBAUD_RATE=9600'
luminet: CFLAGS += '-DVIRTUAL_BOOT_PARTITION'
luminet: AVR_FREQ = 1000000L
luminet: LDSECTION  = --section-start=.text=0x1d00
luminet: $(PROGRAM)_luminet.hex
luminet: $(PROGRAM)_luminet.lst
luminet_isp: luminet
luminet_isp: TARGET = luminet
luminet_isp: MCU_TARGET = attiny84
# Brownout disabled
luminet_isp: HFUSE = DF
# 1MHz internal oscillator, slowly rising power
luminet_isp: LFUSE = 62
# Self-programming enable
luminet_isp: EFUSE = FE
luminet_isp: isp

#
# Generic build instructions
#
#

isp: $(TARGET)
	$(ISPFUSES)
	$(ISPFLASH)

isp-stk500: $(PROGRAM)_$(TARGET).hex
	$(STK500-1)
	$(STK500-2)

%.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
