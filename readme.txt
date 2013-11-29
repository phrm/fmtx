##############################
#                            #
# short-range FM-transmitter #
#                            #
##############################


#########
#       #
# parts #
#       #
#########

Si4710/Si4711/Si4712/Si4713/Si4720/Si4721 (Silicon Laboratories)
http://www.silabs.com/Support%20Documents/TechnicalDocs/Si4710-11-B30.pdf
http://www.silabs.com/Support%20Documents/TechnicalDocs/Si4712-13-B30.pdf
http://www.silabs.com/Support%20Documents/TechnicalDocs/Si4720-21-B20.pdf
http://www.silabs.com/Support%20Documents/TechnicalDocs/AN332.pdf
http://www.silabs.com/Support%20Documents/TechnicalDocs/AN383.pdf

ATtiny45V/ATtiny85V (Atmel)
http://www.atmel.com/images/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf

TPS61070DDCR (Texas Instruments)
http://www.ti.com/lit/ds/symlink/tps61070.pdf

0.12uH	LQW18ANR12J00D (Murata Electronics)	0603
4.7uH	744031004 (Wurth Electronics)

4.7u	x6	0805
.39u	x2	0805
100n	x2	0805
22p	x1	0805

0R	x1      0805	
4K7	x3      0805	
10K	x1      0805	
180K	x1      0805	
1M8	x1      0805	


#########
#       #
# fuses #
#       #
#########

http://www.engbedded.com/fusecalc

L = 0xA2	CKOUT SUT0 CKSEL3 CKSEL2 CKSEL0 
H = 0xDF	SPIEN
E = 0xFF

avrdude -c usbtiny -p t85 -V -U flash:w:firmware.hex:i -U hfuse:w:0xdf:m -U lfuse:w:0xa2:m -U efuse:w:0xff:m


###########
#         #
# arduino #
#         #
###########

http://code.google.com/p/arduino-tiny/downloads/detail?name=tiny-core-2-0100-0002.zip&can=2&q=
https://github.com/adafruit/TinyWireM


####### ARDUINO/hardware/arduino/boards.txt #######

attiny85at8.name=ATtiny85 @ 8 MHz

attiny85at8.upload.using=usbtiny
attiny85at8.upload.maximum_size=8192

attiny85at8.build.mcu=attiny85
attiny85at8.build.f_cpu=8000000L
attiny85at8.build.core=tiny

###################################################
