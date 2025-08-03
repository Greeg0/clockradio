from machine import Pin, SPI, Timer, RTC, PWM, I2C, ADC # SPI is a class associated with the machine library. 
import time
# The below specified libraries have to be included. Also, ssd1306.py must be saved on the Pico. 
from ssd1306 import SSD1306_SPI # this is the driver library and the corresponding class
import framebuf # this is another library for the display. 

import alarmfont # font used to display main clock.
import writer # Used to write custom font data. 

from encoder import Encoder

# Credit to CHATGPT for the bitmap arrays for the iconns.

crescent_moon = bytearray([
    0b00000000, 0b00000000,
    0b00000011, 0b11000000,
    0b00001111, 0b11100000,
    0b00011111, 0b11000000,
    0b00111110, 0b00000000,
    0b01111100, 0b00000000,
    0b01111000, 0b00000000,
    0b01110000, 0b00000000,
    0b01110000, 0b00000000,
    0b01111000, 0b00000000,
    0b01111100, 0b00000000,
    0b00111111, 0b00000000,
    0b00011111, 0b11000000,
    0b00001111, 0b11100000,
    0b00000011, 0b11000000,
    0b00000000, 0b00000000,
])

bell_icon = bytearray([
    0b00000000, 0b10000000,
    0b00000001, 0b11000000,
    0b00000000, 0b10000000,
    0b00000011, 0b11100000,
    0b00000111, 0b11110000,
    0b00001111, 0b11111000,
    0b00011111, 0b11111100,
    0b00111111, 0b11111110,
    0b00111111, 0b11111110,
    0b00111111, 0b11111110,
    0b00111111, 0b11111110,
    0b00111111, 0b11111110,
    0b00000000, 0b00000000,
    0b00000111, 0b11100000,
    0b00000111, 0b11100000,
    0b00000011, 0b11000000,
])


moon_fb = framebuf.FrameBuffer(crescent_moon, 16, 16, framebuf.MONO_HLSB)
bell_fb = framebuf.FrameBuffer(bell_icon, 16, 16, framebuf.MONO_HLSB)

# Define columns and rows of the oled display. These numbers are the standard values. 
SCREEN_WIDTH = 128 #number of columns
SCREEN_HEIGHT = 64 #number of rows


# Initialize I/O pins associated with the oled display SPI interface

spi_sck = Pin(2) # sck stands for serial clock; always be connected to SPI SCK pin of the Pico
spi_sda = Pin(3) # sda stands for serial data;  always be connected to SPI TX pin of the Pico; this is the MOSI
spi_res = Pin(4) # res stands for reset; to be connected to a free GPIO pin
spi_dc  = Pin(6) # dc stands for data/command; to be connected to a free GPIO pin
spi_cs  = Pin(5) # chip select; to be connected to the SPI chip select of the Pico 

#
# SPI Device ID can be 0 or 1. It must match the wiring. 
#
SPI_DEVICE = 0 # Because the peripheral is connected to SPI 0 hardware lines of the Pico

#
# initialize the SPI interface for the OLED display
#
oled_spi = SPI( SPI_DEVICE, baudrate= 100000, sck= spi_sck, mosi= spi_sda )

#
# Initialize the display
#
oled = SSD1306_SPI( SCREEN_WIDTH, SCREEN_HEIGHT, oled_spi, spi_dc, spi_res, spi_cs, True )
wri = writer.Writer(oled, alarmfont)

# radio class provided by professor
class Radio:
    
    def __init__( self, NewFrequency, NewVolume, NewMute ):

#
# set the initial values of the radio
#
        self.Volume = 5
        self.Frequency = 88
        self.Mute = False
#
# Update the values with the ones passed in the initialization code
#
        self.SetVolume( NewVolume )
        self.SetFrequency( NewFrequency )
        self.SetMute( NewMute )
        
      
# Initialize I/O pins associated with the radio's I2C interface

        self.i2c_sda = Pin(26)
        self.i2c_scl = Pin(27)

#
# I2C Device ID can be 0 or 1. It must match the wiring. 
#
# The radio is connected to device number 1 of the I2C device
#
        self.i2c_device = 1 
        self.i2c_device_address = 0x10

#
# Array used to configure the radio
#
        self.Settings = bytearray( 8 )


        self.radio_i2c = I2C( self.i2c_device, scl=self.i2c_scl, sda=self.i2c_sda, freq=200000)
        self.ProgramRadio()

    def SetVolume( self, NewVolume ):
#
# Conver t the string into a integer
#
        try:
            NewVolume = int( NewVolume )
            
        except:
            return( False )
        
#
# Validate the type and range check the volume
#
        if ( not isinstance( NewVolume, int )):
            return( False )
        
        if (( NewVolume < 0 ) or ( NewVolume >= 16 )):
            return( False )

        self.Volume = NewVolume
        return( True )



    def SetFrequency( self, NewFrequency ):
#
# Convert the string into a floating point value
#
        try:
            NewFrequency = float( NewFrequency )
            
        except:
            return( False )
#
# validate the type and range check the frequency
#
        if ( not ( isinstance( NewFrequency, float ))):
            return( False )
 
        if (( NewFrequency < 88.0 ) or ( NewFrequency > 108.0 )):
            return( False )

        self.Frequency = NewFrequency
        return( True )
        
    def SetMute( self, NewMute ):
        
        try:
            self.Mute = bool( int( NewMute ))
            
        except:
            return( False )
        
        return( True )

#
# convert the frequency to 10 bit value for the radio chip
#
    def ComputeChannelSetting( self, Frequency ):
        Frequency = int( Frequency * 10 ) - 870
        
        ByteCode = bytearray( 2 )
#
# split the 10 bits into 2 bytes
#
        ByteCode[0] = ( Frequency >> 2 ) & 0xFF
        ByteCode[1] = (( Frequency & 0x03 ) << 6 ) & 0xC0
        return( ByteCode )

#
# Configure the settings array with the mute, frequency and volume settings
#
    def UpdateSettings( self ):
        
        if ( self.Mute ):
            self.Settings[0] = 0x80
        else:
            self.Settings[0] = 0xC0
  
        self.Settings[1] = 0x09 | 0x04
        self.Settings[2:3] = self.ComputeChannelSetting( self.Frequency )
        self.Settings[3] = self.Settings[3] | 0x10
        self.Settings[4] = 0x04
        self.Settings[5] = 0x00
        self.Settings[6] = 0x84
        self.Settings[7] = (0x80 | (self.Volume & 0x0F))

#        
# Update the settings array and transmitt it to the radio
#
    def ProgramRadio( self ):

        self.UpdateSettings()
        self.radio_i2c.writeto( self.i2c_device_address, self.Settings )

#
# Extract the settings from the radio registers
#
    def GetSettings( self ):
#        
# Need to read the entire register space. This is allow access to the mute and volume settings
# After and address of 255 the 
#
        self.RadioStatus = self.radio_i2c.readfrom( self.i2c_device_address, 256 )

        if (( self.RadioStatus[0xF0] & 0x40 ) != 0x00 ):
            MuteStatus = False
        else:
            MuteStatus = True
            
        VolumeStatus = self.RadioStatus[0xF7] & 0x0F
 
 #
 # Convert the frequency 10 bit count into actual frequency in Mhz
 #
        FrequencyStatus = (( self.RadioStatus[0x00] & 0x03 ) << 8 ) | ( self.RadioStatus[0x01] & 0xFF )
        FrequencyStatus = ( FrequencyStatus * 0.1 ) + 87.0
        
        if (( self.RadioStatus[0x00] & 0x04 ) != 0x00 ):
            StereoStatus = True
        else:
            StereoStatus = False
        
        return( MuteStatus, VolumeStatus, FrequencyStatus, StereoStatus )

radio = Radio( 100.3, 2, True )
radio_frequency = 1003
radio_volume = 5
radio_update = False # Flag to reprogram the radio.
radio_triggered = True
volume_dial = ADC(Pin(28))
radio_on = False

def read_potentiometer_level(adc, samples=20):
    global last_pot_level
    
    total = 0
    for _ in range(samples):
        total += adc.read_u16()
    average = total // samples
    level = (average * 15) // 65000  # Scale to 0-15
    level = 15 - level
    
    return level

class DebouncedButton:
    def __init__(self, button, delay):
        self.button = button
        self.state = False # assume button state is false
        self.last_press_time = 0
        self.delay = delay
        
    def value(self): # Delay is number of milliseconds to delay for debouncing.
        
        button_input = not self.button.value() # read once right away, for stable processing. Not as the value is backwards for pull up.
        current_time = time.ticks_ms()
        
        if (button_input != self.state): # this means button state has changed.
            
            if (time.ticks_diff(current_time, self.last_press_time) >= self.delay): # this is run on a succesful state change.
                self.state = button_input
                self.last_press_time = current_time
                if (button_input == True): # We only care about the value as a result of the press.
                    return True
        else:
            self.last_press_time = current_time
            
        return False
    
    def wait(self): # we use this to wait for a button to be pressed.
        button_input = not self.button.value() # read button value.
        if (button_input == True):
            self.state = True # We must update our state to true.
            self.last_press_time = time.ticks_ms() # update the last time pressed to properly debounce the .value() function.
            
        return not button_input # returns True if button is not pressed, returns false for when button is pressed. ONLY TO BE USED IN WHILE LOOPS.

button1 = Pin( 15, Pin.IN, Pin.PULL_UP ) # encoder button
button2 = Pin( 7, Pin.IN, Pin.PULL_UP ) # auxillary button

button_encoder = DebouncedButton(button1, 40) 
button_other   = DebouncedButton(button2, 40)

# Initialize the encoder
encoder1 = Encoder(13, 14) # Pin A, Pin B

# Declare our settings:
hrfrmt = True # False = 24 hour, True = 12 hour          12/24 hr time
alarm_mode = 0                                  #        0=off, 1=on/beep, 2=on/radio
display_mode = False                            #        0=on, 1=delay mode
snooze_time = 10                                # The duration of snooze, in minutes.
snooze = False # used to check if snooze is enabled or disabled.
snooze_minutes = 0 # stores time in minutes to trigger next snooze.
alarm_not_triggered = True
flash_bell = False
# Initialize the time keeping:
rtc = machine.RTC()

class Clock:
    def __init__(self, hour, minute):
        self.hour = hour
        self.minute = minute
        self.state = 3 # default component to flash.
        self.hour_offset = 0
        self.minute_offset = 0
        
    def update_hour(self, up):
        self.hour = (self.hour + up) % 24
    def update_minute(self, up):
        self.minute = (self.minute + up) % 60
    def sync(self):
        self.hour   = (rtc.datetime()[4]+self.hour_offset ) % 24
        self.minute = (rtc.datetime()[5]+self.minute_offset) % 60
    def sync_hour(self, up):                      # sync_hour and sync_minute are used to offset the RTC within the adjustment system.
        self.hour_offset = (self.hour_offset + up) % 24
        
    def sync_minute(self, up):
        self.minute_offset = (self.minute_offset + up) % 60
 
    def GenerateString(self, flash): # 0 to flash hour, 1 to flash minute, 2 to flash colon.
        
        if (hrfrmt == True):
            hour = (self.hour % 12) or 12 # hour becomes 12 hr version of the 24 hr time of clock.hour 
        else:
            hour = self.hour
                
        if (self.state == 0) and flash:
            hour_string = "  "
        else:
            hour_string = f"{hour}"
            if (hour < 10):
                hour_string = " " + hour_string # we prepend a space to keep stuff aligned.
            
        if (self.state == 1) and flash:
            minute_string = "  "
        else:
            minute_string = f"{self.minute}"
            if (self.minute < 10):
                minute_string = "0" + minute_string
            
        if (self.state == 3) and flash:
            colon = (";")  # I disabled the semicolon in the font, so it's used as a colon-sized space. 
        elif (self.state == 4) and flash:
            colon = " "
        else:
            colon = ":"
        
        return hour_string + colon + minute_string
    
    def String12Hr(self):
        if (self.hour >= 12):
            string_val = "PM"
        else:
            string_val = "AM"
        return string_val
        

# We create two clocks, 1 main, 1 alarm.
main_clock = Clock(rtc.datetime()[4],rtc.datetime()[5]) # set_clock is used to reconfigure RTC time.
alarm_clock = Clock(0, 0)

def CheckAlarm():
    global alarm_not_triggered
    global snooze_minutes
    
    if not snooze: # if not on the snooze mode, we check regular alarm time.
        if (alarm_clock.hour == main_clock.hour) and (alarm_clock.minute == main_clock.minute):
            returnVal = alarm_not_triggered # returnval = False if alarm is already triggered, True if alarm is not already triggered.
            alarm_not_triggered = False
            return returnVal
        else:
            alarm_not_triggered = True
            
    else: # if snooze mode is on, we check the difference between main clock minutes and our snooze_minutes time.
        if (snooze_minutes == main_clock.minute):
            returnVal = alarm_not_triggered # false if alarm is already triggered, true if alarm is not triggered.
            alarm_not_triggered = False
            return returnVal
        else:
            alarm_not_triggered = True
    return False

def UpdateDisplay(clock) : # updatedisplay was modified so that alarm clock can share the same function for displaying.
    global update
    update = False 
    
    oled.fill(0)
    
    wri.set_textpos(oled, 16, 16)
    
    wri.printstring(clock.GenerateString(EN_flash))
        
    if (hrfrmt == True):
        oled.text(clock.String12Hr(), 110, 34)
        
    if (alarm_mode != 0) and (EN_flash or not flash_bell):
        oled.blit(bell_fb, 0, 0)
        
    if (snooze == True):
        oled.blit(moon_fb, 0, 48)
        
    oled.show()

# Configure Timer for flashing display. 
update = True
EN_flash = False
pause_flash = False

def __flash(timer) :
    global EN_flash
    global update
    
    if not pause_flash: 
        EN_flash = not EN_flash
    else:
        EN_flash = False
    update = True

flashtimer = Timer()
flashtimer.init(period=500, callback = __flash)
            
menustr = ["OFF", "ON/TONE", "ON/RDIO", "ON", "DELAY"]

def UpdateRadio():
    global EN_flash
    global update
    
    update = False
    
    oled.fill(0)
    
    wri.set_textpos(oled, 16, 16)
    wri.printstring(f"{radio_frequency / 10}")
    oled.text("MHz", 102, 34)
    oled.text(f"Vol:{radio_volume}", 80, 54)
    oled.text(main_clock.GenerateString(EN_flash), 0, 54)
    if (hrfrmt):
        oled.text(main_clock.String12Hr(), 44, 54)
        
    oled.show()

def UpdateMenu(selection):
    global EN_flash
    global alarm_mode
    global display_mode
    global hrfrmt
    global snooze_time
    global update
    
    update = False
    
    oled.fill(0)
    
    if (EN_flash):
        if (selection < 4):
            oled.text("        >", 0, selection * 11)
            flash_alarm = False
            flash_clock = False
        else:
            flash_alarm = selection in (4, 5)
            flash_clock = selection in (6, 7)
    else:
        flash_alarm = False
        flash_clock = False    
    
    oled.text(f"ALARM    {menustr[alarm_mode]}", 0, 0)
    oled.text(f"DISPLAY  {menustr[display_mode+3]}", 0, 11) #Display mode offset.
    oled.text(f"SNOOZE   {snooze_time}", 0, 22)
    oled.text("            MINS", 0, 22)
    oled.text(f"TIME IN  {24-12*hrfrmt}HR", 0, 33)
    oled.text(f"ALARM    {alarm_clock.GenerateString(flash_alarm)}", 0, 44)
    oled.text(f"CLOCK    {main_clock.GenerateString(flash_clock)}", 0, 55)
    
    if (hrfrmt):
        oled.text(f"              {alarm_clock.String12Hr()}", 0, 44)
        oled.text(f"              {main_clock.String12Hr()}",0, 55)
        
   
            
    oled.show()

#PWM SETUP
pwm = PWM(16) # pin 16
pwm.freq(950) # we want a 1000Hz tone.

while ( True ):
    
    if (update == True):
        main_clock.sync() # sync main clock to real time.
        UpdateDisplay(main_clock) # update display. Update occurs every 500ms in sync with flashing colon.
        
    if (display_mode == True) and (alarm_not_triggered):
        # When display mode is true, clock goes in a sleep mode after 10 seconds.
        if (time.ticks_diff(time.ticks_ms(), last_input_time) > 10000):
            oled.write_cmd(0xAE) # turn off OLED
            while(button_encoder.wait() and button_other.wait()):
                if (update == True):
                    main_clock.sync() # sync the main clock.
                
                if (alarm_mode != 0) and CheckAlarm(): # break out of sleep if alarm is triggered.
                    alarm_not_triggered = True # CheckAlarm() updates this to false if it detects, but we update it to True so that the main code triggers it too.
                    break
            oled.write_cmd(0xAF) #wake up again
            last_input_time = time.ticks_ms()
    
    if (alarm_mode != 0) and CheckAlarm():
        flash_bell = True
        while ( True ):
            if (update == True):
                main_clock.sync()
                UpdateDisplay(main_clock)
            
            if (alarm_mode == 1): # run the PWM alarm sound:
                if (EN_flash):
                    pwm.duty_u16(8192)
                else:
                    pwm.duty_u16(0)
            else: # can only be mode 2 at this point, and it plays the radio in alarm mode (no tuning).
                if not (radio_on):
                    radio.SetMute(False)
                    radio.SetVolume(14)
                    radio.ProgramRadio()
                    radio_on = True
                
            if (button_other.value()):
                snooze = True
                snooze_minutes = (snooze_time + main_clock.minute) % 60 # set the minutes duration for next snooze time.
                pwm.duty_u16(0) # ensure this is shut off
                flash_bell = False
                break
            elif (button_encoder.value()):
                snooze = False # force snooze off
                pwm.duty_u16(0) # ensure it is shut off.
                flash_bell = False
                break
        if (radio_on): # disable radio if radio was turned on for alarm.
            radio_on = False
            radio.SetMute(True)
            radio.SetVolume(radio_volume)
            radio.ProgramRadio()
            
        last_input_time = time.ticks_ms()
                
    # Radio menu:
    if(button_other.value()):
        radio_update = True
        update = True
        radio.SetMute(False)
        radio_volume = read_potentiometer_level(volume_dial)
        radio.SetVolume(radio_volume)
        radio_on = True
        main_clock.state = 4
        last_input_time = time.ticks_ms() - 2000
        while( True ):
                
            if (update == True): # update clock.
                main_clock.sync()
                UpdateRadio()
                
            if (radio_update == True): # update and program the radio.
                radio_update = False
                
                radio.ProgramRadio()
                UpdateRadio()
                
            if(button_other.value()):
                break
            
            vol_input = read_potentiometer_level(volume_dial)
            if (vol_input != radio_volume): #dial in the volume.
                last_input_time = time.ticks_ms()
                radio_volume = vol_input
                radio.SetVolume(radio_volume)
                radio_triggered = False
                update = True
                  
            elif not radio_triggered and (time.ticks_diff(time.ticks_ms(), last_input_time)) >= 2000:
                radio_triggered = True
                radio_update = True
            
            
            
            if (encoder1.update()): # tune radio.
                
                radio_frequency += encoder1.direction * 2 # add multiples of 2 to the number.
                
                if (radio_frequency < 880):
                    radio_frequency = 881
                    
                elif (radio_frequency > 1080):
                    radio_frequency = 1079
                    
                else:
                    last_input_time = time.ticks_ms()
                    update = True
                    radio_triggered = False
                    radio.SetFrequency(radio_frequency / 10)
                    
        radio.SetMute(True)
        update = True
        radio_on = False
        main_clock.state = 3
        radio.ProgramRadio()
        last_input_time = time.ticks_ms()
    
    # Settings Menu:
    if (button_encoder.value()): # enter menu mode
        last_input_time = time.ticks_ms() - 1000 # Important.
        update = True
        select = 0
        while(button_other.wait()):

            if (update == True):
                UpdateMenu(select)
                main_clock.sync()
            
            if (button_encoder.value()):
                
                select = ( select + 1 ) % 8
                
                alarm_clock.state = (select == 5) # these ensure their display states are updated on switch.
                main_clock.state = (select == 7)
            
            if (encoder1.update()): # if the encoder updates, we can increment or decrement our current selected option.
                
                if (select == 0): #receive encoder update
                    alarm_mode = ( alarm_mode + encoder1.direction ) % 3 # cycle the alarm mode.
                    if (alarm_mode == 0): # reset snooze if alarm is turned off in settings. Also a way to reset an unintended snooze.
                        snooze = False
                        snooze_minutes = 0 
                    
                elif (select == 1):
                    display_mode = not display_mode # we can flip the boolean options without encoder direction.
                    
                elif (select == 2):
                    snooze_time = 1 + (snooze_time - 1 + encoder1.direction) % 60  # loop from 1 to 60
                
                elif (select == 3):
                    hrfrmt = not hrfrmt
                    
                else: # at this point, we enter clock adjustment mode.
                    pause_flash = True
                    EN_flash = False
                    last_input_time = time.ticks_ms()
                    
                    if (select == 4):
                        alarm_clock.update_hour(encoder1.direction)
                        
                    elif (select == 5):
                        alarm_clock.update_minute(encoder1.direction)
                        
                    elif (select == 6):
                        main_clock.sync_hour(encoder1.direction)
                        
                        
                    else: # select == 7
                        main_clock.sync_minute(encoder1.direction)
                        
                    
                update = True # push an update to the display on next cycle.

            if pause_flash and time.ticks_diff(time.ticks_ms(), last_input_time) > 1000: # If flash is paused, unpause it after a second of updating time.
                pause_flash = False
                    
        main_clock.state = 3 # reset state to 3 for main clock.
        pause_flash = False # unpause flash for necessary cases.
        update = True # update the menu after exiting.
        last_input_time = time.ticks_ms()
    # End of settings menu code