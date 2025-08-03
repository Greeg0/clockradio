# Encoder module.
#
# A slightly rewritten version of the encoder code provided by the course, to support multiple encoders with classes, and slightly improved code efficiency.

# Use self.update to get the flag that an update occured.
# Use self.direction to get the direction detected on update. 

from machine import Pin

class Encoder:
    def __init__(self, pinA, pinB):
        self.up = False
        self.direction = 1 # 1 for cw, -1 for ccw.
        self.__EncoderState__ = 0
        Pin( pinA, Pin.IN).irq( handler=self. __EncoderAInterrupt__, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, hard=True )
        Pin( pinB, Pin.IN).irq( handler= self.__EncoderBInterrupt__, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, hard=True )
        
    def update(self):
        if (self.up == True):
            self.up = False
            return True
        return False
    
    def __EncoderAInterrupt__(self, Pin ): # For pin A
        if (self.__EncoderState__ == 0) and (Pin.irq().flags() == 4):
            self.__EncoderState__ = 1
            
        elif (self.__EncoderState__ == 2) and (Pin.irq().flags() == 8):
            self.__EncoderState__ = 3
            
        elif (self.__EncoderState__ == 4) and (Pin.irq().flags() == 4):
            self.__EncoderState__ = 5
            
        elif (self.__EncoderState__ == 6) and (Pin.irq().flags() == 8): # State where CCW is activated.
            self.direction = -1
            self.up = True
            self.__EncoderState__ = 0
            
        else:
            self.__EncoderState__ = 0
            
        return(True)
        
    def __EncoderBInterrupt__(self, Pin ): # For pin B
        if (self.__EncoderState__ == 1) and (Pin.irq().flags() == 4):
            self.__EncoderState__ = 2
            
        elif (self.__EncoderState__ == 3) and (Pin.irq().flags() == 8): # This is the state where CW is activated.
            self.__EncoderState__ = 0 
            self.direction = 1
            self.up = True
            
        elif (self.__EncoderState__ == 0) and (Pin.irq().flags() == 4):
            self.__EncoderState__ = 4
            
        elif (self.__EncoderState__ == 5) and (Pin.irq().flags() == 8):
            self.__EncoderState__ = 6
            
        else:
            self.__EncoderState__ = 0
            
        return(True)

