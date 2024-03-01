import smbus
import time
import datetime as dt
from ds1307_jetson import *

"""
class RtcTime:
    def __init__(self, hours: int, minutes: int, seconds:int):
        self.hours = 0
        self.minutes = 0
        self.seconds = 0
        if 24 > hours and hours >= 0: self.hours = hours
        else: raise ValueError("Invalid hour value")
        if 60 > minutes and minutes >= 0: self.minutes = minutes
        else: raise ValueError("Invalid minutes value")
        if 60 > seconds and seconds >= 0: self.seconds = seconds
        else: raise ValueError("Invalid seconds value")

    def __add__(self, other_time):
        self.seconds += other_time.seconds
        if self.seconds > 59: 
            self.seconds -= 60
            self.minutes += 1

        self.minutes += other_time.minutes
        if self.minutes > 59: 
            self.minutes -= 60
            self.hours += 1

        self.hours += other_time.hours
        if self.hours > 23: 
            self.hours -= 24
        
        return self

    def __sub__(self, other_time):
        self.seconds -= other_time.seconds
        if self.seconds < 0: 
            self.seconds += 60
            self.minutes -= 1

        self.minutes -= other_time.minutes
        if self.minutes < 0: 
            self.minutes += 60
            self.hours -= 1

        self.hours -= other_time.hours
        if self.hours < 0: 
            self.hours += 24
        
        return self

    def __str__(self):
        return f"{self.hours}:{self.minutes}:{self.seconds}"
"""
if __name__ == "__main__":
    
    rtc = DS1307_JN(0)
    print(rtc.read_complete_date(), " ", rtc.read_time())
    time.sleep(10)
    print(rtc.read_complete_date(), " ", rtc.read_time())
    """
    current_date = dt.datetime.fromtimestamp(time.time())
    rtc.set_complete_date(current_date)

    print(rtc.read_complete_date(), " ", rtc.read_time())
    time.sleep(10)
    print(rtc.read_complete_date(), " ", rtc.read_time())
    inicio = RtcTime(12, 30, 00)
    final = RtcTime(13, 00, 00)
    elapsed_time = final - inicio
    print(elapsed_time)
    """
