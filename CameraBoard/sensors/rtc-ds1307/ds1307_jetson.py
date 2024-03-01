import smbus
from typing import Tuple
import datetime

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
        return "%02d:%02d:%02d" % (self.hours, self.minutes, self.seconds)

address = 0x68
number_to_day_dict = {
        "1": "Monday",
        "2": "Tuesday",
        "3": "Wednesday",
        "4": "Thursday",
        "5": "Friday",
        "6": "Saturday",
        "7": "Sunday"
        }

day_to_number_dict = {
        "Monday": 1,
        "Tuesday": 2,
        "Wednesday": 3,
        "Thursday": 4,
        "Friday": 5,
        "Saturday": 6,
        "Sunday": 7
        }

class DS1307_JN:
    def __init__(self, jetson_i2cid: int):
        self.i2cbus = None
        self._timer_started = None
        self.hour_format = None
        self.hour_period = None
        try:
            self.i2cbus = smbus.SMBus(jetson_i2cid)
        except Exception as e:
            raise e

    def stop_rtc(self):
        start_byte = self.i2cbus.read_byte_data(address, 0)
        start_byte = start_byte | 0b10000000
        self.i2cbus.write_byte_data(address, 0, start_byte)
        self._timer_started = False

    def start_rtc(self):
        start_byte = self.i2cbus.read_byte_data(address, 0)
        start_byte = start_byte & 0b01111111
        self.i2cbus.write_byte_data(address, 0, start_byte)
        self._timer_started = True
    
    def read_seconds(self) -> int:
        seconds_register = self.i2cbus.read_byte_data(address, 0)
        seconds_value = ((seconds_register & 0b01110000) >> 4)*10 + (seconds_register & 0b00001111)
        return seconds_value

    def read_minutes(self) -> int:
        minutes_register = self.i2cbus.read_byte_data(address, 1)
        minutes_value = ((minutes_register & 0b01110000) >> 4)*10 + (minutes_register & 0b00001111)
        return minutes_value

    def read_hours(self) -> int:
        hours_register = self.i2cbus.read_byte_data(address, 2)
        format_flag = (hours_register & 0b01000000) >> 6
        hours_value = 0
        if format_flag == 0:
            hours_value = ((hours_register & 0b00110000) >> 4)*10 + (hours_register & 0b00001111)
            self.hour_format = "24H"
        elif format_flag == 1:
            hours_value = ((hours_register & 0b00010000) >> 4)*10 + (hours_register & 0b00001111)
            self.hours_period = "AM" if ((hours_register & 0b00100000) >> 5) == 0 else "PM"
            self.hour_format = "12H"
            
        return hours_value
    
    def read_day_of_week(self) -> str:
        day_register = self.i2cbus.read_byte_data(address, 3)
        try:
            return  number_to_day_dict[str((day_register & 0b00000111))]
        except:
            raise KeyError("Couldn't read correctly the day of the week")
    
    def read_date(self) -> int:
        date_register = self.i2cbus.read_byte_data(address, 4)
        date_value = ((date_register & 0b00110000) >> 4)*10 + (date_register & 0b00001111)
        return date_value

    def read_month(self) -> int:
        month_register = self.i2cbus.read_byte_data(address, 5)
        month_value = ((month_register & 0b00010000) >> 4)*10 + (month_register & 0b00001111)
        return month_value

    def read_year(self) -> int:
        year_register = self.i2cbus.read_byte_data(address, 6)
        year_value = ((year_register & 0b11110000) >> 4)*10 + (year_register & 0b00001111)
        return 2000 + year_value

    def read_time_object(self) -> RtcTime:
        seconds = self.read_seconds()
        minutes = self.read_minutes()
        hours = self.read_hours()
        if self.hour_format == "12H" and self.hour_period == "PM": 
            hours += 12
        return RtcTime(hours, minutes, seconds)

    def read_time(self) -> str:
        time = ""
        seconds = self.read_seconds()
        minutes = self.read_minutes()
        hours = self.read_hours()
        if self.hour_format == "24H": time = "%02d:%02d:%02d" % (hours, minutes, seconds)
        elif self.hour_format == "12H": time = ("%02d:%02d:%02d" % (hours, minutes, seconds)) + f" {self.hour_period}"

        return time

    def read_time_for_folder(self) -> str:
        return self.read_time().replace(":", "_")

    def read_complete_date(self) -> str:
        date = self.read_date()
        month = self.read_month()
        year = self.read_year()
        return ("%4d-%02d-%02d" % (year, month, date))

    def read_date_and_time(self) -> str:
        return "%s - %s" % (self.read_complete_date(), self.read_time())

    def set_seconds(self, value: int):
        dozens = int(value/10)
        units = int(value%10)
        seconds_value = ((dozens << 4) | units) & 0x7F
        seconds_register = self.i2cbus.read_byte_data(address, 0)
        seconds_register = (seconds_register & 0x80) | seconds_value
        self.i2cbus.write_byte_data(address, 0, seconds_register)

    def set_minutes(self, value: int):
        dozens = int(value/10)
        units = int(value%10)
        minutes_value = ((dozens << 4) | units) & 0x7F
        minutes_register =  0x00 | minutes_value
        self.i2cbus.write_byte_data(address, 1, minutes_register)

    def set_hours(self, value: int, hours_type: str):
        """
        Sets the hours value in the RTC.
        params:
        value       -- number that represents the hour.
        hour_type   -- 24H if is in 24 hour format, AM/PM if it is in
                       12H format
        """
        hours_value = ((int(value/10) << 4) | int(value%10)) & 0x7F
        if hours_type == "24H":
            hours_register = 0x00 | (hours_value & 0x3F)

        elif hours_type == "AM" or hours_type == "PM":
            hours_register = 0x40 | (hours_value & 0x3F)
            if hours_type == "PM": hours_register | 0x40

        self.i2cbus.write_byte_data(address, 2, hours_register)
    
    def set_day_of_week(self, value: str):
        try:
            day_id = day_to_number_dict[value]
        except:
            raise KeyError(f"Couldn't set correctly the day of the week, invalid name {value}")
        self.i2cbus.write_byte_data(address, 3, day_id)

    def set_day(self, value: int):
        dozens = int(value/10)
        units = int(value%10)
        day_value = ((dozens << 4) | units) & 0x3F
        day_register = 0x00 | day_value
        self.i2cbus.write_byte_data(address, 4, day_register)

    def set_month(self, value: int):
        dozens = int(value/10)
        units = int(value%10)
        month_value = ((dozens << 4) | units) & 0x1F
        month_register = 0x00 | month_value
        self.i2cbus.write_byte_data(address, 5, month_register)

    def set_year(self, value: int):
        value -= 2000
        dozens = int(value/10)
        units = int(value%10)
        year_register = ((dozens << 4) | units) & 0xFF
        self.i2cbus.write_byte_data(address, 6, year_register)
    
    def set_complete_date(self, current_datetime: datetime.datetime):
        self.set_seconds(int(current_datetime.second))
        self.set_minutes(current_datetime.minute)
        self.set_hours(current_datetime.hour, "24H")
        self.set_day(current_datetime.day)
        self.set_month(current_datetime.month)
        self.set_year(current_datetime.year)
