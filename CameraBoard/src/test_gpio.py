import Jetson.GPIO as GPIO
import time

def main():
    camera_capturnig_pin = 11
    lsb_memory_monitor = 13
    msb_memory_monitor = 15
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(camera_capturnig_pin, GPIO.IN, GPIO.PUD_UP)
    GPIO.setup(lsb_memory_monitor, GPIO.IN, GPIO.PUD_UP)
    GPIO.setup(msb_memory_monitor, GPIO.IN, GPIO.PUD_UP)
    try:
        while True:
            capturing_state = "ACTIVE" if not GPIO.input(camera_capturnig_pin) else "DOWN"
            lsb_memory_state = "ACTIVE" if not GPIO.input(lsb_memory_monitor) else "DOWN"
            msb_memory_state = "ACTIVE" if not GPIO.input(msb_memory_monitor) else "DOWN"
            print("Camera: %s, MSB: %s, LSB %s" % (capturing_state, msb_memory_state, lsb_memory_state))
            time.sleep(1.0)
            
    except Exception as e:
        raise e
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[bentayga/src/test_gpio.py] ERROR: ")
        print(e)

    finally:
        GPIO.cleanup()
