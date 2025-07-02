from dronekit import connect, VehicleMode
import time

vehicle = connect('/dev/ttyACM0', baud=57600,wait_ready=True)
print(vehicle)
print(vehicle.mode)

#while True:
#    print(f"Channel 1 PWM Output: {vehicle.channels['1']}")
#    print(f"Channel 2 PWM Output: {vehicle.channels['2']}")
#    time.sleep(0.5)

while True:

    ch1 = int(input("Steering (CH1): ").strip())
    vehicle.channels.overrides = {'1':ch1}



