# from dronekit import connect, VehicleMode
# import time

# vehicle = connect('/dev/ttyACM0', baud=57600,wait_ready=True)
# print(vehicle)
# print(vehicle.mode)

# #while True:
# #    print(f"Channel 1 PWM Output: {vehicle.channels['1']}")
# #    print(f"Channel 2 PWM Output: {vehicle.channels['2']}")
# #    time.sleep(0.5)

# while True:

#     ch1 = int(input("Steering (CH1): ").strip())

# #if lidar_distance < 300 and toggle is on,

#     vehicle.channels.overrides = {'1':ch1}

# #else RC


import rospy
from stair_detection.msg import StairInfo
from dronekit import connect, VehicleMode

lidar_distance = 9999  # Default large value

def distance_callback(msg):
    global lidar_distance
    lidar_distance = msg.distance

def main():
    global lidar_distance

    rospy.init_node('vehicle_control_node', anonymous=True)
    rospy.Subscriber('/stair_info', StairInfo, distance_callback)

    vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
    print(vehicle.mode)

    # toggle = True  # set your condition here

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print(f"Current Distance: {lidar_distance:.2f}")

        att = vehicle.attitude
        roll = att.roll   
        pitch = att.pitch
        yaw = att.yaw

        # Convert to degrees for readability (optional)
        roll_deg = roll * 57.2958
        pitch_deg = pitch * 57.2958
        yaw_deg = yaw * 57.2958

        print(f"Attitude -> Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")


        if lidar_distance < 0.350:
            vehicle.channels.overrides = {'1': 1500}     
        else:
            vehicle.channels.overrides = {'1': 1700}

        rate.sleep()

if __name__ == '__main__':
    # log_pub = rospy.Publisher('/log_detected', Bool, queue_size=1)
    main()
