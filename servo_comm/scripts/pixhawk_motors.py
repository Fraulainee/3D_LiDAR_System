import rospy, time
from stair_detection.msg import StairInfo, Attitude
from dronekit import connect, VehicleMode

happen_once = False
lidar_distance = 9999  # Default large value
MOVE_FORWARD = 1700
STOP = 1500

def distance_callback(msg):
    global lidar_distance
    lidar_distance = msg.distance

def main():
    global lidar_distance

    rospy.init_node('vehicle_control_node', anonymous=True)
    rospy.Subscriber('/stair_info', StairInfo, distance_callback)

    # Publisher for attitude
    attitude_pub = rospy.Publisher('/pixhawk_attitude', Attitude, queue_size=10)

    vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
    rospy.loginfo(f"Vehicle mode: {vehicle.mode.name}")

    rate = rospy.Rate(10)

    

    last_override = None

    while not rospy.is_shutdown():
        att = vehicle.attitude
        roll = att.roll
        pitch = att.pitch
        yaw = att.yaw

        # Convert to degrees (optional)
        roll_deg = roll * 57.2958
        pitch_deg = pitch * 57.2958
        yaw_deg = yaw * 57.2958

        msg = Attitude()
        msg.roll = roll_deg
        msg.pitch = pitch_deg
        msg.yaw = yaw_deg
        attitude_pub.publish(msg)

        rospy.loginfo(f"Attitude -> Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

        if lidar_distance < 0.350 and -0.5 < pitch < 10.0:
            current_override = STOP
            rospy.loginfo("Stopping vehicle")
        elif lidar_distance > 900 and pitch > 10.0:
            current_override = MOVE_FORWARD
            rospy.loginfo("Forward again")
        elif lidar_distance > 900 and pitch >= 37.0 and not happen_once:
            time.sleep(14)
            happen_once = True

            current_override = MOVE_FORWARD
            rospy.loginfo("Climbing, stopping")
        else:
            current_override = MOVE_FORWARD
            rospy.loginfo("Neutral, move forward")

        if current_override != last_override:
            vehicle.channels.overrides = {'1': current_override}
            rospy.loginfo(f"Override changed to: {current_override}")
            last_override = current_override

        rate.sleep()


if __name__ == '__main__':
    main()

