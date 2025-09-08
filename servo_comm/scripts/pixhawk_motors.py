import rospy, time
from stair_detection.msg import StairInfo, Attitude
from dronekit import connect, VehicleMode

happen_once = False
lidar_distance = 9999 
MOVE_FORWARD = 1710
STOP = 1500

def distance_callback(msg):
    global lidar_distance
    lidar_distance = msg.distance

def main():
    global lidar_distance, happen_once

    rospy.init_node('vehicle_control_node', anonymous=True)
    rospy.Subscriber('/log_info', StairInfo, distance_callback)

    # Publisher for attitude
    attitude_pub = rospy.Publisher('/pixhawk_attitude', Attitude, queue_size=10)

    vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
    rospy.loginfo(f"Vehicle mode: {vehicle.mode.name}")

    rate = rospy.Rate(10)

    

    is_descending = False

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

        rospy.loginfo(f"Attitude -> Pitch: {pitch_deg:.2f}°, Lidar Distance: {lidar_distance:.2f} m, Happen once: {happen_once}")

        
        if lidar_distance > 900 and 50.0 > pitch_deg >= 28.0 and not happen_once:
            rospy.loginfo("Climbing, stopping")
            current_override = STOP
            vehicle.channels.overrides = {'1': current_override}

            start_time = time.time()
            while (time.time() - start_time) < 6:
                if rospy.is_shutdown():
                    break

                att = vehicle.attitude
                roll_deg = att.roll * 57.2958
                pitch_deg = att.pitch * 57.2958
                yaw_deg = att.yaw * 57.2958

                msg = Attitude()
                msg.roll = roll_deg
                msg.pitch = pitch_deg
                msg.yaw = yaw_deg
                attitude_pub.publish(msg)

                rate.sleep()

            current_override = MOVE_FORWARD
            vehicle.channels.overrides = {'1': current_override}
            # is_descending = True

        elif lidar_distance < 0.33 and -2.0 < pitch_deg < 10.0:
            current_override = STOP
            vehicle.channels.overrides = {'1': current_override}
            rospy.loginfo("Stopping vehicle")

        elif lidar_distance > 900 and 15.0 > pitch_deg >= 12.0:
            current_override = MOVE_FORWARD
            vehicle.channels.overrides = {'1': current_override}
            rospy.loginfo("Move forward, robot is climbing")

        else:
            current_override = MOVE_FORWARD
            vehicle.channels.overrides = {'1': current_override}
            rospy.loginfo("Neutral, move forward")

        # if current_override != last_override:
        #     vehicle.channels.overrides = {'1': current_override}
        #     rospy.loginfo(f"Override changed to: {current_override}")
        #     last_override = current_override

        rate.sleep()


if __name__ == '__main__':
    main()

