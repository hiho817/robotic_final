#! /usr/bin/env python3

import rospy
from mavros_msgs.msg import State, Trajectory, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def create_position_target(point):
    pos_target = PositionTarget()
    pos_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    pos_target.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                           + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                           + PositionTarget.IGNORE_YAW_RATE
    pos_target.position.x = point[0]
    pos_target.position.y = point[1]
    pos_target.position.z = 1
    return pos_target

if __name__ == "__main__":
    rospy.init_node("min_snap_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_setpoint_trajectory_pub = rospy.Publisher("mavros/setpoint_trajectory/desire", Trajectory, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(50)  # Set rate to 50 Hz

    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Waiting for connection...")
        rate.sleep()

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    last_req = rospy.Time.now()

    path = [
        [1, 0],
        [2, 0],
        [3, 0],
        [4, 0],
        [5, 0]
    ]

    while not rospy.is_shutdown():
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        traj = Trajectory()
        traj.header.stamp = rospy.Time.now()
        traj.type = Trajectory.MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS

        # Assign the 5 points
        traj.point_1 = create_position_target(path[0])
        traj.point_2 = create_position_target(path[1])
        traj.point_3 = create_position_target(path[2])
        traj.point_4 = create_position_target(path[3])
        traj.point_5 = create_position_target(path[4])

        local_setpoint_trajectory_pub.publish(traj)
        rospy.loginfo("Published trajectory: %s", traj)

        rate.sleep()  # Ensure the message is sent at the desired rate
