import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Vector3
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise

update_rate = 50

# UKF parameters
dim_x = 6  # State dimensions [x, y, z, vx, vy, vz]
dim_z = 3  # Measurement dimensions [x, y, z]
dt = 1.0 / update_rate

# Initialize UKF
sigmas = MerweScaledSigmaPoints(dim_x, alpha=0.1, beta=2., kappa=1.)
ukf = UKF(dim_x=dim_x, dim_z=dim_z, dt=dt, hx=lambda x: x[:3], fx=f_cv, points=sigmas)
ukf.x = np.zeros(dim_x)
ukf.P = np.eye(dim_x) * 0.1
ukf.R = np.eye(dim_z) * 0.1  # Measurement noise
ukf.Q = Q_discrete_white_noise(dim=dim_x, dt=dt, var=0.1)  # Process noise

current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def Pose_cb(msg):
    global current_pose
    current_pose = msg

def distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 )

def correct_quaternion_to_rotation_matrix(q):
    """ Convert a quaternion into a rotation matrix """
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
        [2*qx*qy + 2*qw*qz, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qw*qx],
        [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx**2 - 2*qy**2]
    ])

def imu_cb(data):
    quaternion = [
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    ]
    
    rotation_matrix = correct_quaternion_to_rotation_matrix(quaternion)
    
    linear_acceleration_body = np.array([
        data.linear_acceleration.x,
        data.linear_acceleration.y,
        data.linear_acceleration.z
    ])
    
    # Transform linear acceleration to world frame
    linear_acceleration_world = np.dot(rotation_matrix, linear_acceleration_body)
    
    # Subtract gravity to get relative motion
    gravity = np.array([0, 0, 9.81])
    linear_acceleration_motion = linear_acceleration_world - gravity
    
    # Perform UKF prediction step
    ukf.predict(dt=dt, accel=linear_acceleration_motion[:2])
    
    acceleration_msg = Vector3()
    acceleration_msg.x = linear_acceleration_motion[0]
    acceleration_msg.y = linear_acceleration_motion[1]
    acceleration_msg.z = linear_acceleration_motion[2]
    rospy.loginfo("Publishing acceleration: %s", acceleration_msg)
    acceleration_pub.publish(acceleration_msg)

def apriltag_cb(data):
    # Extract AprilTag position from the message
    apriltag_position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    
    # Perform UKF update step
    ukf.update(apriltag_position)
    
    # Publish the estimated position
    estimated_position = Vector3()
    estimated_position.x = ukf.x[0]
    estimated_position.y = ukf.x[1]
    estimated_position.z = ukf.x[2]
    position_pub.publish(estimated_position)

if __name__ == "__main__":
    rospy.init_node("task2_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", Pose_cb)
    rospy.Subscriber('/mavros/imu/data', Imu, imu_cb)
    rospy.Subscriber('/apriltag/pose', PoseStamped, apriltag_cb)  # Assuming AprilTag pose topic

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    acceleration_pub = rospy.Publisher('/world_acceleration', Vector3, queue_size=10)
    velocity_pub = rospy.Publisher('/world_velocity', Vector3, queue_size=10)
    position_pub = rospy.Publisher('/world_position', Vector3, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    rate = rospy.Rate(update_rate)
    # Define waypoints
    def Waypoint(x, y, z):
        set_point = PoseStamped()
        set_point.pose.position.x = x
        set_point.pose.position.y = y
        set_point.pose.position.z = z
        return set_point

    waypoints = [
        Waypoint(5, 0, 1),
        Waypoint(10.5, 0, 1),
        Waypoint(10.5, 3, 1),
        Waypoint(5, 3, 1),
        Waypoint(1, 3, 1),
        Waypoint(1, 6, 1),
        Waypoint(6, 6, 1),
        Waypoint(12, 6, 1),
    ]

    waypoint_index = 0

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1

    for _ in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        if distance(current_pose.pose.position, waypoints[waypoint_index].pose.position) < 2:
            waypoint_index += 1
            if waypoint_index >= len(waypoints):
                waypoint_index -= 1

        # Set pose to the current waypoint
        pose.pose.position.x = waypoints[waypoint_index].pose.position.x
        pose.pose.position.y = waypoints[waypoint_index].pose.position.y
        pose.pose.position.z = waypoints[waypoint_index].pose.position.z
        local_pos_pub.publish(pose)
        
        rate.sleep()
