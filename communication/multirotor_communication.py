#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State
from pyquaternion import Quaternion
from std_msgs.msg import String, Header
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
velocity_pub = TwistStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_construct(x=0, y=0, z=0):
    target_pose = PoseStamped()
    target_pose.header = header
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    return target_pose

def velocity_construct(vx=0.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
    target_velocity = TwistStamped()
    target_velocity.header = header
    target_velocity.twist.linear.x = vx
    target_velocity.twist.linear.y = vy
    target_velocity.twist.linear.z = vz
    target_velocity.twist.angular.x = wx
    target_velocity.twist.angular.y = wy
    target_velocity.twist.angular.z = wz
    return target_velocity

def q2yaw(q):
    #判断是否为四元数
    #获得yaw
    if isinstance(q, Quaternion):
        rotate_z_rad = q.yaw_pitch_roll[0]
    else:
        q_ = Quaternion(q.w, q.x, q.y, q.z)
        rotate_z_rad = q_.yaw_pitch_roll[0]

    return rotate_z_rad

def local_pose_callback(msg):
    global current_position
    global current_yaw
    current_position = msg.pose.position
    current_yaw = q2yaw(msg.pose.orientation)

def cmd_callback(msg):
    return msg

def cmd_vel_flu_callback(msg):
    global velocity_pub
    velocity_pub = velocity_construct(vx=msg.twist.linear.x, vy=msg.twist.linear.y,vz=msg.twist.linear.z, wz=msg.twist.angular.z)

if __name__ == "__main__":
    rospy.init_node("iris_mavros")

    #ros subscriber
    local_pose_sub = rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped, callback = local_pose_callback, queue_size=1)
    state_sub = rospy.Subscriber("/iris_0/mavros/state", State, callback = state_cb)
    cmd_sub = rospy.Subscriber("/xtdrone/iris_0/cmd",String,callback = cmd_callback, queue_size=1)
    cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/iris_0/cmd_vel_flu", TwistStamped, callback = cmd_vel_flu_callback, queue_size=1)

    #ros publisher
    local_pose_pub = rospy.Publisher("/iris_0/mavros/setpoint_position/local", PoseStamped, queue_size=1)
    local_velocity_pub = rospy.Publisher("/iris_0/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)

    #ros service
    rospy.wait_for_service("/iris_0/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/iris_0/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/iris_0/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/iris_0/mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(40)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    header = Header()
    header.frame_id = "base_link"
    #header.stamp = rospy.Time.now()

    construct_target_pub = pose_construct(x=0,y=0,z=0.6)

    #construct_velocity_pub = velocity_construct(vx=0,vy=0,vz=0.1,wx=0,wy=0,wz=0.1)

    #Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pose_pub.publish(construct_target_pub)
        local_velocity_pub.publish(velocity_pub)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    while not rospy.is_shutdown():
        if(current_state.mode == "OFFBOARD"):
            if((rospy.Time.now() - last_req) < rospy.Duration(12)):
                local_pose_pub.publish(construct_target_pub)
            else:
                local_velocity_pub.publish(velocity_pub)
                print(velocity_pub)
            
        rate.sleep()