import rospy
import random
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from std_msgs.msg import String,Header
from sensor_msgs.msg import Image
from pyquaternion import Quaternion
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist,TwistStamped,PoseStamped
import sys 
sys.path.append("/home/robot/firmware/catkin_ws/devel/lib/python3/dist-packages") 

class PIDController():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0.0
        self.last_error = 0.0
    def update(self, target, current, dt):
        error  = target-current
        self.error_sum += error * dt
        error_diff = (error - self.last_error)/dt
        #error_diff = (error - self.last_error)
        output = self.kp * error +self.ki * self.error_sum + self.kd * error_diff
        self.last_error = error
        return output

# 饱和函数（saturation function）
def sat(x,u):
    if x < -u:
        return -u
    elif x > u:
        return u
    else:
        return x 

# 双曲正切函数（tanh function）
def tanh(x):
    return np.tanh(x)

class AccelerateController():
    def __init__(self, tao1, tao2, Et,Ua=0.0):
        self.tao1 = tao1
        self.tao2 = tao2
        self.Et = Et
        self.Sv = 0
        self.Su = 0
        self.Ua = Ua

    def update_X(self,U_,dt,VL,PL):
        #delta_Ua = -self.Et*(self.Sv - VL) + self.Su
        #U = -1*self.tao2*(tanh(VL + self.tao1*PL)+tanh(VL)) - sat(delta_Ua,U_)
        U = -1*self.tao2*(tanh(VL + self.tao1*PL)+tanh(VL))
        acc = U
        #acc = U + self.Ua
        V_out = VL + acc*dt
        #更新Sv和Su
        Sv_1 = self.Sv - self.Et*(self.Sv - VL)*dt
        Su_1 = self.Su - self.Et*(self.Su + U)*dt
        self.Sv = Sv_1
        self.Su = Su_1
        return V_out
    def update_Y(self,U_,dt,VL,PL):
        delta_Ua = -1*self.Et*(self.Sv - VL) + self.Su
        U =-self.tao2*(tanh(VL + self.tao1*PL)+tanh(VL)) - sat(delta_Ua,U_)
        acc = U
        V_out = VL + acc*dt
        #更新Sv和Su
        Sv_1 = self.Sv - self.Et*(self.Sv - VL)*dt
        Su_1 = self.Su - self.Et*(self.Su + U - 9.8)*dt
        self.Sv = Sv_1
        self.Su = Su_1
        return V_out
    def update_Z(self,U_,dt,VL,PL):
        delta_Ua = -1*self.Et*(self.Sv - VL) + self.Su
        U = 9.8 - self.tao2*(tanh(VL + self.tao1*PL)+tanh(VL)) - sat(delta_Ua,U_)
        acc = U - 9.8
        V_out = VL + acc*dt
        #更新Sv和Su
        Sv_1 = self.Sv - self.Et*(self.Sv - VL)*dt
        Su_1 = self.Su - self.Et*(self.Su + U - 9.8)*dt
        self.Sv = Sv_1
        self.Su = Su_1
        return V_out
bridge = CvBridge()

def color_img_callback(msg):
    global color_img
    color_img = bridge.imgmsg_to_cv2(msg, "bgr8")

def depth_img_callback(msg):
    global depth_img
    depth_img = bridge.imgmsg_to_cv2(msg, "32FC1")
    depth_img = np.nan_to_num(depth_img)

def detact_distance(box,randnum):
    distance_list = []
    mid_pos = [(box.xmin + box.xmax)//2, (box.ymin + box.ymax)//2] #确定索引深度的中心像素位置
    min_val = min(abs(box.xmax - box.xmin), abs(box.ymax - box.ymin))#确定深度搜索范围
    #print(box,)
    for i in range(randnum):
        bias = random.randint(-min_val//4, min_val//4)
        dist = depth_img[int(mid_pos[1] + bias), int(mid_pos[0] + bias)]
        #cv2.circle(color_img (int(mid_pos[0] + bias), int(mid_pos[1] + bias)), 4, (255,0,0), -1)
        #print(int(mid_pos[1] + bias), int(mid_pos[0] + bias))
        if dist:
            distance_list.append(dist)
    distance_list = np.array(distance_list)
    distance_list = np.sort(distance_list)[randnum//2-randnum//4:randnum//2+randnum//4] #冒泡排序+中值滤波
    #print(distance_list, np.mean(distance_list))
    return round(np.mean(distance_list/1000),4)
def get_depth_frame():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # 创建对齐对象（深度对齐颜色）
    align = rs.align(rs.stream.color)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            
            # 获取对齐帧集
            aligned_frames = align.process(frames)
            
            # 获取对齐后的深度帧和彩色帧
            aligned_depth_frame = aligned_frames.get_depth_frame()
    except:
        print('cant get picture')
        return aligned_depth_frame
    
def local_velocity_callback(msg):
    global twiststamped
    twiststamped.header = header
    twiststamped.twist.linear.x = msg.twist.linear.x
    twiststamped.twist.linear.y = msg.twist.linear.y
    twiststamped.twist.linear.z = msg.twist.linear.z
    twiststamped.twist.angular.x = msg.twist.angular.x
    twiststamped.twist.angular.y = msg.twist.angular.y
    twiststamped.twist.angular.z = msg.twist.angular.z
    #print (twiststamped)

#返回z轴角速度和x加速度
def darknet_callback(data):
    global find_cnt, cmd, get_time, eval_distance,TwistStampe
    for box in data.bounding_boxes:
        if(box.id == 0 ):
        #if(box.id == 56 ):
            print("find human")
            #print("find chair")
            eval_distance = detact_distance(box,48)
            q_x = eval_distance - target_distance
            print("深度期望为:",q_x)
            u = (box.xmax+box.xmin)/2
            #print("中心像素距离为:",u - ppx)
            v = (box.ymax+box.ymin)/2
            # eval_distance = get_depth_frame().get_distance(u, v)
            # print("深度期望为：",eval_distance)
            TwistStampe.twist.angular.z = z_angvelocity.update(ppx/1000,u/1000,Dt)
            q_y = eval_distance*(u- ppx)/fx
            q_z = eval_distance*(v - ppy)/fy
            #WL = twiststamped.twist.angular.z

            XVL=twiststamped.twist.linear.x
            YVL=twiststamped.twist.linear.y
            ZVL=twiststamped.twist.linear.z
            #print("x_velocity",VL,'\t')
            TwistStampe.twist.linear.x = x_accelerate.update_X(U_=0.5,dt=dDt,VL=XVL,PL=-q_x) 
            TwistStampe.twist.linear.y = y_accelerate.update_Y(U_=0.5,dt=dDt,VL=YVL,PL=q_y)#左手系
            #print(twiststamped.twist.linear.x)
            VZ = z_accelerate.update_Z(U_=0.5,dt=Dt,VL=ZVL,PL=-q_z)
            if VZ > 0:
                TwistStampe.twist.linear.z = z_accelerate.update_Z(U_=0.5,dt=Dt,VL=ZVL,PL=q_z)
            else:
                TwistStampe.twist.linear.z = 0.001

            #录制bag包  
            toast.twist.linear.x = u - ppx
            toast.twist.linear.y = v - ppy
            toast.twist.linear.z = q_x

        else:
            twiststamped.twist.linear.x = 0
            twiststamped.twist.linear.y = 0
            twiststamped.twist.linear.z = 0.001
            twiststamped.twist.angular.z = 0.5

#返回z加速度
def local_pose_callback(msg):
    global height, target_height, target_set
    height = msg.pose.position.z 
    print('高度为： ',height)
    # PH = height - target_height
    # VL=twiststamped.twist.linear.z
    # #print("z_velocity",VL)
    # TwistStampe.twist.linear.z= z_accelerate.update_Z(U_=0.5,dt=Dt,VL=VL,PL=PH)
    # if not target_set:
    #     target_height = height     
    #     target_set = True  

if __name__ == "__main__":

    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    cmd = String()
    twiststamped =TwistStamped()
    TwistStampe = TwistStamped()
    toast = TwistStamped()
    target_distance = 3
    target_height = 0.6

    ppx=318.482
    ppy=241.167
    fx = 616.591
    fy = 616.765

    target_set = True
    find_cnt_last = 0
    not_find_time = 0
    get_time = False

    find_cnt = 0
    height = 0
    Dt = 0.01
    dDt = 0.1
    eval_distance = 0

    #PID control
    # Kp_yaw = 1
    # Ki_yaw = 0.0
    # Kd_yaw = 0.02
    Kp_yaw = 2
    Ki_yaw = 0.0
    Kd_yaw = 0.02
    z_angvelocity = PIDController(Kp_yaw,Ki_yaw,Kd_yaw)
    #ACC control
    ET = 0.1
    # x_tao1 = 2 
    # x_tao2 = 0.6
    x_tao1 = 2
    x_tao2 = 0.6

    # y_tao1 = 2
    # y_tao2 = 4
    y_tao1 = 2
    y_tao2 = 4

    # z_tao1 = 0.05
    # z_tao2 = 0.01
    z_tao1 = 0.05
    z_tao2 = 0.01

    x_accelerate = AccelerateController(x_tao1,x_tao2,ET)
    y_accelerate = AccelerateController(y_tao1,y_tao2,ET)
    z_accelerate = AccelerateController(z_tao1,z_tao2,ET)


    rospy.init_node("yolo_human_tracking")

    rospy.Subscriber("/"+vehicle_type+"_"+vehicle_id+"/camera/color/image_raw",Image,callback = color_img_callback, queue_size=1)

    rospy.Subscriber("/"+vehicle_type+"_"+vehicle_id+"/camera/depth/image_rect_raw", Image, callback = depth_img_callback, queue_size=1)

    rospy.Subscriber("/"+vehicle_type+"_"+vehicle_id+"/mavros/local_position/velocity_local",TwistStamped, callback = local_velocity_callback,queue_size=1)

    rospy.Subscriber("/"+vehicle_type+"_"+vehicle_id+"/mavros/local_position/pose", PoseStamped, callback = local_pose_callback,queue_size=1)

    rospy.Subscriber("/uav_"+vehicle_id+"/darknet_ros/bounding_boxes", BoundingBoxes, callback = darknet_callback,queue_size=1)

    cmd_vel_pub = rospy.Publisher("/xtdrone/"+vehicle_type+"_"+vehicle_id+"/cmd_vel_flu", TwistStamped, queue_size=1)
    cmd_pub = rospy.Publisher("/xtdrone/"+vehicle_type+"_"+vehicle_id+"/cmd", String, queue_size=1)
    errror_pub = rospy.Publisher("/xtdrone/"+vehicle_type+"_"+vehicle_id+"/error",TwistStamped,queue_size=1)
    rate = rospy.Rate(40) 


    header = Header()
    header.frame_id = "base_link"
    while not rospy.is_shutdown():
        rate.sleep()
        cmd_vel_pub.publish(TwistStampe)
        print("twist_x:",TwistStampe.twist.linear.x)
        cmd_pub.publish(cmd)                
        errror_pub.publish(toast)

        if find_cnt - find_cnt_last == 0:
            if not get_time:
                not_find_time = rospy.get_time()
                get_time = True
            if (rospy.get_time() - not_find_time > 3.0)or eval_distance =="nan":
                TwistStampe.twist.linear.x = 0.0
                TwistStampe.twist.linear.y = 0.0
                cmd = "HOVER"
                print(cmd)
                
                get_time = False
        find_cnt_last = find_cnt