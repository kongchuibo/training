import math
from tf_transformations import euler_from_quaternion

class robot_info_update:
    def __init__(self):
        # 按照图片 5.3 要求的顺序
        self.joint_names = ['right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel']
        self.pos = {name: 0.0 for name in self.joint_names}
        self.vel = {name: 0.0 for name in self.joint_names}
        
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.roll_rate, self.pitch_rate, self.yaw_rate = 0.0, 0.0, 0.0

    def update_imu(self, imu_msg):
        # 四元数转欧拉角
        q = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(q)
        # 角速度
        self.roll_rate = imu_msg.angular_velocity.x
        self.pitch_rate = imu_msg.angular_velocity.y
        self.yaw_rate = imu_msg.angular_velocity.z

    def update_joints(self, joint_msg):
        for i, name in enumerate(joint_msg.name):
            if name in self.pos:
                self.pos[name] = joint_msg.position[i]
                self.vel[name] = joint_msg.velocity[i]
