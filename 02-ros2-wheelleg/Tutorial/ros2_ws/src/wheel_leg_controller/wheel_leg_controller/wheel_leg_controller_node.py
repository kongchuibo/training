import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from rm_msgs.msg import WheelLegCmd

from .vmc_cal import five_links_param
from .lqr_update import lqr_controller

# --- 系统常量配置 (严格与 robot.xml 及底层算法对齐，请勿随意更改) ---
MAX_HIP_TORQUE = 6.0      # 髋关节安全力矩上限
MAX_WHEEL_TORQUE = 2.0    # 轮电机安全力矩上限
LQR_WHEEL_LIMIT = 1.5    # LQR输出的轮力矩限制 (比底层上限更紧)
SIM_DT = 0.002            # 仿真步长，绑定 LQR 的 K 矩阵整定
SPEED_UP_RATE = 0.8       # 加速度斜率限制，防猛冲翻车
SPEED_DOWN_RATE = 1.4     # 减速度斜率限制

def limit_val(val, bound):
    """全局安全限幅工具"""
    return max(-bound, min(bound, val))

class PidRegulator:
    """
    移植自原工程的 PID 控制器。
    注意特性：微分项使用误差差分（未除以dt），依靠整定参数适应。
    """
    def __init__(self, p, i, d, out_max, int_max, feed=0.0):
        self.p_gain = p
        self.i_gain = i
        self.d_gain = d
        self.out_max = out_max
        self.int_max = int_max
        self.feed = feed
        self.err_sum = 0.0
        self.err_last = 0.0

    def calculate(self, actual, target):
        err = target - actual
        self.err_sum = limit_val(self.err_sum + err, self.int_max)
        d_err = err - self.err_last
        self.err_last = err
        output = (self.p_gain * err + self.i_gain * self.err_sum + 
                  self.d_gain * d_err + self.feed)
        return limit_val(output, self.out_max)

class WheelLegBalancerNode(Node):
    def __init__(self):
        super().__init__('wheel_leg_balancer_node')

        # 动态参数配置
        self.declare_parameter('target_length', 0.10)  # 站桩目标腿长 (m)
        self.declare_parameter('wheel_radius', 0.06)   # 轮子物理半径 (m)

        self.target_len = float(self.get_parameter('target_length').value)
        self.wheel_r = float(self.get_parameter('wheel_radius').value)

        # 数据缓存
        self.imu_msg = None
        self.joint_msg = None
        self.cmd_vel_msg = Twist()
        self.joint_idx_map = {}
        
        self.step_cnt = 0
        self.err_cnt = 0

        # 控制状态
        self.filtered_v = 0.0
        self.is_airborne = False

        # 算法实例 (左右独立，保存历史状态)
        self.kin_right = five_links_param(dt=SIM_DT)
        self.kin_left = five_links_param(dt=SIM_DT)
        self.lqr_right = lqr_controller(dt=SIM_DT)
        self.lqr_left = lqr_controller(dt=SIM_DT)

        # PID 实例
        self.leg_pid_r = PidRegulator(700.0, 0.0, 5.0, 150.0, 0.0)
        self.leg_pid_l = PidRegulator(700.0, 0.0, 5.0, 150.0, 0.0)
        self.yaw_pid_r = PidRegulator(0.5, 0.0, 1.0, 2.0, 0.0)
        self.yaw_pid_l = PidRegulator(0.5, 0.0, 1.0, 2.0, 0.0)

        # ROS 接口
        self.pub_cmd = self.create_publisher(WheelLegCmd, 'actuator_cmd', 10)
        self.create_subscription(Imu, 'imu', self.imu_cb, 10)
        self.create_subscription(JointState, 'joint_states', self.joints_cb, 10)
        self.create_subscription(Twist, 'cmd_vel', self.vel_cb, 10)

        self.get_logger().info(f'平衡节点启动 | 目标腿长: {self.target_len}m, 轮半径: {self.wheel_r}m')
    def imu_cb(self, msg):
        self.imu_msg = msg

    def vel_cb(self, msg):
        self.cmd_vel_msg = msg

    def joints_cb(self, msg):
        """核心主循环入口：外层包异常保护和倒地保护"""
        self.joint_msg = msg
        self.step_cnt += 1

        try:
            self._execute_control(msg)
        except Exception as e:
            self.err_cnt += 1
            if self.err_cnt <= 5 or self.err_cnt % 500 == 0:
                self.get_logger().error(f'控制异常(第{self.err_cnt}次): {e!r} -> 置零力矩保命')
            self._send_torques(0, 0, 0, 0, 0, 0)

    def _execute_control(self, msg):
        # 建立映射
        if not self.joint_idx_map:
            self.joint_idx_map = {name: i for i, name in enumerate(msg.name)}
            self.get_logger().info(f'关节映射建立: {list(msg.name)}')

        if self.imu_msg is None:
            self._send_torques(0, 0, 0, 0, 0, 0)
            return

        # 倒地保护 (倾角 > 60度)
        pitch_rad, _, _ = self._calc_attitude(self.imu_msg)
        if abs(pitch_rad) > 1.05:
            self._send_torques(0, 0, 0, 0, 0, 0)
            if self.step_cnt % 2000 == 0:
                self.get_logger().warn(f'机器人已倾倒 (pitch={math.degrees(pitch_rad):.1f}°)，请在 MuJoCo 点击 Reset')
            return

        # 姿态提取
        p_rad, gp_rate, gy_rate = self._calc_attitude(self.imu_msg)

        # 共模状态计算
        rw_pos = self._get_joint(msg, 'rightwheel', 'position')
        lw_pos = self._get_joint(msg, 'leftwheel', 'position')
        rw_vel = self._get_joint(msg, 'rightwheel', 'velocity')
        lw_vel = self._get_joint(msg, 'leftwheel', 'velocity')

        # 左减右消除镜像差异
        c_pos = (lw_pos - rw_pos) / 2.0
        c_vel = (lw_vel - rw_vel) / 2.0

        # 目标速度与爬坡
        tgt_v = float(self.cmd_vel_msg.linear.x)
        self.filtered_v = self._ramp_speed(self.filtered_v, tgt_v)

        # x_b_set 冻结逻辑：站桩时目标位置不跟随，保证位置环有效
        if not (self.filtered_v == 0.0 and tgt_v == 0.0):
            self.lqr_left.x_b_set = c_pos * self.wheel_r
            self.lqr_right.x_b_set = c_pos * self.wheel_r
        self.lqr_left.v_set = self.filtered_v
        self.lqr_right.v_set = self.filtered_v

        # 左右腿解算
        r_p1, r_p4, r_w = self._solve_single_leg(
            flag=0, 
            m1=self._get_joint(msg, 'right1', 'position'),
            m2=self._get_joint(msg, 'right2', 'position'),
            kin=self.kin_right, lqr=self.lqr_right,
            pid_leg=self.leg_pid_r, pid_yaw=self.yaw_pid_r,
            p=p_rad, gp=gp_rate, gy=gy_rate,
            c_pos=-c_pos, c_vel=-c_vel # 右腿取反
        )

        l_p1, l_p4, l_w = self._solve_single_leg(
            flag=1,
            m1=self._get_joint(msg, 'left1', 'position'),
            m2=self._get_joint(msg, 'left2', 'position'),
            kin=self.kin_left, lqr=self.lqr_left,
            pid_leg=self.leg_pid_l, pid_yaw=self.yaw_pid_l,
            p=p_rad, gp=gp_rate, gy=gy_rate,
            c_pos=c_pos, c_vel=c_vel
        )

        self._send_torques(r_p1, r_p4, l_p1, l_p4, r_w, l_w)

    def _solve_single_leg(self, flag, m1, m2, kin, lqr, pid_leg, pid_yaw, p, gp, gy, c_pos, c_vel):
        """单腿核心解算"""
        kin.forward_kinematics_cal(flag, m1, m2, p, gp)
        lqr.states_update(flag, kin, p, gp, c_pos, c_vel, self.wheel_r)
        lqr.calc_k_matrix_from_poly(kin.L_0)
        K = lqr.K

        # 虚拟力 F_0
        cos_t = math.cos(kin.theta)
        feed = (4.0 / cos_t) if abs(cos_t) > 1e-6 else 0.0
        f0 = limit_val(feed + pid_leg.calculate(kin.L_0, self.target_len), 100.0)

        # 力矩计算
        tp = (K[1]*kin.theta + K[3]*kin.d_theta + K[5]*(lqr.x_b - lqr.x_b_set) +
              K[7]*(lqr.v_b_whole - lqr.v_set) + K[9]*lqr.phi + K[11]*lqr.d_phi)
        
        wt = (K[0]*kin.theta + K[2]*kin.d_theta + K[4]*(lqr.x_b - lqr.x_b_set) +
              K[6]*(lqr.v_b_whole - lqr.v_set) + K[8]*lqr.phi + K[10]*lqr.d_phi)

        yaw = pid_yaw.calculate(gy, 0.0)
        t0, t1 = kin.VMC_torque_cal(f0, tp)

        if flag == 1: # 左腿
            wheel = limit_val(-wt - yaw, LQR_WHEEL_LIMIT)
            return -t0, -t1, wheel
        else: # 右腿
            wheel = limit_val(wt - yaw, LQR_WHEEL_LIMIT)
            return t0, t1, wheel

    def _calc_attitude(self, imu):
        """姿态解算"""
        q = imu.orientation
        t2 = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = -math.asin(t2)
        gp = -imu.angular_velocity.y
        gy = imu.angular_velocity.x
        return pitch, gp, gy

    def _get_joint(self, msg, name, field='position'):
        idx = self.joint_idx_map.get(name)
        if idx is None: return 0.0
        return getattr(msg, field)[idx]

    @staticmethod
    def _ramp_speed(cur_v, tgt_v):
        if tgt_v > cur_v:
            return min(tgt_v, cur_v + SPEED_UP_RATE * SIM_DT)
        return max(tgt_v, cur_v - SPEED_DOWN_RATE * SIM_DT)

    def _send_torques(self, r1, r4, l1, l4, rw, lw):
        msg = WheelLegCmd()
        msg.right_phi1 = limit_val(r1, MAX_HIP_TORQUE)
        msg.right_phi4 = limit_val(r4, MAX_HIP_TORQUE)
        msg.left_phi1 = limit_val(l1, MAX_HIP_TORQUE)
        msg.left_phi4 = limit_val(l4, MAX_HIP_TORQUE)
        msg.right_wheel = limit_val(rw, MAX_WHEEL_TORQUE)
        msg.left_wheel = limit_val(lw, MAX_WHEEL_TORQUE)
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelLegBalancerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
