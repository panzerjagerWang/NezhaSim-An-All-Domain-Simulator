#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

def clamp(x, a, b):
    return max(a, min(b, x))

def try_import_phase_srv():
    try:
        from nezha_plugins.srv import GetPhaseSample
        return GetPhaseSample
    except Exception:
        return None

class ZOverLController:
    def __init__(self):
        # 常量：航行器总长 L
        self.L = 0.323  # meters

        # 参数
        self.model_name = rospy.get_param("~model_name", "nezha_rocket")
        self.reference_frame = rospy.get_param("~reference_frame", "world")

        self.speed = rospy.get_param("~speed", 1.0)          # m/s
        self.pub_rate = rospy.get_param("~rate", 20.0)       # Hz

        self.target_zol_high = rospy.get_param("~target_z_over_l_high", 1.0)  # 完全出水
        self.target_zol_low  = rospy.get_param("~target_z_over_l_low", 0.0)   # 完全入水
        self.zol_tol         = rospy.get_param("~z_over_l_tolerance", 0.02)

        self.water_surface_z = rospy.get_param("~water_surface_z", 0.0)
        self.z_top_offset    = rospy.get_param("~z_top_offset", 0.0)

        self.surface_service = rospy.get_param("~surface_service", "/transmedia/get_phase_sample")

        # 发布器（负数向上，正数向下）
        self.pub = rospy.Publisher("/gantry/vertical_velocity_controller/command", Float64, queue_size=10)

        # Gazebo GetModelState
        rospy.loginfo("Waiting for /gazebo/get_model_state ...")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.cli_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        # 尝试连接 transmedia 相位/水面服务
        self.phase_srv_type = try_import_phase_srv()
        self.cli_phase = None
        if self.phase_srv_type is not None:
            try:
                rospy.loginfo("Waiting for %s ...", self.surface_service)
                rospy.wait_for_service(self.surface_service, timeout=5.0)
                self.cli_phase = rospy.ServiceProxy(self.surface_service, self.phase_srv_type, persistent=True)
                rospy.loginfo("Connected to %s", self.surface_service)
            except Exception:
                rospy.logwarn("Phase service %s not available, will use fallback estimation.", self.surface_service)
                self.cli_phase = None
        else:
            rospy.logwarn("GetPhaseSample srv not found in Python path; using fallback when needed.")

        # 初始目标：上行到 high
        self.target_zol = self.target_zol_high

        rospy.loginfo("ZOverLController: model=%s, L=%.3f, targets=[%.2f, %.2f], speed=%.2f m/s",
                      self.model_name, self.L, self.target_zol_low, self.target_zol_high, self.speed)

    def get_model_z(self):
        try:
            req = GetModelStateRequest(model_name=self.model_name, relative_entity_name=self.reference_frame)
            resp = self.cli_model(req)
            if not resp.success:
                rospy.logwarn_throttle(2.0, "GetModelState failed for %s", self.model_name)
                return None
            return resp.pose.position.z
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Exception calling GetModelState: %s", str(e))
            return None

    def query_phase_sample(self):
        if self.cli_phase is None:
            return False, None, None, None
        try:
            resp = self.cli_phase()
            # 优先使用服务提供的 surface_z 和 z_over_l
            return True, resp.surface_z, resp.z_over_l, resp.phase_name
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Phase service call failed: %s", str(e))
            return False, None, None, None

    def estimate_z_over_l(self, z_model):
        # 估算顶部高度：z_top = z_model + z_top_offset
        z_top = z_model + self.z_top_offset
        zol = (z_top - self.water_surface_z) / self.L
        return clamp(zol, 0.0, 1.0)

    def compute_command_from_zol(self, zol_now):
        dzol = self.target_zol - zol_now

        # 到达目标 -> 切换
        if abs(dzol) <= self.zol_tol:
            if math.isclose(self.target_zol, self.target_zol_high, abs_tol=1e-9):
                self.target_zol = self.target_zol_low
            else:
                self.target_zol = self.target_zol_high
            rospy.loginfo("Reached z_over_l target, switch target to %.3f", self.target_zol)
            dzol = self.target_zol - zol_now

        # 方向：目标更大 => 需要上移（z 增大）=> 负速度；目标更小 => 下移 => 正速度
        cmd_speed = -self.speed if dzol > 0.0 else self.speed

        # 近区比例缩放（10*tol 内线性缩小，最小 10%）
        band = 10.0 * self.zol_tol
        if abs(dzol) < band:
            scale = max(0.1, abs(dzol) / band)
            cmd_speed *= scale

        return cmd_speed, dzol

    def spin(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            # 优先使用服务提供的 z_over_l
            ok, surface_z, z_over_l, phase_name = self.query_phase_sample()

            if ok and z_over_l is not None:
                zol_now = clamp(z_over_l, 0.0, 1.0)
                cmd, dzol = self.compute_command_from_zol(zol_now)
                self.pub.publish(Float64(data=cmd))
                rospy.loginfo_throttle(1.0, "srv phase=%s, z_over_l=%.3f -> target=%.3f, dzol=%.3f, cmd=%.3f",
                                       phase_name, zol_now, self.target_zol, dzol, cmd)
            else:
                # fallback: 仅依赖模型 z 与配置参数
                z_model = self.get_model_z()
                if z_model is None:
                    rate.sleep()
                    continue
                zol_now = self.estimate_z_over_l(z_model)
                cmd, dzol = self.compute_command_from_zol(zol_now)
                self.pub.publish(Float64(data=cmd))
                rospy.loginfo_throttle(1.0, "[fallback] z_model=%.3f, z_over_l~%.3f -> target=%.3f, dzol=%.3f, cmd=%.3f",
                                       z_model, zol_now, self.target_zol, dzol, cmd)

            rate.sleep()

def main():
    rospy.init_node("z_over_l_controller")
    node = ZOverLController()
    node.spin()

if __name__ == "__main__":
    main()
