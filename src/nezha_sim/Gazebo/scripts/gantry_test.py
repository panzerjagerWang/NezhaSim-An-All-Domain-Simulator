#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
import dynamic_reconfigure.client
import sys


class GantryDiagnostic:
    def __init__(self):
        rospy.init_node('gantry_diagnostic')

        self.joint_state_received = False
        self.controller_state_received = False
        self.joint_position = None
        self.joint_velocity = None

        print("\n" + "=" * 60)
        print("GANTRY VERTICAL CONTROLLER DIAGNOSTIC")
        print("=" * 60 + "\n")

        # 订阅关节状态
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # 订阅控制器状态
        rospy.Subscriber('/gantry/vertical_velocity_controller/state',
                         JointControllerState, self.controller_state_callback)

        rospy.sleep(1)  # 等待订阅建立

        # 执行诊断
        self.run_diagnostics()

    def joint_state_callback(self, msg):
        self.joint_state_received = True
        # 查找vertical关节
        for i, name in enumerate(msg.name):
            if 'vertical' in name.lower():
                self.joint_position = msg.position[i]
                self.joint_velocity = msg.velocity[i] if len(msg.velocity) > i else None
                break

    def controller_state_callback(self, msg):
        self.controller_state_received = True

    def check_controller_manager(self):
        print("\n[1] Checking Controller Manager...")
        try:
            rospy.wait_for_service('/controller_manager/list_controllers', timeout=3)
            list_controllers = rospy.ServiceProxy(
                '/controller_manager/list_controllers',
                ListControllers
            )
            resp = list_controllers()

            found = False
            for controller in resp.controller:
                if 'vertical_velocity' in controller.name:
                    found = True
                    print("    ✓ Controller found: %s" % controller.name)
                    print("      - State: %s" % controller.state)
                    print("      - Type: %s" % controller.type)

                    if controller.state != 'running':
                        print("    ✗ WARNING: Controller is NOT running!")
                        return False
                    else:
                        print("    ✓ Controller is running")
                        return True

            if not found:
                print("    ✗ ERROR: vertical_velocity_controller not found!")
                return False

        except Exception as e:
            print("    ✗ ERROR: Cannot connect to controller_manager: %s" % e)
            return False

    def check_joint_states(self):
        print("\n[2] Checking Joint States...")

        if not self.joint_state_received:
            print("    ✗ ERROR: No joint_states received!")
            return False

        print("    ✓ Joint states are being published")

        if self.joint_position is not None:
            print("    ✓ Vertical joint found")
            print("      - Position: %.4f" % self.joint_position)
            if self.joint_velocity is not None:
                print("      - Velocity: %.4f" % self.joint_velocity)
            else:
                print("      - Velocity: Not published")
        else:
            print("    ✗ WARNING: Vertical joint not found in joint_states")

        return True

    def check_pid_params(self):
        print("\n[3] Checking PID Parameters...")
        try:
            client = dynamic_reconfigure.client.Client(
                '/gantry/vertical_velocity_controller/pid',
                timeout=3
            )
            config = client.get_configuration()

            print("    Current PID values:")
            print("      - P: %.2f" % config['p'])
            print("      - I: %.2f" % config['i'])
            print("      - D: %.2f" % config['d'])
            print("      - i_clamp_min: %.2f" % config['i_clamp_min'])
            print("      - i_clamp_max: %.2f" % config['i_clamp_max'])
            print("      - antiwindup: %s" % config['antiwindup'])

            if config['p'] == 0 and config['i'] == 0 and config['d'] == 0:
                print("    ✗ ERROR: All PID gains are ZERO!")
                print("    → Controller cannot work with zero gains!")
                return False
            elif config['p'] == 0:
                print("    ✗ WARNING: P gain is zero!")
                return False
            else:
                print("    ✓ PID parameters are set")
                return True

        except Exception as e:
            print("    ✗ ERROR: Cannot read PID parameters: %s" % e)
            return False

    def check_controller_state(self):
        print("\n[4] Checking Controller State Topic...")

        if not self.controller_state_received:
            print("    ✗ WARNING: No controller state received")
            return False

        print("    ✓ Controller state is being published")
        return True

    def test_command(self):
        print("\n[5] Testing Command Publishing...")

        pub = rospy.Publisher(
            '/gantry/vertical_velocity_controller/command',
            Float64, queue_size=10
        )

        rospy.sleep(0.5)

        print("    Sending test command: 0.1 m/s")

        # 记录初始状态
        initial_velocity = self.joint_velocity

        # 发送命令
        for i in range(20):
            msg = Float64()
            msg.data = 0.1
            pub.publish(msg)
            rospy.sleep(0.1)

        # 检查是否有变化
        if self.joint_velocity is not None and initial_velocity is not None:
            change = abs(self.joint_velocity - initial_velocity)
            print("    Velocity change: %.6f" % change)

            if change > 0.001:
                print("    ✓ System is responding to commands!")
                return True
            else:
                print("    ✗ No response to command")
                return False
        else:
            print("    ? Cannot determine response (velocity not available)")
            return None

    def run_diagnostics(self):
        results = []

        results.append(self.check_controller_manager())
        results.append(self.check_joint_states())
        results.append(self.check_pid_params())
        results.append(self.check_controller_state())
        results.append(self.test_command())

        print("\n" + "=" * 60)
        print("DIAGNOSTIC SUMMARY")
        print("=" * 60)

        passed = sum(1 for r in results if r is True)
        failed = sum(1 for r in results if r is False)
        unknown = sum(1 for r in results if r is None)

        print("Passed: %d | Failed: %d | Unknown: %d" % (passed, failed, unknown))

        if failed > 0:
            print("\n⚠ ISSUES DETECTED - See details above")
            self.suggest_fixes()
        elif passed == len([r for r in results if r is not None]):
            print("\n✓ All checks passed!")

        print("=" * 60 + "\n")

    def suggest_fixes(self):
        print("\nSUGGESTED FIXES:")
        print("-" * 60)
        print("1. Set PID parameters:")
        print("   rosrun dynamic_reconfigure dynparam set \\")
        print("     /gantry/vertical_velocity_controller/pid p 100.0")
        print("")
        print("2. Start the controller:")
        print("   rosservice call /controller_manager/switch_controller \\")
        print("     \"start_controllers: ['vertical_velocity_controller']")
        print("      stop_controllers: []")
        print("      strictness: 2\"")
        print("")
        print("3. Check YAML configuration file for correct joint name")
        print("-" * 60)


if __name__ == '__main__':
    try:
        diagnostic = GantryDiagnostic()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nDiagnostic interrupted by user")
