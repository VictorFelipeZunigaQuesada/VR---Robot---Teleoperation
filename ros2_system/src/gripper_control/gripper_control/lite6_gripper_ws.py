#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
import websocket
import json
import time

ROBOT_IP = "200.126.19.206"
WS_URL = f"ws://{ROBOT_IP}:18333/ws?channel=prod&lang=en"


class Lite6GripperWS(Node):
    def __init__(self):
        super().__init__("lite6_gripper_ws")

        # WebSocket
        self.ws = websocket.WebSocket()
        self.ws.connect(WS_URL)
        self.get_logger().info("WebSocket conectado al robot")

        # Parámetros del gripper
        self.open_pos = 0.0
        self.close_pos = 0.0079
        self.current_gripper_pos = self.open_pos

        # Buffer para almacenar joints del robot real
        self.robot_joints = {}   # joint1..joint6
        self.robot_joint_order = [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"
        ]

        # Suscripción a /ufactory/joint_states (robot real)
        self.create_subscription(
            JointState,
            "/ufactory/joint_states",
            self.robot_state_callback,
            10
        )

        # Publicador /joint_states FUSIONADO
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Timer para publicar
        self.create_timer(0.02, self.publish_fused_joint_states)  # 50 Hz

        # Action server de MoveIt
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Servicios manuales
        self.create_service(Trigger, "gripper_open", self.cb_open)
        self.create_service(Trigger, "gripper_close", self.cb_close)
        self.create_service(Trigger, "gripper_stop", self.cb_stop)

        self.get_logger().info("Nodo Lite6GripperWS listo (fusionando joint states)")

    # ============================
    # 1) CALLBACK ROBOT REAL
    # ============================

    def robot_state_callback(self, msg):
        """Guarda los joints reales del robot"""
        for name, pos in zip(msg.name, msg.position):
            if name in self.robot_joint_order:
                self.robot_joints[name] = pos

    # ============================
    # 2) PUBLICADOR FUSIONADO
    # ============================

    def publish_fused_joint_states(self):
        """Publica /joint_states completo: robot + gripper"""
        if len(self.robot_joints) < 6:
            # Aún no llegan todos los joints reales
            return

        fused = JointState()
        fused.header.stamp = self.get_clock().now().to_msg()

        # Orden correcto
        fused.name = self.robot_joint_order + [
            "left_finger_joint",
            "right_finger_joint"
        ]

        fused.position = [
            self.robot_joints[j] for j in self.robot_joint_order
        ] + [
            self.current_gripper_pos,
            -self.current_gripper_pos
        ]

        self.joint_pub.publish(fused)

    # ============================
    # 3) COMUNICACIÓN WEBSOCKET
    # ============================

    def send_ws(self, op):
        payload = {
            "cmd": "xarm_set_lite6_gripper",
            "data": {"userid": "ros", "version": "lite6", "op": op},
            "id": "1"
        }
        self.ws.send(json.dumps(payload))
        self.get_logger().info(f"WS → {op}")

    # ============================
    # 4) ACTION SERVER MOVEIT
    # ============================

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        if len(traj.points) == 0:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        target = traj.points[-1].positions[0]
        tol = 0.001

        if abs(target - self.open_pos) < tol:
            self.send_ws("open")
            self.current_gripper_pos = self.open_pos
        elif abs(target - self.close_pos) < tol:
            self.send_ws("close")
            self.current_gripper_pos = self.close_pos
        else:
            # Posición intermedia → elegir la más cercana
            if abs(target - self.open_pos) < abs(target - self.close_pos):
                self.send_ws("open")
                self.current_gripper_pos = self.open_pos
            else:
                self.send_ws("close")
                self.current_gripper_pos = self.close_pos

        time.sleep(1.8)  # tiempo necesario para movimiento real

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ============================
    # 5) SERVICIOS
    # ============================

    def cb_open(self, req, res):
        self.send_ws("open")
        self.current_gripper_pos = self.open_pos
        res.success = True
        res.message = "Gripper OPEN"
        return res

    def cb_close(self, req, res):
        self.send_ws("close")
        self.current_gripper_pos = self.close_pos
        res.success = True
        res.message = "Gripper CLOSE"
        return res

    def cb_stop(self, req, res):
        self.send_ws("stop")
        res.success = True
        res.message = "Gripper STOP"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = Lite6GripperWS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

