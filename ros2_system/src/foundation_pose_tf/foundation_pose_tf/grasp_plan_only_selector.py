#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np
import tf_transformations as tft

from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformListener

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, JointConstraint,
    AttachedCollisionObject, PlanningScene, CollisionObject
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

class GraspPickAttachPlace(Node):
    def __init__(self):
        super().__init__('grasp_pick_attach_place')

        # ---------------- CONFIG ----------------
        self.arm_group = 'lite6'
        self.gripper_group = 'gripper'
        self.world_frame = 'link_base'
        self.tcp_frame = 'tcp_link'

        self.object_frame = 'detected_object_frozen'
        self.ghost_frame = 'ghost_pose'
        self.object_id = 'target_object'
        self.object_dims = [0.16, 0.015, 0.21]

        # ---------------- MOVEIT ----------------
        self.move_action = ActionClient(self, MoveGroup, '/move_action')
        self.ps_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- SUBS ----------------
        self.create_subscription(JointState, '/joint_states', self.cb_joint_state, 10)
        from visualization_msgs.msg import MarkerArray
        self.create_subscription(MarkerArray, '/detected_grasp_valid', self.cb_markers, 10)
        
        self.joint_state_received = False
        self.phase = 'IDLE' 
        self.markers = []
        self.current_index = 0
        self.M_OT = None 
        self.object_in_world = False
        self.lift_z = 0.0 # Guardaremos la altura de seguridad aquí

        self.get_logger().info('🦾 Nodo Pick & Place (Altura Constante 5cm) listo')

    def cb_joint_state(self, msg):
        if msg.name: self.joint_state_received = True

    def cb_markers(self, msg):
        if self.phase != 'IDLE' or not self.joint_state_received: return
        self.markers = [m for m in msg.markers if m.action == 0]
        if not self.markers: return
        
        if not self.object_in_world:
            if self.add_object_to_world_from_tf(self.object_frame):
                self.object_in_world = True

        self.current_index = 0
        self.phase = 'PICK'
        self.try_next_pick()

    # ==================================================
    # ------------------ FASE 1: PICK ------------------
    # ==================================================
    def try_next_pick(self):
        if self.current_index >= len(self.markers):
            self.get_logger().error('❌ No hay grasps alcanzables.')
            self.phase = 'IDLE'
            return

        marker = self.markers[self.current_index]
        
        self.final_pick_pose = Pose()
        self.final_pick_pose.position.x = marker.pose.position.x
        self.final_pick_pose.position.y = marker.pose.position.y
        self.final_pick_pose.position.z = marker.pose.position.z - 0.001 
        self.final_pick_pose.orientation = marker.pose.orientation

        pre_pick_pose = Pose()
        pre_pick_pose.position.x = marker.pose.position.x
        pre_pick_pose.position.y = marker.pose.position.y
        pre_pick_pose.position.z = marker.pose.position.z + 0.045 
        pre_pick_pose.orientation = marker.pose.orientation

        self.get_logger().info(f'➡ [PASO 1] Yendo a Pre-Pick (+05cm)...')
        goal = self.build_arm_goal(pre_pick_pose)
        self.move_action.send_goal_async(goal).add_done_callback(self.on_pre_pick_response)

    def on_pre_pick_response(self, future):
        gh = future.result()
        if not gh or not gh.accepted:
            self.current_index += 1
            self.try_next_pick()
            return
        gh.get_result_async().add_done_callback(self.on_pre_pick_result)

    def on_pre_pick_result(self, future):
        if future.result().result.error_code.val == 1:
            self.get_logger().info('✅ Pre-pick alcanzado.')
            input("\n👉 ALINEADO ARRIBA. Revisa el gripper y presiona ENTER para bajar...")
            
            self.get_logger().info('➡ [PASO 2] Descendiendo linealmente al objeto (Z+0.5cm)...')
            goal = self.build_arm_goal(self.final_pick_pose)
            goal.request.max_velocity_scaling_factor = 0.05 
            self.move_action.send_goal_async(goal).add_done_callback(self.on_pick_response)
        else:
            self.current_index += 1
            self.try_next_pick()

    def on_pick_response(self, future):
        gh = future.result()
        if not gh or not gh.accepted:
            self.current_index += 1
            self.try_next_pick()
            return
        gh.get_result_async().add_done_callback(self.on_pick_result)

    def on_pick_result(self, future):
        if future.result().result.error_code.val == 1:
            if self.compute_and_store_M_OT():
                input("\n👉 POSICIÓN ALCANZADA. Presiona ENTER para cerrar gripper...")
                self.attach_object()
        else:
            self.current_index += 1
            self.try_next_pick()

    def attach_object(self):
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(is_diff=True)
        aco = AttachedCollisionObject()
        aco.link_name = self.tcp_frame
        aco.object.id = self.object_id
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = [self.tcp_frame, 'left_finger', 'right_finger', 'link_eef']
        req.scene.robot_state.attached_collision_objects.append(aco)
        
        obj_remove = CollisionObject(id=self.object_id, operation=CollisionObject.REMOVE)
        req.scene.world.collision_objects.append(obj_remove)
        self.ps_client.call_async(req).add_done_callback(
            lambda _: self.move_gripper(0.008, self.after_pick_closed)
        )

    def after_pick_closed(self, _):
        # Levantamiento de 5cm (Límite visual)
        self.get_logger().info('⬆ Elevando objeto 5cm (Retracción segura)...')
        lift_pose = Pose()
        lift_pose.position.x = self.final_pick_pose.position.x
        lift_pose.position.y = self.final_pick_pose.position.y
        
        # Guardamos la Z actual para usarla en el Pre-Place
        self.lift_z = self.final_pick_pose.position.z + 0.05
        lift_pose.position.z = self.lift_z
        lift_pose.orientation = self.final_pick_pose.orientation

        goal = self.build_arm_goal(lift_pose)
        self.move_action.send_goal_async(goal).add_done_callback(
            lambda f: f.result().get_result_async().add_done_callback(self.on_lift_result)
        )

    def on_lift_result(self, future):
        if future.result().result.error_code.val == 1:
            input("\n✅ ELEVACIÓN OK. Presiona ENTER para ir SOBRE EL DESTINO (Mismo Z)...")
            self.try_pre_place()
        else:
            self.get_logger().error('❌ Falló elevación.')

    def try_pre_place(self):
        try:
            t_ghost = self.tf_buffer.lookup_transform(self.world_frame, self.ghost_frame, rclpy.time.Time())
            m_ghost = self.tf_to_mat(t_ghost)
            m_tcp_goal = np.dot(m_ghost, self.M_OT)
            
            trans = tft.translation_from_matrix(m_tcp_goal)
            quat = tft.quaternion_from_matrix(m_tcp_goal)

            # --- POSE DE PRE-PLACE: Mantenemos la altura Z del Lift ---
            pre_p = Pose()
            pre_p.position.x = trans[0]
            pre_p.position.y = trans[1]
            pre_p.position.z = self.lift_z # Forzamos la altura para no perder visibilidad
            pre_p.orientation.x, pre_p.orientation.y, pre_p.orientation.z, pre_p.orientation.w = quat
            
            # Pose final de destino (esta sí bajará)
            self.final_place_pose = Pose()
            self.final_place_pose.position.x, self.final_place_pose.position.y, self.final_place_pose.position.z = trans
            self.final_place_pose.orientation = pre_p.orientation

            self.get_logger().info(f'➡ Moviendo horizontalmente a altura Z={self.lift_z:.3f}...')
            goal = self.build_arm_goal(pre_p)
            self.move_action.send_goal_async(goal).add_done_callback(
                lambda f: f.result().get_result_async().add_done_callback(self.on_pre_place_result)
            )
        except Exception as e:
            self.get_logger().error(f'❌ Error Pre-Place: {e}')

    def on_pre_place_result(self, future):
        if future.result().result.error_code.val == 1:
            input("\n✅ SOBRE EL DESTINO. Presiona ENTER para bajar al destino final...")
            goal = self.build_arm_goal(self.final_place_pose)
            self.move_action.send_goal_async(goal).add_done_callback(
                lambda f: f.result().get_result_async().add_done_callback(self.on_place_result)
            )
        else:
            self.get_logger().error('❌ Falló Pre-Place.')

    def on_place_result(self, future):
        if future.result().result.error_code.val == 1:
            input("\n👉 EN DESTINO. Presiona ENTER para abrir...")
            self.move_gripper(0.000, self.after_place_opened)
        else:
            self.get_logger().error('❌ Falló Place.')

    def after_place_opened(self, _):
        self.detach_object()

    def detach_object(self):
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(is_diff=True)
        aco = AttachedCollisionObject()
        aco.object.id = self.object_id
        aco.object.operation = CollisionObject.REMOVE
        req.scene.robot_state.attached_collision_objects.append(aco)
        self.ps_client.call_async(req).add_done_callback(self.go_home)

    def go_home(self, _):
        self.get_logger().info('🏠 Volviendo a HOME...')
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        self.move_action.send_goal_async(goal).add_done_callback(
            lambda f: f.result().get_result_async().add_done_callback(self.finished)
        )

    def finished(self, _):
        self.get_logger().info('🎉 CICLO COMPLETADO.')
        self.phase = 'DONE'

    # ==================================================
    # ------------------- HELPERS ----------------------
    # ==================================================
    def move_gripper(self, pos, callback):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.gripper_group
        goal.request.max_velocity_scaling_factor = 0.1
        jc = JointConstraint(joint_name='left_finger_joint', position=pos, tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
        goal.request.goal_constraints.append(Constraints(joint_constraints=[jc]))
        self.move_action.send_goal_async(goal).add_done_callback(lambda f: f.result().get_result_async().add_done_callback(callback))

    def compute_and_store_M_OT(self):
        try:
            t_obj = self.tf_buffer.lookup_transform(self.world_frame, self.object_frame, rclpy.time.Time())
            t_tcp = self.tf_buffer.lookup_transform(self.world_frame, self.tcp_frame, rclpy.time.Time())
            m_wo = self.tf_to_mat(t_obj); m_wt = self.tf_to_mat(t_tcp)
            self.M_OT = np.dot(np.linalg.inv(m_wo), m_wt)
            return True
        except: return False

    def tf_to_mat(self, tf):
        t = tf.transform.translation; q = tf.transform.rotation
        return tft.concatenate_matrices(tft.translation_matrix([t.x, t.y, t.z]), tft.quaternion_matrix([q.x, q.y, q.z, q.w]))

    def add_object_to_world_from_tf(self, frame: str) -> bool:
        try:
            T = self.tf_buffer.lookup_transform(self.world_frame, frame, rclpy.time.Time())
            co = CollisionObject(id=self.object_id, operation=CollisionObject.ADD)
            co.header.frame_id = self.world_frame
            p = Pose()
            p.position.x, p.position.y, p.position.z = T.transform.translation.x, T.transform.translation.y, T.transform.translation.z + 0.005
            p.orientation = T.transform.rotation
            co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=self.object_dims))
            co.primitive_poses.append(p)
            self.ps_client.call_async(ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True, world=PlanningScene().world)))
            return True
        except: return False

    def build_arm_goal(self, pose: Pose) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.allowed_planning_time = 10.0
        
        pc = PositionConstraint(header=PoseStamped().header, link_name=self.tcp_frame)
        pc.header.frame_id = self.world_frame
        pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01]*3))
        pc.constraint_region.primitive_poses.append(pose)
        
        oc = OrientationConstraint(header=pc.header, link_name=self.tcp_frame, orientation=pose.orientation)
        # Bloqueamos giros de muñeca
        oc.absolute_x_axis_tolerance = 0.001
        oc.absolute_y_axis_tolerance = 0.001
        oc.absolute_z_axis_tolerance = 0.001
        oc.weight = 1.0
        
        goal.request.goal_constraints.append(Constraints(position_constraints=[pc], orientation_constraints=[oc]))
        return goal

def main():
    rclpy.init()
    node = GraspPickAttachPlace()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()