#!/usr/bin/env python3

import rclpy
import csv
from datetime import datetime
import numpy as np
from rclpy.node import Node
from rclpy.duration import Duration as rclpyDuration
from sensor_msgs.msg import JointState              
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from std_msgs.msg import Header

#lista dei nomi dei giunti del robot
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]
ACTION_SERVER_NAME = '/scaled_joint_trajectory_controller/follow_joint_trajectory' #nome dell'action server usato per inviare le traiettorie
TIMER_DURATION_SECONDS = 0.4 #tempo di esecuzione della traiettoria

class SimpleRobotMover(Node):
    def __init__(self):
        super().__init__('simple_robot_mover_node')

        #client per il servizio di FK
        self.fk_cli = self.create_client(GetPositionFK, '/compute_fk')

        while not self.fk_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for FK service...')
        self.get_logger().info("FK service available")

        self.joint_state = None

        #Subscriber per aggiornare joint_state
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        #inizializza l'action client per il controllo del robot
        self._action_client = ActionClient(self, FollowJointTrajectory, ACTION_SERVER_NAME)
        self.get_logger().info(f"Waiting for action server: {ACTION_SERVER_NAME}")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server available!")

        #Subscriver che ascolta i comandi dei giunti del nodo RobotController
        self.joint_command_subscription = self.create_subscription(
            JointState,
            '/robot/joint_commands', 
            self.joint_command_callback,
            10
        )
        self.get_logger().info(f"Subscribed to /robot/joint_commands")
        self.get_logger().info("SimpleRobotMover node initialized. Waiting for joint commands...")
        
        #file CSV per salvare le posizioni dell'end-effector raggiunte 
        self.end_effector_log_file = open("end_effector_reached_pos_log.csv", "w", newline='')
        self.end_effector_log_writer = csv.writer(self.end_effector_log_file)
        self.end_effector_log_writer.writerow(["timestamp", "x", "y", "z"])

    def joint_state_callback(self, msg: JointState):
        self.joint_state = msg

    def get_end_effector_pose(self): #calcolo della posa dell'end-effector rispetto al frame di riferimento base_link
        if self.joint_state is None or len(self.joint_state.position) != 6:
            self.get_logger().warn("Joint state non valido o non ancora ricevuto.")
            return None

        #richiesta per il servizio di FK
        fk_req = GetPositionFK.Request()
        fk_req.header = Header()
        fk_req.header.frame_id = 'base_link'
        fk_req.fk_link_names = ['tool0']
        fk_req.robot_state.joint_state.name = self.joint_state.name
        fk_req.robot_state.joint_state.position = self.joint_state.position

        future = self.fk_cli.call_async(fk_req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result() or len(future.result().pose_stamped) == 0:
            self.get_logger().error("Errore nel calcolo della FK.")
            return None

        return future.result().pose_stamped[0].pose


    def joint_command_callback(self, msg: JointState):
        
        #callback per messaggi JointState
        joint_positions = list(msg.position)
        self.get_logger().info(f"Received joint command: {joint_positions}")
        self.send_trajectory(joint_positions, None)  #chiama la funzione che invia la traiettoria al robot

    def send_trajectory(self, joint_positions, desired_pose): #invia la traiettoria di giunti al controller del robot
       
        #creazione dell'obiettivo per l'action server FollowJointTrajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINT_NAMES 
        
        #definizione del punto singolo della traiettoria (posizione e velocità)
        point = JointTrajectoryPoint()
        point.positions = joint_positions #posizioni traget dei giunti
        point.velocities = [0.0] * len(joint_positions) #velocità 0
        point.time_from_start = rclpyDuration(seconds=TIMER_DURATION_SECONDS).to_msg() #durata della traiettoria settata

        goal.trajectory.points.append(point) #aggiunge il punto alla traiettoria
        
        self.get_logger().info(f"Sending trajectory goal for joint positions: {joint_positions}")
        #invia l'obiettivo all'action server in modo asincrono
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future): #callback dopo l'invio della traiettoria
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected') #obiettivo rifiutatio
            return
        self.get_logger().info('Goal accepted') #obiettivo accettato

        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def result_callback(self, future): #callback per il risultato del movimento
        result = future.result().result
        self.get_logger().info(f'Risultato del movimento: {result.error_code}')
        
        #ottiene la posa finale dell'end-effector tramite la FK
        final_pose = self.get_end_effector_pose()
        if final_pose:

            #registra la posizione nel file CSV con un timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            self.end_effector_log_writer.writerow([
                timestamp,
                final_pose.position.x,
                final_pose.position.y,
                final_pose.position.z
            ])
            self.end_effector_log_file.flush()
            self.get_logger().info(f"Registrata posizione finale end-effector (XYZ): "
                                f"X={final_pose.position.x:.3f}, Y={final_pose.position.y:.3f}, Z={final_pose.position.z:.3f}")
        else:
            self.get_logger().error("Impossibile registrare la posizione finale dell'end-effector.")


def main(args=None):

    rclpy.init(args=args)
    node = SimpleRobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()