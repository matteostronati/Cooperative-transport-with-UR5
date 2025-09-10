#!/usr/bin/env python3

import rclpy
import csv
from datetime import datetime
import numpy as np
from rclpy.node import Node
from moveit_msgs.msg import JointConstraint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

#lista dei nomi dei giunti del robot
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]

TIMER = 0.4 #tempo di campionamento
MAX_SPEED=0.1 #massima velocità di movimento in m/s


class IKClient(Node):

    def __init__(self):
        super().__init__('ik_client')
        #creazione dei client per i servizi di cinematica inversa e diretta
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_cli = self.create_client(GetPositionFK, '/compute_fk')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attendo /compute_ik...')
        while not self.fk_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attendo /compute_fk...')

        self.ik_req = GetPositionIK.Request()
        self.fk_req = GetPositionFK.Request()

        self.joint_state = None

        self.subscription = self.create_subscription( #sottoscrizione al topic /joint_states per ricevere gli stati correnti dei giunti
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
    def joint_states_callback(self, msg): #callback che aggiorna la variabile joint_state
        self.joint_state = msg
        self.subscription = None
    
    def get_end_effector_pose(self): #calcolo della posa dell'end-effector rispetto al frame di riferimento base_link
        
        if not hasattr(self, 'joint_state') or len(self.joint_state.position) != 6:
            self.get_logger().warn("Joint state non valido o non ancora ricevuto.")
            return None

        #richiesta FK
        fk_req = GetPositionFK.Request()
        fk_req.header = Header(frame_id='base_link')  # Frame base
        fk_req.fk_link_names = ['tool0']              # Link finale
        fk_req.robot_state.joint_state.name = self.joint_state.name
        fk_req.robot_state.joint_state.position = self.joint_state.position

        #chiama il servizio in modo asincrono e attende il risultato
        future = self.fk_cli.call_async(fk_req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result() or len(future.result().pose_stamped) == 0:
            self.get_logger().error("Errore nel calcolo della FK.")
            return None

        return future.result().pose_stamped[0].pose #ritorna la posa dell'end-effector calcolata dalla FK

    def send_ik_request(self, pose): #invia la richiesta di cinematica inversa

        #imposta il gruppo manipolatore per cui calcolare l'IK e imposta la posa target nel frame base_link
        self.ik_req.ik_request.group_name = 'ur_manipulator'
        self.ik_req.ik_request.pose_stamped = PoseStamped()
        self.ik_req.ik_request.pose_stamped.header.frame_id = 'base_link'
        self.ik_req.ik_request.pose_stamped.pose = pose

        #recupera la posizione corrente del giunto wrist_3_joint
        current_position = self.joint_state.position[
            self.joint_state.name.index("wrist_3_joint")
        ]

        #viene creato un vincolo che sul giunto wrsit_3_joint per bloccarne la posizione con una tolleranza
        constraint = JointConstraint()
        constraint.joint_name = "wrist_3_joint"
        constraint.position = current_position
        constraint.tolerance_above = 0.6
        constraint.tolerance_below = 0.6
        constraint.weight = 1.0

        #viene aggiunto il constraint alla richiesta
        self.ik_req.ik_request.constraints.joint_constraints = [constraint]

        #chiamata asincrona al servizio IK
        future = self.cli.call_async(self.ik_req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result:
            if result.error_code.val != 1:
                self.get_logger().error(f"Errore nella richiesta IK: {result.error_code.val}")
            else:
                self.get_logger().info("Soluzione IK trovata.")
        else:
            self.get_logger().error("Errore nella risposta IK.")

        return result
    
class RobotController(Node): #classe ROS2 Node che gestisce il controllo del robot

    def __init__(self, ik_client):

        super().__init__('robot_controller')

        self.ik_client = ik_client #salva il client IK
        self.red_dot = None #memorizza la posizione del target (punto rosso)

        self.subscription = self.create_subscription( #sottoscrizione al topic '/target_position'
            Point,
            '/target_position',
            self.target_callback,
            10
        )
        
        #creazione di un publisher sul topic '/robot/joint_commands'
        self.joint_command_publisher = self.create_publisher(JointState, '/robot/joint_commands', 10) 

        self.timer = self.create_timer(TIMER, self.execute) #creazione del timer
        
        #inizializzazione dei file CSV per il logging delle posizioni calcolate e target
        self.next_pos_file = open("next_pos_log.csv", "w", newline='')
        self.next_pos_writer = csv.writer(self.next_pos_file)
        self.next_pos_writer.writerow(["timestamp", "x", "y", "z"])

        self.target_pos_file = open("target_pos_log.csv", "w", newline='')
        self.target_pos_writer = csv.writer(self.target_pos_file)
        self.target_pos_writer.writerow(["timestamp", "x", "y", "z"]) 

    def target_callback(self, msg): #callback quando arriva un nuovo messaggio con la posizione target
        self.red_dot = np.array([msg.x, msg.y, msg.z])

    def is_trajectory_safe(self, current_joints, target_joints, thresholds): #funzione per verificare il movimento sicuro

        #controllo che la differenza tra posizione target e posizione attuale non superi la soglia
        for i, (name, c, t, thr) in enumerate(zip(JOINT_NAMES, current_joints, target_joints, thresholds)):
            diff = abs(t - c)
            if diff > thr:
                self.get_logger().warn(
                    f"[SAFETY] Giunto '{name}' supera soglia: |{t:.2f} - {c:.2f}| = {diff:.2f} > {thr:.2f}"
                )
                return False
        return True

    def execute(self): #funzione che calcola la posizione che il robot deve raggiungere e chiama la cinematica inversa

        if self.red_dot is None:
            self.get_logger().info("Aspettando coordinate dal topic /target_position...")
            return

        tool_pose = self.ik_client.get_end_effector_pose() #ottiene la posa corrente dell'end-effector
        if tool_pose is None:
            self.get_logger().error("Impossibile ottenere la pose del tool0.")
            return

        #orientamento desiderato costante (quaternione definito a priori)
        initial_orientation = Quaternion(
            x=0.6139006598391906,
            y=0.536655579596502,
            z=-0.44428853171535015,
            w=-0.3711259480596406
        )

        desired_orientation = initial_orientation
        

        #ottiene la posizione attuale del tool0
        tool_pos = np.array([
            tool_pose.position.x,
            tool_pose.position.y,
            tool_pose.position.z
        ])

        #definisce il target 1 metro davanti il punto rosso sulla x
        target_x = self.red_dot[0] + 1.0

        #la y target deve essere la stessa del punto rosso
        target_y = self.red_dot[1]

        #forza la z a rimanere tra 45 cm e 60 cm per l'altezza
        target_z = np.clip(self.red_dot[2], 0.45, 0.60)

        #posizione desiderata, arrotondata a due cifre decimali
        target_pos = np.round([target_x, target_y, target_z], 2)

        #calcolo del vettore spostamento dal tool alla posizione target
        displacement = target_pos - tool_pos
        dist = np.linalg.norm(displacement)
        self.get_logger().info(f"DISTANZA DA FARE: {dist:.4f} m")
        self.get_logger().info(f"[DEBUG] tool_pos: x={tool_pos[0]:.3f}, y={tool_pos[1]:.3f}, z={tool_pos[2]:.3f}")

        #calcola la distanza massima percorribile in un tempo TIMER con MAX_SPEED
        max_step = MAX_SPEED * TIMER

        #se la distanza da percorrere è troppo grande, si avanza solo di un passo massimo
        if dist > max_step:
            direction = displacement / dist  
            next_pos = tool_pos + direction * max_step
            reached_final_target = False
        else:
            next_pos = target_pos  #posso arrivare direttamente
            reached_final_target = True
        
        if reached_final_target:
            self.get_logger().info("[STEP] POSIZIONE DIRETTA.")
        else:
            self.get_logger().info("[STEP] POSIZIONE INTERMEDIA.")
        
        #arrotonda la posizione calcolata
        next_pos = np.round(next_pos, 2)

        #costruzione della Pose desiderata (posizione+orientamento)
        desired_pose = Pose()
        desired_pose.position = Point(
            x=next_pos[0],
            y=next_pos[1],
            z=next_pos[2]
        )
        desired_pose.orientation = desired_orientation

        #stampa info per debugging
        self.get_logger().info(
            f"[CHECK] TARGET POSE: {target_pos} m"
        )
        self.get_logger().info(
            f"[CHECK] NEXT POS: {next_pos} m"
        )
        self.get_logger().info(
            f"[CHECK] Distanza dal punto rosso: {np.linalg.norm(np.array([desired_pose.position.x, desired_pose.position.y, desired_pose.position.z]) - self.red_dot):.4f} m"
        )

        #chiamata al client IK per trovare una soluzione per la Pose desiderata
        response = self.ik_client.send_ik_request(desired_pose)
        if response is None or response.error_code.val != 1:
            self.get_logger().error("IK fallita.")
            return

        #estrae la soluzione IK
        joint_positions = list(response.solution.joint_state.position)
        joint_names = response.solution.joint_state.name
        ik_solution = {name: pos for name, pos in zip(joint_names, joint_positions)}

        #blocca forzatamente il giunto wirst_3_joint per mantenere costante l'orientamento
        current_joint_state = self.ik_client.joint_state
        if current_joint_state and 'wrist_3_joint' in current_joint_state.name:
            index = current_joint_state.name.index('wrist_3_joint')
            ik_solution['wrist_3_joint'] = current_joint_state.position[index]
            
        #ordina la soluzione IK
        ordered_positions = [ik_solution[name] for name in JOINT_NAMES]
        current_positions = [self.ik_client.joint_state.position[self.ik_client.joint_state.name.index(name)] for name in JOINT_NAMES]

        #soglie massime consentite di variazione (in radianti) per ogni giunto
        joint_thresholds = [1.5, 1.5, 1.3, 0.8, 0.8, 1.0] 

        #controllo della sicurezza del movimento
        if not self.is_trajectory_safe(current_positions, ordered_positions, joint_thresholds):
            self.get_logger().error("Movimento annullato: cambiamento troppo brusco su uno o più giunti.")
            return

        #pubblicazione dei comandi dei giunti
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = JOINT_NAMES # I nomi dei giunti sono essenziali per JointState
        joint_state_msg.position = ordered_positions
        self.joint_command_publisher.publish(joint_state_msg)

        #salvataggio dei log nei file CSV
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        
        self.next_pos_writer.writerow([timestamp, next_pos[0], next_pos[1], next_pos[2]])
        self.next_pos_file.flush() 

        self.target_pos_writer.writerow([timestamp, target_pos[0], target_pos[1], target_pos[2]])
        self.target_pos_file.flush()

    def destroy_node(self):

        #chiude i file CSV quando il nodo viene distrutto
        self.next_pos_file.close()
        self.target_pos_file.close()
        
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    ik_client = IKClient()
    rclpy.spin_once(ik_client)
    
    while ik_client.joint_state is None:
        rclpy.spin_once(ik_client, timeout_sec=0.1)
        ik_client.get_logger().info("Waiting for the first /joint_states message...")
    robot_controller = RobotController(ik_client)
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
