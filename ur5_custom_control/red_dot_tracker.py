#!/usr/bin/env python3

import rclpy
from scipy.spatial.transform import Rotation as R
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import Point
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header
from cv_bridge import CvBridge


class RedDotToBaseNode(Node):
    def __init__(self):
        super().__init__('red_dot_to_base_node') #Inizializzazione nodo ROS2

        self.bridge = CvBridge() #Bridge per la conversione immagini ROS e OpenCV

        self.depth_image = None
        self.intrinsics = None
        self.joint_names = None
        self.joint_positions = None
        self.latest_color_image = None

        self.cli = self.create_client(GetPositionFK, '/compute_fk') #client per la cinematica diretta
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aspettando il servizio /compute_fk...')

        self.req = GetPositionFK.Request()
        self.req.header = Header(frame_id='base_link') #frame di riferimento per il calcolo della FK
        self.req.fk_link_names = ['tool0'] #link di cui calcolare la posa
 
        #si creano le sottoscrizioni alle immagini grazie ai driver Realsense e lo stato dei giunti
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.target_pub = self.create_publisher(Point, '/target_position', 10) #publisher per pubblicare la posizione del punto rosso

    def joint_state_callback(self, msg): #salva i nomi e le posizioni attuali dei giunti del robot
        self.joint_names = msg.name
        self.joint_positions = msg.position

    def camera_info_callback(self, msg): #salva i parametri intrinseci della camera al primo callback ricevuto
        if not self.intrinsics:
            self.intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'ppx': msg.k[2],
                'ppy': msg.k[5]
            }

    def depth_callback(self, msg): #callback per il valore di depth
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') #conversione immagine di profondità
        except Exception as e:
            self.get_logger().error(f'Errore nella conversione depth: {e}')

    def image_callback(self, msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') #conversione immagini a colori
            self.latest_color_image = color_image.copy()
        except Exception as e:
            self.get_logger().error(f'Errore RGB: {e}')
            return

        if self.intrinsics is None or self.depth_image is None or self.joint_positions is None:
            return

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)  #converte l'immagine in HSV e si crea una maschera per riconoscere i pixel rossi
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask = mask1 | mask2
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #trova i contorni della maschera

        if not contours: #se non ci sono contorni ritorna solo l'immagine
            cv2.imshow("RGB", color_image)
            cv2.waitKey(1)
            return

        largest = max(contours, key=cv2.contourArea) #prende il contorno più grande (presumibilmente il punto rosso sul telo)
        M = cv2.moments(largest) 
        if M["m00"] == 0:
            return
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        if 0 <= cy < self.depth_image.shape[0] and 0 <= cx < self.depth_image.shape[1]:
            depth_value = self.depth_image[cy, cx] / 1000.0 #se il punto è dentro i limiti dell'immagine estrae la profonità in metri
            if not (0.01 < depth_value < 2.0):
                return

            fx = self.intrinsics['fx']
            fy = self.intrinsics['fy']
            ppx = self.intrinsics['ppx']
            ppy = self.intrinsics['ppy']

            point_cam = np.array([  #calcolo della posizione 3D del punto nel frame della camera
                (cx - ppx) * depth_value / fx,
                (cy - ppy) * depth_value / fy,
                depth_value
            ])

            ref_point=(643,103)
            cv2.circle(color_image, ref_point, radius=5, color=(255,0,0), thickness=-1) #mostrato un punto di riferimento nello schermo di colore blu

            cv2.circle(color_image, (cx, cy), 5, (0, 255, 0), -1) #visualizza il punto rosso con il valore di profondità
            cv2.putText(color_image, f"{depth_value:.2f} m", (cx+10, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            cv2.imshow("RGB", color_image)
            cv2.waitKey(1)

            self.req.robot_state.joint_state.name = self.joint_names
            self.req.robot_state.joint_state.position = self.joint_positions
            #chiamata asincrona al servizio FK e handle_fk_response con la posizione 3D trovata
            future = self.cli.call_async(self.req)
            future.add_done_callback(lambda fut: self.handle_fk_response(fut, point_cam)) 

    def handle_fk_response(self, future, point_cam): #trasforma la posizione del punto nel frame della camera nel frame base_link del robot
        
        try:
            pose = future.result().pose_stamped[0].pose #ottiene la posa
            pos = np.array([pose.position.x, pose.position.y, pose.position.z]) #estrae la posizione (del tool0 nel base_link)
            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] #estrae l'orientamento come quaternione
            R_tool = R.from_quat(quat).as_matrix()

            offset = np.array([0.0, -0.06, -0.035]) #offset rigido tra tool0 e la camera montata dietro di esso
            cam_origin = pos + R_tool @ offset #posizione della camera nel frame base_link
            R_cam = R_tool #la rotazione della camera corrisponde con quella del tool0
            point_base = cam_origin + R_cam @ point_cam #posizione del punto rosso nel frame base_link

            self.get_logger().info(f"Punto rosso nel frame base_link: X={point_base[0]:.2f}, Y={point_base[1]:.2f}, Z={point_base[2]:.2f}")
            
            #preparazione e pubblicazione del messaggio contenente il punto 3D
            msg = Point()
            msg.x = round(float(point_base[0]), 2)
            msg.y = round(float(point_base[1]), 2)
            msg.z = round(float(point_base[2]), 2)
            self.target_pub.publish(msg)


        except Exception as e:
            self.get_logger().error(f'Errore FK: {e}')
        
def main():
    rclpy.init()
    node = RedDotToBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
