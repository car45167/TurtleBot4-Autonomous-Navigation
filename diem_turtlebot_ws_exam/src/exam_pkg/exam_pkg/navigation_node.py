import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import threading
from irobot_create_msgs.msg import KidnapStatus
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import json
import os
from ament_index_python.packages import get_package_share_directory
from copy import deepcopy

class TryNavigator(Node):
    def __init__(self):
        super().__init__('try_node')

        self.get_logger().info(f"Navigation initializing")
        
        pose_data_final = self.read_positions_from_file('final_pose')
        x_final = pose_data_final['position']['x']
        y_final = pose_data_final['position']['y']
        

        pose_data_initial= self.read_positions_from_file('initial_pose')
        x_initial= pose_data_initial['position']['x']
        y_initial = pose_data_initial['position']['y']
        z_initial, w_initial = yaw_deg_to_quaternion(pose_data_initial['yaw_deg'])


        # Configurazione QoS
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        qos_best_effort_volatile = QoSProfile(
            depth=1, 
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE 
        )
        
        # Inizializzazione navigatore
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Setup TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Lock per thread safety
        self.lock = threading.RLock()
        self.lock_owner = None
        self.lock_acquire_time = None



    
        # Variabili di stato
        self.state = "final"
        self.nav_active = False
        self.current_intermediate_goal = None
        self.final_goal_reached = False
        self.awaiting_cancel_completion = False

        self.final_goal = self.create_pose(x_final, y_final)
        self.last_accepted_goal = None
        self.min_waypoint_distance = 0.5
        self.min_waypoint_angle = math.radians(30)

        # Variabili per gestire kidnapped robot
        self.last_amcl_pose = None
        self.kidnapped = False

        # Setup iniziale
        self.set_initial_pose(x_initial,y_initial,z_initial,w_initial)

        self.current_amcl_pose = None
        self.accepted_goals = []
        

        self.intermediate_goal_reached_pub = self.create_publisher(Bool, 'intermediate_goal_reached', qos)

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            qos
        )
        
        self.create_subscription(
            PoseStamped, 
            '/intermediate_goals', 
            self.intermediate_goal_callback, 
            qos
        )

        self.create_subscription(
            KidnapStatus,
            '/kidnap_status',
            self.kidnapped_status_callback,
            qos_best_effort_volatile
        )

        self.create_timer(0.5, self.control_loop)

    def acquire_lock(self, caller):
        """Acquisisce il lock con logging"""
        self.lock.acquire()
        self.lock_owner = caller

    def release_lock(self, caller):
        """Rilascia il lock con logging"""
        if self.lock_owner != caller:
            self.get_logger().error(f"{caller} tenta di rilasciare lock di {self.lock_owner}!")
        self.lock_owner = None
        self.lock_acquire_time = None
        self.lock.release()


    def read_positions_from_file(self, pose_type):
        """Legge le posizioni dal file JSON"""
        pkg_path = get_package_share_directory('exam_pkg')
        positions_path = os.path.join(pkg_path, 'resource', 'positions.json')
        
        try:
            with open(positions_path, 'r') as f:
                data = json.load(f)
                pose_data = data[pose_type]
                
                
                if 'yaw_deg' in pose_data:
                    yaw_rad = math.radians(pose_data['yaw_deg'])
                    pose_data['orientation'] = {
                        'z': math.sin(yaw_rad / 2.0),
                        'w': math.cos(yaw_rad / 2.0)
                    }
                
                return pose_data
                
        except Exception as e:
            self.get_logger().error(f"Errore nel leggere il file delle posizioni: {str(e)}")
           
            return {
                'position': {'x': 0.0, 'y': 0.0},
                'orientation': {'z': 0.0, 'w': 1.0}
            }


    def set_initial_pose(self,x_initial,y_initial,z_initial,w_initial):
        """Imposta la posizione iniziale usando i dati dal JSON"""
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        # Usa i dati dal JSON
        initial_pose.pose.position.x = x_initial
        initial_pose.pose.position.y = y_initial
        initial_pose.pose.orientation.z = z_initial
        initial_pose.pose.orientation.w = w_initial

        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(
            f"Initial pose set to ({initial_pose.pose.position.x}, {initial_pose.pose.position.y}) "
            f"with yaw {math.degrees(2*math.atan2(initial_pose.pose.orientation.z, initial_pose.pose.orientation.w)):.1f}°"
        )

    def create_pose(self, x, y, yaw_deg=None):
        """Creazione della pose a partire dai valori passati"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        if yaw_deg is not None:
            theta = math.radians(yaw_deg)
            pose.pose.orientation.z = math.sin(theta / 2.0)
            pose.pose.orientation.w = math.cos(theta / 2.0)
        else:
            pose.pose.orientation.w = 1.0
        
        self.get_logger().debug(f"Created pose at ({x}, {y})")
        return pose

    def amcl_pose_callback(self, msg):
        self.current_amcl_pose = msg.pose.pose
        
    def get_current_pose(self):
        """Ottiene la posizione da AMCL"""
        if self.current_amcl_pose is None:
            return None
            
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = self.current_amcl_pose
        return pose
    
    def quaternion_angle_diff(self, q1, q2):
        """Calcola la differenza angolare tra due quaternioni in radianti"""
        dot = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w
        dot = max(-1.0, min(1.0, dot))  
        return 2 * math.acos(abs(dot))  
    



                
    def is_same_pose(self, pose1: PoseStamped, pose2: PoseStamped, 
                    pos_tol=0.05, angle_tol=0.1, 
                    safety_multiplier=1.5) -> bool:

        
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        pos_diff = math.hypot(dx, dy)
        
        
        angle_diff = self.quaternion_angle_diff(
            pose1.pose.orientation,
            pose2.pose.orientation
        )
        
        
        safe_pos_tol = pos_tol * safety_multiplier
        safe_angle_tol = angle_tol * safety_multiplier
        
        
        self.get_logger().debug(f"Pose difference - Position: {pos_diff:.3f}m (tol: {safe_pos_tol:.3f}m), "
                            f"Angle: {math.degrees(angle_diff):.1f}° (tol: {math.degrees(safe_angle_tol):.1f}°)")
        
        # Controlla con range di sicurezza
        return (pos_diff < safe_pos_tol and angle_diff < safe_angle_tol)

    def is_pose_significantly_different(self, pose1: PoseStamped, pose2: PoseStamped) -> bool:
        """Determina se due pose sono significativamente diverse"""
        
        pos_diff = math.hypot(
            pose1.pose.position.x - pose2.pose.position.x,
            pose1.pose.position.y - pose2.pose.position.y
        )
        
        
        angle_diff = self.quaternion_angle_diff(
            pose1.pose.orientation,
            pose2.pose.orientation
        )
        
        return (pos_diff >= self.min_waypoint_distance or 
                angle_diff >= self.min_waypoint_angle)

    def intermediate_goal_callback(self, msg: PoseStamped):
        """Callback per la gestione dei goal intermedi"""
        try:
            self.acquire_lock("intermediate_goal_callback")

            ack_msg=Bool()
            ack_msg.data= False
            
            self.get_logger().info(
                f"Ricevuto goal intermedio - Frame: {msg.header.frame_id}, "
                f"Posizione: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})"
            )

            if self.final_goal_reached:
                self.get_logger().info("Final goal già raggiunto, ignoro nuovo goal")
                return

            
            new_goal = deepcopy(msg)
            
            
            if new_goal.header.frame_id != 'map':
                try:
                    new_goal = self.tf_buffer.transform(new_goal, 'map', timeout=Duration(seconds=1.0))
                    self.get_logger().info(
                        f"Transformed goal to map frame: ({new_goal.pose.position.x:.3f}, "
                        f"{new_goal.pose.position.y:.3f})"
                    )

                except Exception as e:
                    self.get_logger().error(f"Failed to transform goal: {str(e)}")
                    return


            if self.is_goal_too_far(new_goal):
                self.get_logger().warn(
                    f"Goal rifiutato: troppo lontano (>3m) - "
                    f"({new_goal.pose.position.x:.2f}, {new_goal.pose.position.y:.2f})"
                )
                ack_msg.data= True
                self.intermediate_goal_reached_pub.publish(ack_msg)

                return
            

            for accepted_goal in self.accepted_goals:
                if self.is_same_pose(msg, accepted_goal, 
                                    pos_tol=self.min_waypoint_distance,
                                    angle_tol=self.min_waypoint_angle,
                                    safety_multiplier=1.3):  
                    self.get_logger().warn(f"Goal troppo vicino a uno già accettato (con safety margin)")
                    ack_msg.data = True
                    self.intermediate_goal_reached_pub.publish(ack_msg)
                    return

            if (self.last_accepted_goal and not 
                self.is_pose_significantly_different(new_goal, self.last_accepted_goal)):
                self.get_logger().warn(
                    "Goal rifiutato: troppo simile al precedente "
                    f"(diff_pos < {self.min_waypoint_distance}m e "
                    f"diff_ang < {math.degrees(self.min_waypoint_angle):.1f}°)"
                )
                ack_msg.data = True 
                self.intermediate_goal_reached_pub.publish(ack_msg)
                return
                
            self.current_intermediate_goal = new_goal
            self.get_logger().info(
                f"Nuovo goal intermedio accettato: ({new_goal.pose.position.x:.3f}, "
                f"{new_goal.pose.position.y:.3f})"
            )

            if self.nav_active:
                self.get_logger().info("Interrompo navigazione corrente")
                self.navigator.cancelTask()
                self.awaiting_cancel_completion = True

                
            self.state = "intermediate"
        finally:
            self.release_lock("intermediate_goal_callback")

    def navigate_to_intermediate_goal(self):
        if not self.current_intermediate_goal:
            self.get_logger().warn("Nessun goal intermedio disponibile")
            return False

        goal = deepcopy(self.current_intermediate_goal)
        
        self.get_logger().info(
            f"Navigazione verso goal intermedio: ({goal.pose.position.x:.3f}, "
            f"{goal.pose.position.y:.3f})"
        )
        

        current_pose = self.get_current_pose()
        if current_pose:
            dx = self.final_goal.pose.position.x - goal.pose.position.x
            dy = self.final_goal.pose.position.y - goal.pose.position.y
            yaw = math.atan2(dy, dx)
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            self.get_logger().debug(f"Orientamento a yaw: {math.degrees(yaw):.1f}°")
        
        goal.header.stamp = self.get_clock().now().to_msg()
        
        self.navigator.goToPose(goal)
        self.nav_active = True
        self.intermediate_goal_start_time = self.get_clock().now()
        self.get_logger().info(
            f"Goal inviato al navigatore: ({goal.pose.position.x:.3f}, "
            f"{goal.pose.position.y:.3f})"
        )
        return True

    def navigate_to_final_goal(self):
        current_pose = self.get_current_pose()
        if not current_pose:
            self.get_logger().warn("Posizione corrente non disponibile - riprovo...")
            return False

        dx = self.final_goal.pose.position.x - current_pose.pose.position.x
        dy = self.final_goal.pose.position.y - current_pose.pose.position.y
        yaw = math.atan2(dy, dx)

        if self.final_goal.pose.orientation.w == 1.0:  
            self.final_goal.pose.orientation.z = math.sin(yaw / 2.0)
            self.final_goal.pose.orientation.w = math.cos(yaw / 2.0)
            self.get_logger().debug(f"Orientamento final goal settato a  {math.degrees(yaw):.1f}°")

        self.final_goal.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info("Navigazione verso goal finale...")
        self.navigator.goToPose(self.final_goal)
        self.nav_active = True
        return True

    def publish_goal_completion(self, success=True):
        msg = Bool()
        msg.data = success
        self.intermediate_goal_reached_pub.publish(msg)
        self.get_logger().info(f"Pubblicato completamento goal intermedio: {'success' if success else 'failure'}")



    def kidnapped_status_callback(self, msg: KidnapStatus):
        """ Callback per la gestione del robot rapito """
        try:
            self.acquire_lock("kidnapped_status_callback")

            if msg.is_kidnapped and not self.kidnapped:
                self.get_logger().error("KIDNAPPING RILEVATO! Il robot è stato sollevato.")
                self.kidnapped = True
                self.navigator.cancelTask()
                self.nav_active = False

            elif not msg.is_kidnapped and self.kidnapped:
                self.get_logger().info("Kidnapping risolto. Iniziando recovery...")
                self.kidnapped = False
                self.state = "recovery"
                self. current_intermediate_goal = None
                
        finally:
            self.release_lock("kidnapped_status_callback")



    def handle_navigation_success(self):
        if self.state == "intermediate":
            self.get_logger().info("Goal intermedio raggiunto con successo")
            self.publish_goal_completion()

            self.last_accepted_goal = self.current_intermediate_goal
            self.current_intermediate_goal = None
        
            self.state = "final"
            self.nav_active = False

        elif self.state == "final":
            self.get_logger().info("Goal finale raggiunto con successo!")
            self.final_goal_reached = True
            self.state = "idle"
            self.nav_active = False
            self.accepted_goals = []
        


    def handle_navigation_failure(self):
            self.get_logger().warn(f"Navigazione verso goal {self.state} fallita")
            self.nav_active = False
            self.intermediate_goal_start_time = None
            if self.state == "intermediate":
                self.publish_goal_completion(success=False)
                self.current_intermediate_goal = None
                self.state = "final"
               
                
    def control_loop(self):
        try:
            self.acquire_lock("control_loop")
            
            if self.kidnapped:
                self.get_logger().warn("Robot rapito, non posso procedere con la navigazione.")
                return
                

            if self.state == "recovery":

                self.release_lock("control_loop")
                self.perform_full_recovery()
                self.publish_goal_completion(success=False)

                
            if self.awaiting_cancel_completion:
                if self.navigator.isTaskComplete():
                    self.get_logger().info("Cancellazione navigazione completata")
                    self.awaiting_cancel_completion = False
                    self.nav_active = False
                else:
                    return
                    

            if not self.nav_active and not self.final_goal_reached:
                if self.state == "intermediate" and self.current_intermediate_goal:
                    if not self.navigate_to_intermediate_goal():
                        self.get_logger().warn("Navigazione intermedia fallita")
                elif self.state == "final":
                    if not self.navigate_to_final_goal():
                        self.get_logger().warn("Navigazione finale fallita")

            if self.nav_active and self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.handle_navigation_success()
                else:
                    self.handle_navigation_failure()
                    
        except Exception as e:
            self.get_logger().error(f"Error in control_loop: {str(e)}")
        finally:

            if self.lock_owner == "control_loop":
                self.release_lock("control_loop")


    def is_goal_too_far(self, goal_pose: PoseStamped) -> bool:
        """Verifica se il goal è più lontano di 3 metri dalla posizione corrente"""
        current_pose = self.get_current_pose()
        if not current_pose:
            self.get_logger().warn("Impossibile ottenere la posizione corrente")
            return True  
            
        dx = goal_pose.pose.position.x - current_pose.pose.position.x
        dy = goal_pose.pose.position.y - current_pose.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        self.get_logger().info(f"Distanza dal goal: {distance:.2f} metri")  
        return distance > 3.0   
    
    def perform_recovery_rotation(self):
        """Esegue una rotazione completa sul posto attraverso waypoints"""
            
        current_pose = self.get_current_pose()
        if not current_pose:
            self.get_logger().warn("Pose attuale non disponibile per la rotazione.")
            return
            

        waypoints = []
        num_steps = 5  
        
        for i in range(num_steps + 1):  
            angle = 2 * math.pi * i / num_steps
            pose = deepcopy(current_pose)
            pose.pose.orientation.z = math.sin(angle / 2)
            pose.pose.orientation.w = math.cos(angle / 2)
            waypoints.append(pose)
        
        self.get_logger().info("Effettuando una rotazione completa...")
        

        self.navigator.followWaypoints(waypoints)
        self.nav_active = True
        

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().info("Rotazione Completata.")
        self.nav_active = False
        




    def perform_full_recovery(self):
        try:
            self.acquire_lock("perform_full_recovery")
            
            self.get_logger().info("Iniziando la rilocalizzazione...")
            self.perform_recovery_rotation()
            

            self.get_logger().info("Resettando la navigazione...")
            self.state = "final"  
            self.nav_active = False
            self.final_goal_reached = False

            

            self.last_accepted_goal = None
            

            self.publish_goal_completion(success=False)
            self.get_logger().info("Recovery completato con successo. Pronto per nuovi obiettivi.")

        except Exception as e:
            self.get_logger().error(f"Errore durante recovery: {str(e)}")
        finally:
            self.release_lock("perform_full_recovery")

            return
                

        
def yaw_deg_to_quaternion(yaw_deg):
    """Converte yaw in gradi in quaternion (solo z e w)."""
    yaw_rad = math.radians(yaw_deg)
    z = math.sin(yaw_rad / 2.0)
    w = math.cos(yaw_rad / 2.0)
    return z, w  
def main(): 
    rclpy.init()    
    node = TryNavigator()   
    try:    
        rclpy.spin(node)    
    except KeyboardInterrupt:   
        node.get_logger().info("Shutdown richiesto")    
    finally:    
        node.destroy_node() 
        rclpy.shutdown()    

if __name__ == '__main__':  
    main()