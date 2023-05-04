import rclpy
import cv2
from PIL import Image, ImageFilter
import numpy as np
from turtlesim.msg import Pose
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill, Spawn
from math import sqrt, atan2, pi
import time
class DrawArtTurtleSim(Node):
    def __init__(self):
        super().__init__("turtle_sim_draw")
        self.subscription = self.create_subscription(
            Pose,
            '/turtlesim1/turtle1/pose',
            self.pose_callback,
            10)
        self.angle = 0.0
        self.distance = 0.0
        self.subscription  # prevent unused variable warning
        self.image_path = self.declare_parameter('image_path', 'default_value').get_parameter_value().string_value
        self.publisher = self.create_publisher(
            Twist,
            "/turtlesim1/turtle1/cmd_vel",
            10                                  
        )
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)

    def kill_and_spawn_turtlesim_bot(self, spawn_pose=[6.0,6.0]):
        kill_client = self.create_client(Kill, '/turtlesim1/kill')
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kill service not available, waiting again...')
        kill_request = Kill.Request()
        kill_request.name = 'turtle1'
        future = kill_client.call_async(kill_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Previous turtle killed successfully')
        else:
            self.get_logger().error('Failed to kill previous turtle')
        
        client = self.create_client(Spawn, '/turtlesim1/spawn')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = Spawn.Request()
        request.x = spawn_pose[0]
        request.y = spawn_pose[1]
        request.theta = 0.0
        request.name = 'turtle1'

        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Turtle spawned successfully')
        else:
            self.get_logger().error('Failed to spawn turtle')

    def publish_callback(self):
        self.get_logger().info("Inside Publisher")
        twist = Twist()
        twist.linear.x = min(1.0, 2*self.distance)
        twist.angular.z = min(1.0, max(-1.0, 3.0*self.angle))
        self.publisher.publish(twist)
        
    def pose_callback(self, msg):
        self.turtle_x_pose = msg.x
        self.turtle_y_pose = msg.y
        self.theta = msg.theta
        self.get_logger().info("Pose X : {}, Pose Y : {}, Theta : {}".format(self.turtle_x_pose, self.turtle_y_pose, self.theta))

    
    def read_image(self):
        self.get_logger().info("Image Path : {}".format(self.image_path))
        #image = Image.open(self.image_path)
        image = cv2.imread(self.image_path, 0)

        image = cv2.resize(image, (500, 500))
        # Detecting Edges on the Image using the argument ImageFilter.FIND_EDGES
        image = Image.fromarray(image)
        image = image.filter(ImageFilter.FIND_EDGES)
        image = image.filter(ImageFilter.MaxFilter(1))

        ret, thresh = cv2.threshold(np.array(image), 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        blank_image = np.zeros(image.size)
        approx_ = []
        for contour in contours:
            # Perform contour approximation
            epsilon = 0.001 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            approx_.append(approx)
        self.get_logger().info("Contours : {}, Hierarchy : {}".format(len(contours), len(hierarchy)))
        contour_list = []
        for contour in approx_:
            temp = []
            for cont in contour:
                temp.append([cont[0][0]*11/500, 11 - cont[0][1]*11/500])
            contour_list.append((temp))
        self.__contours = np.array(contour_list)
        #self.get_logger().info("Contours filtered: {}".format(self.__contours))
        
        #blank_image = cv2.drawContours(blank_image, contour_list, -1, (255, 255, 255), 3)
        cv2.imwrite("contours.jpg", blank_image)
        cv2.imwrite("Edges.jpg", np.array(image))

    def run(self):
        self.read_image()
        #while rclpy.ok() and cv2.getWindowProperty('Resized_Window', cv2.WND_PROP_VISIBLE) > 1:
            #rclpy.spin_once(self)
        #cv2.waitKey(0)

        """target_point = [5.5445, 8]
        dx = target_point[0] - self.turtle_x_pose
        dy = target_point[1] - self.turtle_y_pose
        self.distance = np.sqrt(dx**2 + dy**2)
        self.angle = np.arctan2(dy,dx) - self.theta
        self.get_logger().info("dX : {}, dY : {}, angle : {}".format(dx, dy, self.angle))
        """
        for contour in self.__contours:
            self.get_logger().info("Contours to be divided : {}".format(contour))
            contour = np.array(contour)
            spawn_point = contour[0]
            self.get_logger().info("SPAWN : {}".format(spawn_point))

            self.kill_and_spawn_turtlesim_bot(spawn_point)
            rclpy.spin_once(self)

            for cont in contour:
                point = cont
                self.get_logger().info("Contour Point : {}".format(point))

                dx = point[0] - self.turtle_x_pose
                dy = point[1] - self.turtle_y_pose
                self.distance = np.sqrt(dx**2 + dy**2)
                self.angle = np.arctan2(dy,dx) - self.theta
                self.get_logger().info("Pose X : {}, Pose Y : {}".format(self.turtle_x_pose, self.turtle_y_pose))
                self.get_logger().info("point x : {}, point y : {}".format(point[0], point[1]))
                    
                while abs(self.angle) > 0.005:
                    dx = point[0] - self.turtle_x_pose
                    dy = point[1] - self.turtle_y_pose
                    self.get_logger().info("dX : {}, dY : {}, angle : {}".format(dx, dy, self.angle))
                    """if(dx < 0 or dy < 0):
                        self.angle = self.theta - np.arctan2(dy,dx)
                    else:
                        self.angle = np.arctan2(dy,dx) - self.theta"""
                    self.angle = np.arctan2(dy,dx) - self.theta
                    
                
                    self.distance = 0.0
                    rclpy.spin_once(self)
                dx = point[0] - self.turtle_x_pose
                dy = point[1] - self.turtle_y_pose
                self.distance = np.sqrt(dx**2 + dy**2)

                while self.distance > 0.05:
                    self.get_logger().info("dX : {}, dY : {}, distance : {}".format(dx, dy, self.distance))
                    
                    dx = point[0] - self.turtle_x_pose
                    dy = point[1] - self.turtle_y_pose
                    self.distance = np.sqrt(dx**2 + dy**2)
                    self.angle = 0.0
                    rclpy.spin_once(self)     

                self.get_logger().info("Contour X : {}, Contour Y : {}".format(point[0], point[1]))

                self.get_logger().info("Angle : {}".format(self.angle))
                self.get_logger().info("Distance : {}".format(self.distance))




        self.destroy_node()



def main(args=None):
    rclpy.init(args=args)
    #node_handle = rclpy.create_node("turtle_sim_draw")
    node_handle = DrawArtTurtleSim()

    #image = cv2.imread(image_path)
    #cv2.namedWindow("Input", cv2.WINDOW_NORMAL)
    #cv2.resizeWindow("Input", 640, 480)
    node_handle.run()
    rclpy.shutdown()


if __file__ == "__main__":
    main()