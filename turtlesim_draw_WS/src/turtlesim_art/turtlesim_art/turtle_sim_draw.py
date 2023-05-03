import rclpy
import cv2
from PIL import Image, ImageFilter
import numpy as np
from turtlesim.msg import Pose
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill, Spawn
from matplotlib import pyplot as plt

PI = 3.14

class DrawArtTurtleSim(Node):
    def __init__(self):
        super().__init__("turtle_sim_draw")
        self.subscription = self.create_subscription(
            Pose,
            '/turtlesim1/turtle1/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.image_path = self.declare_parameter('image_path', 'default_value').get_parameter_value().string_value
        """self.publisher = self.create_publisher(
            Twist,
            "/turtlesim1/turtle1/cmd_vel",
            10                                  
        )
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.draw_shape)"""

    def kill_and_spawn_turtlesim_bot(self):
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
        request.x = 6.0
        request.y = 5.0
        request.theta = -PI
        request.name = 'my_turtle'

        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Turtle spawned successfully')
        else:
            self.get_logger().error('Failed to spawn turtle')

    def draw_shape(self):
        self.kill_and_spawn_turtlesim_bot()
        

    def pose_callback(self, msg):
        self.turtle_x_pose = msg.x
        self.turtle_y_pose = msg.y
        self.theta = msg.theta
        self.get_logger().info("Pose X : {}, Pose Y : {}".format(self.turtle_x_pose, self.turtle_y_pose))

    
    def read_image(self):
        self.get_logger().info("Image Path : {}".format(self.image_path))
        #image = Image.open(self.image_path)
        image = cv2.imread(self.image_path, 0)
        self.get_logger().info(" Image : {}".format(image))
        image = cv2.resize(image, (500, 500))
        # Detecting Edges on the Image using the argument ImageFilter.FIND_EDGES
        image = Image.fromarray(image)
        image = image.filter(ImageFilter.FIND_EDGES)
        image = image.filter(ImageFilter.MaxFilter(3))

        ret, thresh = cv2.threshold(np.array(image), 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().info("Contours : {}, Hierarchy : {}".format(len(contours), len(hierarchy)))
        
        cv2.imwrite("Edges.jpg", np.array(image))

    def run(self):
        self.read_image()
        self.draw_shape()
        #while rclpy.ok() and cv2.getWindowProperty('Resized_Window', cv2.WND_PROP_VISIBLE) > 1:
            #rclpy.spin_once(self)
        #cv2.waitKey(0)
        rclpy.spin(self)
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