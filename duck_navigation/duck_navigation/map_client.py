import sys
import time
import rclpy
from rclpy.node import Node
import os
from nav2_msgs.srv import LoadMap

from ament_index_python.packages import get_package_share_directory
map_file = os.path.join(get_package_share_directory('duck_navigation'), 'config', 'obs_map.yaml')

class Service(Node):
    def __init__(self):
        super().__init__("my_map")
        self.client = self.create_client(LoadMap, '/map_server/load_map')
        self.map_request = LoadMap.Request()
        while not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for service')

        self.send_request()

    def send_request(self):
        while True:
            self.map_request.map_url = map_file
            wait = self.client.call_async(self.map_request)
            rclpy.spin_until_future_complete(self, wait)
            if wait.result() is not None:
                self.get_logger().info('Request was responded')
            else:
                self.get_logger().info('Request Failed')

            # Delay for 5 seconds between consecutive service calls
            time.sleep(1.5)

def main(args=None):
    rclpy.init(args=args)

    map_client = Service()
    # map_client.send_request()  # Remove this line

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(map_client)  # Use spin_once instead of spin
        # rclpy.spin(map_client)  # Remove this line

    map_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
