import rclpy
from rclpy.node import Node
import sys
import rclpy.time
from etasl_interfaces.msg import Output 
from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args

from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        timer_period = 0.25
        #self.timer = self.create_timer(timer_period, self.callback)
        self.sub = self.create_subscription(Output,"/my_topic",self.msg_cb,10)
        self.count = 0
    def callback(self):
        pass
        #if self.count == 10:
        #    #self.destroy_timer(self.timer)
        #    self.destroy_subscription(self.su

    def msg_cb(self,msg):
        if self.count==0:
            print(", ".join([f'{e:23}' for e in ['"'+n+'"' for n in msg.names]]))
            self.count += 1
        print(", ".join( [f'{e:23.16e}' for e in msg.data] ))
        print(msg.is_declared)





def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()



# # main
# def main(args=None):
#     print("enter main")
#     rclpy.init(args=args)
#     node = MyNode()
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)    
#     executor.spin()


if __name__ == "__main__":
     sys.exit(main(sys.argv))


