from flask import Flask
from flask import Flask, request, send_from_directory,Response
import graphviz as gz
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread
import atexit
import sys


generated_dot= """
digraph G {
    rankdir=LR
    base [shape=rectangle style="filled,rounded" fillcolor=red label="No topics received"];
}
"""

def close_running_threads():
    rclpy.shutdown()
    print("closed ROS")
    sys.exit(0)


class GzSubscriber(Node):
    def __init__(self):
        super().__init__("web_server")
        self.subscription = self.create_subscription(String,'/gz',self.listener_callback,10)

    def listener_callback(self,msg):
        self.get_logger().info("received graphviz string")
        global generated_dot
        generated_dot = msg.data

rclpy.init(args=None)
node = GzSubscriber()


def thread_main():
    print("I am started")
    executor = MultiThreadedExecutor()
    executor.add_node(node)    
    executor.spin()
    
    node.destroy()
    rclpy.shutdown()
    print("I am finished")


Thread(target=thread_main).start()






app = Flask(__name__,static_url_path="")



#
# Serving static files:
#

@app.route("/static/<path:path>")
def send_static(path):
    return send_from_directory("static",path)

@app.route("/")
def static_index():
    d= gz.Source(source=generated_dot)
    return Response(d.pipe(format='svg'),mimetype="image/svg+xml",headers={'Refresh': '0.25;url=sm.svg'})
    

@app.route("/sm.svg")
def generate_svg():
    d= gz.Source(source=generated_dot)
    #return Response(Markup(d.pipe(format='svg').decode())
    return Response(d.pipe(format='svg').decode(),mimetype="image/svg+xml",headers={'Refresh': '0.25;url=sm.svg'})


@app.route("/sm.png")
def generate_png():
    d= gz.Source(source=generated_dot)
    return Response(d.pipe(format='png'),mimetype="image/png",headers={'Refresh': '1;url=sm.png'})


def main():
    atexit.register(close_running_threads) 
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = -1
    app.run(debug = True, port = 5000)

if __name__ == "__main__":
    main()
