#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from threading import Thread
from flask import Flask, render_template
import os
import signal
import sys

# ---------- ROS2 Node ----------
class WebLedNode(Node):
    def __init__(self):
        super().__init__("web_led_node")
        self.publisher = self.create_publisher(Bool, "/led_cmd", 10)
        self.get_logger().info("Web LED ROS2 node started")

    def led_on(self):
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
        self.get_logger().info("LED ON")

    def led_off(self):
        msg = Bool()
        msg.data = False
        self.publisher.publish(msg)
        self.get_logger().info("LED OFF")

def ros_spin(node):
    rclpy.spin(node)

# ---------- Flask ----------
def create_flask_app(node: WebLedNode):
    template_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "templates")
    app = Flask(__name__, template_folder=template_dir)

    @app.route("/")
    def index():
        return render_template("index.html")

    @app.route("/on")
    def on():
        node.led_on()
        return "LED ON"

    @app.route("/off")
    def off():
        node.led_off()
        return "LED OFF"

    return app

# ---------- Main ----------
def main():
    rclpy.init()
    ros_node = WebLedNode()
    ros_thread = Thread(target=ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app = create_flask_app(ros_node)

    def shutdown_handler(sig, frame):
        print("Shutting down Flask and ROS2...")
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    app.run(host="0.0.0.0", port=5000)

if __name__ == "__main__":
    main()
