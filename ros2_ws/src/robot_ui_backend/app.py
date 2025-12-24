#!/usr/bin/env python3
from flask import Flask, render_template
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from threading import Thread

# ---------- ROS2 NODE ----------
class WebLedNode(Node):
    def __init__(self):
        super().__init__("web_led_node")
        self.publisher = self.create_publisher(Bool, "/led_cmd", 10)

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


rclpy.init()
ros_node = WebLedNode()
Thread(target=ros_spin, args=(ros_node,), daemon=True).start()

# ---------- FLASK ----------
app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/on")
def on():
    ros_node.led_on()
    return "ON"

@app.route("/off")
def off():
    ros_node.led_off()
    return "OFF"


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
