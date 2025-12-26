#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from threading import Thread, Event
from flask import Flask, render_template
import os
import signal
import sys
import time

# ---------- ROS2 Node ----------
class WebLedNode(Node):
    def __init__(self):
        super().__init__("web_led_node")
        self.publisher = self.create_publisher(Bool, "/led_cmd", 10)
        self.get_logger().info("Web LED ROS2 node started")

        self.blink_event = Event()
        self.blink_thread = None

    def led_on(self):
        self.stop_blink()
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
        self.get_logger().info("LED ON")

    def led_off(self):
        self.stop_blink()
        msg = Bool()
        msg.data = False
        self.publisher.publish(msg)
        self.get_logger().info("LED OFF")

    def led_blink(self, interval=0.5):
        self.stop_blink()  # остановим предыдущий blink если был
        self.blink_event.clear()

        def _blink():
            self.get_logger().info("LED BLINK START")
            state = False
            while not self.blink_event.is_set():
                state = not state
                msg = Bool()
                msg.data = state
                self.publisher.publish(msg)
                time.sleep(interval)
            self.get_logger().info("LED BLINK STOP")

        self.blink_thread = Thread(target=_blink, daemon=True)
        self.blink_thread.start()

    def stop_blink(self):
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_event.set()
            self.blink_thread.join()
            self.blink_thread = None

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

    @app.route("/blink")
    def blink():
        node.led_blink(interval=0.5)  # интервал мигания 0.5 сек
        return "LED BLINK"

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
        ros_node.stop_blink()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    app.run(host="0.0.0.0", port=5000)

if __name__ == "__main__":
    main()
