#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio
import time
from threading import Thread, Event

LED_PIN = 26
CHIP = 4

def claim_gpio_safe(gpio, pin, retries=5, delay=0.5):
    for i in range(retries):
        try:
            lgpio.gpio_claim_output(gpio, pin)
            return True
        except lgpio.error:
            print(f"⚠️ GPIO busy, retry {i+1}/{retries}")
            time.sleep(delay)
    raise RuntimeError(f"❌ Cannot claim GPIO {pin} on chip {CHIP}")

gpio = lgpio.gpiochip_open(CHIP)
claim_gpio_safe(gpio, LED_PIN)

class LedNode(Node):
    def __init__(self):
        super().__init__("led_node")
        self.subscription = self.create_subscription(Bool, '/led_cmd', self.led_callback, 10)
        self.blink_thread = None
        self.blink_stop_event = Event()
        self.get_logger().info("LED node started, waiting for commands...")

    def led_callback(self, msg: Bool):
        if msg.data:
            self.stop_blink()
            lgpio.gpio_write(gpio, LED_PIN, 1)
            self.get_logger().info("LED ON")
        else:
            self.stop_blink()
            lgpio.gpio_write(gpio, LED_PIN, 0)
            self.get_logger().info("LED OFF")

    def start_blink(self, interval=0.5):
        self.stop_blink()
        self.blink_stop_event.clear()
        self.blink_thread = Thread(target=self._blink, args=(interval,), daemon=True)
        self.blink_thread.start()
        self.get_logger().info("LED BLINK mode started")

    def _blink(self, interval):
        while not self.blink_stop_event.is_set():
            lgpio.gpio_write(gpio, LED_PIN, 1)
            time.sleep(interval)
            lgpio.gpio_write(gpio, LED_PIN, 0)
            time.sleep(interval)

    def stop_blink(self):
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_stop_event.set()
            self.blink_thread.join()
            self.blink_thread = None
            lgpio.gpio_write(gpio, LED_PIN, 0)
            self.get_logger().info("LED BLINK mode stopped")

def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_blink()
        lgpio.gpio_write(gpio, LED_PIN, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
