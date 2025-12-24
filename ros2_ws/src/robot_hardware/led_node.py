#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio
import time

# Настройки GPIO
LED_PIN = 26
CHIP = 4

# -------------------------------
# Функция для безопасного захвата пина
# -------------------------------
def claim_gpio_safe(gpio, pin, retries=3, delay=0.5):
    for i in range(retries):
        try:
            lgpio.gpio_claim_output(gpio, pin)
            return True
        except lgpio.error as e:
            print(f"⚠️ GPIO busy, retry {i+1}/{retries}")
            time.sleep(delay)
    raise RuntimeError("❌ Cannot claim GPIO, still busy after retries")

# -------------------------------
# Инициализация GPIO
# -------------------------------
gpio = lgpio.gpiochip_open(CHIP)
claim_gpio_safe(gpio, LED_PIN)

# -------------------------------
# ROS 2 LED Node
# -------------------------------
class LedNode(Node):
    def __init__(self):
        super().__init__("led_node")
        self.subscription = self.create_subscription(
            Bool,
            '/led_cmd',
            self.led_callback,
            10
        )
        self.get_logger().info("LED node started, waiting for commands...")

    def led_callback(self, msg: Bool):
        state = 1 if msg.data else 0
        lgpio.gpio_write(gpio, LED_PIN, state)
        self.get_logger().info(f"LED {'ON' if state else 'OFF'}")

# -------------------------------
# Main
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Безопасно выключаем и закрываем GPIO
        lgpio.gpio_write(gpio, LED_PIN, 0)
        lgpio.gpio_close(gpio)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
