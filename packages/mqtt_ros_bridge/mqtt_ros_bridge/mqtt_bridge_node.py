#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

import paho.mqtt.client as mqtt

MQTT_BROKER_IP = "192.168.211.44"
MQTT_SENSOR_TOPIC = "node/sensor"   
MQTT_LED_TOPIC = "node/led"


class MqttRosBridge(Node):
    def __init__(self):
        super().__init__('mqtt_ros_bridge')

        # ROS Publishers
        self.sensor_pub = self.create_publisher(Float32, 'sensor_value', 10)

        # ROS Subscribers
        self.led_sub = self.create_subscription(Int32, 'led_cmd', self.ros_to_mqtt, 10)

        # MQTT Client Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        self.mqtt_client.connect(MQTT_BROKER_IP, 1883, 60)
        self.mqtt_client.loop_start()

        self.get_logger().info("MQTT <-> ROS2 Bridge Started")

    # MQTT connected
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker")
        client.subscribe(MQTT_SENSOR_TOPIC)

    # Incoming MQTT message -> publish to ROS2
    def on_mqtt_message(self, client, userdata, msg):
        try:
            value = float(msg.payload.decode())
            ros_msg = Float32()
            ros_msg.data = value
            self.sensor_pub.publish(ros_msg)
        except Exception as e:
            self.get_logger().error(f"Error parsing MQTT message: {e}")

    # ROS2 message -> publish to MQTT
    def ros_to_mqtt(self, msg):
        self.mqtt_client.publish(MQTT_LED_TOPIC, str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = MqttRosBridge()
    rclpy.spin(node)
    node.mqtt_client.loop_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
