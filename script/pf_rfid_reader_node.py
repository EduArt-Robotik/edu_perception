#!/usr/bin/env python3
"""
Reads RFID tags via IC-KP2-2HB40-2V1D module connected over ethernet.
"""
import rclpy
from rclpy.node import Node, SetParametersResult, get_logger, Publisher
from std_msgs.msg import String

from dataclasses import dataclass

import requests

@dataclass
class Parameter:
  ip_address: str = "192.168.42.198"
  user: str = "admin"
  password: str = "a327-3e9e-dc44-33d4-22b8-77b6"

class PfRfidReaderNode(Node):
  def __init__(self):
    super().__init__('pf_rfid_reader_node')

    # gather parameters from ROS2 parameter server
    self.parameter = self.get_parameters(Parameter())

    # bring up REST API connection
    self.authenticate()

    # starting timer for periodic RFID tag reading
    self.pub_rfid_tag = self.create_publisher(String, 'rfid_tag', 10)
    self.timer = self.create_timer(1.0, self.read_rfid_tag)


  def authenticate(self):
    self.session = requests.Session()

    ## authenticate with the RFID reader REST API
    response = self.session.post(
      f"http://{self.parameter.ip_address}/api/auth/login",
      json={
        "username": self.parameter.user,
        "password": self.parameter.password
      },
      timeout=10
    )
    response.raise_for_status()
    self.token = response.json()['token']
    self.session.headers.update({'Authorization': f'Bearer {self.token}'})

    ## configure channel 1 for reading RFID tags
    response = self.session.put(
      f"http://{self.parameter.ip_address}/api/rfid/tag/tasks/1",
      json={
        "id": 1,
        "configuration": {
          "mode": "READ",
          "execution_type": "SINGLE",
          "memory": {
            "bank": "UID",
            "byte_address": 0,
            "byte_length": 4,
          },
        },
      },
      timeout=10,
    )
    response.raise_for_status()


  def read_rfid_tag(self):
    self.get_logger().info("reading RFID tag...")

    try:
      response = self.session.post(
        f"http://{self.parameter.ip_address}/api/rfid/tag/tasks/1/start",
        timeout=10,
      )
      response.raise_for_status()

      tags = response.json()["data"]
      self.get_logger().info(f"tags: {tags}")

      # publish each tag via ROS2 topic
      for tag in tags:
        msg = String()
        msg.data = tag["UII"]
        self.pub_rfid_tag.publish(msg)


    except Exception as e:
      self.get_logger().error(f"failed to read RFID tag: {e}")
      # re-authenticate, most likely due to expired token
      self.authenticate()
      return


  def get_parameters(self, default_parameter: Parameter) -> Parameter:
    self.get_logger().info("reading parameters...")

    self.declare_parameter('ip_address', default_parameter.ip_address)
    self.declare_parameter('user', default_parameter.user)
    self.declare_parameter('password', default_parameter.password)

    parameter = Parameter()

    parameter.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
    parameter.user = self.get_parameter('user').get_parameter_value().string_value
    parameter.password = self.get_parameter('password').get_parameter_value().string_value

    return parameter


def main(args=None):
  rclpy.init(args=args)
  node = PfRfidReaderNode()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
