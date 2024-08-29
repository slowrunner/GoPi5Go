#!/usr/bin/env python3

# FILE: say_svc_client.py


"""
    Example of calling the say_node service to speak a phrase with Piper-TextToSpeech


    Uses dave_interfaces.srv Say.srv:
        string saystring
        ---
        bool spoken


"""

import sys

from dave_interfaces.srv import Say
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Say, 'say')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Say.Request()

    def send_request(self, phrase):
        self.req.saystring = phrase
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    phrase = "Say service client used the say node to speak this phrase"
    future = minimal_client.send_request(phrase)
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Service Call Result for say("{}") = spoken: {}'.format(phrase,response.spoken))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
