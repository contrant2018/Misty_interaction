#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from misty_interaction_interfaces.srv import SubmitText

import threading
from queue import Queue, Empty         

class TextInputNode(Node):
    def __init__(self):
        super().__init__('text_input_node')

        # publishers / service
        self.pub_ = self.create_publisher(String, '/interaction_text', 10)
        self.srv_ = self.create_service(SubmitText, 
                                    'submit', self.handle_submit_request)

        # threading
        # safe que for multiple threads
        self._queue = Queue()

        # building but not running the thread and making kill it if the program ends
        self._cli_thread = threading.Thread(
            target=self._blocking_cli, daemon=True)
        self._cli_thread.start()

        # timer to clear our que
        self.create_timer(0.1, self.flush_queue)

        self.get_logger().info(
            'Type a phrase (or "stop") in the same terminal…')

    # our service callback
    def handle_submit_request(self, req, resp):
        self._publish(req.text)
        resp.accepted = True
        return resp

    # CLI worker thread: allows interaction with the console directly 
    # without having to run a command every time
    def _blocking_cli(self):

        # while loop to wait for a response
        while True:
            # catches any errors or shutdowns
            try:
                text = input('> ')
            except (EOFError, KeyboardInterrupt):
                self._queue.put('__quit__')
                break
            
            # makes everything lower cas and looks for these key words/
            # phrases and shuts down if they match
            if text.strip().lower() in ('stop', 'quit', 'exit', 'no mas'):
                self._queue.put('__quit__')
                break
            self._queue.put(text)

    # timer callback
    def flush_queue(self):
        try:
             # empty the queue each tick pop and publish each thing in the que
            while True:
                item = self._queue.get_nowait()
                
                # if we quit shutdown
                if item == '__quit__':
                    rclpy.shutdown()
                    return
                self.publish(item)
        except Empty:
            pass

    # helper
    def publish(self, text: str):
        msg = String()
        msg.data = text
        self.pub_.publish(msg)
        self.get_logger().info(f'→ Sent: "{text}"')


# --------------------------- boilerplate ----------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TextInputNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
