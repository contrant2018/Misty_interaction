#!/usr/bin/env python3

import queue, threading, rclpy, pyaudio
from rclpy.node import Node
from audio_common_msgs.msg import AudioData    

class MicInputNode(Node):
    
    def __init__(self):
        super().__init__('mic_input_node')

        #  parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size',  4000)
        self.rate  = int(self.get_parameter('sample_rate').value)
        self.chunk = int(self.get_parameter('chunk_size').value)

        #  publisher
        self.pub = self.create_publisher(AudioData, '/mic_audio', 10)

        #  queue + background capture thread
        self._q = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        #  timer to flush queue into ROS messages
        self.create_timer(0.02, self._publish_tick)   # 50 Hz tick

        self.get_logger().info(
            f"MicInputNode started  rate={self.rate}Hz  chunk={self.chunk}B"
        )

    # ------------------------------------------------------------
    def _capture_loop(self):
        pa = pyaudio.PyAudio()
        stream = pa.open(format=pyaudio.paInt16,
                         channels=1,
                         rate=self.rate,
                         input=True,
                         frames_per_buffer=self.chunk)

        while not self._stop.is_set():
            data = stream.read(self.chunk, exception_on_overflow=False)
            self._q.put(data)

        stream.stop_stream()
        stream.close()
        pa.terminate()

    # ------------------------------------------------------------
    def _publish_tick(self):
        while not self._q.empty():
            raw = self._q.get_nowait()

            msg = AudioData()
            msg.data = raw                      # ← uint8[] in the message
            # we piggy-back the sample_rate in the header’s frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = str(self.rate)

            self.pub.publish(msg)

    # ------------------------------------------------------------
    def destroy_node(self):
        self._stop.set()
        super().destroy_node()

# ----------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MicInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
