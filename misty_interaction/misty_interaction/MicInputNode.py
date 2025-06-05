#!/usr/bin/env python3

import queue, threading, rclpy, pyaudio
from rclpy.node import Node
from misty_interaction_interfaces.msg import AudioData    

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
        # this lets us keep running while we wait for a mic input
        self._q = queue.Queue() 
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        #  timer to flush queue into ROS messages
        self.create_timer(0.02, self._publish_tick)   # 50 Hz tick

        self.get_logger().info(
            f"MicInputNode started  rate={self.rate}Hz  chunk={self.chunk}B"
        )


    def _capture_loop(self):

        #using pyaidio to open up a mic stream so we can listen for an input
        pa = pyaudio.PyAudio()

        #setting up the stream as 16bit signd integer where each chunk is 2 bytes
        stream = pa.open(format=pyaudio.paInt16,
                         channels=1, #mano auidio
                         rate=self.rate, #sample rate
                         input=True,
                         frames_per_buffer=self.chunk) # amount of audio data comming in

        while not self._stop.is_set():
            #adding our chunk into the que to send to our publisher
            data = stream.read(self.chunk, exception_on_overflow=False)
            self._q.put(data)

        stream.stop_stream()
        stream.close()
        pa.terminate()

    def _publish_tick(self):
        while not self._q.empty():
            #pull whats next in our que (get_nowait lets us grab the next thing without stopping)
            raw = self._q.get_nowait()

            # create a msg using our custom interface this contains: 
            # chunk of audio data, a time stamp, and a string for our rate
            msg = AudioData()
            msg.data = raw 
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = str(self.rate)

            self.pub.publish(msg)
            ### comment out one other nodes working
            self.get_logger().info(f"Publishing audio chunk of size {len(raw)} bytes")

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
