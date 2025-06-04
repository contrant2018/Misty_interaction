#!/usr/bin/env python3
import tempfile
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

class SendToMistyNode(Node):
    def __init__(self):
        super().__init__('send_to_misty_node')
        self.declare_parameter('misty_ip', '172.20.10.12')
        self.misty_ip = self.get_parameter('misty_ip').value

        self.create_subscription(String, '/tts_audio_path', self.on_mp3, 10)
        self.get_logger().info(f"SendToMisty ready — targeting Misty at {self.misty_ip}")

    def on_mp3(self, msg: String):
        mp3_path = Path(msg.data)
        upload_name = mp3_path.name

        try:
            # Upload the MP3 to Misty
            with open(mp3_path, "rb") as f:
                response = requests.post(
                    f"http://{self.misty_ip}/api/audio",
                    params={"filename": upload_name},
                    files={"file": (upload_name, f, "audio/mpeg")}
                )

            if response.status_code != 200:
                self.get_logger().warning(f"Upload failed: {response.status_code} — {response.text}")
            else:
                self.get_logger().info(f"Uploaded: {upload_name}")

                # Play the uploaded file
                response = requests.post(
                    f"http://{self.misty_ip}/api/audio/play",
                    json={"FileName": upload_name}
                )

                if response.status_code != 200:
                    self.get_logger().warning(f"Play failed: {response.status_code} — {response.text}")
                else:
                    self.get_logger().info(f"Playing: {upload_name}")

        except Exception as e:
            self.get_logger().error(f"REST error: {e}")

        # Clean up temp files
        if mp3_path.exists() and str(mp3_path).startswith(tempfile.gettempdir()):
            mp3_path.unlink(missing_ok=True)

def main(args=None):
    rclpy.init(args=args)
    node = SendToMistyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
