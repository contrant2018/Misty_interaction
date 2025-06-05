import rclpy, requests, tempfile, os
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
from datetime import datetime
from pathlib import Path

from misty_interaction_interfaces.srv import SaveMode

class GTTSNode(Node):
    """
    Converts /interaction_text -> MP3 and (optionally) caches each file.
    Always streams the audio to Misty via REST /api/audio/play.
    """
    def __init__(self):
        super().__init__('gtts_tts_node')


        # setting up a language perameter
        self.declare_parameter('language', 'en')
        self.lang = self.get_parameter('language').value

        # parameter for my service
        self.declare_parameter('cache_and_save', True)

        self.cache_and_save = self.get_parameter('cache_and_save').value

        # creating a path and if necessary creating a folder
        self.out_dir = Path.home() / "tts_audio"
        self.out_dir.mkdir(exist_ok=True)


        # creating our subscription, service, and logging it
        self.create_subscription(
            String,'/interaction_text',self.on_text,10)
        
        self.create_service(
            SaveMode, 'set_save_mode', self.on_save_mode_request)
        
        # audio publisher
        self.audio_pub = self.create_publisher(String, '/tts_audio_path', 10)


        self.get_logger().info(
            f"gTTS node ready. Saving audio to {self.out_dir} (cache={self.cache_and_save}).")
        
    def on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        # Pick target path

        #if were saving buld a filename and timestamp it
        if self.cache_and_save:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            mp3_path = self.out_dir / f"speech_{stamp}.mp3"

        # if not just create a temp file so we can play the audio
        else:
            fd, tmp = tempfile.mkstemp(suffix='.mp3')
            os.close(fd)
            mp3_path = Path(tmp)

        try:
            # initiate google text to speach with our format
            gTTS(text=text, lang=self.lang).save(str(mp3_path))
            self.get_logger().info(f'Spoke: "{text}" → {mp3_path.name}')

            #publishes audio path
            self.audio_pub.publish(String(data=str(mp3_path)))


        except Exception as e:
            self.get_logger().error(f"gTTS error: {e}")
        finally:
            # clean up if one-shot
            if not self.cache_and_save and mp3_path.exists():
                mp3_path.unlink(missing_ok=True)        

    #our service callback to turn on/off the mp3 save mode
    def on_save_mode_request(self, req, resp):

        self.cache_and_save = bool(req.cache_and_save)

        mode = "cache+save" if self.cache_and_save else "one-shot"

        self.get_logger().info(f"Save mode switched to: {mode}")
        resp.accepted = True

        return resp


def main(args=None):

    rclpy.init(args=args)
    node = GTTSNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()