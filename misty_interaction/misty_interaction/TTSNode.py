#!/usr/bin/env python3
import os, tempfile, rclpy
from datetime import datetime
from pathlib import Path
from std_msgs.msg import String
from rclpy.node import Node
from TTS.api import TTS
from pydub import AudioSegment
import sys
import os
import ctypes

#this whole piece supresses the hardware unused warnings
def suppress_stderr():
    devnull = os.open(os.devnull, os.O_WRONLY)
    sys.stderr.flush()
    os.dup2(devnull, 2)  # redirect C stderr (fd 2) to /dev/null
    os.close(devnull)

suppress_stderr()

# Load the TTS model — this is where the spam happens
from TTS.api import TTS
tts_engine = TTS(model_name="tts_models/en/ljspeech/vits")

class TTSSNode(Node):
    def __init__(self):
        super().__init__('tts_synth_node')

        # Parameters
        self.declare_parameter('speaker_idx', 28)
        self.declare_parameter('tts_model', 'tts_models/en/ljspeech/vits')
        self.declare_parameter('vocoder_model', 'vocoder_models/en/ljspeech/hifigan_v2')
        default_cache = Path.home() / 'misty_ws' / 'misty_phrases'
        self.declare_parameter('cache_dir', str(default_cache))
        self.declare_parameter('cache_and_save', True)

        # Load params
        self.spk_idx = self.get_parameter('speaker_idx').value
        self.model_name = self.get_parameter('tts_model').value
        self.vocoder_name = self.get_parameter('vocoder_model').value
        self.cache_and_save = self.get_parameter('cache_and_save').value
        self.cache_dir = Path(self.get_parameter('cache_dir').value)
        if self.cache_and_save:
            self.cache_dir.mkdir(parents=True, exist_ok=True)

        # Load TTS model
        self.tts = TTS(model_name=self.model_name, vocoder_name=self.vocoder_name)

        # ROS setup
        self.create_subscription(String, '/interaction_text', self.on_text, 10)
        self.audio_pub = self.create_publisher(String, '/tts_audio_path', 10)

        self.get_logger().info(f"TTS Synth ready (model: {self.model_name})")

    def on_text(self, msg: String):

        #this waits for a string to be published if not keep spinning
        text = msg.data.strip()
        if not text:
            return
        
        #creating a timestamp for the mp3 file
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]

        #if save mode is on create the path
        if self.cache_and_save:
            mp3_path = self.cache_dir / f'speech_{stamp}.mp3'

        # if save mode on create a temporery file
        else:
            fd, tmp = tempfile.mkstemp(suffix='.mp3')
            os.close(fd)
            mp3_path = Path(tmp) #flexibility on path handling

        # creates the wave path
        wav_path = mp3_path.with_suffix('.wav')

        try:
            #use coqui-tts ti synthisize a audio file
            self.tts.tts_to_file(text=text, speaker_idx=self.spk_idx, file_path=str(wav_path))
            #converts the wav file to an mp3
            AudioSegment.from_wav(wav_path).export(mp3_path, format='mp3')
            wav_path.unlink(missing_ok=True)
            self.get_logger().info(f"Synthesized: '{text}' → {mp3_path.name}")
            #publishes our mp3 path
            self.audio_pub.publish(String(data=str(mp3_path)))

        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

            # if save mode of do not save throw out the mp3 file
        finally:
            if not self.cache_and_save:
                mp3_path.unlink(missing_ok=True)

def main(args=None):
    rclpy.init(args=args)
    node = TTSSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()