#!/usr/bin/env python3
import os, tempfile, requests, rclpy
from datetime import datetime
from pathlib import Path
from std_msgs.msg import String
from rclpy.node import Node
from TTS.api import TTS                # Coqui-TTS
from misty_interaction_interfaces.srv import SaveMode
from pydub import AudioSegment

class TTSNode(Node):
    """
    Subscribes to /interaction_text, synthesises speech via Coqui-TTS,
    optionally caches MP3s in a workspace folder, and streams audio to Misty.
    """

    def __init__(self):
        super().__init__('coqui_tts_node')

        # declaring parameters
        self.declare_parameter('misty_ip', '192.168.1.100')
        self.declare_parameter('tts_model', 'tts_models/en/ljspeech/vits')
        self.declare_parameter('vocoder_model', 'vocoder_models/en/ljspeech/hifigan_v2')

        #this parameter to change voices
        self.declare_parameter('speaker_idx', 28)
        self.declare_parameter('cache_and_save', True)
        default_cache = Path.home() / 'misty_ws' / 'misty_phrases'
        self.declare_parameter('cache_dir', str(default_cache))

        # setting them
        self.misty_ip       = self.get_parameter('misty_ip').value
        self.model_name     = self.get_parameter('tts_model').value
        self.vocoder_name   = self.get_parameter('vocoder_model').value
        self.spk_idx        = self.get_parameter('speaker_idx').value
        self.cache_and_save = self.get_parameter('cache_and_save').value
        self.cache_dir      = Path(self.get_parameter('cache_dir').value)

        # does a cache folder exists, if not make one
        if self.cache_and_save:
            self.cache_dir.mkdir(parents=True, exist_ok=True)

        # Load TTS model
        self.tts = TTS(model_name=self.model_name,
                       vocoder_name=self.vocoder_name)

        # Subscriptions and services
        self.create_subscription(String, '/interaction_text', self.on_text, 10)
        self.create_service(SaveMode, 'set_save_mode', self.on_save_mode_request)

        self.get_logger().info(
            f"TTS ready (model='{self.model_name}', vocoder='{self.vocoder_name}', speaker={self.spk_idx}) — "
            f"cache_dir={self.cache_dir} (cache={self.cache_and_save})"
        )

    def on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        # Choose MP3 path based on caching mode (chat helped alot with this)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]

        # check if we are saving
        if self.cache_and_save:
            mp3_path = self.cache_dir / f'speech_{stamp}.mp3'

        #if were not just create a temp file we can send to misty
        else:
            fd, tmp = tempfile.mkstemp(suffix='.mp3')
            os.close(fd)
            mp3_path = Path(tmp)

        # mp3 to wav file
        wav_path = mp3_path.with_suffix('.wav')


        try:
            # Synthesise directly to WAV file
            if self.spk_idx is not None:
                self.tts.tts_to_file(
                    text=text,
                    speaker_idx=int(self.spk_idx),
                    file_path=str(wav_path)
                )
            else:
                self.tts.tts_to_file(
                    text=text,
                    file_path=str(wav_path)
                )

            # Convert WAV -> MP3
            AudioSegment.from_wav(wav_path).export(mp3_path, format='mp3')

            self.get_logger().info(f'Spoke: "{text}" → {mp3_path.name}')
            self._send_to_misty(mp3_path)

        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')

        finally:
            # Cleanup
            wav_path.unlink(missing_ok=True)
            if not self.cache_and_save:
                mp3_path.unlink(missing_ok=True)

    def on_save_mode_request(self, req, resp):

        self.cache_and_save = bool(req.cache_and_save)
        if self.cache_and_save:
            self.cache_dir.mkdir(parents=True, exist_ok=True)

        mode = 'cache+save' if self.cache_and_save else 'one-shot'
        self.get_logger().info(f'Save mode switched to: {mode}')
        resp.accepted = True
        return resp

    def _send_to_misty(self, mp3_path: Path):
        url = f'http://{self.misty_ip}/api/audio/play'
        try:
            with open(mp3_path, 'rb') as f:
                files = {'file': (mp3_path.name, f, 'audio/mpeg')}
                r = requests.post(url, files=files, timeout=6)
            if r.ok:
                self.get_logger().debug(f'Sent {mp3_path.name} ({r.status_code})')
            else:
                self.get_logger().warning(f'Misty responded {r.status_code}: {r.text}')
        except Exception as e:
            self.get_logger().error(f'HTTP error sending audio: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
