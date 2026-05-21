#!/usr/bin/env python3

import time
import re
import onnxruntime as ort
from transformers import AutoTokenizer
 
import numpy as np

import rclpy
import rclpy.node

import hbba_lite

from perception_msgs.msg import Transcript
from audio_utils_msgs.msg import CompleteUtterance, VoiceActivity


EOU_TOKENIZER_MAX_LENGHT = 256
EOU_MODEL_ID = "latishab/turnsense"
EOU_MODEL_PATH = "src0/t-top/ros/utils/audio_utils/audio_utils/models/turnsense/model_quantized.onnx"
OPTIMIZED_MODEL_PATH = "src0/t-top/ros/utils/audio_utils/audio_utils/models/turnsense/optimized_model_quantized.onnx"

#Inspired by https://github.com/latishab/turnsense
class EoUNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('end_of_utterance_node')

        self._utterance_max_delay = self.declare_parameter('utterance_max_wait', 4.0).get_parameter_value().double_value

        self.tokenizer = AutoTokenizer.from_pretrained(EOU_MODEL_ID)
        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_EXTENDED
        options.optimized_model_filepath = OPTIMIZED_MODEL_PATH
        self.session = ort.InferenceSession(EOU_MODEL_PATH, sess_options=options, providers=["CPUExecutionProvider"])

        self._text_buffer = ""

        self._complete_utterance_pub = self.create_publisher(CompleteUtterance, 'utterance', 10)

        self._hbba_filter_state = hbba_lite.OnOffHbbaFilterState(self, 'eou_filter')
        self._stt_sub = self.create_subscription(Transcript, 'transcript', self._stt_cb, 10)
        self._voice_activity_pub = self.create_subscription(VoiceActivity, 'voice_activity', self._voice_activity_cb, 10)
        self._utterance_delay_timer = None
        self._is_voice = False


    def _stt_cb(self, msg):
        if self._hbba_filter_state.is_filtering_all_messages:
            # Clear text buffer.
            self._text_buffer = ""
        else:
            start = time.perf_counter()
            stt_out = msg.text

            # Only look at last sentence in block of text when checking eou.
            match  = re.search(r'[^.!?\n]+(?:[.]{3}|\dots|[.!?])\s*$', stt_out)
            last_sentence = match.group(0).strip() if match else stt_out
            
            inputs = self.tokenizer(
            f"<|user|> {last_sentence}",
            padding="max_length",
            max_length=EOU_TOKENIZER_MAX_LENGHT,
            return_tensors="np"
            )
            
            # Run inference
            ort_inputs = {
                'input_ids': inputs['input_ids'],
                'attention_mask': inputs['attention_mask']
            }
            all_logits = self.session.run(None, ort_inputs)[0]
            logits_for_item = all_logits[0]
            prediction = np.argmax(logits_for_item)

            self._text_buffer += stt_out

            end = time.perf_counter()

            if prediction == 1:
                out_msg = CompleteUtterance()
                out_msg.data = self._text_buffer
                self._complete_utterance_pub.publish(out_msg)
                
                # Clear the text buffer
                self._text_buffer = ""
            else:
                self._text_buffer += " "

                if self._utterance_delay_timer is None:
                    self._utterance_delay_timer = self.create_timer(self._utterance_max_delay, self._on_timer_expired)

            prediction_time_sec = end-start
            self.get_logger().info(f"pred: {'Done' if prediction else 'Still going'}; pred time: {prediction_time_sec:.6f}")


    def _voice_activity_cb(self, msg):
        last_is_voice = self._is_voice
        self._is_voice = msg.is_voice

        if (not last_is_voice and self._is_voice ) and self._utterance_delay_timer is not None:
            # Cancel and delete the timer
            self._utterance_delay_timer.cancel()
            self.destroy_timer(self._utterance_delay_timer)
            self._utterance_delay_timer = None
            

    def _on_timer_expired(self):
        out_msg = CompleteUtterance()
        out_msg.data = self._text_buffer
        self._complete_utterance_pub.publish(out_msg)
        
        # Clear the text buffer
        self._text_buffer = ""

        # Cancel and delete the timer
        self._utterance_delay_timer.cancel()
        self.destroy_timer(self._utterance_delay_timer)
        self._utterance_delay_timer = None


    def run(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    eou_node = EoUNode()

    try:
        eou_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        eou_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
