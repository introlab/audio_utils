#!/usr/bin/env python3

import queue
from collections import deque
import threading
import pdb

import time
import re

import onnxruntime as ort
from transformers import WhisperFeatureExtractor
 
import numpy as np

import rclpy
import rclpy.node

import hbba_lite

from audio_utils_msgs.msg import CompleteUtterance, AudioFrame, VoiceActivity

EOU_TOKENIZER_MAX_LENGHT = 256
EOU_MODEL_PATH = "dev_testing/smart-turn/smart-turn-v3.2-cpu.onnx"

SUPPORTED_CHANNEL_COUNT = 1
SUPPORTED_SAMPLE_RATE_HZ = 16000
REQUIRED_SAMPLE_LENGTH_SEC = 8


class EoUNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('end_of_utterance_node')

        self._utterance_max_delay = self.declare_parameter('utterance_max_wait', 4.0).get_parameter_value().double_value
        self._eou_detection_threshold = self.declare_parameter('eou_detection_threshold', 0.5).get_parameter_value().double_value

        self._feature_extractor = WhisperFeatureExtractor(chunk_length=8)
        so = ort.SessionOptions()
        so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        so.inter_op_num_threads = 1
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._session = ort.InferenceSession(EOU_MODEL_PATH, sess_options=so)

        self._hbba_filter_state = hbba_lite.OnOffHbbaFilterState(self, 'eou_filter')
        self._semantic_analysis_pub = self.create_publisher(CompleteUtterance, 'semantic_analysis', 10)
        
        self._audio_sub = self.create_subscription(AudioFrame, 'audio_in', self._audio_callback, 10)
        self._buffer_lock = threading.Lock()
        self._buffer = deque([0]*(REQUIRED_SAMPLE_LENGTH_SEC*SUPPORTED_SAMPLE_RATE_HZ),
                                  maxlen=(REQUIRED_SAMPLE_LENGTH_SEC*SUPPORTED_SAMPLE_RATE_HZ))
        self._detection_queue = queue.Queue(maxsize=1)
                
        self._voice_activity_sub = self.create_subscription(VoiceActivity, 'voice_activity', self._voice_activity_cb, 10)
        self._is_voice = False
        self._prev_is_voice = False

        self._timer_lock = threading.Lock()
        self._utterance_delay_timer = None
        # --- Utterance detection queue (maxsize=1 for drop/freshness policy) ---
        self._detection_event = threading.Event()
        self.snapshot = None
        self.snapshot_lock = threading.Lock()

        # --- Worker thread ---
        self._worker = threading.Thread(target=self._detection_worker, daemon=True)
        self._worker.start()


    def _audio_callback(self, msg):
        if self._hbba_filter_state.is_filtering_all_messages:
            pass
        elif msg.channel_count != SUPPORTED_CHANNEL_COUNT or msg.sampling_frequency != SUPPORTED_SAMPLE_RATE_HZ:
            self.get_logger().error('Invalid audio frame (msg.channel_count={}, msg.sampling_frequency={}})'
                         .format(msg.channel_count, msg.sampling_frequency))
        else:
            frames = np.frombuffer(msg.data, dtype=np.int16)
            with self._buffer_lock:
                self._buffer.extend(frames.flatten())


    def _voice_activity_cb(self, msg):
        self._prev_is_voice = self._is_voice
        self._is_voice = msg.is_voice
        rising_edge = not self._prev_is_voice and self._is_voice
        falling_edge = self._prev_is_voice and not self._is_voice

        # Clear the timer on rising edge detection for VAD
        if rising_edge:
            with self._timer_lock:
                # Cancel and delete the timer
                if self._utterance_delay_timer is not None:
                    self._utterance_delay_timer.cancel()
                    self.destroy_timer(self._utterance_delay_timer)
                    self._utterance_delay_timer = None
        elif falling_edge and not self._hbba_filter_state.is_filtering_all_messages:
            self._detection_event.set()


    def _detection_worker(self):
        while rclpy.ok():
            triggered = self._detection_event.wait(timeout=1.0)

            if triggered:
                self._detection_event.clear()
                with self._buffer_lock:
                    audio_snapshot = np.array(self._buffer)

                model_prediction = self._run_detection_model(audio_snapshot)

                with self._timer_lock:
                    if not model_prediction and self._utterance_delay_timer is None:
                        self._utterance_delay_timer = self.create_timer(self._utterance_max_delay, self._on_timer_expired)

                msg = CompleteUtterance()
                msg.sentence_complete = model_prediction
                self._semantic_analysis_pub.publish(msg)


    def _run_detection_model(self, audio_in: np.array) -> bool:
        if not self._hbba_filter_state.is_filtering_all_messages:
            start = time.perf_counter()
            inputs = self._feature_extractor(
                audio_in,
                sampling_rate=SUPPORTED_SAMPLE_RATE_HZ,
                return_tensors="np",
                padding="max_length",
                max_length=REQUIRED_SAMPLE_LENGTH_SEC * SUPPORTED_SAMPLE_RATE_HZ,
                truncation=True,
                do_normalize=True,
            )
            
            # Run inference
            model_inputs = inputs.input_features.squeeze(0).astype(np.float32)
            model_inputs = np.expand_dims(model_inputs, axis=0)
            model_outputs = self._session.run(None, {"input_features": model_inputs})

            probability = model_outputs[0][0].item()
            prediction = True if probability > self._eou_detection_threshold else False

            end = time.perf_counter()
            prediction_time_sec = end-start
            self.get_logger().info(f"pred: {'Done talking' if prediction else 'Still going'}; pred time: {prediction_time_sec:.6f}")
            return prediction

    def _on_timer_expired(self):
        msg = CompleteUtterance()
        msg.sentence_complete = True
        self._semantic_analysis_pub.publish(msg)

        # Cancel and delete the timer
        self._utterance_delay_timer.cancel()
        self.destroy_timer(self._utterance_delay_timer)
        self._utterance_delay_timer = None

    def run(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    eou_node = EoUNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(eou_node)

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
