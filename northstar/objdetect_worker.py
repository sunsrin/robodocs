# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import queue
from typing import List, Tuple

import cv2
from config.config import ConfigStore
from output.overlay_util import overlay_circle_obj_detect_observation
from output.StreamServer import MjpegServer
from pipeline.ObjectDetector import CoreMLSlicedObjectDetector
from vision_types import ObjDetectObservation


def objdetect_worker(
    q_in: queue.Queue[Tuple[float, cv2.Mat, ConfigStore]],
    q_out: queue.Queue[Tuple[float, List[ObjDetectObservation]]],
    server_port: int,
):
    object_detector = CoreMLSlicedObjectDetector()
    stream_server = MjpegServer()
    stream_server.start(server_port)

    while True:
        sample = q_in.get()
        timestamp: float = sample[0]
        image: cv2.Mat = sample[1]
        config: ConfigStore = sample[2]

        observations = object_detector.detect(image, config)

        q_out.put((timestamp, observations))
        if stream_server.get_client_count() > 0:
            image = image.copy()
            [overlay_circle_obj_detect_observation(image, x) for x in observations]

            # Render count
            count_text = str(len(observations))
            (text_width, text_height), _ = cv2.getTextSize(count_text, cv2.FONT_HERSHEY_PLAIN, 5, 5)
            cv2.putText(
                image,
                str(len(observations)),
                (image.shape[1] - text_width - 10, text_height + 25),
                cv2.FONT_HERSHEY_PLAIN,
                5,
                (255, 255, 225),
                5
            )

            stream_server.set_frame(image)
