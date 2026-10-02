#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from v2x_msgs.msg import Spat_new

class TrafficLightDisplay:
    def __init__(self):
        rospy.init_node('traffic_light_display')
        rospy.Subscriber("/v2x_data", Spat_new, self.spat_callback)
        self.current_color = "RED"
        self.remaining_time = 0

    def spat_callback(self, msg):
        # 최초 메시지에서 신호등 상태와 남은 시간 추출 (가장 첫 번째 교차로/그룹 예시)
        if msg.interchanges and msg.interchanges[0].states:
            event = msg.interchanges[0].states[0].state_time_speed[0]
            state = event.event_state
            # SAE J2735 표준 세팅에 따라 신호 색상 매핑
            if state == "protected-Movement-Allowed":
                self.current_color = "GREEN"
            elif state == "permissive-clearance":
                self.current_color = "YELLOW"
            elif state == "stop-And-Remain":
                self.current_color = "RED"
            else:
                self.current_color = "RED"
            self.remaining_time = event.timing.minEndTime / 10.0  # 0.1초 단위 -> 초로

    def run(self):
        while not rospy.is_shutdown():
            img = np.ones((300, 200, 3), dtype=np.uint8) * 60  # 어두운 배경
            color_map = {
                "RED": (0, 0, 255),
                "YELLOW": (0, 255, 255),
                "GREEN": (0, 255, 0)
            }
            color = color_map.get(self.current_color, (30, 30, 30))
            # 중앙에 동그라미: 신호등 색상
            cv2.circle(img, (100, 120), 60, color, -1)
            # 그 위에 남은 시간(초)
            cv2.putText(img, f"{int(self.remaining_time)}", (70, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.5, (35, 35, 35), 5, cv2.LINE_AA)
            # 신호 상태 텍스트
            cv2.putText(img, self.current_color, (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3, cv2.LINE_AA)
            cv2.imshow('Traffic Light Signal', img)
            key = cv2.waitKey(200)
            if key == 27:  # ESC
                break
        cv2.destroyAllWindows()

if __name__ == "__main__":
    display = TrafficLightDisplay()
    display.run()
