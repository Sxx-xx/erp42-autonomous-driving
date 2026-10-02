#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from v2x_msgs.msg import Spat_new, IntersectionState, MovementState, MovementEvent, TimeChangeDetails

class V2XSignalSimulator:
    def __init__(self):
        rospy.init_node('v2x_signal_simulator', anonymous=True)

        # Publisher
        self.v2x_pub = rospy.Publisher('/v2x_data', Spat_new, queue_size=1)

        # SAE J2735 상태 문자열 매핑
        self.SIGNAL_STATES = {
            'GREEN': 'protected-Movement-Allowed',
            'YELLOW': 'permissive-clearance',
            'RED': 'stop-And-Remain',
            # 필요 시 우회전 별도 상태를 쓰려면 키를 추가해 사용
            'GREEN_RIGHT': 'protected-Movement-Allowed'
        }

        # ───────────────────────────────────────────────────────────────────
        # ① 교차로/신호그룹 구성 (기존과 동일한 맵)
        # ───────────────────────────────────────────────────────────────────
        self.intersections = {
            200: {  # intersection_id: 200
                'signal_groups': [1],       # waypoint 497(STR)
                'movements': {1: 'STR'}
            },
            300: {  # intersection_id: 300
                'signal_groups': [1],       # waypoint 603(LEFT)
                'movements': {1: 'STR'}
            },
            400: {  # intersection_id: 400
                'signal_groups': [1],       # waypoint 709(LEFT)
                'movements': {1: 'STR'}
            },
            1200: {  # intersection_id: 1200 임의
                    'signal_groups': [1],       # waypoint 603(LEFT)
                    'movements': {1: 'STR'}
            },
            1300: {  # intersection_id: 1300 임의
            'signal_groups': [1],       # waypoint 603(LEFT)
            'movements': {1: 'STR'}
            },
            1400: {  # intersection_id: 1300 임의
            'signal_groups': [1],       # waypoint 603(LEFT)
            'movements': {1: 'STR'}
            },
            
        }

        # ───────────────────────────────────────────────────────────────────
        # ② 시퀀스 정의 (초 단위)
        #    - default_sequence: 명시 없는 교차로/신호그룹에 적용되는 공통 시퀀스
        #    - intersection_plans: 교차로별 커스텀 시퀀스
        #       * "default_sequence": 그 교차로 전체에 적용
        #       * "overrides": 특정 signal_group에만 별도 시퀀스 적용
        #    예시에서 200 교차로는 좌회전(14) 우선신호 포함, 300은 단순 3현시, 610은 YELLOW 짧게 등
        # ───────────────────────────────────────────────────────────────────
        self.default_sequence = [
            {'state': 'GREEN',  'duration': 15},
            {'state': 'YELLOW', 'duration': 3},
            {'state': 'RED',    'duration': 10},
        ]

        self.intersection_plans = {
            200: {
                "default_sequence": [
                    {'state': 'GREEN',    'duration': 10},   # 보행/대향 보호
                    {'state': 'YELLOW',  'duration': 3},  # 직진
                    {'state': 'RED', 'duration': 10},
                ]
            },
            300: {
                "default_sequence": [
                    {'state': 'GREEN',  'duration': 32},
                    {'state': 'YELLOW', 'duration': 3},
                    {'state': 'RED',    'duration': 65},
                ]
            },
            400: {
                "default_sequence": [
                    {'state': 'GREEN',  'duration': 47},
                    {'state': 'YELLOW', 'duration': 3},
                    {'state': 'RED',    'duration': 50},
                ]
            },
            1200: {
                "default_sequence": [
                    {'state': 'GREEN',  'duration': 23},
                    {'state': 'YELLOW', 'duration': 3},
                    {'state': 'RED',    'duration': 74},
                ]
            },
            1300: {
                "default_sequence": [
                    {'state': 'GREEN',  'duration': 23},
                    {'state': 'YELLOW', 'duration': 3},
                    {'state': 'RED',    'duration': 74},
                ]
            },
            1400: {
                "default_sequence": [
                    {'state': 'GREEN',  'duration': 37},
                    {'state': 'YELLOW', 'duration': 3},
                    {'state': 'RED',    'duration': 60},
                ]
            }
        }

        # ───────────────────────────────────────────────────────────────────
        # ③ 상태 트래커: 교차로/신호그룹별로 현재 인덱스와 시작시각을 분리 관리
        #    - overrides가 있으면 해당 그룹은 그룹별 트래커 사용
        # ───────────────────────────────────────────────────────────────────
        self.state_tracker = {}  # { intersection_id: { "default": {...}, "groups": {sg: {...}} } }
        self._build_state_tracker()

        rospy.loginfo("V2X Signal Simulator 시작됨")
        rospy.loginfo("교차로별 시퀀스/오버라이드 구성이 적용되었습니다.")

    # ───────────────────────────────────────────────────────────────────
    # 내부 유틸
    # ───────────────────────────────────────────────────────────────────
    def _now(self):
        return time.time()

    def _build_state_tracker(self):
        self.state_tracker = {}
        t = self._now()
        for iid, info in self.intersections.items():
            plan = self.intersection_plans.get(iid, {})
            self.state_tracker[iid] = {
                "default": {"idx": 0, "start": t},
                "groups": {}
            }
            overrides = plan.get("overrides", {})
            for sg in info['signal_groups']:
                if sg in overrides:
                    self.state_tracker[iid]["groups"][sg] = {"idx": 0, "start": t}

    def _get_sequence_for(self, intersection_id, signal_group=None):
        """교차로/신호그룹에 적용될 시퀀스 반환"""
        plan = self.intersection_plans.get(intersection_id)
        if plan:
            # 신호그룹 오버라이드 우선
            if signal_group is not None:
                overrides = plan.get("overrides", {})
                if signal_group in overrides:
                    return overrides[signal_group]
            # 교차로 기본 시퀀스
            if "default_sequence" in plan:
                return plan["default_sequence"]
        # 전체 공통 기본 시퀀스
        return self.default_sequence

    def _advance_if_needed(self, tracker_entry, sequence):
        """현재 현시가 끝났다면 다음 현시로 진행"""
        now = self._now()
        idx = tracker_entry["idx"]
        start = tracker_entry["start"]
        elapsed = now - start
        duration = sequence[idx]['duration']
        if elapsed >= duration:
            tracker_entry["idx"] = (idx + 1) % len(sequence)
            tracker_entry["start"] = now
            return True
        return False

    def _current_state_and_remaining(self, tracker_entry, sequence):
        """현재 상태와 잔여시간(초) 계산"""
        now = self._now()
        idx = tracker_entry["idx"]
        start = tracker_entry["start"]
        elapsed = now - start
        cur = sequence[idx]
        remaining = max(0, int(round(cur['duration'] - elapsed)))
        return cur['state'], remaining, idx, elapsed

    def _next_predictions(self, sequence, cur_idx, cur_remaining, k=3):
        """다음 k개 현시 예측 (start_time/end_time: 현재 기준 상대초)"""
        preds = []
        cum = float(cur_remaining)
        n = len(sequence)
        for i in range(1, k + 1):
            ni = (cur_idx + i) % n
            dur = float(sequence[ni]['duration'])
            preds.append({
                "state": sequence[ni]['state'],
                "start_time": cum,
                "end_time": cum + dur,
                "duration": dur
            })
            cum += dur
        return preds

    # ───────────────────────────────────────────────────────────────────
    # 메시지 생성
    # ───────────────────────────────────────────────────────────────────
    def create_Spat_new_message(self):
        msg = Spat_new()
        msg.timeStamp = int(self._now() * 1000) % (2**16)

        # 모든 교차로 순회
        for iid, info in self.intersections.items():
            istate = IntersectionState()
            istate.id.id = iid
            istate.revision = 1
            istate.status = 0
            istate.moy = int(self._now() / 60) % (2**20)
            istate.timeStamp = int(self._now() * 10) % (2**16)  # 0.1s 단위

            # 교차로 공통(디폴트) 트래커 advance
            default_seq = self._get_sequence_for(iid, None)
            dtracker = self.state_tracker[iid]["default"]
            self._advance_if_needed(dtracker, default_seq)

            # 각 신호그룹 생성
            for sg in info['signal_groups']:
                m = MovementState()
                m.signalGroup = sg
                m.movementName = info['movements'].get(sg, "")

                # 그룹별 시퀀스/트래커 선택
                seq = self._get_sequence_for(iid, sg)
                if sg in self.state_tracker[iid]["groups"]:
                    tracker = self.state_tracker[iid]["groups"][sg]
                else:
                    tracker = dtracker  # 그룹 오버라이드가 없으면 교차로 디폴트 트래커 공유

                # 필요 시 advance
                self._advance_if_needed(tracker, seq)

                # 현재 상태/남은 시간
                cur_state, remaining, cur_idx, _ = self._current_state_and_remaining(tracker, seq)

                # 현재 이벤트
                ev_now = MovementEvent()
                ev_now.event_state = self.SIGNAL_STATES[cur_state]
                tcd_now = TimeChangeDetails()
                tcd_now.startTime = 0
                tcd_now.minEndTime = int(remaining * 10)
                tcd_now.maxEndTime = int(remaining * 10)
                tcd_now.likelyTime = int(remaining * 10)
                tcd_now.confidence = 95
                ev_now.timing = tcd_now
                m.state_time_speed.append(ev_now)

                # 다음 신호 예측(최대 3개)
                preds = self._next_predictions(seq, cur_idx, remaining, k=3)
                for p in preds:
                    ev_next = MovementEvent()
                    ev_next.event_state = self.SIGNAL_STATES[p['state']]
                    tcd_next = TimeChangeDetails()
                    tcd_next.startTime  = int(p['start_time'] * 10)
                    tcd_next.minEndTime = int(p['end_time']   * 10)
                    tcd_next.maxEndTime = int(p['end_time']   * 10)
                    tcd_next.likelyTime = int(p['end_time']   * 10)
                    tcd_next.confidence = 90
                    ev_next.timing = tcd_next
                    m.state_time_speed.append(ev_next)

                istate.states.append(m)

            msg.interchanges.append(istate)

        return msg

    # ───────────────────────────────────────────────────────────────────
    # 런 루프
    # ───────────────────────────────────────────────────────────────────
    def run(self):
        rate_hz = rospy.get_param("~rate_hz", 5)  # 퍼블리시 주기(기본 5Hz)
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            try:
                spat = self.create_Spat_new_message()
                self.v2x_pub.publish(spat)

                # 간략 로깅(교차로 1개만 대표 출력)
                rep_iid = next(iter(self.intersections.keys()))
                rep_info = self.intersections[rep_iid]
                rep_sg = rep_info['signal_groups'][0]
                seq = self._get_sequence_for(rep_iid, rep_sg)
                tracker = self.state_tracker[rep_iid]["groups"].get(rep_sg, self.state_tracker[rep_iid]["default"])
                cur_state, remaining, _, _ = self._current_state_and_remaining(tracker, seq)
                rospy.loginfo_throttle(2, f"[iid {rep_iid}, sg {rep_sg}] {cur_state} {remaining}s 남음")

                rate.sleep()
            except Exception as e:
                rospy.logerr(f"오류 발생: {e}")

    # ───────────────────────────────────────────────────────────────────
    # 런타임 API
    # ───────────────────────────────────────────────────────────────────
    def update_intersection_sequence(self, intersection_id, sequence):
        """
        교차로 전체 기본 시퀀스를 런타임에 교체.
        sequence 예: [{'state':'GREEN','duration':20}, {'state':'YELLOW','duration':4}, {'state':'RED','duration':15}]
        """
        assert isinstance(sequence, list) and all('state' in s and 'duration' in s for s in sequence)
        if intersection_id not in self.intersection_plans:
            self.intersection_plans[intersection_id] = {}
        self.intersection_plans[intersection_id]["default_sequence"] = sequence
        # 상태 초기화
        self._build_state_tracker()
        rospy.loginfo(f"[iid {intersection_id}] 기본 시퀀스 갱신: {sequence}")

    def update_group_override(self, intersection_id, signal_group, sequence):
        """
        특정 교차로-신호그룹 오버라이드 시퀀스 교체/설정.
        """
        assert isinstance(sequence, list) and all('state' in s and 'duration' in s for s in sequence)
        if intersection_id not in self.intersection_plans:
            self.intersection_plans[intersection_id] = {}
        if "overrides" not in self.intersection_plans[intersection_id]:
            self.intersection_plans[intersection_id]["overrides"] = {}
        self.intersection_plans[intersection_id]["overrides"][signal_group] = sequence
        # 상태 초기화
        self._build_state_tracker()
        rospy.loginfo(f"[iid {intersection_id}, sg {signal_group}] 오버라이드 갱신: {sequence}")

    def set_manual_signal_all(self, state, duration=30):
        """
        전체 교차로/그룹을 특정 상태로 고정(시험용).
        """
        for iid in self.intersections.keys():
            self.intersection_plans[iid] = {"default_sequence": [{'state': state, 'duration': duration}]}
        self._build_state_tracker()
        rospy.loginfo(f"전체 수동신호: {state} ({duration}s)")

if __name__ == '__main__':
    try:
        sim = V2XSignalSimulator()

        # 예) 런타임으로 특정 교차로/그룹 시퀀스 바꾸고 싶다면 아래처럼 사용
        # sim.update_intersection_sequence(300, [
        #     {'state':'GREEN','duration':20},
        #     {'state':'YELLOW','duration':4},
        #     {'state':'RED','duration':15},
        # ])
        # sim.update_group_override(200, 14, [
        #     {'state':'GREEN','duration':8},
        #     {'state':'YELLOW','duration':3},
        #     {'state':'RED','duration':14},
        # ])

        sim.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"시뮬레이터 오류: {e}")