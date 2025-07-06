# vehicle.py - Enhanced Vehicle Class with Improved V2V Intersection Behavior

import traci
import math
import json

class MessagePacket:
    """Simulated V2V message structure with protocol-specific size estimation."""

    def __init__(self, vehicle_id, position, speed, angle, route):
        self.vehicle_id = vehicle_id
        self.position = position
        self.speed = speed
        self.angle = angle
        self.route = route

    def to_bytes(self):
        data = {
            "id": self.vehicle_id,
            "position": {"x": self.position[0], "y": self.position[1]},
            "speed": self.speed,
            "angle": self.angle,
            "route": self.route
        }
        return json.dumps(data).encode("utf-8")

    def payload_size(self):
        return len(self.to_bytes())

    @staticmethod
    def report_sizes(payload, max_overhead=False):
        """
        Reports total size of V2V messages with overheads for DSRC, C-V2X, and 5G NR-V2X.

        References per protocol:
        - DSRC:
            [1] IEEE 802.11p-2010 MAC Header: https://ieeexplore.ieee.org/document/5515717
            [2] IEEE 1609.2-2016 Security Layer: https://standards.ieee.org/standard/1609_2-2016.html
        - C-V2X:
            [3] 3GPP TS 36.331 (RRC/Sidelink): https://www.3gpp.org/ftp/Specs/latest/Rel-14/36_series/36331-g00.zip
            [4] Qualcomm Whitepaper (MAC/RLC/PDCP & security): https://www.qualcomm.com/media/documents/files/why-cellular-v2x.pdf
        - 5G NR-V2X:
            [5] 3GPP TS 38.300 & TS 38.401 (NR sidelink and stack): https://www.3gpp.org/DynaReport/38-series.htm
            [6] IEEE Access Paper on 5G-V2X security: https://ieeexplore.ieee.org/document/9279244
        """

        # === DSRC Overhead ===
        # MAC Layer (IEEE 802.11p): 30–40 bytes → Ref [1]
        # LLC/SNAP: fixed 8 bytes
        # Security Layer (IEEE 1609.2 ECDSA + cert): 60–100 bytes → Ref [2]
        mac_header    = 40 if max_overhead else 30
        llc_snap      = 8
        security_dsrc = 100 if max_overhead else 60
        dsrc_total = payload + mac_header + llc_snap + security_dsrc

        # === C-V2X Overhead ===
        # Sidelink Control Info (SCI, 3GPP TS 36.331): 16–24 bytes → Ref [3]
        # MAC + PDCP + RLC stack (Qualcomm/3GPP): 20–40 bytes → Ref [4]
        # Security (Optional): 48 bytes assumed → Ref [4]
        sci            = 24 if max_overhead else 16
        protocol_stack = 40 if max_overhead else 20
        security_cv2x  = 48
        cv2x_total = payload + sci + protocol_stack + security_cv2x

        # === 5G NR-V2X Overhead ===
        # Scheduling Info (3GPP TS 38.300): 24–40 bytes → Ref [5]
        # NR MAC/PHY/PDCP stack (3GPP): 48 bytes assumed → Ref [5]
        # Security + QoS (IEEE Access): 72 bytes assumed → Ref [6]
        scheduling_info   = 40 if max_overhead else 24
        protocol_stack_5g = 48
        security_5g       = 72
        v2x5g_total = payload + scheduling_info + protocol_stack_5g + security_5g

        return {
            "Payload": payload,
            "DSRC": dsrc_total,
            "C-V2X": cv2x_total,
            "5G NR-V2X": v2x5g_total
        }



class Vehicle:
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        traci.vehicle.setSpeedMode(vehicle_id, 0)
        self.update()
        self.stop_needed = False
        self.lane_blocked = False

    def update(self):
        self.position = traci.vehicle.getPosition(self.vehicle_id)
        self.speed = traci.vehicle.getSpeed(self.vehicle_id)
        self.angle = traci.vehicle.getAngle(self.vehicle_id)
        self.route = traci.vehicle.getRoute(self.vehicle_id)
        self.edge_id = traci.vehicle.getRoadID(self.vehicle_id)
        self.lane_pos = traci.vehicle.getLanePosition(self.vehicle_id)
        self.length = traci.vehicle.getLength(self.vehicle_id)
        self.type_id = traci.vehicle.getTypeID(self.vehicle_id)
        self.current_lane = traci.vehicle.getLaneID(self.vehicle_id)

    def communicate(self, all_vehicles, radius=20.0, safe_distance=15.0):
        packet_size = self._estimate_packet_payload_size()

        # Define intersection area
        INTERSECTION_X_MIN, INTERSECTION_X_MAX = 4370, 4400
        INTERSECTION_Y_MIN, INTERSECTION_Y_MAX = 1320, 1350

        def is_inside_intersection(vehicle):
            x, y = vehicle.position
            return INTERSECTION_X_MIN <= x <= INTERSECTION_X_MAX and INTERSECTION_Y_MIN <= y <= INTERSECTION_Y_MAX

        # Step 1: OUTSIDE the intersection, check for conflicts BEFORE ENTERING
        if not is_inside_intersection(self):
            for other in all_vehicles.values():
                if other.vehicle_id == self.vehicle_id:
                    continue
                if is_inside_intersection(other):
                    distance = self._distance_to(other)
                    if distance < safe_distance:
                        traci.vehicle.setSpeed(self.vehicle_id, 0.0)
                        print(f"{self.vehicle_id} WAIT — conflict with {other.vehicle_id} already in intersection.")
                        return packet_size

        # Step 2: Once I'm INSIDE the intersection, NEVER STOP ME
        # -> No action needed here — we just don’t let others stop us

        # Step 3: Local edge-based safety (rear-end logic)
        nearby_vehicles = []
        for other_id, other in all_vehicles.items():
            if other_id == self.vehicle_id:
                continue
            if self._is_on_same_edge(other) and self._distance_to(other) <= radius:
                nearby_vehicles.append(other)

        leaders = [v for v in nearby_vehicles if self._is_ahead_of(v)]
        if self.stop_needed:
            print(f"{self.vehicle_id} is replanning due to full blockage.")
            self.stop_needed = False
            self._stop_and_wait()
            return packet_size

        if self.lane_blocked:
            num_lanes = traci.edge.getLaneNumber(self.edge_id)
            if num_lanes > 1:
                print(f"{self.vehicle_id} is changing lane due to partial blockage.")
                self._change_lane()
            else:
                print(f"{self.vehicle_id} route has only one lane, treating lane block as full block.")
                self._stop_and_wait()
            self.lane_blocked = False
            return packet_size

        if not leaders or len(leaders) == 0:
            if (self.speed < 5 and not self.speed > 25):
                new_speed = max(0.0, (self.speed if self.speed > 0 else 1.0) * 1.2)
                traci.vehicle.setSpeed(self.vehicle_id, new_speed)
                print(f"{self.vehicle_id} accelerate to {new_speed:.2f} to continue.")
            return packet_size

        nearest_leader = min(leaders, key=lambda v: self._distance_to(v))
        distance = self._distance_to(nearest_leader)
        print(f"distance is distance{distance} this is id {self.vehicle_id} and is intercetion {is_inside_intersection(self)}")
        if distance < safe_distance:
            new_speed = max(0.0, nearest_leader.speed * 0.8)
            traci.vehicle.setSpeed(self.vehicle_id, new_speed)
            print(f"{self.vehicle_id} slows to {new_speed:.2f} to stay {distance:.2f}m behind {nearest_leader.vehicle_id}")
        else: 
            print(f"{self.vehicle_id} maintains speed {self.speed:.2f}, leader {nearest_leader.vehicle_id} is {distance:.2f}m ahead")

        return packet_size


    def handle_unexpected_event(self, event_type="full_block", lane_name = ''):
        print('self.current_lane', self.current_lane)
        if event_type == "full_block" and lane_name in self.current_lane:
            self.stop_needed = True
        elif event_type == "lane_block" and lane_name == self.current_lane:
            self.lane_blocked = True

    def _stop_and_wait(self):
        traci.vehicle.setSpeed(self.vehicle_id, 0)
        print(f"{self.vehicle_id} stopped for full blockage")

    def _change_lane(self):
        lane_index = traci.vehicle.getLaneIndex(self.vehicle_id)
        num_lanes = traci.edge.getLaneNumber(self.edge_id)
        if num_lanes > 1:
            alt_lane = 1 - lane_index if num_lanes == 2 else (lane_index + 1) % num_lanes
            try:
                traci.vehicle.changeLane(self.vehicle_id, alt_lane, 25.0)
                print(f"{self.vehicle_id} switched to lane {alt_lane}")
            except traci.TraCIException as e:
                print(f"Lane change failed for {self.vehicle_id}: {e}")

    def _is_on_same_edge(self, other):
        return self.edge_id == other.edge_id

    def _is_ahead_of(self, other):
        return self.lane_pos < other.lane_pos

    def _distance_to(self, other):
        x1, y1 = self.position
        x2, y2 = other.position
        return math.hypot(x2 - x1, y2 - y1)

    def _estimate_packet_payload_size(self):
        packet = MessagePacket(
            vehicle_id=self.vehicle_id,
            position=self.position,
            speed=self.speed,
            angle=self.angle,
            route=self.route
        )
        return packet.payload_size()
