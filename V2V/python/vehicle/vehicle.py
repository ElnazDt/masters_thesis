# vehicle.py - Enhanced Vehicle Class with Improved V2V Intersection Behavior

import traci
import math
import json

class MessagePacket:
    """Simulated V2V message structure using realistic formatting."""
    def __init__(self, vehicle_id, position, speed, angle, route):
        self.vehicle_id = "VIN-EU-2025-XYZ-veh0"
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

    def size(self):
        return len(self.to_bytes())

class Vehicle:
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        traci.vehicle.setSpeedMode(vehicle_id, 0)
        self.update()
        self.replan_needed = False
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

    def communicate(self, all_vehicles, radius=50.0, safe_distance=20.0):
        packet_size = self._estimate_packet_size()

        # Define intersection area (adjust for your map geometry)
        INTERSECTION_X_MIN, INTERSECTION_X_MAX = 480, 520
        INTERSECTION_Y_MIN, INTERSECTION_Y_MAX = 480, 520

        def is_inside_intersection(vehicle):
            x, y = vehicle.position
            return INTERSECTION_X_MIN <= x <= INTERSECTION_X_MAX and INTERSECTION_Y_MIN <= y <= INTERSECTION_Y_MAX

        # Step 1: If I'm OUTSIDE the intersection, check for conflicts BEFORE ENTERING
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

        if self.replan_needed:
            print(f"{self.vehicle_id} is replanning due to full blockage.")
            self.replan_needed = False
            self._replan_route()
            return packet_size

        if self.lane_blocked:
            num_lanes = traci.edge.getLaneNumber(self.edge_id)
            if num_lanes > 1:
                print(f"{self.vehicle_id} is changing lane due to partial blockage.")
                self._change_lane()
            else:
                print(f"{self.vehicle_id} route has only one lane, treating lane block as full block.")
                self._replan_route()
            self.lane_blocked = False
            return packet_size

        if not leaders:
            return packet_size

        nearest_leader = min(leaders, key=lambda v: self._distance_to(v))
        distance = self._distance_to(nearest_leader)

        if distance < safe_distance:
            new_speed = max(0.0, nearest_leader.speed * 0.8)
            traci.vehicle.setSpeed(self.vehicle_id, new_speed)
            print(f"{self.vehicle_id} slows to {new_speed:.2f} to stay {distance:.2f}m behind {nearest_leader.vehicle_id}")
        else:
            print(f"{self.vehicle_id} maintains speed {self.speed:.2f}, leader {nearest_leader.vehicle_id} is {distance:.2f}m ahead")

        return packet_size


    def handle_unexpected_event(self, event_type="full_block"):
        if event_type == "full_block":
            self.replan_needed = True
        elif event_type == "lane_block":
            self.lane_blocked = True

    def _replan_route(self):
        current_edge = self.edge_id
        if not self.route:
            return
        destination_edge = self.route[-1]
        route_obj = traci.simulation.findRoute(current_edge, destination_edge)
        new_route = route_obj.edges
        traci.vehicle.setRoute(self.vehicle_id, new_route)
        print(f"{self.vehicle_id} replanned route: {new_route}")

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
        return self.lane_pos > other.lane_pos

    def _distance_to(self, other):
        x1, y1 = self.position
        x2, y2 = other.position
        return math.hypot(x2 - x1, y2 - y1)

    def _estimate_packet_size(self):
        packet = MessagePacket(
            vehicle_id=self.vehicle_id,
            position=self.position,
            speed=self.speed,
            angle=self.angle,
            route=self.route
        )
        return packet.size()
