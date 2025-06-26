import traci
import math
import json

class MessagePacket:
    """Simulated V2I message structure using realistic formatting."""
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

class VehicleV2I:
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        traci.vehicle.setSpeedMode(vehicle_id, 0)
        self.update()
        self.replan_needed = False
        self.lane_blocked = False

    def update(self):
        self.last_lane_pos = getattr(self, 'lane_pos', None)
        self.position = traci.vehicle.getPosition(self.vehicle_id)
        self.speed = traci.vehicle.getSpeed(self.vehicle_id)
        self.angle = traci.vehicle.getAngle(self.vehicle_id)
        self.route = traci.vehicle.getRoute(self.vehicle_id)
        self.edge_id = traci.vehicle.getRoadID(self.vehicle_id)
        self.lane_pos = traci.vehicle.getLanePosition(self.vehicle_id)
        self.length = traci.vehicle.getLength(self.vehicle_id)
        self.type_id = traci.vehicle.getTypeID(self.vehicle_id)
        self.current_lane = traci.vehicle.getLaneID(self.vehicle_id)

    def report_state(self):
        return {
            'position': self.position,
            'speed': self.speed,
            'lane_pos': self.lane_pos,
            'last_lane_pos': self.last_lane_pos,
            'edge_id': self.edge_id
        }

    def apply_decision(self, decision):
        if self.lane_pos > 30.0:
        # Already past intersection area — never force stop
            traci.vehicle.setSpeed(self.vehicle_id, -1.0)
            return

        if decision == 'wait':
            traci.vehicle.setSpeed(self.vehicle_id, 0.0)
            print(f"{self.vehicle_id} received decision to WAIT.")
        elif decision == 'go':
            traci.vehicle.setSpeed(self.vehicle_id, -1.0)
            print(f"{self.vehicle_id} received decision to GO.")


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
