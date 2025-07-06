class IntersectionManager:
    def __init__(self):
        self.vehicle_states = {}

    def register_vehicle(self, vehicle_id, state):
        self.vehicle_states[vehicle_id] = state

    def decide_priorities(self):
        decisions = {}

        # only vehicles approaching the intersection
        approaching = {
            vid: v for vid, v in self.vehicle_states.items()
            if v['last_lane_pos'] is not None and v['lane_pos'] < v['last_lane_pos']  
            and v['lane_pos'] < 30.0  
        }

        if not approaching:
            return {vid: 'go' for vid in self.vehicle_states}

        # Choose one to go
        sorted_approaching = sorted(approaching.items(), key=lambda item: item[1]['lane_pos'])
        allowed_vid = sorted_approaching[0][0]

        for vid in self.vehicle_states:
            if vid not in approaching:
                decisions[vid] = 'go'
            elif vid == allowed_vid:
                decisions[vid] = 'go'
            else:
                decisions[vid] = 'wait'

        return decisions




