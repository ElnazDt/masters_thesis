
# sumo_run.py - Enhanced V2V Simulation Controller

import traci
from vehicle.vehicle import Vehicle

vehicle_objects = {}
packet_sizes = []

# Simulate unexpected pedestrian events at fixed intervals
def inject_unexpected_events(step):
    if step % 50 == 0:
        print("[EVENT] Full intersection blockage!")
        for v in vehicle_objects.values():
            v.handle_unexpected_event("full_block")
    elif step % 40 == 0:
        print("[EVENT] One lane blocked!")
        for v in vehicle_objects.values():
            v.handle_unexpected_event("lane_block")

def run_simulation():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        inject_unexpected_events(step)

        current_ids = traci.vehicle.getIDList()
        for vid in current_ids:
            if vid not in vehicle_objects:
                vehicle_objects[vid] = Vehicle(vid)
            else:
                vehicle_objects[vid].update()

        active_vehicles = {vid: vehicle_objects[vid] for vid in current_ids}

        for v in active_vehicles.values():
            size = v.communicate(active_vehicles)
            packet_sizes.append(size)

        step += 1

    traci.close()

    # Report max packet size
    if packet_sizes:
        print(f"\n[REPORT] Maximum V2V packet size observed: {max(packet_sizes)} bytes")

if __name__ == "__main__":
    traci.start(["sumo-gui", "-c", "C:/Users/elnaz/thesis/V2V/osm.sumocfg"])
    run_simulation()