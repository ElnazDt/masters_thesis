# sumo_run_v2i.py - Enhanced V2I Simulation Controller

import tabulate
import traci
from vehicle.vehicle_v2i import MessagePacket, VehicleV2I
from intersection_manager import IntersectionManager

vehicle_objects = {}
packet_sizes = []
intersection_manager = IntersectionManager()

# Simulate unexpected pedestrian events at fixed intervals
def inject_unexpected_events(step):
    if (20 > step) and (step < 30):
        print("[EVENT] Full path blockage!")
        for v in vehicle_objects.values():
            v.handle_unexpected_event("full_block",'41224286#1')
    # if (10 > step) and (step < 30):
    #     print("[EVENT] One lane blocked!")
    #     for v in vehicle_objects.values():
    #         v.handle_unexpected_event("lane_block", '41224286#1_0')

def run_simulation():
    step = 0
    while True:
        expected = traci.simulation.getMinExpectedNumber()
        if not isinstance(expected, (int, float)) or expected <= 0:
            break

        traci.simulationStep()
        inject_unexpected_events(step)
        print('step', step)

        current_ids = traci.vehicle.getIDList()
        for vid in current_ids:
            if vid not in vehicle_objects:
                vehicle_objects[vid] = VehicleV2I(vid)
            else:
                vehicle_objects[vid].update()

        active_vehicles = {vid: vehicle_objects[vid] for vid in current_ids}
        active_vehicles = {vid: vehicle_objects[vid] for vid in current_ids}

        for vid, v in active_vehicles.items():
            intersection_manager.register_vehicle(vid, v.report_state())

        decisions = intersection_manager.decide_priorities()
        for vid, v in active_vehicles.items():
            active_vehicles[vid].apply_decision(decisions[vid])
            packet_sizes.append(active_vehicles[vid]._estimate_packet_payload_size())

        step += 1

    traci.close()

    # Report max packet size
    if packet_sizes:
        min_payload = min(packet_sizes)
        max_payload = max(packet_sizes)
        print('\n======================================================================= Report =======================================================================\n')
        # Collect reports
        reports = {
            "Min OH-Min PL": MessagePacket.report_sizes(min_payload, False),
            "Max OH-Min PL": MessagePacket.report_sizes(min_payload, True),
            "Min OH-Max PL": MessagePacket.report_sizes(max_payload, False),
            "Max OH-Max PL": MessagePacket.report_sizes(max_payload, True)
        }

        # Build table header
        headers = ["Protocol"]
        for label in reports:
            headers.append(f"{label}\n(Size)")
            headers.append(f"{label}\n(OH %)")

        # Build table rows
        protocols = ["DSRC", "C-V2X", "5G NR-V2X"]
        table = []

        for proto in protocols:
            row = [proto]
            for label, report in reports.items():
                total = report[proto]
                payload = report["Payload"]
                overhead = total - payload
                overhead_percentage = (overhead / total) * 100
                row.append(f"{total} bytes")
                row.append(f"{overhead_percentage:.1f}%")
            table.append(row)

        # Print the final table
        print(tabulate.tabulate(table, headers=headers, tablefmt="grid"))

if __name__ == "__main__":
    traci.start(["sumo-gui", "-c", "C:/Users/elnaz/thesis/V2I/osm.sumocfg"])
    run_simulation()
