
# sumo_run.py - Enhanced V2V Simulation Controller

import traci
from vehicle.vehicle import Vehicle
from vehicle.vehicle import MessagePacket
import matplotlib.pyplot as plt
import tabulate

vehicle_objects = {}
packet_sizes = []
packet_sizes_per_steps = []

# Simulate unexpected pedestrian events at fixed intervals like passing a from streets
def inject_unexpected_events(step):
    if (40 > step) and (step < 80):
        print("[EVENT] Full path blockage!")
        for v in vehicle_objects.values():
            v.handle_unexpected_event("full_block",'41224286#1')
    # if (40 > step) and (step < 80):
    #     print("[EVENT] One lane blocked!")
    #     for v in vehicle_objects.values():
    #         v.handle_unexpected_event("lane_block", '41224286#1_0')

def run_simulation():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        inject_unexpected_events(step)
        # print('step: ',step)

        current_ids = traci.vehicle.getIDList()
        for vid in current_ids:
            if vid not in vehicle_objects:
                vehicle_objects[vid] = Vehicle(vid)
            else:
                vehicle_objects[vid].update()

        active_vehicles = {vid: vehicle_objects[vid] for vid in current_ids}
        step_packet_sizes = []
        for v in active_vehicles.values():
            size = v.communicate(active_vehicles)
            step_packet_sizes.append(size)
            packet_sizes.append(size)
        step += 1
        if(len(step_packet_sizes)>0):
            packet_sizes_per_steps.append([max(step_packet_sizes), min(step_packet_sizes), sum(step_packet_sizes)/len(step_packet_sizes)])

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
    if packet_sizes_per_steps:
        steps = list(range(len(packet_sizes_per_steps)))
        max_vals = [x[0] for x in packet_sizes_per_steps]
        min_vals = [x[1] for x in packet_sizes_per_steps]
        avg_vals = [x[2] for x in packet_sizes_per_steps]

        plt.figure(figsize=(10, 6))
        plt.plot(steps, max_vals, label='Max Packet Size', marker='o')
        plt.plot(steps, min_vals, label='Min Packet Size', marker='x')
        plt.plot(steps, avg_vals, label='Avg Packet Size', marker='s')

        plt.xlabel('Simulation Step')
        plt.ylabel('Packet Size (bytes)')
        plt.title('Packet Size Trends Over Time')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig('./figures/v2v_packet_sizes_per_steps.png')
        plt.show()
    # adding the protocols overhead to above feagure
    def map_with_overheads(item):
        report = MessagePacket.report_sizes(item[0], False)  # item[0] is max packet size
        return [
            report["DSRC"],
            report["C-V2X"],
            report["5G NR-V2X"]
        ]

    # Convert the map object to a list
    packet_sizes_per_steps_with_overheads = list(map(map_with_overheads, packet_sizes_per_steps))

    if packet_sizes_per_steps_with_overheads:
        steps = list(range(len(packet_sizes_per_steps_with_overheads)))
        max_vals = [x[0] for x in packet_sizes_per_steps_with_overheads]
        min_vals = [x[1] for x in packet_sizes_per_steps_with_overheads]
        avg_vals = [x[2] for x in packet_sizes_per_steps_with_overheads]

        plt.figure(figsize=(10, 6))
        plt.plot(steps, max_vals, label='DSRC Packet Size with overhead', marker='o')
        plt.plot(steps, min_vals, label='C-V2X Packet Size', marker='x')
        plt.plot(steps, avg_vals, label='5G NR-V2X Packet Size', marker='s')

        plt.xlabel('Simulation Step')
        plt.ylabel('Packet Size (bytes) with OH in each protocol')
        plt.title('Packet Size Trends Over Time')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig('./figures/v2v_packet_sizes_per_steps_with_OH_in_each_protocol.png')
        plt.show()

if __name__ == "__main__":
    traci.start(["sumo-gui", "-c", "C:/Users/elnaz/thesis/V2V/osm.sumocfg"])
    run_simulation()