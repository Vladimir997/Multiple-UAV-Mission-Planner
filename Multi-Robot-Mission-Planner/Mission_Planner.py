import sys, ast, math
import time
import subprocess
import threading
import dronekit
from server import combiserver
from server import Drone_Services

HTTP_PORT = 8000
WEBSOCKET_PORT = 9001
UAV_BASE_PORT = 14550
server = combiserver.CombiServer(HTTP_PORT, WEBSOCKET_PORT)

altitude = 10 + 5.3  # Don't ask why, it just works
vehicles = []
free_vehicles = []

obstacles = []
u_tasks = []


def main():
    Drone_Services.initialize(vehicles, UAV_BASE_PORT, server)
    server.set_fn_message_received(message_received)

    # Start web interface
    with server:
        while True:

            free_vehicles.clear()

            # For each vehicle
            for vehicle in vehicles:
                # If the vehicle does not have a next task, add it to free vehicles
                if not vehicle.nextlocations:
                    free_vehicles.append(vehicle)
                # Else if next task is reached remove it from the vehicles's task lis
                elif Drone_Services.distance(vehicle.nextlocations[0], vehicle.location.global_frame) < 0.001:
                    vehicle.nextlocations.pop(0)
                    # If the vehicle has more tasks, start the next

                    if vehicle.nextlocations:
                        vehicle.simple_goto(vehicle.nextlocations[0])

                    # Else add it to free vehicles
                    else:
                        free_vehicles.append(vehicle)

            # Distribute new tasks	
            distribute_tasks(free_vehicles)
            time.sleep(1)


"""
	Distribute tasks while there are more undone tasks and free vehicles
"""


def distribute_tasks(free_vehicles):
    while free_vehicles and u_tasks:
        task, vehicle = best_vehicle_task(free_vehicles)

        if (task is not None and vehicle is not None):

            free_vehicles.remove(vehicle)
            u_tasks.remove(task)

            vehicle.nextlocations.extend(task)
            vehicle.simple_goto(vehicle.nextlocations[0])

        # If task is None, no more vehicles can do any of the remaining tasks
        else:
            break


"""
	Try to find the shortest distance between a task and vehicle, not optimized
"""


def best_vehicle_task(free_vehicles):
    best_vehicle = None
    best_task = None
    best_distance = None
    path = []
    for vehicle in free_vehicles:
        for task in u_tasks:

            # If this is a pickup task, check that the vehicle is able to carry the weight
            if (len(task) < 4 or task[3] != "weight" or (
                task[3] == "weight" and vehicle.max_carry_weight >= float(task[2]))):

                distance, new_path = Drone_Services.calculate_distance_path(task, vehicle.location.global_frame,
                                                                            obstacles)
                if (best_distance == None) or (distance < best_distance):
                    best_vehicle = vehicle
                    best_task = task
                    best_distance = distance
                    path = new_path

    if (best_task is not None):

        # If this is a pickup task remove the parameters before appending task
        if (len(best_task) > 3 and best_task[3] == "weight"):
            best_task.pop()
            best_task.pop()

        best_vehicle.nextlocations.extend(path)

    return best_task, best_vehicle


# Called when a client clicks on the fly button
def message_received(client, server, message):
    # First position indicates message type
    message_type = ast.literal_eval(message)[0]

    # Adding new tasks that user draw on map
    if (message_type == 0):

        # Line task
        tasks = ast.literal_eval(message)[1]
        lines = tasks['line']
        lines = [[dronekit.LocationGlobal(point["lat"], point["lng"], altitude) for point in line] for line in lines]

        global u_tasks
        u_tasks.extend(lines)

        # Pickup task
        pickup_tasks_temp = []
        for pickup_task in tasks['pickup']:
            line = pickup_task["line"]
            weight = pickup_task["weight"]
            line = [
                dronekit.LocationGlobal(line[0]["lat"], line[0]["lng"], altitude),
                dronekit.LocationGlobal(line[1]["lat"], line[1]["lng"], altitude),
                weight,
                "weight"
            ]
            pickup_tasks_temp.append(line)

        u_tasks.extend(pickup_tasks_temp)

        # Search task
        search_tasks = tasks['search']
        search_tasks_temp = []

        # Convert to DroneKit's Global Position Object
        for task in search_tasks:
            task_temp = []
            for point in task:
                point = dronekit.LocationGlobal(point[0], point[1], altitude)
                task_temp.append(point)
            search_tasks_temp.append(task_temp)
        search_tasks = search_tasks_temp

        if len(search_tasks) > 0:
            search_tasks = Drone_Services.generate_search_coordinates(search_tasks)
            u_tasks.append(search_tasks)

        # Obstacles 
        obstacles_from_client = tasks['obstacles']
        obstacles_temp = []

        for obstacle in obstacles_from_client:
            point_temp = []
            for point in obstacle:
                point = dronekit.LocationGlobal(point[0], point[1], altitude)
                point_temp.append(point)
            obstacles_temp.append(point_temp)

        global obstacles
        obstacles.extend(obstacles_temp)

    # New Parameter recieved
    if (message_type == 1):
        """ Change Parameters """
        vehicle_id = ast.literal_eval(message)[1]
        vehicle = Drone_Services.find_vehicle_by_id(vehicles, vehicle_id)

        new_max_speed = ast.literal_eval(message)[2]
        vehicle.max_speed = float(new_max_speed)
        vehicle.groundspeed = float(new_max_speed)

        new_max_battery_time = ast.literal_eval(message)[3]
        vehicle.max_battery_time = float(new_max_battery_time)
        vehicle.current_battery = float(new_max_battery_time)
        vehicle.start_up_time = time.time()

        new_max_carry_weight = ast.literal_eval(message)[4]
        vehicle.max_carry_weight = float(new_max_carry_weight)

    # Reacharge battery
    if (message_type == 2):
        vehicle_id = ast.literal_eval(message)[1]
        vehicle = Drone_Services.find_vehicle_by_id(vehicles, vehicle_id)

        vehicle.current_battery = vehicle.max_battery_time
        vehicle.start_up_time = time.time()

    # Open new cygwin terminal
    if (message_type == 3):
        subprocess.call('start cd C:\cygwin ^& cygwin', shell=True)

    # Start new simulated drone
    if (message_type == 4):
        Drone_Services.start_new_simulated_drone(vehicles, UAV_BASE_PORT, server)

    # Delete mission and stop vehicles
    if (message_type == 5):
        for vehicle in vehicles:
            vehicle.nextlocations = []
            free_vehicles = []
            obstacles = []
            u_tasks = []
            vehicle.simple_goto(
                dronekit.LocationGlobal(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, altitude))


main()
