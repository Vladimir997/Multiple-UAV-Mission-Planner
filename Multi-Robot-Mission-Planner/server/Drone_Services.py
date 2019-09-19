import sys, ast, math
import time
import threading
import dronekit

altitude = 10 + 5.3
#sys.setrecursionlimit(1500)
"""
	Connect to the number of vehicles spesified as command line arguments
"""


def initialize(vehicles, UAV_BASE_PORT, server):
    t = threading.Thread(target=lambda: initialize_in_new_thread(vehicles, UAV_BASE_PORT, server))
    t.daemon = True
    t.start()


def initialize_in_new_thread(vehicles, UAV_BASE_PORT, server):
    # Number of simulated drones available at startup given as a command line argument
    instance_count = 0
    if (len(sys.argv) > 1):
        instance_count = int(sys.argv[1])

    for instance_index in range(instance_count):

        port = UAV_BASE_PORT + 10 * instance_index
        connection_string = "127.0.0.1:%i" % port  # connect to port on localhost

        # Connect to the Vehicles
        print('Connecting to vehicle %i on: %s' % (instance_index, connection_string))
        while True:
            try:
                vehicle = dronekit.connect(connection_string, wait_ready=True)
            except Exception as e:
                print(e)
                time.sleep(1)
            else:
                vehicle.id = instance_index
                vehicle.max_speed = 5
                vehicle.current_battery = 600
                vehicle.max_battery_time = 600
                vehicle.max_carry_weight = 10
                vehicle.start_up_time = time.time()
                vehicle.nextlocations = []
                vehicle.groundspeed = 50
                vehicle.airspeed = 40
                vehicles.append(vehicle)
                break

    start_data_updating(vehicles, server)
    do_for_all(vehicles, lambda v: arm_and_takeoff(v, 10, server))


"""
	Start a new simulated drone
"""


def start_new_simulated_drone(vehicles, UAV_BASE_PORT, server):
    t = threading.Thread(target=lambda: start_new_simulated_drone_in_new_thread(vehicles, UAV_BASE_PORT, server))
    t.daemon = True
    t.start()


def start_new_simulated_drone_in_new_thread(vehicles, UAV_BASE_PORT, server):
    port = UAV_BASE_PORT + 10 * len(vehicles)
    connection_string = "127.0.0.1:%i" % port  # connect to port on localhost

    # Connect to the Vehicles
    print('Connecting to vehicle %i on: %s' % (len(vehicles), connection_string))
    server.send_message_to_all("[\"-1\", \"Connecting to vehicle...\"]")

    while True:
        try:
            vehicle = dronekit.connect(connection_string, wait_ready=True)
        except Exception as e:
            print(e)
            time.sleep(1)
        else:
            vehicle.id = len(vehicles)
            vehicle.max_speed = 5
            vehicle.current_battery = 600
            vehicle.max_battery_time = 600
            vehicle.max_carry_weight = 10
            vehicle.start_up_time = time.time()
            vehicle.nextlocations = []
            vehicles.append(vehicle)
            t = threading.Thread(target=lambda: start_data_updating_thread(vehicle, server))
            t.daemon = True
            t.start()
            arm_and_takeoff(vehicle, 10, server)
            break


"""
	Arms vehicle and fly to aTargetAltitude.
"""


def arm_and_takeoff(vehicle, aTargetAltitude, server):
    print("Vehicle %i: Basic pre-arm checks" % vehicle.id)
    # Don't try to arm until autopilot is ready
    print("Vehicle %i: Waiting for vehicle to initialise..." % vehicle.id)
    server.send_message_to_all("[\"-1\", \"Waiting for vehicle to initialise...\"]")
    while not vehicle.is_armable:
        time.sleep(1)

    print("Vehicle %i: Arming motors" % vehicle.id)
    # Copter should arm in GUIDED mode
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    print("Vehicle %i: Waiting for arming..." % vehicle.id)
    server.send_message_to_all("[\"-1\", \"Waiting for arming...\"]")
    while not vehicle.armed:
        time.sleep(1)

    print("Vehicle %i: Taking off!" % vehicle.id)
    server.send_message_to_all("[\"-1\", \"Taking off!\"]")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print("Vehicle %i: Altitude: " % vehicle.id, vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Vehicle %i: Reached target altitude" % vehicle.id)
            server.send_message_to_all("[\"-1\", \"Reached target altitude\"]")
            break
        time.sleep(1)


"""
Start to send vehicle information to client
"""


def start_data_updating(vehicles, server):
    for vehicle in vehicles:
        t = threading.Thread(target=lambda: start_data_updating_thread(vehicle, server))
        t.daemon = True
        t.start()


def start_data_updating_thread(vehicle, server):
    while True:
        vehicle_id = vehicle.id
        vehicle_position = [vehicle.location.global_frame.lat, vehicle.location.global_frame.lon]

        # Flight data
        vehicle_altitude = vehicle.location.global_relative_frame.alt
        vehicle_speed = vehicle.groundspeed
        vehicle_heading = vehicle.heading
        vehicle_current_battery = vehicle.current_battery - (time.time() - vehicle.start_up_time)

        # if(vehicle_current_battery <= 0):
        # vehicle.close()

        # Parameters
        vehicle_max_speed = vehicle.max_speed
        vehicle_max_battery_time = vehicle.max_battery_time
        vehicle_max_carry_weight = vehicle.max_carry_weight

        server.send_message_to_all(repr([
            vehicle_id,
            vehicle_position,
            vehicle_altitude,
            vehicle_speed,
            vehicle_heading,
            vehicle_current_battery,
            vehicle_max_speed,
            vehicle_max_battery_time,
            vehicle_max_carry_weight
        ]))
        time.sleep(0.1)


def do_for_all(vehicles, function):
    threads = [threading.Thread(target=lambda v=v: function(v)) for v in vehicles]
    for t in threads:
        t.start()
    for t in threads:
        t.join()


def find_vehicle_by_id(vehicles, id):
    for vehicle in vehicles:
        if vehicle.id == id:
            return vehicle


"""
	Calculate the distance between 2 points
"""


def distance(location1, location2):
    (x1, y1) = location1.lat, location1.lon
    (x2, y2) = location2.lat, location2.lon
    try:
        d = math.acos(math.sin(math.radians(x1)) * math.sin(math.radians(x2)) + math.cos(math.radians(x1)) * math.cos(
            math.radians(x2)) * math.cos(math.radians(y2 - y1))) * 6378.137
    except ValueError:
        print("ValueError in distance calculation for ", x1, y1, "to", x2, y2)
        d = 0.0
    return d


"""
	Functions to check if two lines intersect
"""


# Given three colinear points (x1, y1), (x2, y2), (x3, y3), the function checks if 
# point (x2, y2) lies on line segment (x1,y1)(x3,y3)
def onSegment(x1, y1, x2, y2, x3, y3):
    if x2 <= max(x1, x3) and x2 >= min(x1, x3) and y2 <= max(y1, y3) and y2 >= min(y1, y3):
        return True
    return False


def orientation(x1, y1, x2, y2, x3, y3):
    value = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2)

    if value == 0:
        return 0

    return 1 if value > 0 else 2


def doIntersect(p1, q1, p2, q2):
    o1 = orientation(p1.lat, p1.lon, q1.lat, q1.lon, p2.lat, p2.lon)
    o2 = orientation(p1.lat, p1.lon, q1.lat, q1.lon, q2.lat, q2.lon)
    o3 = orientation(p2.lat, p2.lon, q2.lat, q2.lon, p1.lat, p1.lon)
    o4 = orientation(p2.lat, p2.lon, q2.lat, q2.lon, q1.lat, q1.lon)

    if o1 != o2 and o3 != o4:
        return True

    return False


def is_on_line(drone, location1, location2):
    return distance(location1, drone) + distance(location2, drone) == distance(location1, location2)


"""
	Check if there is an obstacle between 2 points and returns the closest
"""


def is_obstacle(p1, p2, obstacles):
    closest_obstacle = None
    for obstacle in obstacles:
        n = len(obstacle)
        flag = False
        for i, point in enumerate(obstacle):
                flag = flag or doIntersect(obstacle[i], obstacle[(i+1)%n], p1, p2)

        if flag:
            if closest_obstacle is None:
                closest_obstacle = obstacle
            else:
                closest_obstacle = calculate_closest_obstacle(p1, closest_obstacle, obstacle)
    

    return closest_obstacle


"""
	Returns the obstacle closest to p
"""


def calculate_closest_obstacle(p, obstacle_1, obstacle_2):
    shortest_distance = 9999999
    closest_obstacle = None
    for i in range(4):
        if distance(p, obstacle_1[i]) < shortest_distance:
            shortest_distance = distance(p, obstacle_1[i])
            closest_obstacle = obstacle_1
    for i in range(4):
        if distance(p, obstacle_2[i]) < shortest_distance:
            shortest_distance = distance(p, obstacle_2[i])
            closest_obstacle = obstacle_2

    return closest_obstacle


"""
	Check if the spesific obstacle is between 2 points
"""


def is_spesific_obstacle_between(p1, p2, obstacle):
    flag = False
    n = len(obstacle)
    for i, point in enumerate(obstacle):
            flag = flag or doIntersect(obstacle[i], obstacle[(i+1)%n], p1, p2)

    if flag:
        return True
    return False


"""
	Creates a bigger obstacle, so corner is outside the obstacle, so drone don't crash
"""

'''
def makeOffset(prev_point, point, next_point, r):
    x1, y1 = prev_point.lon, prev_point.lat
    x2, y2 = point.lon, point.lat
    x3, y3 = next_point.lon, next_point.lat
    if x2 - x1 == 0:
        x1 += r * 0.1
    if y2 - y1 == 0:
        y1 += r * 0.1
    if x3 - x2 == 0:
        x3 += r * 0.1
    if y3 - y2 == 0:
        y3 += r * 0.1
    m1 = (x2 - x1)/(y2 - y1)
    b1 = y1 - m1 * x1
    m2 = (x3 - x2)/(y3 - y2)
    b2 = y3 - m2 * x3
    theta = math.atan2(m2-m1,1+m1*m2)
    theta *= 180/math.pi
    theta /= 2
    theta += 180
    x_line = x2 + r * math.cos(theta)
    y_line = y2 + m1 * x2 + b1
    y_final = y_line + r * math.sin(theta)
    x_final = x_line - (y_final - b1)/m1
    return dronekit.LocationGlobal(y_final, x_final, altitude)

def make_obstacle_with_offset(obstacle, r):
    obstacle_with_offset = []
    n = len(obstacle)
    for i, point in enumerate(obstacle):
        obstacle_with_offset.append(makeOffset(obstacle[(i-1)%n], point, obstacle[(i+1)%n]))
    return obstacle_with_offset
    
'''
def make_obstacle_with_offset(obstacle, n):
    obstacle_with_offset = []
    obstacle_with_offset.append(makeOffset(obstacle[0], 0, n))
    obstacle_with_offset.append(makeOffset(obstacle[1], 1, n))
    obstacle_with_offset.append(makeOffset(obstacle[2], 2, n))
    obstacle_with_offset.append(makeOffset(obstacle[3], 3, n))
    return obstacle_with_offset


"""
	Creates a point outside the corner of a square
"""


def makeOffset(p, corner_nr, n):
    if corner_nr == 0:
        return dronekit.LocationGlobal(p.lat + n, p.lon - n, altitude)
    if corner_nr == 1:
        return dronekit.LocationGlobal(p.lat + n, p.lon + n, altitude)
    if corner_nr == 2:
        return dronekit.LocationGlobal(p.lat - n, p.lon + n, altitude)
    if corner_nr == 3:
        return dronekit.LocationGlobal(p.lat - n, p.lon - n, altitude)


"""
	Calculates the distance and path from a spesific vehicle to a spesific task
	If an obstacle is in the way, try to calculate path around, not optimized.
"""

'''
def calculate_distance_path(task, current_location, obstacles):
    obstacle = is_obstacle(current_location, task[0], obstacles)
    distance_ = None
    path = []

    if obstacle == None:
        distance_ = distance(current_location, task[0])
    else:
        shortest_distance_around_obstacle, shortest_path_around_obstacle = calculate_distance_path_around_obstacle(
            current_location, task, obstacle, obstacles)
        distance_ = shortest_distance_around_obstacle
        path.extend(shortest_path_around_obstacle)

    return distance_, path


"""
	Try to calculate the shortest distance and path around the obstacle
	This function is recursiv and will try to calculate the shortest path around additional
	obstacles, it is not completly optimized, and will result in pythons max recursive depth error if
	there are many obstacles on the map
"""


def calculate_distance_path_around_obstacle(drone_location, task, obstacle, obstacles):
    obstacle_with_offset = make_obstacle_with_offset(obstacle)
    shortest_distance = 999999999999999
    shortest_path = []

    # For each corner of the obstacle
    for i in range(4):

        # If corner is in line of sight from drone
        if not is_spesific_obstacle_between(drone_location, obstacle_with_offset[i], obstacle):

            # Calculate distance from corner to task
            distance_from_corner_to_task, path_from_corner = calculate_distance_path_from_corner_to_task(i, task,
                                                                                                         obstacle,
                                                                                                         obstacles)

            distance_ = distance(drone_location, obstacle_with_offset[i]) + distance_from_corner_to_task
            if distance_ < shortest_distance:
                shortest_distance = distance_
                shortest_path = [obstacle_with_offset[i]]
                shortest_path.extend(path_from_corner)

    return shortest_distance, shortest_path


"""
	Calculate the distance from the spesific corner around the obstacle and to the task
"""


def calculate_distance_path_from_corner_to_task(corner_nr, task, obstacle, obstacles):
    distance_around_obstacle_clockwise, path_around_clockwise = calculate_distance_path_from_corner_clockwise(corner_nr,
                                                                                                              task,
                                                                                                              obstacle,
                                                                                                              obstacles)
    distance_around_obstacle_anti_clockwise, path_around_anti_clockwise = calculate_distance_path_from_corner_anti_clockwise(
        corner_nr, task, obstacle, obstacles)

    if (distance_around_obstacle_clockwise < distance_around_obstacle_anti_clockwise):
        return distance_around_obstacle_clockwise, path_around_clockwise
    else:
        return distance_around_obstacle_anti_clockwise, path_around_anti_clockwise


"""
	Calculate the shortest distance around the obstacle clockwise recursivly
"""


def calculate_distance_path_from_corner_clockwise(corner_nr, task, obstacle, obstacles):
    obstacle_with_offset = make_obstacle_with_offset(obstacle)
    corner = obstacle_with_offset[corner_nr]

    if not is_spesific_obstacle_between(corner, task[0], obstacle):
        new_obstacle = is_obstacle(corner, task[0], obstacles)
        if new_obstacle == None:
            return distance(corner, task[0]), []
        else:
            return calculate_distance_path_around_obstacle(corner, task, new_obstacle, obstacles)
    else:
        distance_from_next_corner, path_from_next_corner = calculate_distance_path_from_corner_clockwise(
            (corner_nr + 1) % 4, task, obstacle_with_offset, obstacles)
        distance_around_obstacle_clockwise = distance(corner, obstacle_with_offset[
            (corner_nr + 1) % 4]) + distance_from_next_corner
        path_from_corner = [obstacle_with_offset[(corner_nr + 1) % 4]]
        if (path_from_next_corner):
            path_from_corner.extend(path_from_next_corner)

        return distance_around_obstacle_clockwise, path_from_corner


"""
	Calculate the shortest distance around the obstacle counter clockwise recursivly
"""


def calculate_distance_path_from_corner_anti_clockwise(corner_nr, task, obstacle, obstacles):
    obstacle_with_offset = make_obstacle_with_offset(obstacle)
    corner = obstacle_with_offset[corner_nr]

    if not is_spesific_obstacle_between(corner, task[0], obstacle):
        new_obstacle = is_obstacle(corner, task[0], obstacles)
        if new_obstacle == None:
            return distance(corner, task[0]), []
        else:
            return calculate_distance_path_around_obstacle(corner, task, new_obstacle, obstacles)
    else:
        distance_from_next_corner, path_from_next_corner = calculate_distance_path_from_corner_anti_clockwise(
            (corner_nr - 1) % 4, task, obstacle_with_offset, obstacles)
        distance_around_obstacle_anti_clockwise = distance(corner, obstacle_with_offset[
            (corner_nr - 1) % 4]) + distance_from_next_corner
        path_from_corner = [obstacle_with_offset[(corner_nr - 1) % 4]]
        if (path_from_next_corner):
            path_from_corner.extend(path_from_next_corner)

        return distance_around_obstacle_anti_clockwise, path_from_corner
'''


def calculate_distance_path(task, current_location, obstacles):
    flag_clk = False
    flag_aclk = False
    rec_clk = False
    rec_aclk = False
    rec = None
    stack = []
    stack_2 = []
    temp_obstacles = list(obstacles)
    obstacles = add_offset_to_obstacles(obstacles, 0.00003)
    obstacle = is_obstacle(current_location, task[0], obstacles)
    distance_ = None
    path = []

    if obstacle is None:
        distance_ = distance(current_location, task[0])
    else:
        while True:
            print('Main recursion')
            print(len(stack))
            print(len(stack_2))
            shortest_distance = 999999999999999
            shortest_path = []

            corner_nr = 0

            if rec_clk or rec_aclk:
                corner_nr = rec[3]
                obstacle = rec[4]
                temp_location = current_location
                current_location = rec[5]
                shortest_distance = rec[6]
                shortest_path = rec[7]

            obstacle_with_offset = make_obstacle_with_offset(obstacle, 0.00001)
            print(obstacle)

            while corner_nr < 4:

                # If corner is in line of sight from drone
                if not is_spesific_obstacle_between(current_location, obstacle_with_offset[corner_nr], obstacle):


                    # Calculate distance from corner to task
                    if not (rec_clk or rec_aclk):
                        distance_around_obstacle_clockwise = 0
                        path_around_clockwise = []
                        temp_corner = corner_nr
                        while True:
                            print('Recursion clk')
                            corner = obstacle_with_offset[corner_nr]

                            if not is_spesific_obstacle_between(corner, task[0], obstacle):
                                new_obstacle = is_obstacle(corner, task[0], obstacles)
                                print(type(new_obstacle))
                                if new_obstacle is None:
                                    distance_around_obstacle_clockwise += distance(corner, task[0])
                                    break
                                else:
                                    flag_clk = True
                                    break
                            else:
                                distance_around_obstacle_clockwise += distance(corner, obstacle_with_offset[
                                    (corner_nr + 1) % 4])
                                path_around_clockwise.extend([obstacle_with_offset[(corner_nr + 1) % 4]])
                                corner_nr += 1
                                corner_nr %= 4

                        corner_nr = temp_corner

                        if flag_clk:
                            print('Main recursion from clk')
                            stack.append(('clk', distance_around_obstacle_clockwise, path_around_clockwise, corner_nr, obstacle, current_location, shortest_distance, shortest_path))
                            flag_clk = False
                            current_location = corner
                            obstacle = new_obstacle
                            break
                    elif rec_clk:
                        print('Out of main recursion from clk')
                        shortest = stack_2.pop()
                        start = temp_location
                        finish = shortest[1][0]
                        print(temp_location)
                        distance_around_obstacle_clockwise, path_around_clockwise = distance_path_around_new_obstacle(start, finish, rec, shortest, obstacles)
                        rec_clk = False

                    if not rec_aclk:
                        distance_around_obstacle_anti_clockwise = 0
                        path_around_anti_clockwise = []
                        temp_corner = corner_nr
                        while True:
                            print('Recursion aclk')
                            corner = obstacle_with_offset[corner_nr]

                            if not is_spesific_obstacle_between(corner, task[0], obstacle):
                                new_obstacle = is_obstacle(corner, task[0], obstacles)
                                print(type(new_obstacle))
                                if new_obstacle is None:
                                    distance_around_obstacle_anti_clockwise += distance(corner, task[0])
                                    break
                                else:
                                    flag_aclk = True
                                    break
                            else:
                                distance_around_obstacle_anti_clockwise += distance(corner, obstacle_with_offset[
                                    (corner_nr - 1) % 4])
                                path_around_anti_clockwise.extend([obstacle_with_offset[(corner_nr - 1) % 4]])
                                corner_nr -= 1
                                corner_nr %= 4

                        corner_nr = temp_corner

                        if flag_aclk:
                            print('Main recursion from aclk')
                            stack.append(('aclk', distance_around_obstacle_anti_clockwise, path_around_anti_clockwise, corner_nr, obstacle, current_location, shortest_distance, shortest_path, distance_around_obstacle_clockwise, path_around_clockwise))
                            flag_aclk = False
                            current_location = corner
                            obstacle = new_obstacle
                            break
                    else:
                        print('Out of main recursion from aclk')
                        shortest = stack_2.pop()
                        start = temp_location
                        finish = shortest[1][0]
                        print(temp_location)
                        distance_around_obstacle_anti_clockwise, path_around_anti_clockwise = distance_path_around_new_obstacle(start, finish, rec, shortest, obstacles)
                        distance_around_obstacle_clockwise = rec[8]
                        path_around_clockwise = rec[9]
                        rec_aclk = False

                    distance_around_obstacle_clockwise += distance(current_location, obstacle_with_offset[corner_nr])
                    path_around_clockwise_new = [obstacle_with_offset[corner_nr]]
                    path_around_clockwise_new.extend(path_around_clockwise)

                    distance_around_obstacle_clockwise, path_around_clockwise = shorter_path(current_location, distance_around_obstacle_clockwise, path_around_clockwise_new, obstacles)

                    distance_around_obstacle_anti_clockwise += distance(current_location, obstacle_with_offset[corner_nr])
                    path_around_anti_clockwise_new = [obstacle_with_offset[corner_nr]]
                    path_around_anti_clockwise_new.extend(path_around_anti_clockwise)

                    distance_around_obstacle_anti_clockwise, path_around_anti_clockwise = shorter_path(current_location, distance_around_obstacle_anti_clockwise, path_around_anti_clockwise_new, obstacles)

                    if distance_around_obstacle_clockwise < distance_around_obstacle_anti_clockwise:
                        distance_to_task = distance_around_obstacle_clockwise
                        path_to_task = path_around_clockwise
                    else:
                        distance_to_task = distance_around_obstacle_anti_clockwise
                        path_to_task = path_around_anti_clockwise

                    print("Distance_: %f" %distance_to_task)
                    print('Calculating for corner_nr: %d' %corner_nr)
                    if distance_to_task < shortest_distance:
                        shortest_distance = distance_to_task
                        shortest_path = path_to_task

                corner_nr += 1

            if len(stack) > 0 and corner_nr == 4:
                rec = list(stack.pop())
                stack_2.append((shortest_distance, shortest_path))
                if rec[0] == 'clk':
                    rec_clk = True
                elif rec[0] == 'aclk':
                    rec_aclk = True
            elif corner_nr == 4:
                break

            print('Shortest distance: %f' %shortest_distance)

        distance_ = shortest_distance
        path.extend(shortest_path)
        obstacles = temp_obstacles
        print('Finished')

    return distance_, path

def distance_path_around_new_obstacle(start, finish, rec, shortest, obstacles):

    while True:
        print(finish)
        print(rec[1])
        print(rec[2])
        newest_obstacle = is_obstacle(start, finish, obstacles)
        if newest_obstacle is None:
            distance_new = rec[1] + shortest[0]
            path_new = rec[2]
            path_new.extend(shortest[1])
            print('finally')
            break
        else:
            newest_obstacle_offset = make_obstacle_with_offset(newest_obstacle, 0.00001)
            id_new, nearest_corner = find_nearest_corner(start, newest_obstacle)
            if not is_spesific_obstacle_between(nearest_corner, finish, newest_obstacle):
                rec[1] += distance(start, nearest_corner)
                rec[2].extend([nearest_corner])
                start = rec[2][-1]
                print('close')
                print(start)
            elif is_spesific_obstacle_between(newest_obstacle_offset[(id_new + 1) % 4], finish, newest_obstacle):
                if not is_spesific_obstacle_between(start, newest_obstacle_offset[id_new - 1], newest_obstacle):
                    rec[1] += distance(start, newest_obstacle_offset[id_new - 1])
                    rec[2].extend([newest_obstacle_offset[id_new - 1]])
                else:
                    rec[1] += distance(nearest_corner, newest_obstacle_offset[id_new - 1]) + distance(start, nearest_corner)
                    rec[2].extend([nearest_corner])
                    rec[2].extend([newest_obstacle_offset[id_new - 1]])
                print('aclk')
                start = rec[2][-1]
                print(start)
            elif is_spesific_obstacle_between(newest_obstacle_offset[id_new - 1], finish, newest_obstacle):
                if not is_spesific_obstacle_between(start, newest_obstacle_offset[(id_new + 1) % 4], newest_obstacle):
                    rec[1] += distance(start, newest_obstacle_offset[(id_new + 1) % 4])
                    rec[2].extend([newest_obstacle_offset[(id_new + 1) % 4]])
                else:
                    rec[1] += distance(nearest_corner, newest_obstacle_offset[(id_new + 1) % 4]) + distance(start, nearest_corner)
                    rec[2].extend([nearest_corner])
                    rec[2].extend([newest_obstacle_offset[(id_new + 1) % 4]])
                print('clk')
                start = rec[2][-1]
                print(start)
            else:
                distance1 = 0
                distance2 = 0
                flag1 = False
                flag2 = False
                if not is_spesific_obstacle_between(start, newest_obstacle_offset[id_new - 1], newest_obstacle):
                    distance1 += distance(start, newest_obstacle_offset[id_new - 1])
                else:
                    distance1 += distance(nearest_corner, newest_obstacle_offset[id_new - 1]) + distance(start, nearest_corner)
                    flag1 = True
                if not is_spesific_obstacle_between(start, newest_obstacle_offset[(id_new + 1) % 4], newest_obstacle):
                    distance2 += distance(start, newest_obstacle_offset[(id_new + 1) % 4])
                else:
                    distance2 += distance(nearest_corner, newest_obstacle_offset[(id_new + 1) % 4]) + distance(start, nearest_corner)
                    flag2 = True
                distance1 += distance(newest_obstacle_offset[id_new - 1], finish)
                distance2 += distance(newest_obstacle_offset[(id_new + 1) % 4], finish)
                if distance1 < distance2:
                    rec[1] += distance1
                    if flag1:
                        rec[2].extend([nearest_corner])
                        rec[2].extend([newest_obstacle_offset[id_new - 1]])
                    else:
                        rec[2].extend([newest_obstacle_offset[id_new - 1]])
                else:
                    rec[1] += distance2
                    if flag2:
                        rec[2].extend([nearest_corner])
                        rec[2].extend([newest_obstacle_offset[(id_new + 1) % 4]])
                    else:
                        rec[2].extend([newest_obstacle_offset[(id_new + 1) % 4]])
                start = rec[2][-1]
                print('cmp')
            print(start)
    return distance_new, path_new

def shorter_path(current_location, distance_short, path_short, obstacles):
    path_temp = [current_location]
    path_temp.extend(path_short)
    print('shortest')

    for i, start in enumerate(path_temp):
        last = 0
        l = []
        flag = False
        for j, finish in enumerate(path_temp):
            if j > i + 1:
                o = is_obstacle(start, finish, obstacles)
                if o is None:
                    for k in range(i, j):
                        if k == i and not flag:
                            distance_short -= distance(path_temp[k], path_temp[k+1])
                            flag = True
                        elif path_temp[k] not in l and not k == i:
                            distance_short -= distance(path_temp[k], path_temp[k+1])
                            l.append(path_temp[k])

                    distance_short -= last
                    last = distance(start, finish)
                    distance_short += last

        for p in l:
            print([path_temp, p, l])
            path_temp.remove(p)

    path_temp.pop(0)
    path_short = path_temp
    return distance_short, path_short

def find_nearest_corner(start, obstacle):
    min_new = 9999999999
    id_new = 0
    nearest_corner = None
    offset_obstacle = make_obstacle_with_offset(obstacle, 0.00003)

    for i, cor in enumerate(offset_obstacle):
        d = distance(start, cor)
        if min_new > d:
            min_new = d
            nearest_corner = cor
            id_new = i

    return id_new, nearest_corner

def add_offset_to_obstacles(obstacles, offset_):
    temp_obstacles = []

    for obstacle in obstacles:
        temp_obstacles.append(make_obstacle_with_offset(obstacle, offset_))

    obstacles = temp_obstacles

    return obstacles


"""
	Creates a sequence of points to follow when searching over an area
"""


def generate_search_coordinates(search_tasks):
    points = search_tasks[0]

    '''
	Indexes of "points":
	0 = North West, 1 = North East, 2 = South East, 3 = South West
	0---------------1
	|				|
	|				|
	|				|
	3---------------2
	'''
    # This is the North West corner of the search area
    starting_point = points[0]
    # How far south the drone shall search
    lowest_lat = points[3].lat
    # How far west the drone shall go 
    lowest_lon = points[0].lon
    # How far east the drone shall go
    highest_lon = points[1].lon

    paths = []

    current_location = starting_point
    next_location = None
    moveLatDirection = False

    while current_location.lat > lowest_lat:
        if next_location == None:
            # Start in northwest corner
            next_location = starting_point
        else:
            if moveLatDirection:
                next_location = current_location
                next_location = dronekit.LocationGlobal(next_location.lat - 0.0001, next_location.lon, 10)
                moveLatDirection = False
            else:
                # If we are max east, move left
                if current_location.lon >= highest_lon:
                    next_location = dronekit.LocationGlobal(current_location.lat, lowest_lon, 10)
                # Move right
                else:
                    next_location = dronekit.LocationGlobal(current_location.lat, highest_lon, 10)
                moveLatDirection = True

        paths.append(next_location)
        current_location = next_location

    return paths
