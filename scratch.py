def calculate_distance_path(task, current_location, obstacles):
    """
    Calculates the distance and path from a specific vehicle to a specific task
	If an obstacle is in the way, try to calculate path around, not optimized.
	"""
    
    flag_clk = False        # Flags used when another obstacle is in the way (clockwise and anticlockwise)
    flag_aclk = False
    rec_clk = False         # Flags used when returning to the calculations from the previous obstacle (clockwise and anticlockwise)
    rec_aclk = False
    rec = []                # List used when returning the values of the calculations from the previous obstacle
    stack = []              # Stack which keeps the values when another obstacle is in the way
    stack_2 = []            # Stores the shortest distance and path to return and add them to the previous obstacle
    temp_obstacles = list(obstacles)
    obstacles = add_offset_to_obstacles(obstacles, 0.00003)
    #obstacles = merge_obstacles(obstacles)         # It needs to be corrected
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

            if rec_clk or rec_aclk:         # Returning the values of the calculations from the previous obstacle
                corner_nr = rec[3]
                obstacle = rec[4]
                temp_location = current_location
                current_location = rec[5]
                shortest_distance = rec[6]
                shortest_path = rec[7]

            n = len(obstacle)
            obstacle_with_offset = make_obstacle_with_offset(obstacle, 0.00001)
            print(obstacle)

            while corner_nr < n:

                # If corner is in line of sight from drone
                if not is_specific_obstacle_between(current_location, obstacle_with_offset[corner_nr], obstacle):


                    # Calculate distance from corner to task clockwise if not returning to a previous obstacle
                    if not (rec_clk or rec_aclk):
                        distance_around_obstacle_clockwise = 0
                        path_around_clockwise = []
                        temp_corner = corner_nr

                        while True:
                            print('Recursion clk')
                            corner = obstacle_with_offset[corner_nr]

                            if not is_specific_obstacle_between(corner, task[0], obstacle):
                                new_obstacle = is_obstacle(corner, task[0], obstacles)
                                print(type(new_obstacle))
                                if new_obstacle is None:        # Reached the task
                                    distance_around_obstacle_clockwise += distance(corner, task[0])
                                    break
                                else:                           # Another obstacle is in the way
                                    flag_clk = True
                                    break
                            else:                               # Going around obstacle
                                distance_around_obstacle_clockwise += distance(corner, obstacle_with_offset[
                                    (corner_nr + 1) % n])
                                path_around_clockwise.extend([obstacle_with_offset[(corner_nr + 1) % n]])
                                corner_nr += 1
                                corner_nr %= n

                        corner_nr = temp_corner

                        if flag_clk:
                            print('Main recursion from clk')
                            stack.append(['clk', distance_around_obstacle_clockwise, path_around_clockwise, corner_nr, obstacle, current_location, shortest_distance, shortest_path])
                            flag_clk = False
                            current_location = corner
                            obstacle = new_obstacle
                            break

                    elif rec_clk:           # Returning to a previous obstacle from clockwise
                        print('Out of main recursion from clk')
                        shortest = stack_2.pop()
                        start = temp_location
                        print(shortest)
                        print(shortest[1])
                        finish = shortest[1][0]
                        print(temp_location)
                        distance_around_obstacle_clockwise, path_around_clockwise = distance_path_around_new_obstacle(start, finish, rec, shortest, obstacles)
                        rec_clk = False

                    # Calculate distance from corner to task anticlockwise
                    # if not returning to a previous obstacle from anticlockwise
                    if not rec_aclk:
                        distance_around_obstacle_anti_clockwise = 0
                        path_around_anti_clockwise = []
                        temp_corner = corner_nr

                        while True:
                            print('Recursion aclk')
                            corner = obstacle_with_offset[corner_nr]

                            if not is_specific_obstacle_between(corner, task[0], obstacle):
                                new_obstacle = is_obstacle(corner, task[0], obstacles)
                                print(type(new_obstacle))
                                if new_obstacle is None:            # Reached the task
                                    distance_around_obstacle_anti_clockwise += distance(corner, task[0])
                                    break
                                else:                               # Another obstacle is in the way
                                    flag_aclk = True
                                    break
                            else:                                   # Going around obstacle
                                distance_around_obstacle_anti_clockwise += distance(corner, obstacle_with_offset[
                                    (corner_nr - 1) % n])
                                path_around_anti_clockwise.extend([obstacle_with_offset[(corner_nr - 1) % n]])
                                corner_nr -= 1
                                corner_nr %= n

                        corner_nr = temp_corner

                        if flag_aclk:
                            print('Main recursion from aclk')
                            stack.append(['aclk', distance_around_obstacle_anti_clockwise, path_around_anti_clockwise, corner_nr, obstacle, current_location, shortest_distance, shortest_path, distance_around_obstacle_clockwise, path_around_clockwise])
                            flag_aclk = False
                            current_location = corner
                            obstacle = new_obstacle
                            break

                    else:           # Returning to a previous obstacle from anticlockwise
                        print('Out of main recursion from aclk')
                        shortest = stack_2.pop()
                        start = temp_location
                        print(shortest)
                        print(shortest[1])
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

            if len(stack) > 0 and corner_nr == n:           # If calculated for all corners and started from previous obstacle
                rec = stack.pop()
                stack_2.append((shortest_distance, shortest_path))
                if rec[0] == 'clk':
                    rec_clk = True
                elif rec[0] == 'aclk':
                    rec_aclk = True
            elif corner_nr == n:                            # If calculated for all corners and no previous obstacles
                break

            print('Shortest distance: %f' %shortest_distance)
            
        distance_ = shortest_distance
        path.extend(shortest_path)
        obstacles = temp_obstacles
        print('Finished')

    return distance_, path


def distance_path_around_new_obstacle(start, finish, rec, shortest, obstacles):
    """
    Adjusts the path and distance between obstacles if the corner of the next obstacle is not visible
    from the previous point of the path
    Made before the merge obstacles function, may need changes to work with merged obstacles
    """

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
            n = len(newest_obstacle)
            newest_obstacle_offset = make_obstacle_with_offset(newest_obstacle, 0.00001)
            id_new, nearest_corner = find_nearest_corner(start, newest_obstacle)
            # If the obstacle is not in the way from the nearest corner of the obstacle
            if not is_specific_obstacle_between(nearest_corner, finish, newest_obstacle):
                rec[1] += distance(start, nearest_corner)
                rec[2].extend([nearest_corner])
                start = rec[2][-1]
                print('close')
                print(start)
            # If the obstacle is in the way from the next corner in clockwise direction, go anticlockwise
            elif is_specific_obstacle_between(newest_obstacle_offset[(id_new + 1) % n], finish, newest_obstacle):
                # If the obstacle is not in the way to the next corner
                if not is_specific_obstacle_between(start, newest_obstacle_offset[id_new - 1], newest_obstacle):
                    rec[1] += distance(start, newest_obstacle_offset[id_new - 1])
                    rec[2].extend([newest_obstacle_offset[id_new - 1]])
                else:
                    rec[1] += distance(nearest_corner, newest_obstacle_offset[id_new - 1]) + distance(start, nearest_corner)
                    rec[2].extend([nearest_corner])
                    rec[2].extend([newest_obstacle_offset[id_new - 1]])
                print('aclk')
                start = rec[2][-1]
                print(start)
            # If the obstacle is in the way from the next corner in anticlockwise direction, go clockwise
            elif is_specific_obstacle_between(newest_obstacle_offset[id_new - 1], finish, newest_obstacle):
                # If the obstacle is not in the way to the next corner
                if not is_specific_obstacle_between(start, newest_obstacle_offset[(id_new + 1) % n], newest_obstacle):
                    rec[1] += distance(start, newest_obstacle_offset[(id_new + 1) % n])
                    rec[2].extend([newest_obstacle_offset[(id_new + 1) % n]])
                else:
                    rec[1] += distance(nearest_corner, newest_obstacle_offset[(id_new + 1) % n]) + distance(start, nearest_corner)
                    rec[2].extend([nearest_corner])
                    rec[2].extend([newest_obstacle_offset[(id_new + 1) % n]])
                print('clk')
                start = rec[2][-1]
                print(start)
            else:            # If the obstacle is not in the way from both of the next corners, compare and choose shorter path
                distance1 = 0
                distance2 = 0
                flag1 = False
                flag2 = False

                # If the obstacle is not in the way to the next anticlockwise corner
                if not is_specific_obstacle_between(start, newest_obstacle_offset[id_new - 1], newest_obstacle):
                    distance1 += distance(start, newest_obstacle_offset[id_new - 1])
                else:
                    distance1 += distance(nearest_corner, newest_obstacle_offset[id_new - 1]) + distance(start, nearest_corner)
                    flag1 = True

                # If the obstacle is not in the way to the next clockwise corner
                if not is_specific_obstacle_between(start, newest_obstacle_offset[(id_new + 1) % n], newest_obstacle):
                    distance2 += distance(start, newest_obstacle_offset[(id_new + 1) % n])
                else:
                    distance2 += distance(nearest_corner, newest_obstacle_offset[(id_new + 1) % n]) + distance(start, nearest_corner)
                    flag2 = True

                distance1 += distance(newest_obstacle_offset[id_new - 1], finish)
                distance2 += distance(newest_obstacle_offset[(id_new + 1) % n], finish)

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
                        rec[2].extend([newest_obstacle_offset[(id_new + 1) % n]])
                    else:
                        rec[2].extend([newest_obstacle_offset[(id_new + 1) % n]])

                start = rec[2][-1]
                print('cmp')
            print(start)
    return distance_new, path_new


def shorter_path(current_location, distance_short, path_short, obstacles):
    """
    Removes unnecessary points from path
    """

    path_temp = [current_location]          # Because some point might be reachable from the start
    path_temp.extend(path_short)
    print('shortest')

    for i, start in enumerate(path_temp):       # Starting point
        last = 0
        l = []          # List for the points that need to be removed
        flag = False
        for j, finish in enumerate(path_temp):  # Ending point
            if j > i + 1:                       # At least one point between them
                o = is_obstacle(start, finish, obstacles)
                if o is None:           # If the point is reachable from a previous position
                    for k in range(i, j):
                        # Remove only the points between, without the starting and ending point
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

    path_temp.pop(0)            # The current location is not needed in the actual path, only the next points
    path_short = path_temp
    return distance_short, path_short


def find_nearest_corner(start, obstacle):
    """
    Finds nearest corner of a specific obstacle, from a specific position
    """

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


def is_inside(check_point, obstacles):
    """
    Checks if a point is inside any obstacle
    """

    north_pole = dronekit.LocationGlobal(90.0000, 0.0000, altitude)

    for obstacle in obstacles:
        i = 0
        n = len(obstacle)
        for k, point in enumerate(obstacle):
            f = doIntersect(obstacle[k], obstacle[(k+1)%n], check_point, north_pole)
            if f:
                i += 1
        # If the line segment from the point to the north pole intersects with an uneven number of sides of an obstacle,
        # the point is inside that obstacle
        if not i % 2 == 0:
            return True, obstacle

    return False, None


def merge_obstacles(obstacles):
    """
    Merges obstacles that are too close to each other and the vehicle can't go between them,
    so that they are considered as one obstacle
    The points of the merged obstacle are ordered in a clockwise direction
    There are still mistakes, it needs to be corrected
    """

    temp_obstacles = list(obstacles)
    obstacles = add_offset_to_obstacles(obstacles, 0.00001)
    merged_obstacles = []
    print('Merge')

    for o1 in obstacles:
        f1 = False
        already_merged1 = []
        print('o1')
        print(o1)
        for x in merged_obstacles:          # Check if the obstacle is already merged
            for y in x:
                print('y')
                print(y)
                if compare_obstacles(o1, y):
                    already_merged1 = x
        print('alreadyM1')
        print(already_merged1)
        if not already_merged1 == []:
            print('already1')
            o1 = already_merged1[0]         # Take the already merged obstacle
            f1 = True
        o3 = o1         # Iterate only over this obstacle, o1 will change as it is merged with other obstacles
        for id_point, point in enumerate(o3):
            f2 = False
            f, o2 = is_inside(o3[id_point], temp_obstacles)
            print('o2')
            print(o2)
            if f:
                already_merged2 = []
                for x in merged_obstacles:          # Check if the obstacle is already merged
                    for y in x:
                        if compare_obstacles(o2, y):
                            already_merged2 = x
                print('alreadyM2')
                print(already_merged2)
                if not already_merged2 == []:
                    print('already2')
                    if o1 in already_merged2:       # If it's the same obstacle, just remove the point
                        print('cont')
                        merged_obstacles.remove(already_merged2)
                        already_merged2[0].remove(point)
                        merged_obstacles.append(already_merged2)
                        continue
                    o2 = already_merged2[0]         # Take the already merged obstacle
                    f2 = True
                n1 = len(o1)
                n2 = len(o2)

                id_nearest, nearest = find_nearest_corner(o3[id_point], o2)
                print('o1')
                print(o1)
                # If you can iterate clockwise over every point of the two obstacles uninterrupted
                # excluding the point that is inside, add the points to a new obstacle in that order
                if not is_specific_obstacle_between(o2[id_nearest], o2[(id_nearest + 1) % n2], o1):

                    new_obstacle = [o2[id_nearest]]         # Start from the nearest of the second obstacle
                    j = (id_nearest + 1) % n2

                    while not j == id_nearest:              # Add all the points of the second obstacle
                        new_obstacle.append(o2[j])
                        j += 1
                        j %= n2

                    # Find the point from the merged obstacle o1 which corresponds to the point from o3
                    for id_other, point_other in enumerate(o1):
                        if compare_points(point, point_other):
                            j = (id_other + 1) % n1         # And skip it
                            id_point = id_other

                    while not j == id_point:                # Add all the other points of the merged obstacle o1
                        new_obstacle.append(o1[j])
                        j += 1
                        j %= n1
                # Else you can iterate anticlockwise over every point of the two obstacles uninterrupted
                # excluding the point that is inside, and add the points to a new obstacle in reverse order
                else:

                    new_obstacle = [o2[id_nearest]]         # Start from the nearest of the second obstacle
                    j = (id_nearest - 1) % n2

                    while not j == id_nearest:              # Add all the points of the second obstacle
                        new_obstacle.insert(0, o2[j])
                        j -= 1
                        j %= n2

                    # Find the point from the merged obstacle o1 which corresponds to the point from o3
                    for id_other, point_other in enumerate(o1):
                        if compare_points(point, point_other):
                            j = (id_other - 1) % n1         # And skip it
                            id_point = id_other

                    while not j == id_point:                # Add all the other points of the merged obstacle o1
                        new_obstacle.insert(0, o1[j])
                        j -= 1
                        j %= n1

                if f1 and f2:
                    temp_list = [new_obstacle]
                    temp_list.extend(already_merged1)
                    temp_list.extend(already_merged2)
                    merged_obstacles.remove(already_merged1)
                    merged_obstacles.remove(already_merged2)
                    merged_obstacles.append(temp_list)
                    already_merged1 = temp_list
                    o1 = new_obstacle
                elif f1:
                    temp_list = [new_obstacle]
                    temp_list.extend(already_merged1)
                    merged_obstacles.remove(already_merged1)
                    temp_list.append(o2)
                    merged_obstacles.append(temp_list)
                    already_merged1 = temp_list
                    o1 = new_obstacle
                elif f2:
                    temp_list = [new_obstacle]
                    temp_list.extend(already_merged2)
                    merged_obstacles.remove(already_merged2)
                    temp_list.append(o1)
                    merged_obstacles.append(temp_list)
                    already_merged1 = temp_list
                    o1 = new_obstacle
                    f1 = True
                else:
                    merged_obstacles.append([new_obstacle, o1, o2])
                    already_merged1 = [new_obstacle, o1, o2]
                    o1 = new_obstacle
                    f1 = True
                print('merged')
                print(merged_obstacles)
                print('alreadyM1')
                print(already_merged1)
                print('new')
                print(new_obstacle)

    # If there are unreachable points that are not inside any obstacle, remove them
    without_trapped = []
    for new_obstacles in merged_obstacles:
        new_obstacle = new_obstacles[0]
        n3 = len(new_obstacle)
        print('trapped')
        print(new_obstacle)
        new_obstacle_offset = make_obstacle_with_offset(new_obstacle, -0.00001)
        to_be_removed = []
        for i, trapped_point in enumerate(new_obstacle):
            print('point')
            print(trapped_point)
            print('remove')
            print(to_be_removed)
            if trapped_point in to_be_removed:
                continue
            # If the obstacle itself is in the way between two of it's points,
            # iterate the points in opposite directions until it's in the way again
            if is_specific_obstacle_between(new_obstacle[i], new_obstacle[(i + 1) % n3], new_obstacle_offset):
                p = i
                q = (i + 1) % n3
                while True:
                    # Whichever way is blocked first, go backwards and remove the points
                    if is_specific_obstacle_between(new_obstacle[q], new_obstacle[(q + 1) % n3], new_obstacle_offset):
                        while not q == i:
                            to_be_removed.append(new_obstacle[q])
                            q = (q - 1) % n3
                        break
                    elif is_specific_obstacle_between(new_obstacle[p], new_obstacle[(p - 1) % n3], new_obstacle_offset):
                        while not p == (i+1)%n3:
                            to_be_removed.append(new_obstacle[p])
                            p = (p + 1) % n3
                        break
                    p = (p - 1) % n3
                    q = (q + 1) % n3

        for remove_point in to_be_removed:
            new_obstacle.remove(remove_point)
        without_trapped.append([new_obstacle, new_obstacles])

    print('without')
    print(without_trapped)
    l = []
    for obstacle in obstacles:
        k = 0
        for merged_obstacle in without_trapped:
            if obstacle not in merged_obstacle:
                k += 1
        if k == len(merged_obstacles):          # Find the obstacles that aren't merged
            l.append(obstacle)

    for merged_obstacle in without_trapped:
        l.append(merged_obstacle[0])            # Add the ones that are merged

    print('begin')
    for o in l:
        print(o)
    print('end')

    obstacles = add_offset_to_obstacles(l, -0.00001)

    return obstacles


def compare_points(p1, p2):
    if p1.lat == p2.lat and p1.lon == p2.lon and p1.alt == p2.alt:
        return True
    return False


def compare_obstacles(o1, o2):
    j = 0
    for i, p in enumerate(o2):
        if compare_points(o1[0], p):
            j = i
            break
    n = len(o2)
    for i, p in enumerate(o1):
        if not compare_points(p, o2[j]):
            return False
        j += 1
        j %= n

    return True


def rotate_around_point(x_to_rotate, y_to_rotate, around_x, around_y, for_theta):
    """
    Rotate one point around another
    """

    # Bring the point you need to rotate to the global coordinate system
    x_temp = x_to_rotate - around_x
    y_temp = y_to_rotate - around_y
    # Rotate it around (0,0) in the global coordinate system
    new_x = x_temp * math.cos(for_theta) - y_temp * math.sin(for_theta)
    new_y = x_temp * math.sin(for_theta) + y_temp * math.cos(for_theta)
    # Bring it back to the local coordinate system with the beginning in the point around which it was rotated
    new_x += around_x
    new_y += around_y
    return new_x, new_y


def makeOffset(prev_point, point, next_point, r):
    """
    Adds an offset to a point
    There are still mistakes, it needs to be corrected
    """

    x1, y1 = prev_point.lon, prev_point.lat
    x2, y2 = point.lon, point.lat
    x3, y3 = next_point.lon, next_point.lat

    # Add a little offset so that there are no problems with the calculations
    if x2 - x1 == 0:
        x1 += 0.000001
    if y2 - y1 == 0:
        y1 += 0.000001
    if x3 - x2 == 0:
        x3 += 0.000001
    if y3 - y2 == 0:
        y3 += 0.000001

    # The sides of the triangle created by the points
    a = math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1))
    b = math.sqrt((x3 - x2)*(x3 - x2) + (y3 - y2)*(y3 - y2))
    c = math.sqrt((x1 - x3)*(x1 - x3) + (y1 - y3)*(y1 - y3))

    theta = math.acos((a*a + b*b - c*c)/(2*a*b))            # The angle at the point
    theta_half = theta/2                                    # So that the offset is positioned centrally

    # Angle between the line through the previous point and the point and the x axis
    new_x_angle = math.atan2(y2-y1,x2-x1)
    # Add the offset on the x axis and rotate it first to align it with the line through the previous point
    # and the x axis, and then for theta half
    x_final, y_final = rotate_around_point(x2 + r, y2, x2, y2, new_x_angle + theta_half)

    return dronekit.LocationGlobal(y_final, x_final, altitude)


def make_obstacle_with_offset(obstacle, r):
    obstacle_with_offset = []
    n = len(obstacle)
    for i, point in enumerate(obstacle):
        obstacle_with_offset.append(makeOffset(obstacle[(i-1)%n], point, obstacle[(i+1)%n], r))
    return obstacle_with_offset


