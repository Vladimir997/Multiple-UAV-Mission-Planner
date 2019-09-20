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

def is_inside(check_point, obstacles):
    north_pole = dronekit.LocationGlobal(90.0000, 0.0000, altitude)

    for obstacle in obstacles:
        i = 0
        n = len(obstacle)
        for k, point in enumerate(obstacle):
            f = doIntersect(obstacle[k], obstacle[(k+1)%n], check_point, north_pole)
        if f:
            i += 1
        if not i % 2 == 0:
            return True, obstacle

    return False, None

def merge_obstacles(obstacles):
    obstacles = add_offset_to_obstacles(obstacles, 0.00001)
    merged_obstacles = []
    print('Merge')

    for o1 in obstacles:
        f1 = False
        already_merged1 = [x for x in merged_obstacles if o1 in x]
        if not already_merged1 == []:
            o1 = already_merged1[0][0]
            f1 = True
        o3 = o1

        for id_point, point in enumerate(o3):
            f2 = False
            f, o2 = is_inside(o3[id_point], obstacles)
            if f:
                already_merged2 = [x for x in merged_obstacles if o2 in x]
                if not already_merged2 == []:
                    if o1 in already_merged2[0]:
                        merged_obstacles.remove(already_merged2[0])
                        already_merged2[0][0].remove(point)
                        merged_obstacles.append(already_merged2[0])
                        continue
                    o2 = already_merged2[0][0]
                    f2 = True
                n1 = len(o1)
                n2 = len(o2)

                id_nearest, nearest = find_nearest_corner(o3[id_point], o2)
                if not is_spesific_obstacle_between(o2[id_nearest], o2[(id_nearest + 1)%n2], o1):
                    new_obstacle = [o2[id_nearest]]

                    j = (id_nearest + 1) % n2
                    while not j == id_nearest:
                        new_obstacle.append(o2[j])
                        j += 1
                        j %= n2
                        
                    for id_other, point_other in o1:
                        if point_other == point:
                            j = (id_other + 1) % n1
                            
                    while not j == id_point:
                        new_obstacle.append(o1[j])
                        j += 1
                        j %= n1
                else:
                    new_obstacle = [o2[id_nearest]]

                    j = (id_nearest - 1) % n2
                    while not j == id_nearest:
                        new_obstacle.insert(0, o2[j])
                        j -= 1
                        j %= n2

                    for id_other, point_other in o1:
                        if point_other == point:
                            j = (id_other - 1) % n1
                            
                    while not j == id_point:
                        new_obstacle.insert(0, o1[j])
                        j -= 1
                        j %= n1

                n3 = len(new_obstacle)
                print(new_obstacle)
                new_obstacle_offset = make_obstacle_with_offset(new_obstacle, -0.00001)
                for i, trapped_point in new_obstacle:
                    if is_spesific_obstacle_between(new_obstacle[i], new_obstacle[(i+1)%n3], new_obstacle_offset):
                        p = i
                        q = (i + 1) % n3
                        while true:
                            if is_spesific_obstacle_between(new_obstacle[q], new_obstacle[(q+1)%n3], new_obstacle_offset):
                                while not q == i:
                                    new_obstacle.remove(new_obstacle[q])
                                    q = (q - 1) % n3
                                break
                            elif is_spesific_obstacle_between(new_obstacle[p], new_obstacle[(p-1)%n3], new_obstacle_offset):
                                while not p == (i+1)%n3:
                                    new_obstacle.remove(new_obstacle[p])
                                    p = (p + 1) % n3
                                break
                            p = (p - 1) % n3
                            q = (q + 1) % n3

                if f1 and f2:
                    temp_list = [new_obstacle]
                    temp_list.extend(already_merged1[0])
                    temp_list.extend(already_merged2[0])
                    merged_obstacles.remove(already_merged1[0])
                    merged_obstacles.remove(already_merged2[0])
                    merged_obstacles.append(temp_list)
                    already_merged1[0] = temp_list
                    o1 = new_obstacle
                elif f1:
                    temp_list = [new_obstacle]
                    temp_list.extend(already_merged1[0])
                    merged_obstacles.remove(already_merged1[0])
                    temp_list.append(o2)
                    merged_obstacles.append(temp_list)
                    already_merged1[0] = temp_list
                    o1 = new_obstacle
                elif f2:
                    temp_list = [new_obstacle]
                    temp_list.extend(already_merged2[0])
                    merged_obstacles.remove(already_merged2[0])
                    temp_list.append(o1)
                    merged_obstacles.append(temp_list)
                    already_merged1[0] = temp_list
                    o1 = new_obstacle
                    f1 = True
                else:
                    merged_obstacles.append([new_obstacle, o1, o2])
                    already_merged1[0] = [new_obstacle, o1, o2]
                    o1 = new_obstacle
                    f1 = True

    l = []
    for obstacle in obstacles:
        k = 0
        for merged_obstacle in merged_obstacles:
            if obstacle not in merged_obstacle:
                k += 1
        if k == len(merged_obstacles):
            l.append(obstacle)

    for merged_obstacle in merged_obstacles:
        l.append(merged_obstacle[0])

    obstacles = add_offset_to_obstacles(l, -0.00001)

    return obstacles

def rotate_around_point(x_to_rotate, y_to_rotate, around_x, around_y, for_theta):
    x_temp = x_to_rotate - around_x
    y_temp = y_to_rotate - around_y
    new_x = x_temp * math.cos(for_theta) - y_temp * math.sin(for_theta)
    new_y = x_temp * math.sin(for_theta) + y_temp * math.cos(for_theta)
    new_x += around_x
    new_y += around_y
    return new_x, new_y

def makeOffset(prev_point, point, next_point, r):
    x1, y1 = prev_point.lon, prev_point.lat
    x2, y2 = point.lon, point.lat
    x3, y3 = next_point.lon, next_point.lat

    if x2 - x1 == 0:
        x1 += 0.00001
    if y2 - y1 == 0:
        y1 += 0.00001
    if x3 - x2 == 0:
        x3 += 0.00001
    if y3 - y2 == 0:
        y3 += 0.00001

    m1 = (y2 - y1)/(x2 - x1)
    #b1 = y1 - m1 * x1
    m2 = (y3 - y2)/(x3 - x2)
    #b2 = y3 - m2 * x3

    theta = math.atan2(m1-m2,1+m1*m2)
    theta_half = theta/2
    new_x_angle = math.atan2(y2-y1,x2-x1)

    if theta < 0:
        print('check')
        theta_half = -theta_half

    x_final, y_final = rotate_around_point(x2 + r, y2, x2, y2, new_x_angle + theta_half)

    return x_final, y_final

def make_obstacle_with_offset(obstacle, r):
    obstacle_with_offset = []
    n = len(obstacle)
    for i, point in enumerate(obstacle):
        obstacle_with_offset.append(makeOffset(obstacle[(i-1)%n], point, obstacle[(i+1)%n], r))
    return obstacle_with_offset
