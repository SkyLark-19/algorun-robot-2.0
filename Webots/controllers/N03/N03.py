# floodfill - stops at centre, then returns to start exploring all cells
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('motor1')
right_motor = robot.getDevice('motor2')
left_pos_sensor = robot.getDevice('pos2')
right_pos_sensor = robot.getDevice('pos1')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_pos_sensor.enable(timestep)
right_pos_sensor.enable(timestep)

dsF = robot.getDevice("ds2")
dsL = robot.getDevice("ds4")
dsR = robot.getDevice("ds0")
dsFL = robot.getDevice("ds3")
dsFR = robot.getDevice("ds1")

for ds in [dsF, dsL, dsR, dsFL, dsFR]:
    ds.enable(timestep)

WHEEL_RADIUS = 0.022
AXLE_LENGTH = 0.087
TILE_SIZE = 0.192
MAZE_SIZE = 16
alpha = 0.3

maze = [[{'N': False, 'S': False, 'E': False, 'W': False, 'visited': False}
         for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
for i in range(MAZE_SIZE):
    maze[0][i]['S'] = True
    maze[MAZE_SIZE-1][i]['N'] = True
    maze[i][0]['W'] = True
    maze[i][MAZE_SIZE-1]['E'] = True

current_x = 0
current_y = 0
current_direction = 'N'
goals = [(7, 7), (7, 8), (8, 7), (8, 8)]
start_pos = (0, 0)  
WALL_THRESHOLD = 100
SEARCH_SPEED = 2.5
STOP_FRONT_THRESHOLD = 18

disF = disL = disR = disFL = disFR = 0

def update_sensors():
    global disF, disL, disR, disFL, disFR
    rawF = dsF.getValue() * 1000
    rawL = dsL.getValue() * 1000
    rawR = dsR.getValue() * 1000
    rawFL = dsFL.getValue() * 1000
    rawFR = dsFR.getValue() * 1000
    if rawF < 0: rawF = 1000
    if rawL < 0: rawL = 1000
    if rawR < 0: rawR = 1000
    if rawFL < 0: rawFL = 1000
    if rawFR < 0: rawFR = 1000
    disF = alpha * rawF + (1 - alpha) * disF
    disL = alpha * rawL + (1 - alpha) * disL
    disR = alpha * rawR + (1 - alpha) * disR
    disFL = alpha * rawFL + (1 - alpha) * disFL
    disFR = alpha * rawFR + (1 - alpha) * disFR

def detect_walls():
    walls = {'N': False, 'E': False, 'S': False, 'W': False}
    def is_wall(sensor_value): return sensor_value < WALL_THRESHOLD
    if current_direction == 'N':
        walls['N'] = is_wall(disF)
        walls['W'] = is_wall(disL)
        walls['E'] = is_wall(disR)
    elif current_direction == 'E':
        walls['E'] = is_wall(disF)
        walls['N'] = is_wall(disL)
        walls['S'] = is_wall(disR)
    elif current_direction == 'S':
        walls['S'] = is_wall(disF)
        walls['E'] = is_wall(disL)
        walls['W'] = is_wall(disR)
    elif current_direction == 'W':
        walls['W'] = is_wall(disF)
        walls['S'] = is_wall(disL)
        walls['N'] = is_wall(disR)
    return walls

def update_maze_walls(walls):
    global maze
    x, y = current_x, current_y
    for direction, wall_exists in walls.items():
        if wall_exists:
            maze[y][x][direction] = True
            dx, dy = get_direction_offset(direction)
            adj_x, adj_y = x + dx, y + dy
            if 0 <= adj_x < MAZE_SIZE and 0 <= adj_y < MAZE_SIZE:
                opposite = get_opposite_direction(direction)
                maze[adj_y][adj_x][opposite] = True
    maze[y][x]['visited'] = True

def get_direction_offset(direction):
    offsets = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}
    return offsets[direction]

def get_opposite_direction(direction):
    opposites = {'N': 'S', 'E': 'W', 'S': 'N', 'W': 'E'}
    return opposites[direction]

def flood_fill(targets):
    distances = [[float('inf')] * MAZE_SIZE for _ in range(MAZE_SIZE)]
    queue = []
    for target_x, target_y in targets:
        distances[target_y][target_x] = 0
        queue.append((target_x, target_y))
    while queue:
        x, y = queue.pop(0)
        current_dist = distances[y][x]
        for direction in ['N', 'E', 'S', 'W']:
            if not maze[y][x][direction]:
                dx, dy = get_direction_offset(direction)
                nx, ny = x + dx, y + dy
                if (0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE and
                    distances[ny][nx] > current_dist + 1):
                    distances[ny][nx] = current_dist + 1
                    queue.append((nx, ny))
    return distances

def get_best_direction(distances):
    x, y = current_x, current_y
    best_direction = None
    best_distance = float('inf')
    for direction in ['N', 'E', 'S', 'W']:
        if not maze[y][x][direction]:
            dx, dy = get_direction_offset(direction)
            nx, ny = x + dx, y + dy
            if (0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE):
                dist = distances[ny][nx]
                if dist < best_distance:
                    best_distance = dist
                    best_direction = direction
    return best_direction

def turn_to_direction(target_direction):
    global current_direction
    if target_direction == current_direction:
        return
    directions = ['N', 'E', 'S', 'W']
    current_idx = directions.index(current_direction)
    target_idx = directions.index(target_direction)
    turn_steps = (target_idx - current_idx) % 4
    if turn_steps == 2:
        turn_angle(180)
        current_direction = target_direction
    elif turn_steps == 1:
        turn_angle(90)
        current_direction = target_direction
    elif turn_steps == 3:
        turn_angle(-90)
        current_direction = target_direction

def turn_angle(angle_deg, max_speed=3.0, accel=0.1):
    angle_rad = angle_deg * 3.14159 / 180.0
    arc_length = (AXLE_LENGTH / 2.0) * angle_rad
    target_rotation = arc_length / WHEEL_RADIUS
    start_left = left_pos_sensor.getValue()
    start_right = right_pos_sensor.getValue()
    current_speed = 2
    while robot.step(timestep) != -1:
        update_sensors()
        left_pos = left_pos_sensor.getValue() - start_left
        right_pos = right_pos_sensor.getValue() - start_right
        avg_turn = (abs(left_pos) + abs(right_pos)) / 2.0
        if avg_turn >= abs(target_rotation):
            break
        direction = 1 if angle_rad > 0 else -1
        if current_speed < max_speed:
            current_speed += accel
        else:
            current_speed = max_speed
        left_motor.setVelocity(-current_speed * direction)
        right_motor.setVelocity(current_speed * direction)
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

def move_forward_distance(distance, speed=None):
    if speed is None:
        speed = SEARCH_SPEED
    target_rotation = distance / WHEEL_RADIUS
    start_left = left_pos_sensor.getValue()
    start_right = right_pos_sensor.getValue()
    stopped_early = False
    while robot.step(timestep) != -1:
        update_sensors()
        left_pos = left_pos_sensor.getValue() - start_left
        right_pos = right_pos_sensor.getValue() - start_right
        avg_pos = (left_pos + right_pos) / 2.0
        if disF < STOP_FRONT_THRESHOLD:
            stopped_early = True
            break
        if abs(avg_pos) >= abs(target_rotation):
            break
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    return stopped_early

def at_goal():
    return (current_x, current_y) in goals

def at_start():
    return (current_x, current_y) == start_pos

def is_maze_fully_explored():
    for y in range(MAZE_SIZE):
        for x in range(MAZE_SIZE):
            if not maze[y][x]['visited']:
                return False
    return True

def get_unvisited_neighbor_direction():
    x, y = current_x, current_y
    for direction in ['N', 'E', 'S', 'W']:
        dx, dy = get_direction_offset(direction)
        nx, ny = x + dx, y + dy
        if (0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE):
            if not maze[y][x][direction] and not maze[ny][nx]['visited']:
                return direction
    return None

def search_run():
    global current_x, current_y, current_direction
    step_count = 0
    while not at_goal():
        update_sensors()
        walls = detect_walls()
        update_maze_walls(walls)
        distances = flood_fill(goals)
        best_direction = get_best_direction(distances)
        if best_direction is None:
            for direction in ['N', 'E', 'S', 'W']:
                if not maze[current_y][current_x][direction]:
                    best_direction = direction
                    break
            if best_direction is None:
                break
        if best_direction != current_direction:
            turn_to_direction(best_direction)
        stopped_early = move_forward_distance(TILE_SIZE)
        dx, dy = get_direction_offset(current_direction)
        current_x += dx
        current_y += dy
        if stopped_early:
            walls = detect_walls()
            walls[current_direction] = True
            update_maze_walls(walls)
        step_count += 1

def flood_fill_to_start():
    distances = [[float('inf')] * MAZE_SIZE for _ in range(MAZE_SIZE)]
    queue = []
    sx, sy = start_pos
    distances[sy][sx] = 0
    queue.append((sx, sy))
    while queue:
        x, y = queue.pop(0)
        current_dist = distances[y][x]
        for direction in ['N', 'E', 'S', 'W']:
            if not maze[y][x][direction]:
                dx, dy = get_direction_offset(direction)
                nx, ny = x + dx, y + dy
                if 0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE:
                    if distances[ny][nx] > current_dist + 1:
                        distances[ny][nx] = current_dist + 1
                        queue.append((nx, ny))
    return distances

def get_best_direction_to_start(distances):
    x, y = current_x, current_y
    best_direction = None
    best_distance = float('inf')
    for direction in ['N', 'E', 'S', 'W']:
        if not maze[y][x][direction]:
            dx, dy = get_direction_offset(direction)
            nx, ny = x + dx, y + dy
            if (0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE):
                dist = distances[ny][nx]
                if dist < best_distance:
                    best_distance = dist
                    best_direction = direction
    return best_direction

def return_to_start_with_exploration():
    global current_x, current_y, current_direction
    print("Starting return to start with exploration...")
    maze_fully_explored = False
    while not at_start():
        update_sensors()
        walls = detect_walls()
        update_maze_walls(walls)
        if not maze_fully_explored:
            maze_fully_explored = is_maze_fully_explored()
        exploration_dir = None
        if not maze_fully_explored:
            exploration_dir = get_unvisited_neighbor_direction()
        if exploration_dir is not None and not maze_fully_explored:
            if exploration_dir != current_direction:
                turn_to_direction(exploration_dir)
            stopped_early = move_forward_distance(TILE_SIZE)
            dx, dy = get_direction_offset(current_direction)
            current_x += dx
            current_y += dy
            if stopped_early:
                walls = detect_walls()
                walls[current_direction] = True
                update_maze_walls(walls)
        else:
            distances = flood_fill_to_start()
            best_dir = get_best_direction_to_start(distances)
            if best_dir is not None and best_dir != current_direction:
                turn_to_direction(best_dir)
            stopped_early = move_forward_distance(TILE_SIZE)
            dx, dy = get_direction_offset(current_direction)
            current_x += dx
            current_y += dy
            if stopped_early:
                walls = detect_walls()
                walls[current_direction] = True
                update_maze_walls(walls)

def create_navigation_arrays():
    distances = flood_fill(goals)
    floodfillArray = [[0 for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
    verticalWallArray = [[False for _ in range(MAZE_SIZE+1)] for _ in range(MAZE_SIZE)]
    horizontalWallArray = [[False for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE+1)]
    for y in range(MAZE_SIZE):
        for x in range(MAZE_SIZE):
            floodfillArray[x][y] = int(distances[y][x]) if distances[y][x] != float('inf') else 999
            if maze[y][x]['W']:
                verticalWallArray[y][x] = True
            if maze[y][x]['E']:
                verticalWallArray[y][x+1] = True
            if maze[y][x]['S']:
                horizontalWallArray[y][x] = True
            if maze[y][x]['N']:
                horizontalWallArray[y+1][x] = True
    return floodfillArray, verticalWallArray, horizontalWallArray

def print_navigation_arrays(floodfillArray, verticalWallArray, horizontalWallArray):
    print("\nFinal maze memory (walls and visited cells):")
    for y in range(MAZE_SIZE):
        for x in range(MAZE_SIZE):
            cell = maze[y][x]
            print(f"Cell ({x},{y}): N={cell['N']} S={cell['S']} E={cell['E']} W={cell['W']} visited={cell['visited']}")
    print(f"\nFlood Fill Array (floodfillArray[x][y]):")
    print("-" * 40)
    print("   ", end="")
    for x in range(MAZE_SIZE):
        print(f"{x:3}", end="")
    print()
    for y in range(MAZE_SIZE-1, -1, -1):
        print(f"{y:2}:", end="")
        for x in range(MAZE_SIZE):
            val = floodfillArray[x][y]
            if val == 999:
                print("inf", end="")
            else:
                print(f"{val:3}", end="")
        print()
    print(f"\nVertical Wall Array (verticalWallArray[y][x]) - {MAZE_SIZE}x{MAZE_SIZE+1}:")
    print("-" * 40)
    print("For robot at (x,y): verticalWallArray[y][x] = left wall, verticalWallArray[y][x+1] = right wall")
    for y in range(MAZE_SIZE-1, -1, -1):
        print(f"y={y:2}: ", end="")
        for x in range(MAZE_SIZE+1):
            print("1" if verticalWallArray[y][x] else "0", end="")
        print()
    print(f"\nHorizontal Wall Array (horizontalWallArray[y][x]) - {MAZE_SIZE+1}x{MAZE_SIZE}:")
    print("-" * 40)
    print("For robot at (x,y): horizontalWallArray[y+1][x] = north wall, horizontalWallArray[y][x] = south wall")
    for y in range(MAZE_SIZE, -1, -1):
        print(f"y={y:2}: ", end="")
        for x in range(MAZE_SIZE):
            print("1" if horizontalWallArray[y][x] else "0", end="")
        print()

def print_visited_stats():
    total_cells = MAZE_SIZE * MAZE_SIZE
    visited_cells = 0
    for y in range(MAZE_SIZE):
        for x in range(MAZE_SIZE):
            if maze[y][x]['visited']:
                visited_cells += 1
    unvisited_cells = total_cells - visited_cells
    print(f"\nMaze exploration summary:")
    print(f"  Total cells:     {total_cells}")
    print(f"  Visited cells:   {visited_cells}")
    print(f"  Unvisited cells: {unvisited_cells}")

def run():
    global timestep, current_x, current_y, current_direction
    for _ in range(15):
        robot.step(timestep)
        update_sensors()
    search_run()
    return_to_start_with_exploration()
    floodfillArray, verticalWallArray, horizontalWallArray = create_navigation_arrays()
    print_navigation_arrays(floodfillArray, verticalWallArray, horizontalWallArray)
    print_visited_stats()

if __name__ == "__main__":
    run()