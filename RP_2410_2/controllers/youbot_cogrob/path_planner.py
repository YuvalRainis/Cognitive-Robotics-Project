import heapq
import math
# Environment limits 
X_MIN, X_MAX = -1.8, 1.8
Y_MIN, Y_MAX = -1.8, 1.8
RESOLUTION = 0.2  # grid size in meters
WORLD_MARGIN = 0.12  # safe margin from world borders

# Helper functions 
def heuristic(a, b):
    """Euclidean distance used as the A* heuristic."""
    return math.hypot(b[0] - a[0], b[1] - a[1])

def neighbors(node):
    """Generate valid 8 connected neighboring grid points within world limits."""
    x, y = node
    for dx, dy in [
        (-RESOLUTION, 0), (RESOLUTION, 0),
        (0, -RESOLUTION), (0, RESOLUTION),
        (-RESOLUTION, -RESOLUTION), (-RESOLUTION, RESOLUTION),
        (RESOLUTION, -RESOLUTION), (RESOLUTION, RESOLUTION)
    ]:
        nx, ny = x + dx, y + dy
        if (X_MIN + WORLD_MARGIN) <= nx <= (X_MAX - WORLD_MARGIN) and \
           (Y_MIN + WORLD_MARGIN) <= ny <= (Y_MAX - WORLD_MARGIN):
            yield (nx, ny)

def is_free(x, y, obstacles, clearance=0.65):
    """Return True if the point (x, y) is inside world bounds and not colliding with any obstacle."""
    if x < (X_MIN + WORLD_MARGIN) or x > (X_MAX - WORLD_MARGIN) or \
       y < (Y_MIN + WORLD_MARGIN) or y > (Y_MAX - WORLD_MARGIN):
        return False
    for ox, oy in obstacles:
        if math.hypot(x - ox, y - oy) < clearance:
            return False
    return True

# Inflate obstacles (used for cubes that are not yet collected)
def inflate_obstacles(obstacles, inflate_radius=0.5, step=0.05):
    """
    Inflate each obstacle into a zone by sampling additional points within a given radius.
    This ensures safe clearance around physical objects.
    """
    inflated = []
    for ox, oy in obstacles:
        for dx in range(-int(inflate_radius / step), int(inflate_radius / step) + 1):
            for dy in range(-int(inflate_radius / step), int(inflate_radius / step) + 1):
                nx = ox + dx * step
                ny = oy + dy * step
                inflated.append((nx, ny))
    print(f"Inflated {len(obstacles)} obstacles → {len(inflated)} points (radius={inflate_radius:.2f})")
    return inflated

# Core A* 
def astar(start, goal, obstacles):
    """A* planner from start → goal. Returns a list of waypoints."""
    # Snap to grid
    start = (round(start[0] / RESOLUTION) * RESOLUTION,
             round(start[1] / RESOLUTION) * RESOLUTION)
    goal = (round(goal[0] / RESOLUTION) * RESOLUTION,
            round(goal[1] / RESOLUTION) * RESOLUTION)
    # Clamp to world bounds
    start = (max(X_MIN, min(X_MAX, start[0])), max(Y_MIN, min(Y_MAX, start[1])))
    goal = (max(X_MIN, min(X_MAX, goal[0])), max(Y_MIN, min(Y_MAX, goal[1])))
    # Make sure start not hugging border
    sx, sy = start
    if sx >= X_MAX - WORLD_MARGIN: sx -= 1e-3
    if sx <= X_MIN + WORLD_MARGIN: sx += 1e-3
    if sy >= Y_MAX - WORLD_MARGIN: sy -= 1e-3
    if sy <= Y_MIN + WORLD_MARGIN: sy += 1e-3
    start = (sx, sy)
    print(f"Starting A* from {start} to {goal}, with {len(obstacles)} obstacles")
    if not is_free(start[0], start[1], obstacles):
        print(f"Start {start} not free! Searching nearest free point...")
        start = find_nearest_free(start, obstacles)
    if not is_free(goal[0], goal[1], obstacles):
        print(f"Goal {goal} not free! Searching nearest free point...")
        goal = find_nearest_free(goal, obstacles)

    # Core A*
    open_set = [(0 + heuristic(start, goal), 0, start, None)]
    came_from = {}
    cost_so_far = {start: 0}
    while open_set:
        _, cost, current, parent = heapq.heappop(open_set)
        if current in came_from:
            continue
        came_from[current] = parent
        if heuristic(current, goal) < RESOLUTION:
            break
        for nb in neighbors(current):
            if not is_free(nb[0], nb[1], obstacles):
                continue
            new_cost = cost + heuristic(current, nb)
            if nb not in cost_so_far or new_cost < cost_so_far[nb]:
                cost_so_far[nb] = new_cost
                priority = new_cost + heuristic(nb, goal)
                heapq.heappush(open_set, (priority, new_cost, nb, current))
    if goal not in came_from:
        if came_from:
            goal = min(came_from.keys(), key=lambda n: heuristic(n, goal))
            print(f"Could not reach exact goal, using closest point: {goal}")
        else:
            print("No path found!")
            return None
        
    # Reconstruct
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from.get(node)
    path.reverse()
    print(f"A* completed: {len(path)} waypoints")
    path = smooth_path(path, obstacles)
    return path

def find_nearest_free(position, obstacles, max_search=1.0):
    """
    Search for the nearest collision-free point around a given position.
    Expands in circular increments up to max_search distance.
    """
    x, y = position
    for radius in [0.1, 0.2, 0.3, 0.5, 0.7, 1.0]:
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            tx = x + radius * math.cos(rad)
            ty = y + radius * math.sin(rad)
            if (X_MIN + WORLD_MARGIN) <= tx <= (X_MAX - WORLD_MARGIN) and \
               (Y_MIN + WORLD_MARGIN) <= ty <= (Y_MAX - WORLD_MARGIN):
                if is_free(tx, ty, obstacles):
                    print(f"Found nearby free spot at ({tx:.2f}, {ty:.2f})")
                    return (tx, ty)
    return position

def smooth_path(path, obstacles):
    """
    Reduce unnecessary waypoints by merging consecutive visible segments.
    """
    if len(path) <= 2:
        return path
    smoothed = [path[0]]
    current_idx = 0
    # Attempt to connect each waypoint directly to the farthest visible waypoint
    while current_idx < len(path) - 1:
        for look_ahead in range(len(path) - 1, current_idx, -1):
            if has_line_of_sight(path[current_idx], path[look_ahead], obstacles):
                smoothed.append(path[look_ahead])
                current_idx = look_ahead
                break
        else:
            current_idx += 1
    print(f"Path smoothed: {len(path)} → {len(smoothed)} waypoints")
    return smoothed

def has_line_of_sight(p1, p2, obstacles, clearance=0.5):
    """
    Check if the straight line between two points is free of obstacles.
    Used for path smoothing.
    """
    steps = int(heuristic(p1, p2) / (RESOLUTION / 2))
    if steps == 0:
        return True
    for i in range(steps + 1):
        t = i / steps
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        if not is_free(x, y, obstacles, clearance):
            return False
    return True
