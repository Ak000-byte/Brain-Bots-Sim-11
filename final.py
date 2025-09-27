
# import requests
# import json
# import time
# import math
# import heapq
# from typing import List, Tuple, Dict, Any, Optional, Set, Union
# from dataclasses import dataclass, field
# from enum import Enum
# import numpy as np
# from collections import defaultdict

# # Enhanced Configuration Management
# class Config:
#     """Centralized configuration for better maintainability and performance"""

#     # Server Configuration
#     BASE_URL = "http://localhost:5001"
#     REQUEST_TIMEOUT = 5.0
#     MAX_RETRIES = 3

#     # Canvas and Grid Configuration
#     CANVAS_WIDTH = 650
#     CANVAS_HEIGHT = 600
#     GRID_RESOLUTION = 10  # Adjustable resolution for performance tuning

#     # Robot Configuration
#     ROBOT_RADIUS = 18
#     ROBOT_START = (320, 300)
#     SAFETY_MARGIN_MULTIPLIER = 1.5  # Dynamic safety margin

#     # Performance Configuration
#     MAX_SEARCH_NODES = 10000
#     PATH_SMOOTHING_ITERATIONS = 3
#     WAYPOINT_DISTANCE_THRESHOLD = 25  # Minimum distance between waypoints

#     # Algorithm Selection
#     USE_JPS = True  # Use JPS when possible, fallback to A*
#     ENABLE_PATH_SMOOTHING = True
#     ENABLE_DYNAMIC_OBSTACLES = True

#     # Navigation Configuration
#     GOAL_TOLERANCE = 33
#     MOVEMENT_DELAY = 0.3  # Reduced for faster navigation
#     MAX_PLANNING_TIME = 5.0  # Timeout for pathfinding

# class PathfindingAlgorithm(Enum):
#     """Available pathfinding algorithms"""
#     A_STAR = "a_star"
#     JPS = "jps"
#     HYBRID = "hybrid"  # Automatic selection based on environment

# @dataclass
# class Point:
#     """Enhanced Point class with additional utilities"""
#     x: float
#     y: float

#     def distance_to(self, other: 'Point') -> float:
#         return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

#     def manhattan_distance_to(self, other: 'Point') -> float:
#         """Manhattan distance for grid-based calculations"""
#         return abs(self.x - other.x) + abs(self.y - other.y)

#     def to_grid(self, resolution: int = Config.GRID_RESOLUTION) -> Tuple[int, int]:
#         """Convert to grid coordinates"""
#         return (int(self.x // resolution), int(self.y // resolution))

#     @classmethod
#     def from_grid(cls, grid_x: int, grid_y: int, resolution: int = Config.GRID_RESOLUTION) -> 'Point':
#         """Create point from grid coordinates"""
#         return cls(grid_x * resolution, grid_y * resolution)

#     def is_valid(self, canvas_width: int = Config.CANVAS_WIDTH, 
#                  canvas_height: int = Config.CANVAS_HEIGHT, 
#                  robot_radius: int = Config.ROBOT_RADIUS) -> bool:
#         """Check if point is within valid bounds"""
#         return (robot_radius <= self.x <= canvas_width - robot_radius and 
#                 robot_radius <= self.y <= canvas_height - robot_radius)

#     def __hash__(self):
#         return hash((round(self.x, 2), round(self.y, 2)))

#     def __eq__(self, other):
#         if not isinstance(other, Point):
#             return False
#         return abs(self.x - other.x) < 0.01 and abs(self.y - other.y) < 0.01

# @dataclass
# class Obstacle:
#     """Enhanced Obstacle class with dynamic properties"""
#     x: float
#     y: float
#     size: float = 25
#     velocity_x: float = 0.0  # For dynamic obstacles
#     velocity_y: float = 0.0
#     prediction_time: float = 2.0  # How far to predict movement
#     is_dynamic: bool = False

#     def point(self) -> Point:
#         return Point(self.x, self.y)

#     def predicted_point(self, time_ahead: float) -> Point:
#         """Get predicted position for dynamic obstacles"""
#         if not self.is_dynamic:
#             return self.point()

#         pred_x = self.x + self.velocity_x * time_ahead
#         pred_y = self.y + self.velocity_y * time_ahead
#         return Point(pred_x, pred_y)

#     def contains_point(self, point: Point, robot_radius: float = Config.ROBOT_RADIUS, 
#                       time_ahead: float = 0.0) -> bool:
#         """Check if point collides with obstacle (with prediction)"""
#         obstacle_pos = self.predicted_point(time_ahead)
#         safety_radius = (self.size/2 + robot_radius) * Config.SAFETY_MARGIN_MULTIPLIER
#         return point.distance_to(obstacle_pos) <= safety_radius

# class GridMap:
#     """Enhanced grid-based map representation"""

#     def __init__(self, width: int, height: int, resolution: int = Config.GRID_RESOLUTION):
#         self.width = width
#         self.height = height
#         self.resolution = resolution
#         self.grid_width = width // resolution
#         self.grid_height = height // resolution
#         try:
#             self.grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
#         except ImportError:
#             # Fallback if numpy not available
#             self.grid = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]

#     def is_valid_grid_pos(self, gx: int, gy: int) -> bool:
#         """Check if grid position is valid"""
#         return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

#     def is_walkable(self, gx: int, gy: int) -> bool:
#         """Check if grid cell is walkable"""
#         if not self.is_valid_grid_pos(gx, gy):
#             return False
#         return self.grid[gy][gx] == 0

#     def set_obstacle(self, gx: int, gy: int, value: int = 1):
#         """Set obstacle in grid"""
#         if self.is_valid_grid_pos(gx, gy):
#             self.grid[gy][gx] = value

#     def add_obstacle(self, obstacle: Obstacle, robot_radius: float = Config.ROBOT_RADIUS):
#         """Add obstacle to grid with safety margins"""
#         center_gx = int(obstacle.x // self.resolution)
#         center_gy = int(obstacle.y // self.resolution)

#         # Calculate expanded radius with safety margin
#         expanded_radius = (obstacle.size/2 + robot_radius) * Config.SAFETY_MARGIN_MULTIPLIER
#         grid_radius = int(math.ceil(expanded_radius / self.resolution))

#         # Mark cells as obstacles
#         for dy in range(-grid_radius, grid_radius + 1):
#             for dx in range(-grid_radius, grid_radius + 1):
#                 gx, gy = center_gx + dx, center_gy + dy
#                 if self.is_valid_grid_pos(gx, gy):
#                     # Use distance check for circular obstacles
#                     dist = math.sqrt(dx*dx + dy*dy) * self.resolution
#                     if dist <= expanded_radius:
#                         self.set_obstacle(gx, gy, 1)

# class JumpPointSearch:
#     """Optimized Jump Point Search implementation"""

#     def __init__(self, grid_map: GridMap):
#         self.grid_map = grid_map
#         self.grid = grid_map.grid
#         self.grid_width = grid_map.grid_width
#         self.grid_height = grid_map.grid_height

#     def find_path(self, start: Point, goal: Point) -> List[Point]:
#         """Find path using JPS algorithm"""
#         start_grid = start.to_grid(self.grid_map.resolution)
#         goal_grid = goal.to_grid(self.grid_map.resolution)

#         # Check if start and goal are valid
#         if not (self.grid_map.is_walkable(start_grid[0], start_grid[1]) and 
#                 self.grid_map.is_walkable(goal_grid[0], goal_grid[1])):
#             return []

#         open_set = [(0, start_grid)]
#         came_from = {}
#         g_score = {start_grid: 0}
#         closed_set = set()

#         while open_set:
#             current_f, current = heapq.heappop(open_set)

#             if current in closed_set:
#                 continue

#             closed_set.add(current)

#             if current == goal_grid:
#                 return self._reconstruct_path(came_from, current)

#             # Get jump points
#             for dx in [-1, 0, 1]:
#                 for dy in [-1, 0, 1]:
#                     if dx == 0 and dy == 0:
#                         continue

#                     jump_point = self._jump(current[0], current[1], dx, dy, goal_grid)
#                     if jump_point and jump_point not in closed_set:
#                         distance = math.sqrt((jump_point[0] - current[0])**2 + 
#                                            (jump_point[1] - current[1])**2)
#                         tentative_g = g_score[current] + distance

#                         if jump_point not in g_score or tentative_g < g_score[jump_point]:
#                             g_score[jump_point] = tentative_g
#                             h_score = math.sqrt((jump_point[0] - goal_grid[0])**2 + 
#                                               (jump_point[1] - goal_grid[1])**2)
#                             f_score = tentative_g + h_score
#                             heapq.heappush(open_set, (f_score, jump_point))
#                             came_from[jump_point] = current

#         return []

#     def _jump(self, x: int, y: int, dx: int, dy: int, goal: Tuple[int, int]) -> Optional[Tuple[int, int]]:
#         """Jump point search core logic"""
#         nx, ny = x + dx, y + dy

#         if not self.grid_map.is_walkable(nx, ny):
#             return None

#         if (nx, ny) == goal:
#             return (nx, ny)

#         # Check for forced neighbors
#         if dx != 0 and dy != 0:  # Diagonal movement
#             if ((self.grid_map.is_walkable(nx - dx, ny) and not self.grid_map.is_walkable(nx - dx, ny - dy)) or
#                 (self.grid_map.is_walkable(nx, ny - dy) and not self.grid_map.is_walkable(nx - dx, ny - dy))):
#                 return (nx, ny)

#             # Recursive horizontal and vertical jumps
#             if (self._jump(nx, ny, dx, 0, goal) or self._jump(nx, ny, 0, dy, goal)):
#                 return (nx, ny)
#         else:  # Horizontal or vertical movement
#             if dx != 0:  # Horizontal
#                 if ((self.grid_map.is_walkable(nx, ny + 1) and not self.grid_map.is_walkable(nx - dx, ny + 1)) or
#                     (self.grid_map.is_walkable(nx, ny - 1) and not self.grid_map.is_walkable(nx - dx, ny - 1))):
#                     return (nx, ny)
#             else:  # Vertical
#                 if ((self.grid_map.is_walkable(nx + 1, ny) and not self.grid_map.is_walkable(nx + 1, ny - dy)) or
#                     (self.grid_map.is_walkable(nx - 1, ny) and not self.grid_map.is_walkable(nx - 1, ny - dy))):
#                     return (nx, ny)

#         return self._jump(nx, ny, dx, dy, goal)

#     def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Point]:
#         """Reconstruct path from grid coordinates to Points"""
#         path_grid = [current]
#         while current in came_from:
#             current = came_from[current]
#             path_grid.append(current)
#         path_grid.reverse()

#         # Convert to Point objects
#         path_points = [Point.from_grid(gx, gy, self.grid_map.resolution) 
#                       for gx, gy in path_grid]
#         return path_points

# class EnhancedAStarPathfinder:
#     """Enhanced A* pathfinder with modern optimizations"""

#     def __init__(self, canvas_width: int = Config.CANVAS_WIDTH, 
#                  canvas_height: int = Config.CANVAS_HEIGHT, 
#                  robot_radius: int = Config.ROBOT_RADIUS, 
#                  step_size: float = 15.0):
#         self.canvas_width = canvas_width
#         self.canvas_height = canvas_height
#         self.robot_radius = robot_radius
#         self.step_size = step_size
#         self.grid_map = GridMap(canvas_width, canvas_height)

#     def is_valid_point(self, point: Point, obstacles: List[Obstacle], time_ahead: float = 0.0) -> bool:
#         """Enhanced validity check with dynamic obstacles"""
#         if not point.is_valid(self.canvas_width, self.canvas_height, self.robot_radius):
#             return False

#         # Check dynamic obstacles with prediction
#         for obstacle in obstacles:
#             if obstacle.contains_point(point, self.robot_radius, time_ahead):
#                 return False

#         return True

#     def is_line_clear(self, start: Point, end: Point, obstacles: List[Obstacle], 
#                      step_size: float = 2.0, time_ahead: float = 0.0) -> bool:
#         """Enhanced line clearance check"""
#         distance = start.distance_to(end)
#         if distance == 0:
#             return True

#         steps = int(math.ceil(distance / step_size))
#         dx = (end.x - start.x) / steps
#         dy = (end.y - start.y) / steps

#         for i in range(steps + 1):
#             test_point = Point(start.x + i * dx, start.y + i * dy)
#             if not self.is_valid_point(test_point, obstacles, time_ahead):
#                 return False

#         return True

#     def heuristic(self, point: Point, goal: Point, algorithm: str = "euclidean") -> float:
#         """Enhanced heuristic with multiple options"""
#         if algorithm == "manhattan":
#             return point.manhattan_distance_to(goal)
#         elif algorithm == "diagonal":
#             dx = abs(point.x - goal.x)
#             dy = abs(point.y - goal.y)
#             return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
#         else:  # euclidean (default)
#             return point.distance_to(goal)

#     def get_neighbors(self, point: Point, obstacles: List[Obstacle], 
#                      time_ahead: float = 0.0) -> List[Point]:
#         """Enhanced neighbor generation with optimizations"""
#         neighbors = []
#         directions = [
#             (1, 0), (-1, 0), (0, 1), (0, -1),  # Cardinal
#             (1, 1), (-1, -1), (1, -1), (-1, 1)  # Diagonal
#         ]

#         for dx, dy in directions:
#             if dx != 0 and dy != 0:
#                 factor = self.step_size / math.sqrt(2)
#             else:
#                 factor = self.step_size

#             new_point = Point(point.x + dx * factor, point.y + dy * factor)
#             if (self.is_valid_point(new_point, obstacles, time_ahead) and 
#                 self.is_line_clear(point, new_point, obstacles, time_ahead=time_ahead)):
#                 neighbors.append(new_point)

#         return neighbors

#     def find_path(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> List[Point]:
#         """Enhanced A* with tie-breaking and optimizations"""
#         open_set: List[Tuple[float, int, Point]] = []
#         counter = 0
#         heapq.heappush(open_set, (0.0, counter, start))

#         came_from: Dict[Point, Point] = {}
#         g_score = {start: 0.0}
#         f_score = {start: self.heuristic(start, goal)}
#         closed = set()
#         nodes_explored = 0

#         start_time = time.time()

#         while open_set and nodes_explored < Config.MAX_SEARCH_NODES:
#             # Check timeout
#             if time.time() - start_time > Config.MAX_PLANNING_TIME:
#                 print(f"⚠️ Pathfinding timeout after {Config.MAX_PLANNING_TIME}s")
#                 break

#             _, _, current = heapq.heappop(open_set)
#             if current in closed:
#                 continue

#             closed.add(current)
#             nodes_explored += 1

#             if current.distance_to(goal) <= Config.GOAL_TOLERANCE:
#                 path = []
#                 while current in came_from:
#                     path.append(current)
#                     current = came_from[current]
#                 path.append(start)
#                 print(f"✅ A* found path with {nodes_explored} nodes explored")
#                 return list(reversed(path))

#             for neighbor in self.get_neighbors(current, obstacles):
#                 if neighbor in closed:
#                     continue

#                 tentative_g = g_score[current] + current.distance_to(neighbor)

#                 if neighbor not in g_score or tentative_g < g_score[neighbor]:
#                     came_from[neighbor] = current
#                     g_score[neighbor] = tentative_g
#                     f_val = tentative_g + self.heuristic(neighbor, goal)
#                     counter += 1
#                     heapq.heappush(open_set, (f_val, counter, neighbor))

#         print(f"❌ A* failed to find path after exploring {nodes_explored} nodes")
#         return []

# class HybridPathfinder:
#     """Intelligent pathfinder that chooses the best algorithm"""

#     def __init__(self):
#         self.a_star = EnhancedAStarPathfinder()
#         self.grid_map = GridMap(Config.CANVAS_WIDTH, Config.CANVAS_HEIGHT)
#         self.jps = None

#     def _analyze_environment(self, obstacles: List[Obstacle]) -> Dict[str, Any]:
#         """Analyze environment to choose optimal algorithm"""
#         total_cells = self.grid_map.grid_width * self.grid_map.grid_height

#         # Reset grid
#         try:
#             self.grid_map.grid.fill(0)
#         except AttributeError:
#             # Fallback for list-based grid
#             for y in range(self.grid_map.grid_height):
#                 for x in range(self.grid_map.grid_width):
#                     self.grid_map.grid[y][x] = 0

#         # Add obstacles to grid
#         for obstacle in obstacles:
#             self.grid_map.add_obstacle(obstacle)

#         try:
#             occupied_cells = np.count_nonzero(self.grid_map.grid)
#         except:
#             # Fallback counting
#             occupied_cells = sum(sum(1 for cell in row if cell != 0) 
#                                for row in self.grid_map.grid)

#         obstacle_density = occupied_cells / total_cells

#         # Check for large open areas (good for JPS)
#         open_area_score = self._calculate_open_area_score()

#         return {
#             'obstacle_density': obstacle_density,
#             'open_area_score': open_area_score,
#             'total_obstacles': len(obstacles),
#             'dynamic_obstacles': sum(1 for obs in obstacles if obs.is_dynamic)
#         }

#     def _calculate_open_area_score(self) -> float:
#         """Calculate how much open area exists (higher score = more open)"""
#         try:
#             visited = np.zeros_like(self.grid_map.grid, dtype=bool)
#         except:
#             # Fallback for list-based grid
#             visited = [[False for _ in range(self.grid_map.grid_width)] 
#                       for _ in range(self.grid_map.grid_height)]

#         largest_area = 0

#         for y in range(self.grid_map.grid_height):
#             for x in range(self.grid_map.grid_width):
#                 if not visited[y][x] and self.grid_map.is_walkable(x, y):
#                     area_size = self._flood_fill_size(x, y, visited)
#                     largest_area = max(largest_area, area_size)

#         try:
#             total_walkable = np.count_nonzero(np.array(self.grid_map.grid) == 0)
#         except:
#             total_walkable = sum(sum(1 for cell in row if cell == 0) 
#                                for row in self.grid_map.grid)

#         return (largest_area / total_walkable) if total_walkable > 0 else 0

#     def _flood_fill_size(self, start_x: int, start_y: int, visited) -> int:
#         """Calculate size of connected open area using flood fill"""
#         stack = [(start_x, start_y)]
#         size = 0

#         while stack:
#             x, y = stack.pop()
#             if (not self.grid_map.is_valid_grid_pos(x, y) or 
#                 visited[y][x] or not self.grid_map.is_walkable(x, y)):
#                 continue

#             visited[y][x] = True
#             size += 1

#             # Add neighbors
#             for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
#                 stack.append((x + dx, y + dy))

#         return size

#     def find_path(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> List[Point]:
#         """Find path using the most appropriate algorithm"""
#         env_analysis = self._analyze_environment(obstacles)

#         # Decision logic for algorithm selection
#         use_jps = (Config.USE_JPS and 
#                   env_analysis['obstacle_density'] < 0.4 and  # Not too cluttered
#                   env_analysis['open_area_score'] > 0.3 and   # Has decent open areas
#                   env_analysis['dynamic_obstacles'] == 0)     # No dynamic obstacles

#         if use_jps:
#             print(f"🔥 Using JPS (Open area: {env_analysis['open_area_score']:.2f}, Density: {env_analysis['obstacle_density']:.2f})")
#             if self.jps is None:
#                 self.jps = JumpPointSearch(self.grid_map)
#             try:
#                 path = self.jps.find_path(start, goal)
#                 if path:
#                     return self._post_process_path(path, obstacles)
#             except Exception as e:
#                 print(f"⚠️ JPS failed: {e}, falling back to A*")

#         # Fallback to A* or if JPS not suitable
#         print(f"🎯 Using Enhanced A* (Obstacles: {env_analysis['total_obstacles']}, Dynamic: {env_analysis['dynamic_obstacles']})")
#         path = self.a_star.find_path(start, goal, obstacles)
#         return self._post_process_path(path, obstacles)

#     def _post_process_path(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
#         """Post-process path for optimization"""
#         if not path or not Config.ENABLE_PATH_SMOOTHING:
#             return path

#         # Path smoothing
#         smoothed_path = self._smooth_path(path, obstacles)

#         # Remove redundant waypoints
#         optimized_path = self._remove_redundant_waypoints(smoothed_path, obstacles)

#         print(f"📈 Path optimized: {len(path)} -> {len(optimized_path)} waypoints")
#         return optimized_path

#     def _smooth_path(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
#         """Smooth path by removing unnecessary waypoints"""
#         if len(path) <= 2:
#             return path

#         smoothed = [path[0]]  # Keep start point

#         i = 0
#         while i < len(path) - 1:
#             # Try to connect current point to the farthest possible point
#             farthest_idx = i + 1

#             for j in range(i + 2, len(path)):
#                 if self.a_star.is_line_clear(path[i], path[j], obstacles):
#                     farthest_idx = j
#                 else:
#                     break

#             smoothed.append(path[farthest_idx])
#             i = farthest_idx

#         return smoothed

#     def _remove_redundant_waypoints(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
#         """Remove waypoints that are too close together"""
#         if len(path) <= 2:
#             return path

#         optimized = [path[0]]

#         for i in range(1, len(path)):
#             if (i == len(path) - 1 or  # Keep last point
#                 optimized[-1].distance_to(path[i]) >= Config.WAYPOINT_DISTANCE_THRESHOLD):
#                 optimized.append(path[i])

#         return optimized

# class EnhancedRobotController:
#     """Enhanced robot controller with modern features"""

#     def __init__(self, server_url: str = Config.BASE_URL):
#         self.server_url = server_url
#         self.pathfinder = HybridPathfinder()
#         try:
#             self.session = requests.Session()  # Reuse connections
#         except:
#             self.session = None
#         self.last_obstacles = []
#         self.performance_stats = {
#             'total_navigations': 0,
#             'successful_navigations': 0,
#             'average_path_length': 0,
#             'average_compute_time': 0
#         }

#     def _make_request(self, method: str, endpoint: str, **kwargs) -> Optional[requests.Response]:
#         """Enhanced request handling with retries and error handling"""
#         url = f"{self.server_url}{endpoint}"

#         for attempt in range(Config.MAX_RETRIES):
#             try:
#                 if self.session:
#                     response = self.session.request(
#                         method, url, 
#                         timeout=Config.REQUEST_TIMEOUT,
#                         **kwargs
#                     )
#                 else:
#                     response = requests.request(
#                         method, url,
#                         timeout=Config.REQUEST_TIMEOUT,
#                         **kwargs
#                     )

#                 if response.status_code == 200:
#                     return response
#                 else:
#                     print(f"⚠️ Server returned status {response.status_code}")

#             except requests.exceptions.RequestException as e:
#                 if attempt < Config.MAX_RETRIES - 1:
#                     print(f"🔄 Request failed (attempt {attempt + 1}), retrying... Error: {e}")
#                     time.sleep(0.5 * (attempt + 1))  # Exponential backoff
#                 else:
#                     print(f"❌ Request failed after {Config.MAX_RETRIES} attempts: {e}")
#             except Exception as e:
#                 print(f"❌ Unexpected error: {e}")

#         return None

#     def get_robot_status(self) -> Dict[str, Any]:
#         """Get current robot status with enhanced error handling"""
#         response = self._make_request("GET", "/status")
#         return response.json() if response else {}

#     def get_obstacles(self) -> List[Dict[str, Any]]:
#         """Get current obstacles from server"""
#         response = self._make_request("GET", "/obstacles")
#         if response:
#             return response.json().get('obstacles', [])
#         return []

#     def move_robot(self, x: float, y: float) -> bool:
#         """Move robot to specific coordinates"""
#         response = self._make_request(
#             "POST", "/move", 
#             json={"x": x, "y": y},
#             headers={"Content-Type": "application/json"}
#         )
#         return response is not None

#     def set_goal(self, x: float, y: float) -> bool:
#         """Set goal position"""
#         response = self._make_request(
#             "POST", "/goal", 
#             json={"x": x, "y": y},
#             headers={"Content-Type": "application/json"}
#         )
#         if response:
#             print(f"🎯 Goal set at ({x:.1f}, {y:.1f})")
#         return response is not None

#     def reset_simulation(self) -> bool:
#         """Reset simulation state"""
#         response = self._make_request("POST", "/reset")
#         if response:
#             print("✅ Simulation reset successful")
#         return response is not None

#     def generate_random_obstacles(self, count: int = 8) -> bool:
#         """Generate random obstacles"""
#         response = self._make_request(
#             "POST", "/obstacles/random", 
#             json={"count": count},
#             headers={"Content-Type": "application/json"}
#         )
#         if response:
#             print(f"🚧 Generated {count} random obstacles")
#         return response is not None

#     def broadcast_reroute(self, obstacle_coords: Tuple[float, float]) -> bool:
#         """Broadcast reroute notification"""
#         response = self._make_request(
#             "POST", "/broadcast",
#             json={"message": f"Rerouting around obstacle at {obstacle_coords}"},
#             headers={"Content-Type": "application/json"}
#         )
#         return response is not None

#     def is_goal_reached(self) -> bool:
#         """Check if goal is reached"""
#         response = self._make_request("GET", "/goal/status")
#         if response:
#             return response.json().get('goal_reached', False)
#         return False

#     def navigate_to_goal(self, start_x: float, start_y: float, 
#                         goal_x: float, goal_y: float, 
#                         enable_dynamic_replanning: bool = True) -> bool:
#         """Enhanced navigation with dynamic replanning"""

#         print(f"🚀 Starting enhanced navigation from ({start_x:.1f}, {start_y:.1f}) to ({goal_x:.1f}, {goal_y:.1f})")

#         # Performance tracking
#         navigation_start = time.time()
#         self.performance_stats['total_navigations'] += 1

#         # Set the goal
#         if not self.set_goal(goal_x, goal_y):
#             print("❌ Failed to set goal")
#             return False

#         # Get current obstacles and create enhanced obstacle objects
#         obstacles_data = self.get_obstacles()
#         obstacles = []

#         for obs_data in obstacles_data:
#             obstacle = Obstacle(
#                 x=obs_data['x'], 
#                 y=obs_data['y'], 
#                 size=obs_data.get('size', 25)
#             )
#             # Add dynamic properties if available
#             if 'velocity_x' in obs_data:
#                 obstacle.velocity_x = obs_data['velocity_x']
#                 obstacle.velocity_y = obs_data.get('velocity_y', 0)
#                 obstacle.is_dynamic = True

#             obstacles.append(obstacle)

#         print(f"🗺️ Environment: {len(obstacles)} obstacles ({sum(1 for o in obstacles if o.is_dynamic)} dynamic)")

#         # Initial path planning
#         start_point = Point(start_x, start_y)
#         goal_point = Point(goal_x, goal_y)

#         print("🧠 Computing optimal path...")
#         path_start_time = time.time()
#         path = self.pathfinder.find_path(start_point, goal_point, obstacles)
#         compute_time = time.time() - path_start_time

#         if not path:
#             print("❌ No collision-free path found!")
#             return False

#         # Calculate path statistics
#         path_length = sum(path[i].distance_to(path[i+1]) for i in range(len(path)-1))
#         print(f"✅ Path computed! Length: {path_length:.1f}px, Time: {compute_time:.3f}s, Waypoints: {len(path)}")

#         # Update performance stats
#         self.performance_stats['average_compute_time'] = (
#             (self.performance_stats['average_compute_time'] * (self.performance_stats['total_navigations'] - 1) + compute_time) / 
#             self.performance_stats['total_navigations']
#         )

#         # Execute path with dynamic replanning
#         success = self._execute_path_with_replanning(path, goal_point, obstacles, enable_dynamic_replanning)

#         # Update performance statistics
#         total_time = time.time() - navigation_start
#         if success:
#             self.performance_stats['successful_navigations'] += 1
#             self.performance_stats['average_path_length'] = (
#                 (self.performance_stats['average_path_length'] * (self.performance_stats['successful_navigations'] - 1) + path_length) / 
#                 self.performance_stats['successful_navigations']
#             )
#             print(f"🎉 Navigation completed successfully in {total_time:.2f}s!")
#         else:
#             print(f"❌ Navigation failed after {total_time:.2f}s")

#         return success

#     def _execute_path_with_replanning(self, path: List[Point], goal: Point, 
#                                     original_obstacles: List[Obstacle], 
#                                     enable_replanning: bool) -> bool:
#         """Execute path with dynamic replanning capabilities"""

#         current_path = path[:]
#         waypoint_idx = 0
#         replanning_count = 0
#         max_replanning = 3

#         while waypoint_idx < len(current_path):
#             waypoint = current_path[waypoint_idx]
#             print(f"📍 Moving to waypoint {waypoint_idx + 1}/{len(current_path)}: ({waypoint.x:.1f}, {waypoint.y:.1f})")

#             if not self.move_robot(waypoint.x, waypoint.y):
#                 print(f"❌ Failed to move to waypoint {waypoint_idx + 1}")
#                 return False

#             # Wait for movement
#             time.sleep(Config.MOVEMENT_DELAY)

#             # Check for goal reached
#             if self.is_goal_reached():
#                 print("🎯 Goal reached!")
#                 return True

#             # Dynamic obstacle detection and replanning
#             if enable_replanning and Config.ENABLE_DYNAMIC_OBSTACLES:
#                 current_obstacles_data = self.get_obstacles()

#                 # Check if obstacles have changed significantly
#                 if self._obstacles_changed(original_obstacles, current_obstacles_data):
#                     print("⚠️ Obstacle configuration changed, replanning...")

#                     if replanning_count < max_replanning:
#                         new_obstacles = [Obstacle(obs['x'], obs['y'], obs.get('size', 25)) 
#                                        for obs in current_obstacles_data]

#                         current_pos = Point(waypoint.x, waypoint.y)
#                         new_path = self.pathfinder.find_path(current_pos, goal, new_obstacles)

#                         if new_path:
#                             current_path = new_path
#                             waypoint_idx = 0
#                             replanning_count += 1
#                             print(f"🔄 Replanned path with {len(new_path)} waypoints (attempt {replanning_count})")

#                             # Broadcast reroute notification
#                             self.broadcast_reroute((waypoint.x, waypoint.y))
#                             continue
#                         else:
#                             print("⚠️ Replanning failed, continuing with original path")
#                     else:
#                         print(f"⚠️ Max replanning attempts ({max_replanning}) reached")

#             waypoint_idx += 1

#         # Final goal check
#         return self.is_goal_reached()

#     def _obstacles_changed(self, original: List[Obstacle], current_data: List[Dict]) -> bool:
#         """Check if obstacles have changed significantly"""
#         if len(original) != len(current_data):
#             return True

#         # Simple change detection - could be enhanced
#         threshold = 10.0  # pixels

#         for i, obs_data in enumerate(current_data):
#             if i >= len(original):
#                 return True

#             orig = original[i]
#             if (abs(orig.x - obs_data['x']) > threshold or 
#                 abs(orig.y - obs_data['y']) > threshold):
#                 return True

#         return False

#     def get_performance_stats(self) -> Dict[str, Any]:
#         """Get performance statistics"""
#         success_rate = (self.performance_stats['successful_navigations'] / 
#                        max(1, self.performance_stats['total_navigations']) * 100)

#         return {
#             **self.performance_stats,
#             'success_rate_percent': success_rate
#         }

#     def print_performance_stats(self):
#         """Print detailed performance statistics"""
#         stats = self.get_performance_stats()
#         print(f"""
# 📊 Performance Statistics:
# ├── Total Navigations: {stats['total_navigations']}
# ├── Successful: {stats['successful_navigations']} ({stats['success_rate_percent']:.1f}%)
# ├── Average Path Length: {stats['average_path_length']:.1f} pixels
# └── Average Compute Time: {stats['average_compute_time']:.3f} seconds
#         """)

# def main():
#     """Enhanced main execution function"""
#     print("=" * 60)
#     print("🤖 ENHANCED ROBOT NAVIGATION SYSTEM v2.0")
#     print("🔥 Features: Hybrid JPS/A*, Dynamic Replanning, Smart Config")
#     print("=" * 60)

#     controller = EnhancedRobotController()

#     # System checks
#     print("🔍 System Checks:")
#     print(f"├── Server URL: {Config.BASE_URL}")
#     print(f"├── Grid Resolution: {Config.GRID_RESOLUTION}px")
#     print(f"├── JPS Enabled: {Config.USE_JPS}")
#     print(f"├── Path Smoothing: {Config.ENABLE_PATH_SMOOTHING}")
#     print(f"└── Dynamic Obstacles: {Config.ENABLE_DYNAMIC_OBSTACLES}")
#     print()

#     # Test server connection
#     status = controller.get_robot_status()
#     if not status:
#         print("❌ Cannot connect to server. Please ensure server.py is running on localhost:5001")
#         return
#     else:
#         print("✅ Server connection successful")

#     while True:
#         print(f"""
# 🎮 Navigation Options:
# 1. Quick Test (Default scenario with 8 obstacles)
# 2. Custom Navigation (Specify coordinates)
# 3. Corner Navigation (NE, NW, SE, SW)
# 4. Benchmark Suite (Multiple test scenarios)
# 5. Obstacle Stress Test (High obstacle density)
# 6. Performance Statistics
# 7. Reset Simulation
# 8. Advanced Configuration
# 9. Exit
#         """)

#         choice = input("Choose option (1-9): ").strip()

#         try:
#             if choice == '1':
#                 # Quick test scenario
#                 print("🚀 Quick Test Scenario")
#                 controller.reset_simulation()
#                 time.sleep(1)

#                 controller.generate_random_obstacles(8)
#                 success = controller.navigate_to_goal(320, 300, 550, 80)
#                 print(f"Result: {'✅ SUCCESS' if success else '❌ FAILED'}")

#             elif choice == '2':
#                 # Custom navigation
#                 print("📍 Custom Navigation")
#                 try:
#                     start_x = float(input(f"Start X (default {Config.ROBOT_START[0]}): ") or str(Config.ROBOT_START[0]))
#                     start_y = float(input(f"Start Y (default {Config.ROBOT_START[1]}): ") or str(Config.ROBOT_START[1]))
#                     goal_x = float(input("Goal X: "))
#                     goal_y = float(input("Goal Y: "))

#                     success = controller.navigate_to_goal(start_x, start_y, goal_x, goal_y)
#                     print(f"Result: {'✅ SUCCESS' if success else '❌ FAILED'}")
#                 except ValueError:
#                     print("❌ Invalid coordinates")

#             elif choice == '3':
#                 # Corner navigation
#                 corners = {
#                     'NE': (630, 20), 'NW': (20, 20),
#                     'SE': (630, 580), 'SW': (20, 580)
#                 }

#                 print("🏃 Corner Navigation")
#                 corner = input("Enter corner (NE, NW, SE, SW): ").strip().upper()

#                 if corner in corners:
#                     goal_x, goal_y = corners[corner]
#                     success = controller.navigate_to_goal(320, 300, goal_x, goal_y)
#                     print(f"Result: {'✅ SUCCESS' if success else '❌ FAILED'}")
#                 else:
#                     print("❌ Invalid corner")

#             elif choice == '4':
#                 # Benchmark suite
#                 print("🏁 Running Benchmark Suite...")

#                 scenarios = [
#                     ("Light obstacles", 320, 300, 550, 80, 5),
#                     ("Medium obstacles", 320, 300, 100, 100, 8),
#                     ("Heavy obstacles", 320, 300, 630, 300, 12),
#                     ("Extreme obstacles", 320, 300, 20, 580, 15),
#                 ]

#                 results = []
#                 for name, sx, sy, gx, gy, obs_count in scenarios:
#                     print(f"\n📋 {name} ({obs_count} obstacles)")
#                     controller.reset_simulation()
#                     time.sleep(0.5)
#                     controller.generate_random_obstacles(obs_count)

#                     success = controller.navigate_to_goal(sx, sy, gx, gy)
#                     results.append((name, success))
#                     time.sleep(1)

#                 print(f"\n📊 Benchmark Results:")
#                 for name, success in results:
#                     status = "✅ PASS" if success else "❌ FAIL"
#                     print(f"├── {name}: {status}")

#             elif choice == '5':
#                 # Obstacle stress test
#                 print("🔥 Obstacle Stress Test")
#                 obstacle_counts = [10, 15, 20, 25]

#                 for count in obstacle_counts:
#                     print(f"\n🧪 Testing with {count} obstacles...")
#                     controller.reset_simulation()
#                     time.sleep(0.5)
#                     controller.generate_random_obstacles(count)

#                     success = controller.navigate_to_goal(320, 300, 550, 80)
#                     result = "✅ PASSED" if success else "❌ FAILED"
#                     print(f"   {count} obstacles: {result}")

#                     if not success:
#                         print("   ⚠️ Stress test limit reached")
#                         break
#                     time.sleep(1)

#             elif choice == '6':
#                 # Performance statistics
#                 controller.print_performance_stats()

#             elif choice == '7':
#                 # Reset simulation
#                 if controller.reset_simulation():
#                     print("✅ Simulation reset complete")
#                 else:
#                     print("❌ Reset failed")

#             elif choice == '8':
#                 # Advanced configuration
#                 print("⚙️ Advanced Configuration")
#                 print(f"Current settings:")
#                 print(f"├── Grid Resolution: {Config.GRID_RESOLUTION}")
#                 print(f"├── Use JPS: {Config.USE_JPS}")
#                 print(f"├── Path Smoothing: {Config.ENABLE_PATH_SMOOTHING}")
#                 print(f"├── Safety Margin: {Config.SAFETY_MARGIN_MULTIPLIER}x")
#                 print(f"└── Movement Delay: {Config.MOVEMENT_DELAY}s")

#                 modify = input("Modify settings? (y/n): ").lower().strip()
#                 if modify == 'y':
#                     # Configuration modification could be implemented here
#                     print("⚠️ Configuration modification not implemented in this demo")

#             elif choice == '9':
#                 print("👋 Goodbye!")
#                 break

#             else:
#                 print("❌ Invalid option")

#         except KeyboardInterrupt:
#             print("\n⚠️ Operation cancelled")
#         except Exception as e:
#             print(f"❌ Error: {e}")

# if __name__ == "__main__":
#     main()



# import requests
# import json
# import time
# import math
# import heapq
# import threading
# from typing import List, Tuple, Dict, Any, Optional, Set, Union
# from dataclasses import dataclass, field
# from enum import Enum
# import numpy as np
# from collections import defaultdict

# # Enhanced Configuration Management
# class Config:
#     """Centralized configuration for better maintainability and performance"""

#     # Server Configuration
#     BASE_URL = "http://localhost:5001"
#     REQUEST_TIMEOUT = 5.0
#     MAX_RETRIES = 3

#     # Canvas and Grid Configuration
#     CANVAS_WIDTH = 650
#     CANVAS_HEIGHT = 600
#     GRID_RESOLUTION = 10  # Adjustable resolution for performance tuning

#     # Robot Configuration
#     ROBOT_RADIUS = 18
#     ROBOT_START = (320, 300)
#     SAFETY_MARGIN_MULTIPLIER = 1.5  # Dynamic safety margin

#     # Performance Configuration
#     MAX_SEARCH_NODES = 10000
#     PATH_SMOOTHING_ITERATIONS = 3
#     WAYPOINT_DISTANCE_THRESHOLD = 25  # Minimum distance between waypoints

#     # Algorithm Selection
#     USE_JPS = True  # Use JPS when possible, fallback to A*
#     ENABLE_PATH_SMOOTHING = True
#     ENABLE_DYNAMIC_OBSTACLES = True

#     # Navigation Configuration
#     GOAL_TOLERANCE = 33
#     MOVEMENT_DELAY = 0.3  # Reduced for faster navigation
#     MAX_PLANNING_TIME = 5.0  # Timeout for pathfinding

#     # NEW: Dynamic Goal Tracking Configuration
#     ENABLE_DYNAMIC_GOAL_TRACKING = True  # Enable/disable goal tracking
#     GOAL_CHECK_INTERVAL = 3.0  # Check goal position every 3 seconds
#     GOAL_POSITION_THRESHOLD = 20.0  # Minimum distance to trigger replan (pixels)
#     MAX_GOAL_CHANGES = 5  # Maximum goal changes before warning

# class PathfindingAlgorithm(Enum):
#     """Available pathfinding algorithms"""
#     A_STAR = "a_star"
#     JPS = "jps"
#     HYBRID = "hybrid"  # Automatic selection based on environment

# @dataclass
# class Point:
#     """Enhanced Point class with additional utilities"""
#     x: float
#     y: float

#     def distance_to(self, other: 'Point') -> float:
#         return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

#     def manhattan_distance_to(self, other: 'Point') -> float:
#         """Manhattan distance for grid-based calculations"""
#         return abs(self.x - other.x) + abs(self.y - other.y)

#     def to_grid(self, resolution: int = Config.GRID_RESOLUTION) -> Tuple[int, int]:
#         """Convert to grid coordinates"""
#         return (int(self.x // resolution), int(self.y // resolution))

#     @classmethod
#     def from_grid(cls, grid_x: int, grid_y: int, resolution: int = Config.GRID_RESOLUTION) -> 'Point':
#         """Create point from grid coordinates"""
#         return cls(grid_x * resolution, grid_y * resolution)

#     def is_valid(self, canvas_width: int = Config.CANVAS_WIDTH, 
#                  canvas_height: int = Config.CANVAS_HEIGHT, 
#                  robot_radius: int = Config.ROBOT_RADIUS) -> bool:
#         """Check if point is within valid bounds"""
#         return (robot_radius <= self.x <= canvas_width - robot_radius and 
#                 robot_radius <= self.y <= canvas_height - robot_radius)

#     def __hash__(self):
#         return hash((round(self.x, 2), round(self.y, 2)))

#     def __eq__(self, other):
#         if not isinstance(other, Point):
#             return False
#         return abs(self.x - other.x) < 0.01 and abs(self.y - other.y) < 0.01

# @dataclass
# class Obstacle:
#     """Enhanced Obstacle class with dynamic properties"""
#     x: float
#     y: float
#     size: float = 25
#     velocity_x: float = 0.0  # For dynamic obstacles
#     velocity_y: float = 0.0
#     prediction_time: float = 2.0  # How far to predict movement
#     is_dynamic: bool = False

#     def point(self) -> Point:
#         return Point(self.x, self.y)

#     def predicted_point(self, time_ahead: float) -> Point:
#         """Get predicted position for dynamic obstacles"""
#         if not self.is_dynamic:
#             return self.point()

#         pred_x = self.x + self.velocity_x * time_ahead
#         pred_y = self.y + self.velocity_y * time_ahead
#         return Point(pred_x, pred_y)

#     def contains_point(self, point: Point, robot_radius: float = Config.ROBOT_RADIUS, 
#                       time_ahead: float = 0.0) -> bool:
#         """Check if point collides with obstacle (with prediction)"""
#         obstacle_pos = self.predicted_point(time_ahead)
#         safety_radius = (self.size/2 + robot_radius) * Config.SAFETY_MARGIN_MULTIPLIER
#         return point.distance_to(obstacle_pos) <= safety_radius

# @dataclass 
# class GoalStatus:
#     """Track goal position and changes"""
#     position: Point
#     last_updated: float
#     change_count: int = 0
#     is_changed: bool = False

# class GridMap:
#     """Enhanced grid-based map representation"""

#     def __init__(self, width: int, height: int, resolution: int = Config.GRID_RESOLUTION):
#         self.width = width
#         self.height = height
#         self.resolution = resolution
#         self.grid_width = width // resolution
#         self.grid_height = height // resolution
#         try:
#             self.grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
#         except ImportError:
#             # Fallback if numpy not available
#             self.grid = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]

#     def is_valid_grid_pos(self, gx: int, gy: int) -> bool:
#         """Check if grid position is valid"""
#         return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

#     def is_walkable(self, gx: int, gy: int) -> bool:
#         """Check if grid cell is walkable"""
#         if not self.is_valid_grid_pos(gx, gy):
#             return False
#         return self.grid[gy][gx] == 0

#     def set_obstacle(self, gx: int, gy: int, value: int = 1):
#         """Set obstacle in grid"""
#         if self.is_valid_grid_pos(gx, gy):
#             self.grid[gy][gx] = value

#     def add_obstacle(self, obstacle: Obstacle, robot_radius: float = Config.ROBOT_RADIUS):
#         """Add obstacle to grid with safety margins"""
#         center_gx = int(obstacle.x // self.resolution)
#         center_gy = int(obstacle.y // self.resolution)

#         # Calculate expanded radius with safety margin
#         expanded_radius = (obstacle.size/2 + robot_radius) * Config.SAFETY_MARGIN_MULTIPLIER
#         grid_radius = int(math.ceil(expanded_radius / self.resolution))

#         # Mark cells as obstacles
#         for dy in range(-grid_radius, grid_radius + 1):
#             for dx in range(-grid_radius, grid_radius + 1):
#                 gx, gy = center_gx + dx, center_gy + dy
#                 if self.is_valid_grid_pos(gx, gy):
#                     # Use distance check for circular obstacles
#                     dist = math.sqrt(dx*dx + dy*dy) * self.resolution
#                     if dist <= expanded_radius:
#                         self.set_obstacle(gx, gy, 1)

# class JumpPointSearch:
#     """Optimized Jump Point Search implementation"""

#     def __init__(self, grid_map: GridMap):
#         self.grid_map = grid_map
#         self.grid = grid_map.grid
#         self.grid_width = grid_map.grid_width
#         self.grid_height = grid_map.grid_height

#     def find_path(self, start: Point, goal: Point) -> List[Point]:
#         """Find path using JPS algorithm"""
#         start_grid = start.to_grid(self.grid_map.resolution)
#         goal_grid = goal.to_grid(self.grid_map.resolution)

#         # Check if start and goal are valid
#         if not (self.grid_map.is_walkable(start_grid[0], start_grid[1]) and 
#                 self.grid_map.is_walkable(goal_grid[0], goal_grid[1])):
#             return []

#         open_set = [(0, start_grid)]
#         came_from = {}
#         g_score = {start_grid: 0}
#         closed_set = set()

#         while open_set:
#             current_f, current = heapq.heappop(open_set)

#             if current in closed_set:
#                 continue

#             closed_set.add(current)

#             if current == goal_grid:
#                 return self._reconstruct_path(came_from, current)

#             # Get jump points
#             for dx in [-1, 0, 1]:
#                 for dy in [-1, 0, 1]:
#                     if dx == 0 and dy == 0:
#                         continue

#                     jump_point = self._jump(current[0], current[1], dx, dy, goal_grid)
#                     if jump_point and jump_point not in closed_set:
#                         distance = math.sqrt((jump_point[0] - current[0])**2 + 
#                                            (jump_point[1] - current[1])**2)
#                         tentative_g = g_score[current] + distance

#                         if jump_point not in g_score or tentative_g < g_score[jump_point]:
#                             g_score[jump_point] = tentative_g
#                             h_score = math.sqrt((jump_point[0] - goal_grid[0])**2 + 
#                                               (jump_point[1] - goal_grid[1])**2)
#                             f_score = tentative_g + h_score
#                             heapq.heappush(open_set, (f_score, jump_point))
#                             came_from[jump_point] = current

#         return []

#     def _jump(self, x: int, y: int, dx: int, dy: int, goal: Tuple[int, int]) -> Optional[Tuple[int, int]]:
#         """Jump point search core logic"""
#         nx, ny = x + dx, y + dy

#         if not self.grid_map.is_walkable(nx, ny):
#             return None

#         if (nx, ny) == goal:
#             return (nx, ny)

#         # Check for forced neighbors
#         if dx != 0 and dy != 0:  # Diagonal movement
#             if ((self.grid_map.is_walkable(nx - dx, ny) and not self.grid_map.is_walkable(nx - dx, ny - dy)) or
#                 (self.grid_map.is_walkable(nx, ny - dy) and not self.grid_map.is_walkable(nx - dx, ny - dy))):
#                 return (nx, ny)

#             # Recursive horizontal and vertical jumps
#             if (self._jump(nx, ny, dx, 0, goal) or self._jump(nx, ny, 0, dy, goal)):
#                 return (nx, ny)
#         else:  # Horizontal or vertical movement
#             if dx != 0:  # Horizontal
#                 if ((self.grid_map.is_walkable(nx, ny + 1) and not self.grid_map.is_walkable(nx - dx, ny + 1)) or
#                     (self.grid_map.is_walkable(nx, ny - 1) and not self.grid_map.is_walkable(nx - dx, ny - 1))):
#                     return (nx, ny)
#             else:  # Vertical
#                 if ((self.grid_map.is_walkable(nx + 1, ny) and not self.grid_map.is_walkable(nx + 1, ny - dy)) or
#                     (self.grid_map.is_walkable(nx - 1, ny) and not self.grid_map.is_walkable(nx - 1, ny - dy))):
#                     return (nx, ny)

#         return self._jump(nx, ny, dx, dy, goal)

#     def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Point]:
#         """Reconstruct path from grid coordinates to Points"""
#         path_grid = [current]
#         while current in came_from:
#             current = came_from[current]
#             path_grid.append(current)
#         path_grid.reverse()

#         # Convert to Point objects
#         path_points = [Point.from_grid(gx, gy, self.grid_map.resolution) 
#                       for gx, gy in path_grid]
#         return path_points

# class EnhancedAStarPathfinder:
#     """Enhanced A* pathfinder with modern optimizations"""

#     def __init__(self, canvas_width: int = Config.CANVAS_WIDTH, 
#                  canvas_height: int = Config.CANVAS_HEIGHT, 
#                  robot_radius: int = Config.ROBOT_RADIUS, 
#                  step_size: float = 15.0):
#         self.canvas_width = canvas_width
#         self.canvas_height = canvas_height
#         self.robot_radius = robot_radius
#         self.step_size = step_size
#         self.grid_map = GridMap(canvas_width, canvas_height)

#     def is_valid_point(self, point: Point, obstacles: List[Obstacle], time_ahead: float = 0.0) -> bool:
#         """Enhanced validity check with dynamic obstacles"""
#         if not point.is_valid(self.canvas_width, self.canvas_height, self.robot_radius):
#             return False

#         # Check dynamic obstacles with prediction
#         for obstacle in obstacles:
#             if obstacle.contains_point(point, self.robot_radius, time_ahead):
#                 return False

#         return True

#     def is_line_clear(self, start: Point, end: Point, obstacles: List[Obstacle], 
#                      step_size: float = 2.0, time_ahead: float = 0.0) -> bool:
#         """Enhanced line clearance check"""
#         distance = start.distance_to(end)
#         if distance == 0:
#             return True

#         steps = int(math.ceil(distance / step_size))
#         dx = (end.x - start.x) / steps
#         dy = (end.y - start.y) / steps

#         for i in range(steps + 1):
#             test_point = Point(start.x + i * dx, start.y + i * dy)
#             if not self.is_valid_point(test_point, obstacles, time_ahead):
#                 return False

#         return True

#     def heuristic(self, point: Point, goal: Point, algorithm: str = "euclidean") -> float:
#         """Enhanced heuristic with multiple options"""
#         if algorithm == "manhattan":
#             return point.manhattan_distance_to(goal)
#         elif algorithm == "diagonal":
#             dx = abs(point.x - goal.x)
#             dy = abs(point.y - goal.y)
#             return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
#         else:  # euclidean (default)
#             return point.distance_to(goal)

#     def get_neighbors(self, point: Point, obstacles: List[Obstacle], 
#                      time_ahead: float = 0.0) -> List[Point]:
#         """Enhanced neighbor generation with optimizations"""
#         neighbors = []
#         directions = [
#             (1, 0), (-1, 0), (0, 1), (0, -1),  # Cardinal
#             (1, 1), (-1, -1), (1, -1), (-1, 1)  # Diagonal
#         ]

#         for dx, dy in directions:
#             if dx != 0 and dy != 0:
#                 factor = self.step_size / math.sqrt(2)
#             else:
#                 factor = self.step_size

#             new_point = Point(point.x + dx * factor, point.y + dy * factor)
#             if (self.is_valid_point(new_point, obstacles, time_ahead) and 
#                 self.is_line_clear(point, new_point, obstacles, time_ahead=time_ahead)):
#                 neighbors.append(new_point)

#         return neighbors

#     def find_path(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> List[Point]:
#         """Enhanced A* with tie-breaking and optimizations"""
#         open_set: List[Tuple[float, int, Point]] = []
#         counter = 0
#         heapq.heappush(open_set, (0.0, counter, start))

#         came_from: Dict[Point, Point] = {}
#         g_score = {start: 0.0}
#         f_score = {start: self.heuristic(start, goal)}
#         closed = set()
#         nodes_explored = 0

#         start_time = time.time()

#         while open_set and nodes_explored < Config.MAX_SEARCH_NODES:
#             # Check timeout
#             if time.time() - start_time > Config.MAX_PLANNING_TIME:
#                 print(f"⚠️ Pathfinding timeout after {Config.MAX_PLANNING_TIME}s")
#                 break

#             _, _, current = heapq.heappop(open_set)
#             if current in closed:
#                 continue

#             closed.add(current)
#             nodes_explored += 1

#             if current.distance_to(goal) <= Config.GOAL_TOLERANCE:
#                 path = []
#                 while current in came_from:
#                     path.append(current)
#                     current = came_from[current]
#                 path.append(start)
#                 print(f"✅ A* found path with {nodes_explored} nodes explored")
#                 return list(reversed(path))

#             for neighbor in self.get_neighbors(current, obstacles):
#                 if neighbor in closed:
#                     continue

#                 tentative_g = g_score[current] + current.distance_to(neighbor)

#                 if neighbor not in g_score or tentative_g < g_score[neighbor]:
#                     came_from[neighbor] = current
#                     g_score[neighbor] = tentative_g
#                     f_val = tentative_g + self.heuristic(neighbor, goal)
#                     counter += 1
#                     heapq.heappush(open_set, (f_val, counter, neighbor))

#         print(f"❌ A* failed to find path after exploring {nodes_explored} nodes")
#         return []

# class HybridPathfinder:
#     """Intelligent pathfinder that chooses the best algorithm"""

#     def __init__(self):
#         self.a_star = EnhancedAStarPathfinder()
#         self.grid_map = GridMap(Config.CANVAS_WIDTH, Config.CANVAS_HEIGHT)
#         self.jps = None

#     def _analyze_environment(self, obstacles: List[Obstacle]) -> Dict[str, Any]:
#         """Analyze environment to choose optimal algorithm"""
#         total_cells = self.grid_map.grid_width * self.grid_map.grid_height

#         # Reset grid
#         try:
#             self.grid_map.grid.fill(0)
#         except AttributeError:
#             # Fallback for list-based grid
#             for y in range(self.grid_map.grid_height):
#                 for x in range(self.grid_map.grid_width):
#                     self.grid_map.grid[y][x] = 0

#         # Add obstacles to grid
#         for obstacle in obstacles:
#             self.grid_map.add_obstacle(obstacle)

#         try:
#             occupied_cells = np.count_nonzero(self.grid_map.grid)
#         except:
#             # Fallback counting
#             occupied_cells = sum(sum(1 for cell in row if cell != 0) 
#                                for row in self.grid_map.grid)

#         obstacle_density = occupied_cells / total_cells

#         # Check for large open areas (good for JPS)
#         open_area_score = self._calculate_open_area_score()

#         return {
#             'obstacle_density': obstacle_density,
#             'open_area_score': open_area_score,
#             'total_obstacles': len(obstacles),
#             'dynamic_obstacles': sum(1 for obs in obstacles if obs.is_dynamic)
#         }

#     def _calculate_open_area_score(self) -> float:
#         """Calculate how much open area exists (higher score = more open)"""
#         try:
#             visited = np.zeros_like(self.grid_map.grid, dtype=bool)
#         except:
#             # Fallback for list-based grid
#             visited = [[False for _ in range(self.grid_map.grid_width)] 
#                       for _ in range(self.grid_map.grid_height)]

#         largest_area = 0

#         for y in range(self.grid_map.grid_height):
#             for x in range(self.grid_map.grid_width):
#                 if not visited[y][x] and self.grid_map.is_walkable(x, y):
#                     area_size = self._flood_fill_size(x, y, visited)
#                     largest_area = max(largest_area, area_size)

#         try:
#             total_walkable = np.count_nonzero(np.array(self.grid_map.grid) == 0)
#         except:
#             total_walkable = sum(sum(1 for cell in row if cell == 0) 
#                                for row in self.grid_map.grid)

#         return (largest_area / total_walkable) if total_walkable > 0 else 0

#     def _flood_fill_size(self, start_x: int, start_y: int, visited) -> int:
#         """Calculate size of connected open area using flood fill"""
#         stack = [(start_x, start_y)]
#         size = 0

#         while stack:
#             x, y = stack.pop()
#             if (not self.grid_map.is_valid_grid_pos(x, y) or 
#                 visited[y][x] or not self.grid_map.is_walkable(x, y)):
#                 continue

#             visited[y][x] = True
#             size += 1

#             # Add neighbors
#             for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
#                 stack.append((x + dx, y + dy))

#         return size

#     def find_path(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> List[Point]:
#         """Find path using the most appropriate algorithm"""
#         env_analysis = self._analyze_environment(obstacles)

#         # Decision logic for algorithm selection
#         use_jps = (Config.USE_JPS and 
#                   env_analysis['obstacle_density'] < 0.4 and  # Not too cluttered
#                   env_analysis['open_area_score'] > 0.3 and   # Has decent open areas
#                   env_analysis['dynamic_obstacles'] == 0)     # No dynamic obstacles

#         if use_jps:
#             print(f"🔥 Using JPS (Open area: {env_analysis['open_area_score']:.2f}, Density: {env_analysis['obstacle_density']:.2f})")
#             if self.jps is None:
#                 self.jps = JumpPointSearch(self.grid_map)
#             try:
#                 path = self.jps.find_path(start, goal)
#                 if path:
#                     return self._post_process_path(path, obstacles)
#             except Exception as e:
#                 print(f"⚠️ JPS failed: {e}, falling back to A*")

#         # Fallback to A* or if JPS not suitable
#         print(f"🎯 Using Enhanced A* (Obstacles: {env_analysis['total_obstacles']}, Dynamic: {env_analysis['dynamic_obstacles']})")
#         path = self.a_star.find_path(start, goal, obstacles)
#         return self._post_process_path(path, obstacles)

#     def _post_process_path(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
#         """Post-process path for optimization"""
#         if not path or not Config.ENABLE_PATH_SMOOTHING:
#             return path

#         # Path smoothing
#         smoothed_path = self._smooth_path(path, obstacles)

#         # Remove redundant waypoints
#         optimized_path = self._remove_redundant_waypoints(smoothed_path, obstacles)

#         print(f"📈 Path optimized: {len(path)} -> {len(optimized_path)} waypoints")
#         return optimized_path

#     def _smooth_path(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
#         """Smooth path by removing unnecessary waypoints"""
#         if len(path) <= 2:
#             return path

#         smoothed = [path[0]]  # Keep start point

#         i = 0
#         while i < len(path) - 1:
#             # Try to connect current point to the farthest possible point
#             farthest_idx = i + 1

#             for j in range(i + 2, len(path)):
#                 if self.a_star.is_line_clear(path[i], path[j], obstacles):
#                     farthest_idx = j
#                 else:
#                     break

#             smoothed.append(path[farthest_idx])
#             i = farthest_idx

#         return smoothed

#     def _remove_redundant_waypoints(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
#         """Remove waypoints that are too close together"""
#         if len(path) <= 2:
#             return path

#         optimized = [path[0]]

#         for i in range(1, len(path)):
#             if (i == len(path) - 1 or  # Keep last point
#                 optimized[-1].distance_to(path[i]) >= Config.WAYPOINT_DISTANCE_THRESHOLD):
#                 optimized.append(path[i])

#         return optimized

# class DynamicGoalTracker:
#     """NEW: Track and handle dynamic goal changes"""

#     def __init__(self, controller):
#         self.controller = controller
#         self.current_goal: Optional[GoalStatus] = None
#         self.is_tracking = False
#         self.tracking_thread = None
#         self.stop_tracking = False

#     def start_tracking(self, initial_goal: Point):
#         """Start tracking goal position changes"""
#         if not Config.ENABLE_DYNAMIC_GOAL_TRACKING:
#             return

#         self.current_goal = GoalStatus(
#             position=initial_goal,
#             last_updated=time.time(),
#             change_count=0
#         )

#         self.is_tracking = True
#         self.stop_tracking = False

#         # Start background tracking thread
#         self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
#         self.tracking_thread.start()

#         print(f"🎯 Started goal tracking at ({initial_goal.x:.1f}, {initial_goal.y:.1f})")

#     def stop_goal_tracking(self):
#         """Stop tracking goal position"""
#         self.stop_tracking = True
#         self.is_tracking = False

#         if self.tracking_thread and self.tracking_thread.is_alive():
#             self.tracking_thread.join(timeout=1.0)

#         print("🛑 Goal tracking stopped")

#     def _tracking_loop(self):
#         """Background thread to track goal position changes"""
#         while not self.stop_tracking and self.is_tracking:
#             try:
#                 # Get current goal position from server
#                 current_server_goal = self._get_server_goal_position()

#                 if current_server_goal and self.current_goal:
#                     distance_moved = self.current_goal.position.distance_to(current_server_goal)

#                     # Check if goal has moved significantly
#                     if distance_moved >= Config.GOAL_POSITION_THRESHOLD:
#                         self.current_goal.position = current_server_goal
#                         self.current_goal.last_updated = time.time()
#                         self.current_goal.change_count += 1
#                         self.current_goal.is_changed = True

#                         print(f"🎯 Goal moved {distance_moved:.1f}px to ({current_server_goal.x:.1f}, {current_server_goal.y:.1f})")
#                         print(f"🔄 Goal change #{self.current_goal.change_count}")

#                         # Warning if too many goal changes
#                         if self.current_goal.change_count >= Config.MAX_GOAL_CHANGES:
#                             print(f"⚠️ Warning: Goal changed {self.current_goal.change_count} times!")

#                 # Sleep for the specified interval
#                 time.sleep(Config.GOAL_CHECK_INTERVAL)

#             except Exception as e:
#                 print(f"⚠️ Goal tracking error: {e}")
#                 time.sleep(Config.GOAL_CHECK_INTERVAL)

#     def _get_server_goal_position(self) -> Optional[Point]:
#         """Get current goal position from server"""
#         try:
#             response = self.controller._make_request("GET", "/goal")
#             if response:
#                 goal_data = response.json()
#                 if 'x' in goal_data and 'y' in goal_data:
#                     return Point(goal_data['x'], goal_data['y'])
#         except Exception:
#             pass
#         return None

#     def has_goal_changed(self) -> bool:
#         """Check if goal has changed and reset the flag"""
#         if self.current_goal and self.current_goal.is_changed:
#             self.current_goal.is_changed = False
#             return True
#         return False

#     def get_current_goal(self) -> Optional[Point]:
#         """Get current goal position"""
#         return self.current_goal.position if self.current_goal else None

# class EnhancedRobotController:
#     """Enhanced robot controller with dynamic goal tracking"""

#     def __init__(self, server_url: str = Config.BASE_URL):
#         self.server_url = server_url
#         self.pathfinder = HybridPathfinder()
#         try:
#             self.session = requests.Session()  # Reuse connections
#         except:
#             self.session = None
#         self.last_obstacles = []
#         self.performance_stats = {
#             'total_navigations': 0,
#             'successful_navigations': 0,
#             'average_path_length': 0,
#             'average_compute_time': 0,
#             'goal_changes': 0,  # NEW: Track goal changes
#             'replanning_due_to_goal_change': 0  # NEW: Track goal-based replanning
#         }

#         # NEW: Initialize dynamic goal tracker
#         self.goal_tracker = DynamicGoalTracker(self)

#     def _make_request(self, method: str, endpoint: str, **kwargs) -> Optional[requests.Response]:
#         """Enhanced request handling with retries and error handling"""
#         url = f"{self.server_url}{endpoint}"

#         for attempt in range(Config.MAX_RETRIES):
#             try:
#                 if self.session:
#                     response = self.session.request(
#                         method, url, 
#                         timeout=Config.REQUEST_TIMEOUT,
#                         **kwargs
#                     )
#                 else:
#                     response = requests.request(
#                         method, url,
#                         timeout=Config.REQUEST_TIMEOUT,
#                         **kwargs
#                     )

#                 if response.status_code == 200:
#                     return response
#                 else:
#                     print(f"⚠️ Server returned status {response.status_code}")

#             except requests.exceptions.RequestException as e:
#                 if attempt < Config.MAX_RETRIES - 1:
#                     print(f"🔄 Request failed (attempt {attempt + 1}), retrying... Error: {e}")
#                     time.sleep(0.5 * (attempt + 1))  # Exponential backoff
#                 else:
#                     print(f"❌ Request failed after {Config.MAX_RETRIES} attempts: {e}")
#             except Exception as e:
#                 print(f"❌ Unexpected error: {e}")

#         return None

#     def get_robot_status(self) -> Dict[str, Any]:
#         """Get current robot status with enhanced error handling"""
#         response = self._make_request("GET", "/status")
#         return response.json() if response else {}

#     def get_obstacles(self) -> List[Dict[str, Any]]:
#         """Get current obstacles from server"""
#         response = self._make_request("GET", "/obstacles")
#         if response:
#             return response.json().get('obstacles', [])
#         return []

#     def move_robot(self, x: float, y: float) -> bool:
#         """Move robot to specific coordinates"""
#         response = self._make_request(
#             "POST", "/move", 
#             json={"x": x, "y": y},
#             headers={"Content-Type": "application/json"}
#         )
#         return response is not None

#     def set_goal(self, x: float, y: float) -> bool:
#         """Set goal position"""
#         response = self._make_request(
#             "POST", "/goal", 
#             json={"x": x, "y": y},
#             headers={"Content-Type": "application/json"}
#         )
#         if response:
#             print(f"🎯 Goal set at ({x:.1f}, {y:.1f})")
#         return response is not None

#     def reset_simulation(self) -> bool:
#         """Reset simulation state"""
#         # Stop goal tracking if active
#         if self.goal_tracker.is_tracking:
#             self.goal_tracker.stop_goal_tracking()

#         response = self._make_request("POST", "/reset")
#         if response:
#             print("✅ Simulation reset successful")
#         return response is not None

#     def generate_random_obstacles(self, count: int = 8) -> bool:
#         """Generate random obstacles"""
#         response = self._make_request(
#             "POST", "/obstacles/random", 
#             json={"count": count},
#             headers={"Content-Type": "application/json"}
#         )
#         if response:
#             print(f"🚧 Generated {count} random obstacles")
#         return response is not None

#     def broadcast_reroute(self, obstacle_coords: Tuple[float, float]) -> bool:
#         """Broadcast reroute notification"""
#         response = self._make_request(
#             "POST", "/broadcast",
#             json={"message": f"Rerouting around obstacle at {obstacle_coords}"},
#             headers={"Content-Type": "application/json"}
#         )
#         return response is not None

#     def is_goal_reached(self) -> bool:
#         """Check if goal is reached"""
#         response = self._make_request("GET", "/goal/status")
#         if response:
#             return response.json().get('goal_reached', False)
#         return False

#     def navigate_to_goal(self, start_x: float, start_y: float, 
#                         goal_x: float, goal_y: float, 
#                         enable_dynamic_replanning: bool = True,
#                         enable_goal_tracking: bool = True) -> bool:
#         """Enhanced navigation with dynamic goal tracking"""

#         print(f"🚀 Starting enhanced navigation with goal tracking from ({start_x:.1f}, {start_y:.1f}) to ({goal_x:.1f}, {goal_y:.1f})")

#         # Performance tracking
#         navigation_start = time.time()
#         self.performance_stats['total_navigations'] += 1

#         # Set the goal
#         if not self.set_goal(goal_x, goal_y):
#             print("❌ Failed to set goal")
#             return False

#         # NEW: Start dynamic goal tracking
#         initial_goal = Point(goal_x, goal_y)
#         if enable_goal_tracking and Config.ENABLE_DYNAMIC_GOAL_TRACKING:
#             self.goal_tracker.start_tracking(initial_goal)

#         # Get current obstacles and create enhanced obstacle objects
#         obstacles_data = self.get_obstacles()
#         obstacles = []

#         for obs_data in obstacles_data:
#             obstacle = Obstacle(
#                 x=obs_data['x'], 
#                 y=obs_data['y'], 
#                 size=obs_data.get('size', 25)
#             )
#             # Add dynamic properties if available
#             if 'velocity_x' in obs_data:
#                 obstacle.velocity_x = obs_data['velocity_x']
#                 obstacle.velocity_y = obs_data.get('velocity_y', 0)
#                 obstacle.is_dynamic = True

#             obstacles.append(obstacle)

#         print(f"🗺️ Environment: {len(obstacles)} obstacles ({sum(1 for o in obstacles if o.is_dynamic)} dynamic)")

#         # Initial path planning
#         start_point = Point(start_x, start_y)
#         current_goal_point = initial_goal

#         print("🧠 Computing optimal path...")
#         path_start_time = time.time()
#         path = self.pathfinder.find_path(start_point, current_goal_point, obstacles)
#         compute_time = time.time() - path_start_time

#         if not path:
#             print("❌ No collision-free path found!")
#             if enable_goal_tracking:
#                 self.goal_tracker.stop_goal_tracking()
#             return False

#         # Calculate path statistics
#         path_length = sum(path[i].distance_to(path[i+1]) for i in range(len(path)-1))
#         print(f"✅ Path computed! Length: {path_length:.1f}px, Time: {compute_time:.3f}s, Waypoints: {len(path)}")

#         # Update performance stats
#         self.performance_stats['average_compute_time'] = (
#             (self.performance_stats['average_compute_time'] * (self.performance_stats['total_navigations'] - 1) + compute_time) / 
#             self.performance_stats['total_navigations']
#         )

#         # Execute path with dynamic replanning and goal tracking
#         success = self._execute_path_with_goal_tracking(
#             path, current_goal_point, obstacles, 
#             enable_dynamic_replanning, enable_goal_tracking
#         )

#         # Stop goal tracking
#         if enable_goal_tracking:
#             self.goal_tracker.stop_goal_tracking()

#         # Update performance statistics
#         total_time = time.time() - navigation_start
#         if success:
#             self.performance_stats['successful_navigations'] += 1
#             self.performance_stats['average_path_length'] = (
#                 (self.performance_stats['average_path_length'] * (self.performance_stats['successful_navigations'] - 1) + path_length) / 
#                 self.performance_stats['successful_navigations']
#             )
#             print(f"🎉 Navigation completed successfully in {total_time:.2f}s!")
#         else:
#             print(f"❌ Navigation failed after {total_time:.2f}s")

#         return success

#     def _execute_path_with_goal_tracking(self, path: List[Point], initial_goal: Point, 
#                                        original_obstacles: List[Obstacle], 
#                                        enable_replanning: bool, 
#                                        enable_goal_tracking: bool) -> bool:
#         """NEW: Execute path with both obstacle and goal tracking"""

#         current_path = path[:]
#         current_goal = initial_goal
#         waypoint_idx = 0
#         replanning_count = 0
#         max_replanning = 3

#         while waypoint_idx < len(current_path):
#             waypoint = current_path[waypoint_idx]
#             print(f"📍 Moving to waypoint {waypoint_idx + 1}/{len(current_path)}: ({waypoint.x:.1f}, {waypoint.y:.1f})")

#             if not self.move_robot(waypoint.x, waypoint.y):
#                 print(f"❌ Failed to move to waypoint {waypoint_idx + 1}")
#                 return False

#             # Wait for movement
#             time.sleep(Config.MOVEMENT_DELAY)

#             # Check for goal reached (using current goal position)
#             if self.is_goal_reached():
#                 print("🎯 Goal reached!")
#                 return True

#             # NEW: Check for goal position changes
#             if enable_goal_tracking and self.goal_tracker.has_goal_changed():
#                 new_goal = self.goal_tracker.get_current_goal()
#                 if new_goal:
#                     print(f"🎯➡️ Goal changed! New target: ({new_goal.x:.1f}, {new_goal.y:.1f})")
#                     current_goal = new_goal
#                     self.performance_stats['goal_changes'] += 1

#                     # Replan path to new goal
#                     current_pos = Point(waypoint.x, waypoint.y)
#                     current_obstacles_data = self.get_obstacles()
#                     new_obstacles = [Obstacle(obs['x'], obs['y'], obs.get('size', 25)) 
#                                    for obs in current_obstacles_data]

#                     new_path = self.pathfinder.find_path(current_pos, current_goal, new_obstacles)

#                     if new_path:
#                         current_path = new_path
#                         waypoint_idx = 0
#                         self.performance_stats['replanning_due_to_goal_change'] += 1
#                         print(f"🔄 Replanned path to new goal with {len(new_path)} waypoints")
#                         continue
#                     else:
#                         print("⚠️ Failed to plan path to new goal, continuing with current path")

#             # Dynamic obstacle detection and replanning
#             if enable_replanning and Config.ENABLE_DYNAMIC_OBSTACLES:
#                 current_obstacles_data = self.get_obstacles()

#                 # Check if obstacles have changed significantly
#                 if self._obstacles_changed(original_obstacles, current_obstacles_data):
#                     print("⚠️ Obstacle configuration changed, replanning...")

#                     if replanning_count < max_replanning:
#                         new_obstacles = [Obstacle(obs['x'], obs['y'], obs.get('size', 25)) 
#                                        for obs in current_obstacles_data]

#                         current_pos = Point(waypoint.x, waypoint.y)
#                         new_path = self.pathfinder.find_path(current_pos, current_goal, new_obstacles)

#                         if new_path:
#                             current_path = new_path
#                             waypoint_idx = 0
#                             replanning_count += 1
#                             print(f"🔄 Replanned path with {len(new_path)} waypoints (attempt {replanning_count})")

#                             # Broadcast reroute notification
#                             self.broadcast_reroute((waypoint.x, waypoint.y))
#                             continue
#                         else:
#                             print("⚠️ Replanning failed, continuing with original path")
#                     else:
#                         print(f"⚠️ Max replanning attempts ({max_replanning}) reached")

#             waypoint_idx += 1

#         # Final goal check
#         return self.is_goal_reached()

#     def _obstacles_changed(self, original: List[Obstacle], current_data: List[Dict]) -> bool:
#         """Check if obstacles have changed significantly"""
#         if len(original) != len(current_data):
#             return True

#         # Simple change detection - could be enhanced
#         threshold = 10.0  # pixels

#         for i, obs_data in enumerate(current_data):
#             if i >= len(original):
#                 return True

#             orig = original[i]
#             if (abs(orig.x - obs_data['x']) > threshold or 
#                 abs(orig.y - obs_data['y']) > threshold):
#                 return True

#         return False

#     def get_performance_stats(self) -> Dict[str, Any]:
#         """Get performance statistics with goal tracking data"""
#         success_rate = (self.performance_stats['successful_navigations'] / 
#                        max(1, self.performance_stats['total_navigations']) * 100)

#         return {
#             **self.performance_stats,
#             'success_rate_percent': success_rate
#         }

#     def print_performance_stats(self):
#         """Print detailed performance statistics including goal tracking"""
#         stats = self.get_performance_stats()
#         print(f"""
# 📊 Performance Statistics:
# ├── Total Navigations: {stats['total_navigations']}
# ├── Successful: {stats['successful_navigations']} ({stats['success_rate_percent']:.1f}%)
# ├── Average Path Length: {stats['average_path_length']:.1f} pixels
# ├── Average Compute Time: {stats['average_compute_time']:.3f} seconds
# ├── 🎯 Goal Changes: {stats['goal_changes']}
# └── 🔄 Goal-based Replannings: {stats['replanning_due_to_goal_change']}
#         """)

# def main():
#     """Enhanced main execution function with goal tracking"""
#     print("=" * 60)
#     print("🤖 ENHANCED ROBOT NAVIGATION SYSTEM v2.1")
#     print("🔥 Features: Hybrid JPS/A*, Dynamic Goals, Smart Tracking")
#     print("=" * 60)

#     controller = EnhancedRobotController()

#     # System checks
#     print("🔍 System Checks:")
#     print(f"├── Server URL: {Config.BASE_URL}")
#     print(f"├── Grid Resolution: {Config.GRID_RESOLUTION}px")
#     print(f"├── JPS Enabled: {Config.USE_JPS}")
#     print(f"├── Path Smoothing: {Config.ENABLE_PATH_SMOOTHING}")
#     print(f"├── Dynamic Obstacles: {Config.ENABLE_DYNAMIC_OBSTACLES}")
#     print(f"└── 🎯 Goal Tracking: {Config.ENABLE_DYNAMIC_GOAL_TRACKING} (Check every {Config.GOAL_CHECK_INTERVAL}s)")
#     print()

#     # Test server connection
#     status = controller.get_robot_status()
#     if not status:
#         print("❌ Cannot connect to server. Please ensure server.py is running on localhost:5001")
#         return
#     else:
#         print("✅ Server connection successful")

#     while True:
#         print(f"""
# 🎮 Navigation Options:
# 1. Quick Test (Default scenario with goal tracking)
# 2. Custom Navigation (Specify coordinates)
# 3. Corner Navigation (NE, NW, SE, SW)
# 4. 🎯 Dynamic Goal Test (Goal tracking demo)
# 5. Benchmark Suite (Multiple test scenarios)
# 6. Obstacle Stress Test (High obstacle density)
# 7. Performance Statistics
# 8. Reset Simulation
# 9. Advanced Configuration
# 0. Exit
#         """)

#         choice = input("Choose option (0-9): ").strip()

#         try:
#             if choice == '1':
#                 # Quick test scenario with goal tracking
#                 print("🚀 Quick Test Scenario with Goal Tracking")
#                 controller.reset_simulation()
#                 time.sleep(1)

#                 controller.generate_random_obstacles(8)
#                 success = controller.navigate_to_goal(
#                     320, 300, 550, 80, 
#                     enable_dynamic_replanning=True,
#                     enable_goal_tracking=True  # NEW: Enable goal tracking
#                 )
#                 print(f"Result: {'✅ SUCCESS' if success else '❌ FAILED'}")

#             elif choice == '2':
#                 # Custom navigation
#                 print("📍 Custom Navigation with Goal Tracking")
#                 try:
#                     start_x = float(input(f"Start X (default {Config.ROBOT_START[0]}): ") or str(Config.ROBOT_START[0]))
#                     start_y = float(input(f"Start Y (default {Config.ROBOT_START[1]}): ") or str(Config.ROBOT_START[1]))
#                     goal_x = float(input("Goal X: "))
#                     goal_y = float(input("Goal Y: "))

#                     enable_tracking = input("Enable goal tracking? (y/n, default y): ").lower().strip()
#                     enable_tracking = enable_tracking != 'n'

#                     success = controller.navigate_to_goal(
#                         start_x, start_y, goal_x, goal_y,
#                         enable_goal_tracking=enable_tracking
#                     )
#                     print(f"Result: {'✅ SUCCESS' if success else '❌ FAILED'}")
#                 except ValueError:
#                     print("❌ Invalid coordinates")

#             elif choice == '3':
#                 # Corner navigation
#                 corners = {
#                     'NE': (630, 20), 'NW': (20, 20),
#                     'SE': (630, 580), 'SW': (20, 580)
#                 }

#                 print("🏃 Corner Navigation with Goal Tracking")
#                 corner = input("Enter corner (NE, NW, SE, SW): ").strip().upper()

#                 if corner in corners:
#                     goal_x, goal_y = corners[corner]
#                     success = controller.navigate_to_goal(320, 300, goal_x, goal_y, enable_goal_tracking=True)
#                     print(f"Result: {'✅ SUCCESS' if success else '❌ FAILED'}")
#                 else:
#                     print("❌ Invalid corner")

#             elif choice == '4':
#                 # NEW: Dynamic goal tracking demo
#                 print("🎯 Dynamic Goal Tracking Demo")
#                 print("This will start navigation and track goal changes every 3 seconds.")
#                 print("You can manually change the goal in the simulator to see real-time replanning!")

#                 controller.reset_simulation()
#                 time.sleep(1)
#                 controller.generate_random_obstacles(6)  # Fewer obstacles for demo

#                 print("🔄 Starting navigation with goal tracking enabled...")
#                 print("💡 TIP: Change the goal position in the simulator to see dynamic replanning!")

#                 success = controller.navigate_to_goal(
#                     100, 300, 550, 100,
#                     enable_dynamic_replanning=True,
#                     enable_goal_tracking=True
#                 )
#                 print(f"Demo Result: {'✅ SUCCESS' if success else '❌ FAILED'}")

#             elif choice == '5':
#                 # Benchmark suite
#                 print("🏁 Running Benchmark Suite with Goal Tracking...")

#                 scenarios = [
#                     ("Light obstacles + Goal tracking", 320, 300, 550, 80, 5, True),
#                     ("Medium obstacles + Goal tracking", 320, 300, 100, 100, 8, True),
#                     ("Heavy obstacles (No goal tracking)", 320, 300, 630, 300, 12, False),
#                     ("Extreme obstacles (No goal tracking)", 320, 300, 20, 580, 15, False),
#                 ]

#                 results = []
#                 for name, sx, sy, gx, gy, obs_count, goal_tracking in scenarios:
#                     print(f"\n📋 {name} ({obs_count} obstacles)")
#                     controller.reset_simulation()
#                     time.sleep(0.5)
#                     controller.generate_random_obstacles(obs_count)

#                     success = controller.navigate_to_goal(
#                         sx, sy, gx, gy, 
#                         enable_goal_tracking=goal_tracking
#                     )
#                     results.append((name, success))
#                     time.sleep(1)

#                 print(f"\n📊 Benchmark Results:")
#                 for name, success in results:
#                     status = "✅ PASS" if success else "❌ FAIL"
#                     print(f"├── {name}: {status}")

#             elif choice == '6':
#                 # Obstacle stress test
#                 print("🔥 Obstacle Stress Test")
#                 obstacle_counts = [10, 15, 20, 25]

#                 for count in obstacle_counts:
#                     print(f"\n🧪 Testing with {count} obstacles...")
#                     controller.reset_simulation()
#                     time.sleep(0.5)
#                     controller.generate_random_obstacles(count)

#                     # Disable goal tracking for stress test (focus on pathfinding)
#                     success = controller.navigate_to_goal(
#                         320, 300, 550, 80,
#                         enable_goal_tracking=False
#                     )
#                     result = "✅ PASSED" if success else "❌ FAILED"
#                     print(f"   {count} obstacles: {result}")

#                     if not success:
#                         print("   ⚠️ Stress test limit reached")
#                         break
#                     time.sleep(1)

#             elif choice == '7':
#                 # Performance statistics
#                 controller.print_performance_stats()

#             elif choice == '8':
#                 # Reset simulation
#                 if controller.reset_simulation():
#                     print("✅ Simulation reset complete")
#                 else:
#                     print("❌ Reset failed")

#             elif choice == '9':
#                 # Advanced configuration
#                 print("⚙️ Advanced Configuration")
#                 print(f"Current settings:")
#                 print(f"├── Grid Resolution: {Config.GRID_RESOLUTION}")
#                 print(f"├── Use JPS: {Config.USE_JPS}")
#                 print(f"├── Path Smoothing: {Config.ENABLE_PATH_SMOOTHING}")
#                 print(f"├── Safety Margin: {Config.SAFETY_MARGIN_MULTIPLIER}x")
#                 print(f"├── Movement Delay: {Config.MOVEMENT_DELAY}s")
#                 print(f"├── 🎯 Goal Tracking: {Config.ENABLE_DYNAMIC_GOAL_TRACKING}")
#                 print(f"├── Goal Check Interval: {Config.GOAL_CHECK_INTERVAL}s")
#                 print(f"└── Goal Position Threshold: {Config.GOAL_POSITION_THRESHOLD}px")

#                 modify = input("Modify settings? (y/n): ").lower().strip()
#                 if modify == 'y':
#                     try:
#                         print("\nConfigurable parameters:")
#                         new_interval = input(f"Goal check interval (current: {Config.GOAL_CHECK_INTERVAL}s): ").strip()
#                         if new_interval:
#                             Config.GOAL_CHECK_INTERVAL = float(new_interval)
#                             print(f"✅ Goal check interval updated to {Config.GOAL_CHECK_INTERVAL}s")

#                         new_threshold = input(f"Goal position threshold (current: {Config.GOAL_POSITION_THRESHOLD}px): ").strip()
#                         if new_threshold:
#                             Config.GOAL_POSITION_THRESHOLD = float(new_threshold)
#                             print(f"✅ Goal position threshold updated to {Config.GOAL_POSITION_THRESHOLD}px")

#                     except ValueError:
#                         print("❌ Invalid input")

#             elif choice == '0':
#                 print("👋 Goodbye!")
#                 break

#             else:
#                 print("❌ Invalid option")

#         except KeyboardInterrupt:
#             print("\n⚠️ Operation cancelled")
#             # Make sure to stop goal tracking on interrupt
#             if controller.goal_tracker.is_tracking:
#                 controller.goal_tracker.stop_goal_tracking()
#         except Exception as e:
#             print(f"❌ Error: {e}")
#             # Make sure to stop goal tracking on error
#             if controller.goal_tracker.is_tracking:
#                 controller.goal_tracker.stop_goal_tracking()

# if __name__ == "__main__":
#     main()




import requests
import json
import time
import math
import heapq
import threading
import random
from typing import List, Tuple, Dict, Any, Optional, Set, Union
from dataclasses import dataclass, field
from enum import Enum
import numpy as np
from collections import defaultdict

# Enhanced Configuration Management
class Config:
    """Centralized configuration for better maintainability and performance"""

    # Server Configuration
    BASE_URL = "http://localhost:5001"
    REQUEST_TIMEOUT = 5.0
    MAX_RETRIES = 3

    # Canvas and Grid Configuration
    CANVAS_WIDTH = 650
    CANVAS_HEIGHT = 600
    GRID_RESOLUTION = 10  # Adjustable resolution for performance tuning

    # Robot Configuration
    ROBOT_RADIUS = 18
    ROBOT_START = (320, 300)
    SAFETY_MARGIN_MULTIPLIER = 1.5  # Dynamic safety margin

    # Performance Configuration
    MAX_SEARCH_NODES = 10000
    PATH_SMOOTHING_ITERATIONS = 3
    WAYPOINT_DISTANCE_THRESHOLD = 25  # Minimum distance between waypoints

    # Algorithm Selection
    USE_JPS = True  # Use JPS when possible, fallback to A*
    ENABLE_PATH_SMOOTHING = True
    ENABLE_DYNAMIC_OBSTACLES = True

    # Navigation Configuration
    GOAL_TOLERANCE = 33
    MOVEMENT_DELAY = 0.3  # Reduced for faster navigation
    MAX_PLANNING_TIME = 5.0  # Timeout for pathfinding

    # Dynamic Goal Tracking Configuration
    ENABLE_DYNAMIC_GOAL_TRACKING = True  # Enable/disable goal tracking
    GOAL_CHECK_INTERVAL = 3.0  # Check goal position every 3 seconds
    GOAL_POSITION_THRESHOLD = 20.0  # Minimum distance to trigger replan (pixels)
    MAX_GOAL_CHANGES = 5  # Maximum goal changes before warning

    # NEW: Dynamic Goal Generation Configuration
    ENABLE_DYNAMIC_GOAL_GENERATION = True  # Enable automatic goal changing
    GOAL_GENERATION_INTERVAL = 10.0  # Change goal every 10 seconds
    GOAL_GENERATION_RADIUS = 100.0  # How far from current goal to generate new one
    GOAL_GENERATION_MODES = ["random", "spiral", "corners", "patrol"]  # Available modes
    DEFAULT_GOAL_GENERATION_MODE = "random"

    # Goal Generation Boundaries
    GOAL_MIN_X = 50
    GOAL_MAX_X = 600
    GOAL_MIN_Y = 50
    GOAL_MAX_Y = 550

class PathfindingAlgorithm(Enum):
    """Available pathfinding algorithms"""
    A_STAR = "a_star"
    JPS = "jps"
    HYBRID = "hybrid"  # Automatic selection based on environment

class GoalGenerationMode(Enum):
    """Different modes for automatic goal generation"""
    RANDOM = "random"      # Random positions within bounds
    SPIRAL = "spiral"      # Spiral pattern around center
    CORNERS = "corners"    # Move between corners
    PATROL = "patrol"      # Patrol predefined waypoints
    CHASE = "chase"        # Follow a moving target pattern

@dataclass
class Point:
    """Enhanced Point class with additional utilities"""
    x: float
    y: float

    def distance_to(self, other: 'Point') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def manhattan_distance_to(self, other: 'Point') -> float:
        """Manhattan distance for grid-based calculations"""
        return abs(self.x - other.x) + abs(self.y - other.y)

    def to_grid(self, resolution: int = Config.GRID_RESOLUTION) -> Tuple[int, int]:
        """Convert to grid coordinates"""
        return (int(self.x // resolution), int(self.y // resolution))

    @classmethod
    def from_grid(cls, grid_x: int, grid_y: int, resolution: int = Config.GRID_RESOLUTION) -> 'Point':
        """Create point from grid coordinates"""
        return cls(grid_x * resolution, grid_y * resolution)

    def is_valid(self, canvas_width: int = Config.CANVAS_WIDTH, 
                 canvas_height: int = Config.CANVAS_HEIGHT, 
                 robot_radius: int = Config.ROBOT_RADIUS) -> bool:
        """Check if point is within valid bounds"""
        return (robot_radius <= self.x <= canvas_width - robot_radius and 
                robot_radius <= self.y <= canvas_height - robot_radius)

    def __hash__(self):
        return hash((round(self.x, 2), round(self.y, 2)))

    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return abs(self.x - other.x) < 0.01 and abs(self.y - other.y) < 0.01

@dataclass
class Obstacle:
    """Enhanced Obstacle class with dynamic properties"""
    x: float
    y: float
    size: float = 25
    velocity_x: float = 0.0  # For dynamic obstacles
    velocity_y: float = 0.0
    prediction_time: float = 2.0  # How far to predict movement
    is_dynamic: bool = False

    def point(self) -> Point:
        return Point(self.x, self.y)

    def predicted_point(self, time_ahead: float) -> Point:
        """Get predicted position for dynamic obstacles"""
        if not self.is_dynamic:
            return self.point()

        pred_x = self.x + self.velocity_x * time_ahead
        pred_y = self.y + self.velocity_y * time_ahead
        return Point(pred_x, pred_y)

    def contains_point(self, point: Point, robot_radius: float = Config.ROBOT_RADIUS, 
                      time_ahead: float = 0.0) -> bool:
        """Check if point collides with obstacle (with prediction)"""
        obstacle_pos = self.predicted_point(time_ahead)
        safety_radius = (self.size/2 + robot_radius) * Config.SAFETY_MARGIN_MULTIPLIER
        return point.distance_to(obstacle_pos) <= safety_radius

@dataclass 
class GoalStatus:
    """Track goal position and changes"""
    position: Point
    last_updated: float
    change_count: int = 0
    is_changed: bool = False
    generation_mode: str = "manual"
    is_auto_generated: bool = False

class DynamicGoalGenerator:
    """NEW: Generate dynamic goals in different patterns"""

    def __init__(self):
        self.current_mode = GoalGenerationMode.RANDOM
        self.sequence_index = 0
        self.center_point = Point(Config.CANVAS_WIDTH // 2, Config.CANVAS_HEIGHT // 2)
        self.spiral_angle = 0
        self.spiral_radius = 50

        # Predefined patrol points
        self.patrol_points = [
            Point(100, 100),   # NW area
            Point(550, 100),   # NE area  
            Point(550, 500),   # SE area
            Point(100, 500),   # SW area
            Point(325, 300),   # Center
        ]

        # Corner points
        self.corner_points = [
            Point(Config.GOAL_MIN_X, Config.GOAL_MIN_Y),      # Top-left
            Point(Config.GOAL_MAX_X, Config.GOAL_MIN_Y),      # Top-right
            Point(Config.GOAL_MAX_X, Config.GOAL_MAX_Y),      # Bottom-right
            Point(Config.GOAL_MIN_X, Config.GOAL_MAX_Y),      # Bottom-left
        ]

    def set_mode(self, mode: Union[str, GoalGenerationMode]):
        """Set goal generation mode"""
        if isinstance(mode, str):
            try:
                self.current_mode = GoalGenerationMode(mode.lower())
            except ValueError:
                print(f"⚠️ Invalid mode '{mode}', using RANDOM")
                self.current_mode = GoalGenerationMode.RANDOM
        else:
            self.current_mode = mode

        self.sequence_index = 0  # Reset sequence
        print(f"🎯 Goal generation mode set to: {self.current_mode.value.upper()}")

    def generate_next_goal(self, current_goal: Optional[Point] = None, 
                          obstacles: Optional[List[Obstacle]] = None) -> Point:
        """Generate next goal based on current mode"""

        if self.current_mode == GoalGenerationMode.RANDOM:
            return self._generate_random_goal(current_goal, obstacles)
        elif self.current_mode == GoalGenerationMode.SPIRAL:
            return self._generate_spiral_goal()
        elif self.current_mode == GoalGenerationMode.CORNERS:
            return self._generate_corner_goal()
        elif self.current_mode == GoalGenerationMode.PATROL:
            return self._generate_patrol_goal()
        elif self.current_mode == GoalGenerationMode.CHASE:
            return self._generate_chase_goal(current_goal)
        else:
            return self._generate_random_goal(current_goal, obstacles)

    def _generate_random_goal(self, current_goal: Optional[Point], 
                             obstacles: Optional[List[Obstacle]]) -> Point:
        """Generate random goal within bounds, avoiding obstacles"""
        max_attempts = 50

        for attempt in range(max_attempts):
            # Generate random position within bounds
            x = random.uniform(Config.GOAL_MIN_X, Config.GOAL_MAX_X)
            y = random.uniform(Config.GOAL_MIN_Y, Config.GOAL_MAX_Y)
            new_goal = Point(x, y)

            # Check distance from current goal if specified
            if current_goal:
                distance = new_goal.distance_to(current_goal)
                if distance < 50:  # Too close to current goal
                    continue
                if distance > Config.GOAL_GENERATION_RADIUS * 2:  # Too far
                    continue

            # Check if goal is valid (not in obstacles)
            if self._is_goal_valid(new_goal, obstacles):
                return new_goal

        # Fallback: return a safe corner position
        return Point(Config.GOAL_MIN_X + 50, Config.GOAL_MIN_Y + 50)

    def _generate_spiral_goal(self) -> Point:
        """Generate goal in spiral pattern"""
        # Calculate spiral position
        x = self.center_point.x + self.spiral_radius * math.cos(self.spiral_angle)
        y = self.center_point.y + self.spiral_radius * math.sin(self.spiral_angle)

        # Update spiral parameters
        self.spiral_angle += math.pi / 4  # 45-degree increments
        if self.spiral_angle >= 2 * math.pi:
            self.spiral_angle = 0
            self.spiral_radius += 30  # Expand spiral
            if self.spiral_radius > 200:  # Reset spiral
                self.spiral_radius = 50

        # Clamp to bounds
        x = max(Config.GOAL_MIN_X, min(Config.GOAL_MAX_X, x))
        y = max(Config.GOAL_MIN_Y, min(Config.GOAL_MAX_Y, y))

        return Point(x, y)

    def _generate_corner_goal(self) -> Point:
        """Generate goal moving between corners"""
        goal = self.corner_points[self.sequence_index % len(self.corner_points)]
        self.sequence_index += 1
        return goal

    def _generate_patrol_goal(self) -> Point:
        """Generate goal following patrol pattern"""
        goal = self.patrol_points[self.sequence_index % len(self.patrol_points)]
        self.sequence_index += 1
        return goal

    def _generate_chase_goal(self, current_goal: Optional[Point]) -> Point:
        """Generate goal that moves in chase pattern"""
        if not current_goal:
            return Point(Config.CANVAS_WIDTH // 2, Config.CANVAS_HEIGHT // 2)

        # Move goal in circular pattern
        angle = (time.time() * 0.5) % (2 * math.pi)  # Slow circular motion
        radius = 100

        x = Config.CANVAS_WIDTH // 2 + radius * math.cos(angle)
        y = Config.CANVAS_HEIGHT // 2 + radius * math.sin(angle)

        # Clamp to bounds
        x = max(Config.GOAL_MIN_X, min(Config.GOAL_MAX_X, x))
        y = max(Config.GOAL_MIN_Y, min(Config.GOAL_MAX_Y, y))

        return Point(x, y)

    def _is_goal_valid(self, goal: Point, obstacles: Optional[List[Obstacle]]) -> bool:
        """Check if goal position is valid (not in obstacles)"""
        if not goal.is_valid():
            return False

        if obstacles:
            for obstacle in obstacles:
                if obstacle.contains_point(goal, Config.ROBOT_RADIUS * 2):  # Extra safety
                    return False

        return True

class GridMap:
    """Enhanced grid-based map representation"""

    def __init__(self, width: int, height: int, resolution: int = Config.GRID_RESOLUTION):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.grid_width = width // resolution
        self.grid_height = height // resolution
        try:
            self.grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        except ImportError:
            # Fallback if numpy not available
            self.grid = [[0 for _ in range(self.grid_width)] for _ in range(self.grid_height)]

    def is_valid_grid_pos(self, gx: int, gy: int) -> bool:
        """Check if grid position is valid"""
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

    def is_walkable(self, gx: int, gy: int) -> bool:
        """Check if grid cell is walkable"""
        if not self.is_valid_grid_pos(gx, gy):
            return False
        return self.grid[gy][gx] == 0

    def set_obstacle(self, gx: int, gy: int, value: int = 1):
        """Set obstacle in grid"""
        if self.is_valid_grid_pos(gx, gy):
            self.grid[gy][gx] = value

    def add_obstacle(self, obstacle: Obstacle, robot_radius: float = Config.ROBOT_RADIUS):
        """Add obstacle to grid with safety margins"""
        center_gx = int(obstacle.x // self.resolution)
        center_gy = int(obstacle.y // self.resolution)

        # Calculate expanded radius with safety margin
        expanded_radius = (obstacle.size/2 + robot_radius) * Config.SAFETY_MARGIN_MULTIPLIER
        grid_radius = int(math.ceil(expanded_radius / self.resolution))

        # Mark cells as obstacles
        for dy in range(-grid_radius, grid_radius + 1):
            for dx in range(-grid_radius, grid_radius + 1):
                gx, gy = center_gx + dx, center_gy + dy
                if self.is_valid_grid_pos(gx, gy):
                    # Use distance check for circular obstacles
                    dist = math.sqrt(dx*dx + dy*dy) * self.resolution
                    if dist <= expanded_radius:
                        self.set_obstacle(gx, gy, 1)

class JumpPointSearch:
    """Optimized Jump Point Search implementation"""

    def __init__(self, grid_map: GridMap):
        self.grid_map = grid_map
        self.grid = grid_map.grid
        self.grid_width = grid_map.grid_width
        self.grid_height = grid_map.grid_height

    def find_path(self, start: Point, goal: Point) -> List[Point]:
        """Find path using JPS algorithm"""
        start_grid = start.to_grid(self.grid_map.resolution)
        goal_grid = goal.to_grid(self.grid_map.resolution)

        # Check if start and goal are valid
        if not (self.grid_map.is_walkable(start_grid[0], start_grid[1]) and 
                self.grid_map.is_walkable(goal_grid[0], goal_grid[1])):
            return []

        open_set = [(0, start_grid)]
        came_from = {}
        g_score = {start_grid: 0}
        closed_set = set()

        while open_set:
            current_f, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)

            if current == goal_grid:
                return self._reconstruct_path(came_from, current)

            # Get jump points
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue

                    jump_point = self._jump(current[0], current[1], dx, dy, goal_grid)
                    if jump_point and jump_point not in closed_set:
                        distance = math.sqrt((jump_point[0] - current[0])**2 + 
                                           (jump_point[1] - current[1])**2)
                        tentative_g = g_score[current] + distance

                        if jump_point not in g_score or tentative_g < g_score[jump_point]:
                            g_score[jump_point] = tentative_g
                            h_score = math.sqrt((jump_point[0] - goal_grid[0])**2 + 
                                              (jump_point[1] - goal_grid[1])**2)
                            f_score = tentative_g + h_score
                            heapq.heappush(open_set, (f_score, jump_point))
                            came_from[jump_point] = current

        return []

    def _jump(self, x: int, y: int, dx: int, dy: int, goal: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """Jump point search core logic"""
        nx, ny = x + dx, y + dy

        if not self.grid_map.is_walkable(nx, ny):
            return None

        if (nx, ny) == goal:
            return (nx, ny)

        # Check for forced neighbors
        if dx != 0 and dy != 0:  # Diagonal movement
            if ((self.grid_map.is_walkable(nx - dx, ny) and not self.grid_map.is_walkable(nx - dx, ny - dy)) or
                (self.grid_map.is_walkable(nx, ny - dy) and not self.grid_map.is_walkable(nx - dx, ny - dy))):
                return (nx, ny)

            # Recursive horizontal and vertical jumps
            if (self._jump(nx, ny, dx, 0, goal) or self._jump(nx, ny, 0, dy, goal)):
                return (nx, ny)
        else:  # Horizontal or vertical movement
            if dx != 0:  # Horizontal
                if ((self.grid_map.is_walkable(nx, ny + 1) and not self.grid_map.is_walkable(nx - dx, ny + 1)) or
                    (self.grid_map.is_walkable(nx, ny - 1) and not self.grid_map.is_walkable(nx - dx, ny - 1))):
                    return (nx, ny)
            else:  # Vertical
                if ((self.grid_map.is_walkable(nx + 1, ny) and not self.grid_map.is_walkable(nx + 1, ny - dy)) or
                    (self.grid_map.is_walkable(nx - 1, ny) and not self.grid_map.is_walkable(nx - 1, ny - dy))):
                    return (nx, ny)

        return self._jump(nx, ny, dx, dy, goal)

    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Point]:
        """Reconstruct path from grid coordinates to Points"""
        path_grid = [current]
        while current in came_from:
            current = came_from[current]
            path_grid.append(current)
        path_grid.reverse()

        # Convert to Point objects
        path_points = [Point.from_grid(gx, gy, self.grid_map.resolution) 
                      for gx, gy in path_grid]
        return path_points

class EnhancedAStarPathfinder:
    """Enhanced A* pathfinder with modern optimizations"""

    def __init__(self, canvas_width: int = Config.CANVAS_WIDTH, 
                 canvas_height: int = Config.CANVAS_HEIGHT, 
                 robot_radius: int = Config.ROBOT_RADIUS, 
                 step_size: float = 15.0):
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.robot_radius = robot_radius
        self.step_size = step_size
        self.grid_map = GridMap(canvas_width, canvas_height)

    def is_valid_point(self, point: Point, obstacles: List[Obstacle], time_ahead: float = 0.0) -> bool:
        """Enhanced validity check with dynamic obstacles"""
        if not point.is_valid(self.canvas_width, self.canvas_height, self.robot_radius):
            return False

        # Check dynamic obstacles with prediction
        for obstacle in obstacles:
            if obstacle.contains_point(point, self.robot_radius, time_ahead):
                return False

        return True

    def is_line_clear(self, start: Point, end: Point, obstacles: List[Obstacle], 
                     step_size: float = 2.0, time_ahead: float = 0.0) -> bool:
        """Enhanced line clearance check"""
        distance = start.distance_to(end)
        if distance == 0:
            return True

        steps = int(math.ceil(distance / step_size))
        dx = (end.x - start.x) / steps
        dy = (end.y - start.y) / steps

        for i in range(steps + 1):
            test_point = Point(start.x + i * dx, start.y + i * dy)
            if not self.is_valid_point(test_point, obstacles, time_ahead):
                return False

        return True

    def heuristic(self, point: Point, goal: Point, algorithm: str = "euclidean") -> float:
        """Enhanced heuristic with multiple options"""
        if algorithm == "manhattan":
            return point.manhattan_distance_to(goal)
        elif algorithm == "diagonal":
            dx = abs(point.x - goal.x)
            dy = abs(point.y - goal.y)
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
        else:  # euclidean (default)
            return point.distance_to(goal)

    def get_neighbors(self, point: Point, obstacles: List[Obstacle], 
                     time_ahead: float = 0.0) -> List[Point]:
        """Enhanced neighbor generation with optimizations"""
        neighbors = []
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),  # Cardinal
            (1, 1), (-1, -1), (1, -1), (-1, 1)  # Diagonal
        ]

        for dx, dy in directions:
            if dx != 0 and dy != 0:
                factor = self.step_size / math.sqrt(2)
            else:
                factor = self.step_size

            new_point = Point(point.x + dx * factor, point.y + dy * factor)
            if (self.is_valid_point(new_point, obstacles, time_ahead) and 
                self.is_line_clear(point, new_point, obstacles, time_ahead=time_ahead)):
                neighbors.append(new_point)

        return neighbors

    def find_path(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> List[Point]:
        """Enhanced A* with tie-breaking and optimizations"""
        open_set: List[Tuple[float, int, Point]] = []
        counter = 0
        heapq.heappush(open_set, (0.0, counter, start))

        came_from: Dict[Point, Point] = {}
        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}
        closed = set()
        nodes_explored = 0

        start_time = time.time()

        while open_set and nodes_explored < Config.MAX_SEARCH_NODES:
            # Check timeout
            if time.time() - start_time > Config.MAX_PLANNING_TIME:
                print(f"⚠️ Pathfinding timeout after {Config.MAX_PLANNING_TIME}s")
                break

            _, _, current = heapq.heappop(open_set)
            if current in closed:
                continue

            closed.add(current)
            nodes_explored += 1

            if current.distance_to(goal) <= Config.GOAL_TOLERANCE:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                print(f"✅ A* found path with {nodes_explored} nodes explored")
                return list(reversed(path))

            for neighbor in self.get_neighbors(current, obstacles):
                if neighbor in closed:
                    continue

                tentative_g = g_score[current] + current.distance_to(neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_val = tentative_g + self.heuristic(neighbor, goal)
                    counter += 1
                    heapq.heappush(open_set, (f_val, counter, neighbor))

        print(f"❌ A* failed to find path after exploring {nodes_explored} nodes")
        return []

class HybridPathfinder:
    """Intelligent pathfinder that chooses the best algorithm"""

    def __init__(self):
        self.a_star = EnhancedAStarPathfinder()
        self.grid_map = GridMap(Config.CANVAS_WIDTH, Config.CANVAS_HEIGHT)
        self.jps = None

    def _analyze_environment(self, obstacles: List[Obstacle]) -> Dict[str, Any]:
        """Analyze environment to choose optimal algorithm"""
        total_cells = self.grid_map.grid_width * self.grid_map.grid_height

        # Reset grid
        try:
            self.grid_map.grid.fill(0)
        except AttributeError:
            # Fallback for list-based grid
            for y in range(self.grid_map.grid_height):
                for x in range(self.grid_map.grid_width):
                    self.grid_map.grid[y][x] = 0

        # Add obstacles to grid
        for obstacle in obstacles:
            self.grid_map.add_obstacle(obstacle)

        try:
            occupied_cells = np.count_nonzero(self.grid_map.grid)
        except:
            # Fallback counting
            occupied_cells = sum(sum(1 for cell in row if cell != 0) 
                               for row in self.grid_map.grid)

        obstacle_density = occupied_cells / total_cells

        # Check for large open areas (good for JPS)
        open_area_score = self._calculate_open_area_score()

        return {
            'obstacle_density': obstacle_density,
            'open_area_score': open_area_score,
            'total_obstacles': len(obstacles),
            'dynamic_obstacles': sum(1 for obs in obstacles if obs.is_dynamic)
        }

    def _calculate_open_area_score(self) -> float:
        """Calculate how much open area exists (higher score = more open)"""
        try:
            visited = np.zeros_like(self.grid_map.grid, dtype=bool)
        except:
            # Fallback for list-based grid
            visited = [[False for _ in range(self.grid_map.grid_width)] 
                      for _ in range(self.grid_map.grid_height)]

        largest_area = 0

        for y in range(self.grid_map.grid_height):
            for x in range(self.grid_map.grid_width):
                if not visited[y][x] and self.grid_map.is_walkable(x, y):
                    area_size = self._flood_fill_size(x, y, visited)
                    largest_area = max(largest_area, area_size)

        try:
            total_walkable = np.count_nonzero(np.array(self.grid_map.grid) == 0)
        except:
            total_walkable = sum(sum(1 for cell in row if cell == 0) 
                               for row in self.grid_map.grid)

        return (largest_area / total_walkable) if total_walkable > 0 else 0

    def _flood_fill_size(self, start_x: int, start_y: int, visited) -> int:
        """Calculate size of connected open area using flood fill"""
        stack = [(start_x, start_y)]
        size = 0

        while stack:
            x, y = stack.pop()
            if (not self.grid_map.is_valid_grid_pos(x, y) or 
                visited[y][x] or not self.grid_map.is_walkable(x, y)):
                continue

            visited[y][x] = True
            size += 1

            # Add neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                stack.append((x + dx, y + dy))

        return size

    def find_path(self, start: Point, goal: Point, obstacles: List[Obstacle]) -> List[Point]:
        """Find path using the most appropriate algorithm"""
        env_analysis = self._analyze_environment(obstacles)

        # Decision logic for algorithm selection
        use_jps = (Config.USE_JPS and 
                  env_analysis['obstacle_density'] < 0.4 and  # Not too cluttered
                  env_analysis['open_area_score'] > 0.3 and   # Has decent open areas
                  env_analysis['dynamic_obstacles'] == 0)     # No dynamic obstacles

        if use_jps:
            print(f"🔥 Using JPS (Open area: {env_analysis['open_area_score']:.2f}, Density: {env_analysis['obstacle_density']:.2f})")
            if self.jps is None:
                self.jps = JumpPointSearch(self.grid_map)
            try:
                path = self.jps.find_path(start, goal)
                if path:
                    return self._post_process_path(path, obstacles)
            except Exception as e:
                print(f"⚠️ JPS failed: {e}, falling back to A*")

        # Fallback to A* or if JPS not suitable
        print(f"🎯 Using Enhanced A* (Obstacles: {env_analysis['total_obstacles']}, Dynamic: {env_analysis['dynamic_obstacles']})")
        path = self.a_star.find_path(start, goal, obstacles)
        return self._post_process_path(path, obstacles)

    def _post_process_path(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
        """Post-process path for optimization"""
        if not path or not Config.ENABLE_PATH_SMOOTHING:
            return path

        # Path smoothing
        smoothed_path = self._smooth_path(path, obstacles)

        # Remove redundant waypoints
        optimized_path = self._remove_redundant_waypoints(smoothed_path, obstacles)

        print(f"📈 Path optimized: {len(path)} -> {len(optimized_path)} waypoints")
        return optimized_path

    def _smooth_path(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
        """Smooth path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]  # Keep start point

        i = 0
        while i < len(path) - 1:
            # Try to connect current point to the farthest possible point
            farthest_idx = i + 1

            for j in range(i + 2, len(path)):
                if self.a_star.is_line_clear(path[i], path[j], obstacles):
                    farthest_idx = j
                else:
                    break

            smoothed.append(path[farthest_idx])
            i = farthest_idx

        return smoothed

    def _remove_redundant_waypoints(self, path: List[Point], obstacles: List[Obstacle]) -> List[Point]:
        """Remove waypoints that are too close together"""
        if len(path) <= 2:
            return path

        optimized = [path[0]]

        for i in range(1, len(path)):
            if (i == len(path) - 1 or  # Keep last point
                optimized[-1].distance_to(path[i]) >= Config.WAYPOINT_DISTANCE_THRESHOLD):
                optimized.append(path[i])

        return optimized

class DynamicGoalTracker:
    """Enhanced goal tracker with dynamic goal generation"""

    def __init__(self, controller):
        self.controller = controller
        self.current_goal: Optional[GoalStatus] = None
        self.is_tracking = False
        self.tracking_thread = None
        self.stop_tracking = False

        # NEW: Goal generation components
        self.goal_generator = DynamicGoalGenerator()
        self.is_auto_generating = False
        self.auto_generation_thread = None
        self.last_generation_time = 0

    def start_tracking(self, initial_goal: Point, enable_auto_generation: bool = False, 
                      generation_mode: str = "random"):
        """Start tracking goal position changes with optional auto-generation"""
        if not Config.ENABLE_DYNAMIC_GOAL_TRACKING:
            return

        self.current_goal = GoalStatus(
            position=initial_goal,
            last_updated=time.time(),
            change_count=0,
            generation_mode="manual"
        )

        self.is_tracking = True
        self.stop_tracking = False

        # Start background tracking thread
        self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.tracking_thread.start()

        # NEW: Start auto-generation if enabled
        if enable_auto_generation and Config.ENABLE_DYNAMIC_GOAL_GENERATION:
            self.start_auto_generation(generation_mode)

        print(f"🎯 Started goal tracking at ({initial_goal.x:.1f}, {initial_goal.y:.1f})")
        if enable_auto_generation:
            print(f"🔄 Auto-generation enabled in {generation_mode.upper()} mode")

    def start_auto_generation(self, mode: str = "random"):
        """NEW: Start automatic goal generation"""
        if not Config.ENABLE_DYNAMIC_GOAL_GENERATION:
            return

        self.goal_generator.set_mode(mode)
        self.is_auto_generating = True
        self.last_generation_time = time.time()

        # Start auto-generation thread
        self.auto_generation_thread = threading.Thread(target=self._auto_generation_loop, daemon=True)
        self.auto_generation_thread.start()

        print(f"🔄 Started automatic goal generation in {mode.upper()} mode")

    def stop_auto_generation(self):
        """NEW: Stop automatic goal generation"""
        self.is_auto_generating = False
        if self.auto_generation_thread and self.auto_generation_thread.is_alive():
            self.auto_generation_thread.join(timeout=1.0)
        print("🛑 Auto goal generation stopped")

    def stop_goal_tracking(self):
        """Stop tracking goal position"""
        self.stop_tracking = True
        self.is_tracking = False
        self.stop_auto_generation()

        if self.tracking_thread and self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=1.0)

        print("🛑 Goal tracking stopped")

    def _tracking_loop(self):
        """Background thread to track goal position changes"""
        while not self.stop_tracking and self.is_tracking:
            try:
                # Get current goal position from server
                current_server_goal = self._get_server_goal_position()

                if current_server_goal and self.current_goal:
                    distance_moved = self.current_goal.position.distance_to(current_server_goal)

                    # Check if goal has moved significantly
                    if distance_moved >= Config.GOAL_POSITION_THRESHOLD:
                        self.current_goal.position = current_server_goal
                        self.current_goal.last_updated = time.time()
                        self.current_goal.change_count += 1
                        self.current_goal.is_changed = True

                        # Determine if this was auto-generated or manual
                        if self.is_auto_generating:
                            self.current_goal.generation_mode = self.goal_generator.current_mode.value
                            self.current_goal.is_auto_generated = True

                        print(f"🎯 Goal moved {distance_moved:.1f}px to ({current_server_goal.x:.1f}, {current_server_goal.y:.1f})")
                        print(f"🔄 Goal change #{self.current_goal.change_count} ({self.current_goal.generation_mode})")

                        # Warning if too many goal changes
                        if self.current_goal.change_count >= Config.MAX_GOAL_CHANGES:
                            print(f"⚠️ Warning: Goal changed {self.current_goal.change_count} times!")

                # Sleep for the specified interval
                time.sleep(Config.GOAL_CHECK_INTERVAL)

            except Exception as e:
                print(f"⚠️ Goal tracking error: {e}")
                time.sleep(Config.GOAL_CHECK_INTERVAL)

    def _auto_generation_loop(self):
        """NEW: Background thread for automatic goal generation"""
        while not self.stop_tracking and self.is_auto_generating:
            try:
                current_time = time.time()

                # Check if it's time to generate a new goal
                if current_time - self.last_generation_time >= Config.GOAL_GENERATION_INTERVAL:
                    # Get current obstacles for better goal placement
                    obstacles_data = self.controller.get_obstacles()
                    obstacles = [Obstacle(obs['x'], obs['y'], obs.get('size', 25)) 
                               for obs in obstacles_data]

                    # Generate new goal
                    current_goal_pos = self.current_goal.position if self.current_goal else None
                    new_goal = self.goal_generator.generate_next_goal(current_goal_pos, obstacles)

                    # Set the new goal on the server
                    if self.controller.set_goal(new_goal.x, new_goal.y):
                        self.last_generation_time = current_time
                        print(f"🔄 Auto-generated new goal: ({new_goal.x:.1f}, {new_goal.y:.1f}) [{self.goal_generator.current_mode.value}]")

                # Sleep for a short time before checking again
                time.sleep(1.0)

            except Exception as e:
                print(f"⚠️ Auto-generation error: {e}")
                time.sleep(2.0)

    def _get_server_goal_position(self) -> Optional[Point]:
        """Get current goal position from server"""
        try:
            response = self.controller._make_request("GET", "/goal")
            if response:
                goal_data = response.json()
                if 'x' in goal_data and 'y' in goal_data:
                    return Point(goal_data['x'], goal_data['y'])
        except Exception:
            pass
        return None

    def has_goal_changed(self) -> bool:
        """Check if goal has changed and reset the flag"""
        if self.current_goal and self.current_goal.is_changed:
            self.current_goal.is_changed = False
            return True
        return False

    def get_current_goal(self) -> Optional[Point]:
        """Get current goal position"""
        return self.current_goal.position if self.current_goal else None

    def set_generation_mode(self, mode: str):
        """NEW: Change goal generation mode"""
        if self.is_auto_generating:
            self.goal_generator.set_mode(mode)
        else:
            print("⚠️ Auto-generation is not currently active")

class EnhancedRobotController:
    """Enhanced robot controller with dynamic goal generation"""

    def __init__(self, server_url: str = Config.BASE_URL):
        self.server_url = server_url
        self.pathfinder = HybridPathfinder()
        try:
            self.session = requests.Session()  # Reuse connections
        except:
            self.session = None
        self.last_obstacles = []
        self.performance_stats = {
            'total_navigations': 0,
            'successful_navigations': 0,
            'average_path_length': 0,
            'average_compute_time': 0,
            'goal_changes': 0,
            'replanning_due_to_goal_change': 0,
            'auto_generated_goals': 0,  # NEW: Track auto-generated goals
            'manual_goal_changes': 0    # NEW: Track manual goal changes
        }

        # Initialize enhanced goal tracker
        self.goal_tracker = DynamicGoalTracker(self)

    def _make_request(self, method: str, endpoint: str, **kwargs) -> Optional[requests.Response]:
        """Enhanced request handling with retries and error handling"""
        url = f"{self.server_url}{endpoint}"

        for attempt in range(Config.MAX_RETRIES):
            try:
                if self.session:
                    response = self.session.request(
                        method, url, 
                        timeout=Config.REQUEST_TIMEOUT,
                        **kwargs
                    )
                else:
                    response = requests.request(
                        method, url,
                        timeout=Config.REQUEST_TIMEOUT,
                        **kwargs
                    )

                if response.status_code == 200:
                    return response
                else:
                    print(f"⚠️ Server returned status {response.status_code}")

            except requests.exceptions.RequestException as e:
                if attempt < Config.MAX_RETRIES - 1:
                    print(f"🔄 Request failed (attempt {attempt + 1}), retrying... Error: {e}")
                    time.sleep(0.5 * (attempt + 1))  # Exponential backoff
                else:
                    print(f"❌ Request failed after {Config.MAX_RETRIES} attempts: {e}")
            except Exception as e:
                print(f"❌ Unexpected error: {e}")

        return None

    def get_robot_status(self) -> Dict[str, Any]:
        """Get current robot status with enhanced error handling"""
        response = self._make_request("GET", "/status")
        return response.json() if response else {}

    def get_obstacles(self) -> List[Dict[str, Any]]:
        """Get current obstacles from server"""
        response = self._make_request("GET", "/obstacles")
        if response:
            return response.json().get('obstacles', [])
        return []

    def move_robot(self, x: float, y: float) -> bool:
        """Move robot to specific coordinates"""
        response = self._make_request(
            "POST", "/move", 
            json={"x": x, "y": y},
            headers={"Content-Type": "application/json"}
        )
        return response is not None

    def set_goal(self, x: float, y: float) -> bool:
        """Set goal position"""
        response = self._make_request(
            "POST", "/goal", 
            json={"x": x, "y": y},
            headers={"Content-Type": "application/json"}
        )
        if response:
            print(f"🎯 Goal set at ({x:.1f}, {y:.1f})")
        return response is not None

    def reset_simulation(self) -> bool:
        """Reset simulation state"""
        # Stop goal tracking if active
        if self.goal_tracker.is_tracking:
            self.goal_tracker.stop_goal_tracking()

        response = self._make_request("POST", "/reset")
        if response:
            print("✅ Simulation reset successful")
        return response is not None

    def generate_random_obstacles(self, count: int = 8) -> bool:
        """Generate random obstacles"""
        response = self._make_request(
            "POST", "/obstacles/random", 
            json={"count": count},
            headers={"Content-Type": "application/json"}
        )
        if response:
            print(f"🚧 Generated {count} random obstacles")
        return response is not None

    def broadcast_reroute(self, obstacle_coords: Tuple[float, float]) -> bool:
        """Broadcast reroute notification"""
        response = self._make_request(
            "POST", "/broadcast",
            json={"message": f"Rerouting around obstacle at {obstacle_coords}"},
            headers={"Content-Type": "application/json"}
        )
        return response is not None

    def is_goal_reached(self) -> bool:
        """Check if goal is reached"""
        response = self._make_request("GET", "/goal/status")
        if response:
            return response.json().get('goal_reached', False)
        return False

    def navigate_to_goal(self, start_x: float, start_y: float, 
                        goal_x: float, goal_y: float, 
                        enable_dynamic_replanning: bool = True,
                        enable_goal_tracking: bool = True,
                        enable_auto_goal_generation: bool = False,
                        goal_generation_mode: str = "random") -> bool:
        """Enhanced navigation with dynamic goal generation"""

        print(f"🚀 Starting enhanced navigation with dynamic goals from ({start_x:.1f}, {start_y:.1f}) to ({goal_x:.1f}, {goal_y:.1f})")
        if enable_auto_goal_generation:
            print(f"🔄 Auto goal generation: {goal_generation_mode.upper()} mode")

        # Performance tracking
        navigation_start = time.time()
        self.performance_stats['total_navigations'] += 1

        # Set the goal
        if not self.set_goal(goal_x, goal_y):
            print("❌ Failed to set goal")
            return False

        # NEW: Start enhanced goal tracking with auto-generation
        initial_goal = Point(goal_x, goal_y)
        if enable_goal_tracking and Config.ENABLE_DYNAMIC_GOAL_TRACKING:
            self.goal_tracker.start_tracking(
                initial_goal, 
                enable_auto_generation=enable_auto_goal_generation,
                generation_mode=goal_generation_mode
            )

        # Get current obstacles and create enhanced obstacle objects
        obstacles_data = self.get_obstacles()
        obstacles = []

        for obs_data in obstacles_data:
            obstacle = Obstacle(
                x=obs_data['x'], 
                y=obs_data['y'], 
                size=obs_data.get('size', 25)
            )
            # Add dynamic properties if available
            if 'velocity_x' in obs_data:
                obstacle.velocity_x = obs_data['velocity_x']
                obstacle.velocity_y = obs_data.get('velocity_y', 0)
                obstacle.is_dynamic = True

            obstacles.append(obstacle)

        print(f"🗺️ Environment: {len(obstacles)} obstacles ({sum(1 for o in obstacles if o.is_dynamic)} dynamic)")

        # Initial path planning
        start_point = Point(start_x, start_y)
        current_goal_point = initial_goal

        print("🧠 Computing optimal path...")
        path_start_time = time.time()
        path = self.pathfinder.find_path(start_point, current_goal_point, obstacles)
        compute_time = time.time() - path_start_time

        if not path:
            print("❌ No collision-free path found!")
            if enable_goal_tracking:
                self.goal_tracker.stop_goal_tracking()
            return False

        # Calculate path statistics
        path_length = sum(path[i].distance_to(path[i+1]) for i in range(len(path)-1))
        print(f"✅ Path computed! Length: {path_length:.1f}px, Time: {compute_time:.3f}s, Waypoints: {len(path)}")

        # Update performance stats
        self.performance_stats['average_compute_time'] = (
            (self.performance_stats['average_compute_time'] * (self.performance_stats['total_navigations'] - 1) + compute_time) / 
            self.performance_stats['total_navigations']
        )

        # Execute path with enhanced goal tracking
        success = self._execute_path_with_dynamic_goals(
            path, current_goal_point, obstacles, 
            enable_dynamic_replanning, enable_goal_tracking
        )

        # Stop goal tracking
        if enable_goal_tracking:
            self.goal_tracker.stop_goal_tracking()

        # Update performance statistics
        total_time = time.time() - navigation_start
        if success:
            self.performance_stats['successful_navigations'] += 1
            self.performance_stats['average_path_length'] = (
                (self.performance_stats['average_path_length'] * (self.performance_stats['successful_navigations'] - 1) + path_length) / 
                self.performance_stats['successful_navigations']
            )
            print(f"🎉 Navigation completed successfully in {total_time:.2f}s!")
        else:
            print(f"❌ Navigation failed after {total_time:.2f}s")

        return success

    def _execute_path_with_dynamic_goals(self, path: List[Point], initial_goal: Point, 
                                        original_obstacles: List[Obstacle], 
                                        enable_replanning: bool, 
                                        enable_goal_tracking: bool) -> bool:
        """NEW: Execute path with enhanced goal generation and tracking"""

        current_path = path[:]
        current_goal = initial_goal
        waypoint_idx = 0
        replanning_count = 0
        max_replanning = 3

        while waypoint_idx < len(current_path):
            waypoint = current_path[waypoint_idx]
            print(f"📍 Moving to waypoint {waypoint_idx + 1}/{len(current_path)}: ({waypoint.x:.1f}, {waypoint.y:.1f})")

            if not self.move_robot(waypoint.x, waypoint.y):
                print(f"❌ Failed to move to waypoint {waypoint_idx + 1}")
                return False

            # Wait for movement
            time.sleep(Config.MOVEMENT_DELAY)

            # Check for goal reached (using current goal position)
            if self.is_goal_reached():
                print("🎯 Goal reached!")
                return True

            # Enhanced goal change detection
            if enable_goal_tracking and self.goal_tracker.has_goal_changed():
                new_goal = self.goal_tracker.get_current_goal()
                if new_goal:
                    print(f"🎯➡️ Goal changed! New target: ({new_goal.x:.1f}, {new_goal.y:.1f})")
                    current_goal = new_goal
                    self.performance_stats['goal_changes'] += 1

                    # Track if it was auto-generated or manual
                    if (self.goal_tracker.current_goal and 
                        self.goal_tracker.current_goal.is_auto_generated):
                        self.performance_stats['auto_generated_goals'] += 1
                    else:
                        self.performance_stats['manual_goal_changes'] += 1

                    # Replan path to new goal
                    current_pos = Point(waypoint.x, waypoint.y)
                    current_obstacles_data = self.get_obstacles()
                    new_obstacles = [Obstacle(obs['x'], obs['y'], obs.get('size', 25)) 
                                   for obs in current_obstacles_data]

                    new_path = self.pathfinder.find_path(current_pos, current_goal, new_obstacles)

                    if new_path:
                        current_path = new_path
                        waypoint_idx = 0
                        self.performance_stats['replanning_due_to_goal_change'] += 1

                        mode = "AUTO" if (self.goal_tracker.current_goal and 
                                        self.goal_tracker.current_goal.is_auto_generated) else "MANUAL"
                        print(f"🔄 Replanned path to new goal with {len(new_path)} waypoints [{mode}]")
                        continue
                    else:
                        print("⚠️ Failed to plan path to new goal, continuing with current path")

            # Dynamic obstacle detection and replanning
            if enable_replanning and Config.ENABLE_DYNAMIC_OBSTACLES:
                current_obstacles_data = self.get_obstacles()

                # Check if obstacles have changed significantly
                if self._obstacles_changed(original_obstacles, current_obstacles_data):
                    print("⚠️ Obstacle configuration changed, replanning...")

                    if replanning_count < max_replanning:
                        new_obstacles = [Obstacle(obs['x'], obs['y'], obs.get('size', 25)) 
                                       for obs in current_obstacles_data]

                        current_pos = Point(waypoint.x, waypoint.y)
                        new_path = self.pathfinder.find_path(current_pos, current_goal, new_obstacles)

                        if new_path:
                            current_path = new_path
                            waypoint_idx = 0
                            replanning_count += 1
                            print(f"🔄 Replanned path with {len(new_path)} waypoints (attempt {replanning_count})")

                            # Broadcast reroute notification
                            self.broadcast_reroute((waypoint.x, waypoint.y))
                            continue
                        else:
                            print("⚠️ Replanning failed, continuing with original path")
                    else:
                        print(f"⚠️ Max replanning attempts ({max_replanning}) reached")

            waypoint_idx += 1

        # Final goal check
        return self.is_goal_reached()

    def _obstacles_changed(self, original: List[Obstacle], current_data: List[Dict]) -> bool:
        """Check if obstacles have changed significantly"""
        if len(original) != len(current_data):
            return True

        # Simple change detection - could be enhanced
        threshold = 10.0  # pixels

        for i, obs_data in enumerate(current_data):
            if i >= len(original):
                return True

            orig = original[i]
            if (abs(orig.x - obs_data['x']) > threshold or 
                abs(orig.y - obs_data['y']) > threshold):
                return True

        return False

    def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics with goal generation data"""
        success_rate = (self.performance_stats['successful_navigations'] / 
                       max(1, self.performance_stats['total_navigations']) * 100)

        return {
            **self.performance_stats,
            'success_rate_percent': success_rate
        }

    def print_performance_stats(self):
        """Print detailed performance statistics including goal generation"""
        stats = self.get_performance_stats()
        print(f"""
📊 Performance Statistics:
├── Total Navigations: {stats['total_navigations']}
├── Successful: {stats['successful_navigations']} ({stats['success_rate_percent']:.1f}%)
├── Average Path Length: {stats['average_path_length']:.1f} pixels
├── Average Compute Time: {stats['average_compute_time']:.3f} seconds
├── 🎯 Total Goal Changes: {stats['goal_changes']}
├── 🔄 Goal-based Replannings: {stats['replanning_due_to_goal_change']}
├── 🤖 Auto-Generated Goals: {stats['auto_generated_goals']}
└── 👤 Manual Goal Changes: {stats['manual_goal_changes']}
        """)

def main():
    """Enhanced main execution function with dynamic goal generation"""
    print("=" * 70)
    print("🤖 ENHANCED ROBOT NAVIGATION SYSTEM v2.2")
    print("🔥 Features: Hybrid JPS/A*, Dynamic Goals, Auto-Generation")
    print("=" * 70)

    controller = EnhancedRobotController()

    # System checks
    print("🔍 System Checks:")
    print(f"├── Server URL: {Config.BASE_URL}")
    print(f"├── Grid Resolution: {Config.GRID_RESOLUTION}px")
    print(f"├── JPS Enabled: {Config.USE_JPS}")
    print(f"├── Path Smoothing: {Config.ENABLE_PATH_SMOOTHING}")
    print(f"├── Dynamic Obstacles: {Config.ENABLE_DYNAMIC_OBSTACLES}")
    print(f"├── 🎯 Goal Tracking: {Config.ENABLE_DYNAMIC_GOAL_TRACKING} (Check every {Config.GOAL_CHECK_INTERVAL}s)")
    print(f"└── 🔄 Goal Generation: {Config.ENABLE_DYNAMIC_GOAL_GENERATION} (Change every {Config.GOAL_GENERATION_INTERVAL}s)")
    print()

    # Test server connection
    status = controller.get_robot_status()
    if not status:
        print("❌ Cannot connect to server. Please ensure server.py is running on localhost:5001")
        return
    else:
        print("✅ Server connection successful")

    while True:
        print(f"""
🎮 Navigation Options:
1. Quick Test (Default scenario)
2. Custom Navigation (Specify coordinates)
3. Corner Navigation (NE, NW, SE, SW)
4. 🎯 Dynamic Goal Test (Goal tracking demo)
5. 🔄 Auto-Generation Demo (Automatic goal changing)
6. 🎨 Goal Pattern Showcase (All generation modes)
7. Benchmark Suite (Multiple test scenarios)
8. Obstacle Stress Test (High obstacle density)
9. Performance Statistics
A. Reset Simulation
B. Advanced Configuration
0. Exit
        """)

        choice = input("Choose option (0-9, A-B): ").strip().upper()

        try:
            if choice == '1':
                # Quick test scenario
                print("🚀 Quick Test Scenario")
                controller.reset_simulation()
                time.sleep(1)

                controller.generate_random_obstacles(8)
                success = controller.navigate_to_goal(
                    320, 300, 550, 80,
                    enable_goal_tracking=True
                )
                print(f"Result: {'✅ SUCCESS' if success else '❌ FAILED'}")

            elif choice == '2':
                # Custom navigation with all options
                print("📍 Custom Navigation with Dynamic Goals")
                try:
                    start_x = float(input(f"Start X (default {Config.ROBOT_START[0]}): ") or str(Config.ROBOT_START[0]))
                    start_y = float(input(f"Start Y (default {Config.ROBOT_START[1]}): ") or str(Config.ROBOT_START[1]))
                    goal_x = float(input("Goal X: "))
                    goal_y = float(input("Goal Y: "))

                    enable_tracking = input("Enable goal tracking? (y/n, default y): ").lower().strip() != 'n'
                    enable_auto_gen = input("Enable auto goal generation? (y/n, default n): ").lower().strip() == 'y'

                    gen_mode = "random"
                    if enable_auto_gen:
                        print("Available modes: random, spiral, corners, patrol, chase")
                        gen_mode = input("Generation mode (default random): ").strip() or "random"

                    success = controller.navigate_to_goal(
                        start_x, start_y, goal_x, goal_y,
                        enable_goal_tracking=enable_tracking,
                        enable_auto_goal_generation=enable_auto_gen,
                        goal_generation_mode=gen_mode
                    )
                    print(f"Result: {'✅ SUCCESS' if success else '❌ FAILED'}")
                except ValueError:
                    print("❌ Invalid coordinates")

            elif choice == '3':
                # Corner navigation
                corners = {
                    'NE': (630, 20), 'NW': (20, 20),
                    'SE': (630, 580), 'SW': (20, 580)
                }

                print("🏃 Corner Navigation")
                corner = input("Enter corner (NE, NW, SE, SW): ").strip().upper()

                if corner in corners:
                    goal_x, goal_y = corners[corner]
                    success = controller.navigate_to_goal(
                        320, 300, goal_x, goal_y, 
                        enable_goal_tracking=True
                    )
                    print(f"Result: {'✅ SUCCESS' if success else '❌ FAILED'}")
                else:
                    print("❌ Invalid corner")

            elif choice == '4':
                # Dynamic goal tracking demo
                print("🎯 Dynamic Goal Tracking Demo")
                print("This will start navigation and track goal changes every 3 seconds.")
                print("You can manually change the goal in the simulator!")

                controller.reset_simulation()
                time.sleep(1)
                controller.generate_random_obstacles(6)

                success = controller.navigate_to_goal(
                    100, 300, 550, 100,
                    enable_goal_tracking=True
                )
                print(f"Demo Result: {'✅ SUCCESS' if success else '❌ FAILED'}")

            elif choice == '5':
                # NEW: Auto-generation demo
                print("🔄 Auto-Generation Demo")
                print("Robot will automatically generate new goals every 10 seconds!")

                available_modes = ["random", "spiral", "corners", "patrol", "chase"]
                print(f"Available modes: {', '.join(available_modes)}")
                mode = input("Choose generation mode (default: random): ").strip() or "random"

                if mode not in available_modes:
                    print(f"⚠️ Invalid mode, using 'random'")
                    mode = "random"

                controller.reset_simulation()
                time.sleep(1)
                controller.generate_random_obstacles(8)

                print(f"🚀 Starting navigation with {mode.upper()} goal generation...")
                print("💡 Watch as the robot automatically changes its destination!")

                success = controller.navigate_to_goal(
                    320, 300, 400, 200,
                    enable_goal_tracking=True,
                    enable_auto_goal_generation=True,
                    goal_generation_mode=mode
                )
                print(f"Demo Result: {'✅ SUCCESS' if success else '❌ FAILED'}")

            elif choice == '6':
                # NEW: Goal pattern showcase
                print("🎨 Goal Pattern Showcase")
                print("Demonstrating all goal generation patterns...")

                patterns = ["random", "spiral", "corners", "patrol"]

                for i, pattern in enumerate(patterns, 1):
                    print(f"\n--- Pattern {i}/4: {pattern.upper()} ---")

                    controller.reset_simulation()
                    time.sleep(1)
                    controller.generate_random_obstacles(5)  # Fewer obstacles for demo

                    # Short demo of each pattern
                    print(f"🔄 Demonstrating {pattern} goal generation for 30 seconds...")

                    # Start navigation in a separate thread for demo
                    def demo_navigation():
                        controller.navigate_to_goal(
                            320, 300, 450, 150,
                            enable_goal_tracking=True,
                            enable_auto_goal_generation=True,
                            goal_generation_mode=pattern
                        )

                    import threading
                    demo_thread = threading.Thread(target=demo_navigation, daemon=True)
                    demo_thread.start()

                    # Let it run for 30 seconds
                    time.sleep(30)

                    # Stop the demo
                    controller.goal_tracker.stop_goal_tracking()

                    if i < len(patterns):
                        input("Press Enter to continue to next pattern...")

                print("\n🎉 Pattern showcase complete!")

            elif choice == '7':
                # Benchmark suite with goal generation
                print("🏁 Running Enhanced Benchmark Suite...")

                scenarios = [
                    ("Standard navigation", False, "none"),
                    ("Goal tracking", True, "none"), 
                    ("Random goal generation", True, "random"),
                    ("Spiral goal generation", True, "spiral"),
                    ("Patrol goal generation", True, "patrol"),
                ]

                results = []
                for name, enable_gen, mode in scenarios:
                    print(f"\n📋 {name}")
                    controller.reset_simulation()
                    time.sleep(0.5)
                    controller.generate_random_obstacles(8)

                    # Run for shorter time for demo
                    def timed_navigation():
                        return controller.navigate_to_goal(
                            320, 300, 550, 100,
                            enable_goal_tracking=True,
                            enable_auto_goal_generation=enable_gen,
                            goal_generation_mode=mode
                        )

                    # Run with timeout
                    import threading
                    result_container = [False]

                    def run_nav():
                        result_container[0] = timed_navigation()

                    nav_thread = threading.Thread(target=run_nav, daemon=True)
                    nav_thread.start()
                    nav_thread.join(timeout=60)  # 1 minute timeout

                    # Stop any ongoing tracking
                    controller.goal_tracker.stop_goal_tracking()

                    results.append((name, result_container[0]))
                    time.sleep(1)

                print(f"\n📊 Benchmark Results:")
                for name, success in results:
                    status = "✅ COMPLETED" if success else "🔄 PARTIAL"
                    print(f"├── {name}: {status}")

            elif choice == '8':
                # Obstacle stress test
                print("🔥 Obstacle Stress Test")
                obstacle_counts = [10, 15, 20, 25]

                for count in obstacle_counts:
                    print(f"\n🧪 Testing with {count} obstacles...")
                    controller.reset_simulation()
                    time.sleep(0.5)
                    controller.generate_random_obstacles(count)

                    success = controller.navigate_to_goal(
                        320, 300, 550, 80,
                        enable_goal_tracking=False  # Disable for stress test
                    )
                    result = "✅ PASSED" if success else "❌ FAILED"
                    print(f"   {count} obstacles: {result}")

                    if not success:
                        print("   ⚠️ Stress test limit reached")
                        break
                    time.sleep(1)

            elif choice == '9':
                # Performance statistics
                controller.print_performance_stats()

            elif choice == 'A':
                # Reset simulation
                if controller.reset_simulation():
                    print("✅ Simulation reset complete")
                else:
                    print("❌ Reset failed")

            elif choice == 'B':
                # Advanced configuration
                print("⚙️ Advanced Configuration")
                print(f"Current settings:")
                print(f"├── Grid Resolution: {Config.GRID_RESOLUTION}")
                print(f"├── Use JPS: {Config.USE_JPS}")
                print(f"├── Path Smoothing: {Config.ENABLE_PATH_SMOOTHING}")
                print(f"├── Safety Margin: {Config.SAFETY_MARGIN_MULTIPLIER}x")
                print(f"├── Movement Delay: {Config.MOVEMENT_DELAY}s")
                print(f"├── 🎯 Goal Tracking: {Config.ENABLE_DYNAMIC_GOAL_TRACKING}")
                print(f"├── Goal Check Interval: {Config.GOAL_CHECK_INTERVAL}s")
                print(f"├── Goal Position Threshold: {Config.GOAL_POSITION_THRESHOLD}px")
                print(f"├── 🔄 Goal Generation: {Config.ENABLE_DYNAMIC_GOAL_GENERATION}")
                print(f"├── Generation Interval: {Config.GOAL_GENERATION_INTERVAL}s")
                print(f"└── Generation Radius: {Config.GOAL_GENERATION_RADIUS}px")

                modify = input("Modify settings? (y/n): ").lower().strip()
                if modify == 'y':
                    try:
                        print("\nConfigurable parameters:")

                        new_interval = input(f"Goal check interval (current: {Config.GOAL_CHECK_INTERVAL}s): ").strip()
                        if new_interval:
                            Config.GOAL_CHECK_INTERVAL = float(new_interval)
                            print(f"✅ Goal check interval updated to {Config.GOAL_CHECK_INTERVAL}s")

                        new_threshold = input(f"Goal position threshold (current: {Config.GOAL_POSITION_THRESHOLD}px): ").strip()
                        if new_threshold:
                            Config.GOAL_POSITION_THRESHOLD = float(new_threshold)
                            print(f"✅ Goal position threshold updated to {Config.GOAL_POSITION_THRESHOLD}px")

                        new_gen_interval = input(f"Goal generation interval (current: {Config.GOAL_GENERATION_INTERVAL}s): ").strip()
                        if new_gen_interval:
                            Config.GOAL_GENERATION_INTERVAL = float(new_gen_interval)
                            print(f"✅ Goal generation interval updated to {Config.GOAL_GENERATION_INTERVAL}s")

                        new_gen_radius = input(f"Goal generation radius (current: {Config.GOAL_GENERATION_RADIUS}px): ").strip()
                        if new_gen_radius:
                            Config.GOAL_GENERATION_RADIUS = float(new_gen_radius)
                            print(f"✅ Goal generation radius updated to {Config.GOAL_GENERATION_RADIUS}px")

                    except ValueError:
                        print("❌ Invalid input")

            elif choice == '0':
                print("👋 Goodbye!")
                break

            else:
                print("❌ Invalid option")

        except KeyboardInterrupt:
            print("\n⚠️ Operation cancelled")
            # Make sure to stop goal tracking on interrupt
            if controller.goal_tracker.is_tracking:
                controller.goal_tracker.stop_goal_tracking()
        except Exception as e:
            print(f"❌ Error: {e}")
            # Make sure to stop goal tracking on error
            if controller.goal_tracker.is_tracking:
                controller.goal_tracker.stop_goal_tracking()

if __name__ == "__main__":
    main()
