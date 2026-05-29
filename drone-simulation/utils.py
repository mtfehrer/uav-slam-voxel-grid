import numpy as np
import math
import pybullet as p
from collections import defaultdict

VOXEL_SIZE = 0.5

def fibonacci_sphere(n_points, randomize=False):
    points = np.zeros((n_points, 3))
    phi = np.pi * (3 - np.sqrt(5))
    for i in range(n_points):
        y = 1 - (i / float(n_points - 1)) * 2
        radius = np.sqrt(1 - y * y)
        theta = phi * i
        if randomize:
            theta = theta + np.random.uniform(-0.1, 0.1)
        x = np.cos(theta) * radius
        z = np.sin(theta) * radius
        points[i] = [x, y, z]
    return points

def voxel_traversal(start_point, end_point):
    direction = [
        end_point[0] - start_point[0],
        end_point[1] - start_point[1],
        end_point[2] - start_point[2]
    ]

    distance = (direction[0]**2 + direction[1]**2 + direction[2]**2)**0.5
    
    if distance < 1e-6:
        voxel = [
            int(start_point[0] / VOXEL_SIZE),
            int(start_point[1] / VOXEL_SIZE),
            int(start_point[2] / VOXEL_SIZE)
        ]
        return [tuple(voxel)]
    
    direction = [
        direction[0] / distance,
        direction[1] / distance,
        direction[2] / distance
    ]
    
    voxel = [
        int(start_point[0] / VOXEL_SIZE), 
        int(start_point[1] / VOXEL_SIZE), 
        int(start_point[2] / VOXEL_SIZE)
    ]
    
    step = [
        1 if direction[0] > 0 else -1 if direction[0] < 0 else 0,
        1 if direction[1] > 0 else -1 if direction[1] < 0 else 0,
        1 if direction[2] > 0 else -1 if direction[2] < 0 else 0
    ]
    
    delta_t = [
        float('inf') if abs(direction[0]) < 1e-6 else VOXEL_SIZE / abs(direction[0]),
        float('inf') if abs(direction[1]) < 1e-6 else VOXEL_SIZE / abs(direction[1]),
        float('inf') if abs(direction[2]) < 1e-6 else VOXEL_SIZE / abs(direction[2])
    ]
    
    next_boundary = [
        (voxel[0] + (1 if step[0] > 0 else 0)) * VOXEL_SIZE,
        (voxel[1] + (1 if step[1] > 0 else 0)) * VOXEL_SIZE,
        (voxel[2] + (1 if step[2] > 0 else 0)) * VOXEL_SIZE
    ]
    
    t_max = [
        float('inf') if abs(direction[0]) < 1e-6 else (next_boundary[0] - start_point[0]) / direction[0],
        float('inf') if abs(direction[1]) < 1e-6 else (next_boundary[1] - start_point[1]) / direction[1],
        float('inf') if abs(direction[2]) < 1e-6 else (next_boundary[2] - start_point[2]) / direction[2]
    ]
    
    voxel_list = []
    distance_traveled = 0
    
    while distance_traveled < distance:
        voxel_list.append(tuple(voxel))
        min_t_idx = t_max.index(min(t_max))
        distance_traveled = t_max[min_t_idx]
        voxel[min_t_idx] += step[min_t_idx]
        t_max[min_t_idx] += delta_t[min_t_idx]
    
    end_voxel = (
        int(end_point[0] / VOXEL_SIZE),
        int(end_point[1] / VOXEL_SIZE),
        int(end_point[2] / VOXEL_SIZE)
    )
    
    if end_voxel not in voxel_list:
        voxel_list.append(end_voxel)
    
    return voxel_list

def get_voxel_index(point):
    x, y, z = point
    sx = sy = sz = float(VOXEL_SIZE)
    i = math.floor(x / sx)
    j = math.floor(y / sy)
    k = math.floor(z / sz)
    return (i, j, k)

def get_voxel_center(voxel_indices):
    return (np.array(voxel_indices) + 0.5) * VOXEL_SIZE

def get_voxel_neighbor_indices(voxel_indices):
    neighbors = []
    x, y, z = voxel_indices
    deltas = [(1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)]
    for dx, dy, dz in deltas:
        neighbors.append((x+dx, y+dy, z+dz))
    return neighbors

def distance(start, end):
    return ((end[0] - start[0])**2 + (end[1] - start[1])**2 + (end[2] - start[2])**2) ** 0.5

def a_star(start_voxel_index, goal_voxel_index, occupied_voxels):
    if start_voxel_index in occupied_voxels:
        raise Exception("start is occupied")
    if goal_voxel_index in occupied_voxels:
        raise Exception("goal is occupied")

    open_set = set([start_voxel_index])
    prev = {}

    g_score = defaultdict(lambda: math.inf)
    g_score[start_voxel_index] = 0

    f_score = defaultdict(lambda: math.inf)
    f_score[start_voxel_index] = distance(start_voxel_index, goal_voxel_index)

    while open_set:
        lowest = math.inf
        for n in open_set:
            if f_score[n] < lowest:
                lowest = f_score[n]
                current = n
        open_set.remove(current)
        if current == goal_voxel_index:
            total_path = [current]
            while current in prev:
                current = prev[current]
                total_path.insert(0, current)
            return total_path

        for neighbor_index in get_voxel_neighbor_indices(current):
            if neighbor_index in occupied_voxels:
                continue
            tentative_g_score = g_score[current] + distance(current, neighbor_index)
            if tentative_g_score < g_score[neighbor_index]:
                prev[neighbor_index] = current
                g_score[neighbor_index] = tentative_g_score
                f_score[neighbor_index] = tentative_g_score + distance(neighbor_index, goal_voxel_index)
                if neighbor_index not in open_set:
                    open_set.add(neighbor_index)

    raise Exception("A* algorithm couldn't find a valid path")

def place_trajectory(points, color):
    line_ids = []
    
    for i in range(len(points)-1):
        line_id = p.addUserDebugLine(
            lineFromXYZ=points[i],
            lineToXYZ=points[i+1],
            lineColorRGB=color,
            lineWidth=2.0,
            lifeTime=0
        )
        line_ids.append(line_id)

    return line_ids

def remove_trajectory(line_ids):
    for line_id in line_ids:
        p.removeUserDebugItem(line_id)

def place_obstacles(obstacles):
    for obstacle in obstacles:
        box_col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=obstacle["half-extents"])
        box_vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=obstacle["half-extents"], rgbaColor=[1, 0, 0, 1])
        obstacle_id = p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=box_col_id,
                                        baseVisualShapeIndex=box_vis_id,
                                        basePosition=get_voxel_center(obstacle["center"]))

def get_cube_surface_voxels(half_width):
    surface_voxels = set()
    min_coord = -half_width
    max_coord = half_width
    coord_range = range(min_coord, max_coord + 1)

    for y in coord_range:
        for z in coord_range:
            surface_voxels.add((min_coord, y, z))
    for y in coord_range:
        for z in coord_range:
            surface_voxels.add((max_coord, y, z))
    for x in coord_range:
        for z in coord_range:
            surface_voxels.add((x, min_coord, z))
    for x in coord_range:
        for z in coord_range:
            surface_voxels.add((x, max_coord, z))
    for x in coord_range:
        for y in coord_range:
            surface_voxels.add((x, y, min_coord))
    for x in coord_range:
        for y in coord_range:
            surface_voxels.add((x, y, max_coord))

    return surface_voxels
