import time
import numpy as np
import pybullet as p
import utils
import socket
import json

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync

SIMULATION_FREQ_HZ = 240
CONTROL_FREQ_HZ = 48
DURATION_SEC = 200
H = 0.1
H_STEP = 0.05
R = 0.3
N_RAYS = 1000
RAY_LENGTH_SCALAR = 1.5
RAY_DIRECTIONS = np.array(utils.fibonacci_sphere(N_RAYS))
norms = np.linalg.norm(RAY_DIRECTIONS, axis=1).reshape((N_RAYS, 1))
safe_norms = np.where(norms == 0, 1, norms)
NORMALIZED_RAY_DIRECTIONS = RAY_DIRECTIONS / safe_norms
MAX_TARGET_DISTANCE_FROM_DRONE = 1
CAMERA_YAW = -30
CAMERA_PITCH = -30
DRONE = DroneModel("cf2x")
VOXEL_SIZE = 0.5
NEXT_TARGET_STEPS = int(CONTROL_FREQ_HZ * 0.4)
MAP_UPDATE_INTERVAL_STEPS = int(CONTROL_FREQ_HZ * 0.2)
OBSTACLES = [{"half-extents": (0.5, 0.5, 2), "center": (0, 3, 0)},
             {"half-extents": (2, 0.5, 0.5), "center": (1, 6, 1)}]
occupied_voxels = utils.get_cube_surface_voxels(11)

class Pathfinder:
    def __init__(self):
        self.original_trajectory = [(0, 0, 1), (0, 1, 1), (0, 2, 1), (0, 3, 1), (0, 4, 1), (0, 5, 1)]
        self.original_trajectory_index = 0
        self.detour_active = False
        self.detour_trajectory = None
        self.detour_trajectory_index = None
        self.new_detour = False
    
    def get_next_clear_target_index(self):
        i = self.original_trajectory_index
        while self.original_trajectory[i] in occupied_voxels:
            i += 1
        return i
    
    def increment_target(self):
        if self.detour_active:
            self.detour_trajectory_index += 1
        else:
            self.original_trajectory_index += 1

        if (self.detour_active and self.detour_trajectory_index >= len(self.detour_trajectory)):
            self.detour_active = False

    def get_current_target(self):
        if self.detour_active:
            current_target = self.detour_trajectory[self.detour_trajectory_index]
        else:
            current_target = self.original_trajectory[self.original_trajectory_index]
        
        if current_target in occupied_voxels:
            if self.detour_active:
                start = self.detour_trajectory[self.detour_trajectory_index - 1]
            else:
                self.detour_active = True
                self.original_trajectory_next_index = self.get_next_clear_target_index()
                start = self.original_trajectory[self.original_trajectory_index - 1]
            end = self.original_trajectory[self.original_trajectory_next_index]
            self.detour_trajectory = utils.a_star(start, end, occupied_voxels)
            self.detour_trajectory_index = 0
            self.new_detour = True
            current_target = self.detour_trajectory[0]
                
        return current_target
    
    def is_detour_new(self):
        if self.new_detour == True:
            self.new_detour = False
            return True
        else:
            return False

def get_new_occupied_voxels(drone_position):
    # refactor: this getter function mutates global state
    from_positions = np.tile(drone_position, (N_RAYS, 1))
    to_positions = from_positions + NORMALIZED_RAY_DIRECTIONS * RAY_LENGTH_SCALAR
    rayTestBatch_result = p.rayTestBatch(from_positions, to_positions)
    new_occupied_voxels = []

    for r in rayTestBatch_result:
        if r[0] != -1:
            hit_voxel_index = utils.get_voxel_index(r[3])
            if hit_voxel_index not in occupied_voxels:
                occupied_voxels.add(hit_voxel_index)
                new_occupied_voxels.append(hit_voxel_index)

    return new_occupied_voxels

def send_info_to_server(data, type_of_data):
    encoded_data = json.dumps(data).encode('utf-8')
    message_byte_size = len(encoded_data)
    encoded_message_byte_size = message_byte_size.to_bytes(4, byteorder="big")
    if type_of_data == "occupied voxels":
        byte = b'0'
    elif type_of_data == "trajectory":
        byte = b'1'

    client_socket.sendall(byte)
    client_socket.sendall(encoded_message_byte_size)
    client_socket.sendall(encoded_data)

pathfinder = Pathfinder()
env = CtrlAviary(
    drone_model=DRONE,
    num_drones=1,
    initial_xyzs=np.array([utils.get_voxel_center(pathfinder.original_trajectory[0])]),
    initial_rpys=np.array([(0, 0, 0)]),
    physics=Physics("pyb"),
    neighbourhood_radius=10,
    pyb_freq=SIMULATION_FREQ_HZ,
    ctrl_freq=CONTROL_FREQ_HZ,
    gui=True,
    record=False,
    obstacles=False,
    user_debug_gui=False,
)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
ctrl = DSLPIDControl(drone_model=DRONE)
utils.place_obstacles(OBSTACLES)
utils.place_trajectory([utils.get_voxel_center(pathfinder.original_trajectory[0]), utils.get_voxel_center(pathfinder.original_trajectory[-1])], (1, 0, 0))

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
while True:
    try:
        client_socket.connect(("localhost", 12345))
        break
    except ConnectionRefusedError:
        print(f"[!] Connection refused. Is the server running at {'localhost'}:{12345}?")
        time.sleep(1)

trajectory_line_ids = []
detour_end = None
action = np.zeros((1, 4))
START = time.time()
for i in range(0, int(DURATION_SEC * env.CTRL_FREQ)):
    obs, reward, terminated, truncated, info = env.step(action)
    drone_position = obs[0, :3]
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=CAMERA_YAW, cameraPitch=CAMERA_PITCH, cameraTargetPosition=drone_position)

    if i % MAP_UPDATE_INTERVAL_STEPS == 0:
        new_occupied_voxels = get_new_occupied_voxels(drone_position)
        send_info_to_server(new_occupied_voxels, "occupied voxels")

    if i % NEXT_TARGET_STEPS == 0:
        pathfinder.increment_target()
        target_pos = pathfinder.get_current_target()
        if pathfinder.detour_active == True and pathfinder.is_detour_new():
            send_info_to_server(pathfinder.detour_trajectory, "trajectory")

    action[0, :], _, _ = ctrl.computeControlFromState(
        control_timestep=env.CTRL_TIMESTEP, state=obs[0], target_pos=utils.get_voxel_center(target_pos)
    )
    sync(i, START, env.CTRL_TIMESTEP)

env.close()
client_socket.close()