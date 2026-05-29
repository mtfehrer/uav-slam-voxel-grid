import socket
import json
import threading
import time
import pybullet as p
import utils

VOXEL_SIZE = 0.5
trajectory = [(0, 0, 1), (0, 1, 1), (0, 2, 1), (0, 3, 1), (0, 4, 1), (0, 5, 1)]
trajectory_line_ids = []

def place_voxels(voxels, color=(0.7, 0.2, 0.2, 1.0)):
    global voxel_grid
    global update_vis
    for v_index in voxels:
        pos = [utils.get_voxel_center(i) for i in v_index]
        visualShapeId = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[VOXEL_SIZE / 2, VOXEL_SIZE / 2, VOXEL_SIZE / 2],
                rgbaColor=color,
                visualFramePosition=pos
            )
        p.createMultiBody(baseVisualShapeIndex=visualShapeId, basePosition=[0,0,0])

def start_server(host='localhost', port=12345):
    global trajectory_line_ids
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    conn, addr = server_socket.accept()

    while True:
        byte = conn.recv(1)
        if byte == b'0':
            encoded_message_byte_size = conn.recv(4)
            message_byte_size = int.from_bytes(encoded_message_byte_size, byteorder="big")
            encoded_received_voxels = conn.recv(message_byte_size)
            received_voxels = [tuple(v) for v in json.loads(encoded_received_voxels.decode('utf-8'))]
            place_voxels(received_voxels)
        elif byte == b'1':
            encoded_message_byte_size = conn.recv(4)
            message_byte_size = int.from_bytes(encoded_message_byte_size, byteorder="big")
            encoded_trajectory = conn.recv(message_byte_size)
            trajectory = [tuple(v) for v in json.loads(encoded_trajectory.decode('utf-8'))]
            if trajectory_line_ids != []:
                utils.remove_trajectory(trajectory_line_ids)
            trajectory_line_ids = utils.place_trajectory([utils.get_voxel_center(i) for i in trajectory], (0, 1, 0))

t = threading.Thread(None, start_server)
t.start()

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

#show initial trajectory
#utils.place_trajectory(trajectory, (1, 0, 0))
#show border voxels
#place_voxels(list(utils.get_cube_surface_voxels(10)), (0, 0, 1, 0.1))

while p.isConnected():
    p.stepSimulation()
    time.sleep(1/120)

t.join()