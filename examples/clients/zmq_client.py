import zmq
import json
import time


def main():
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")

    # 1. Ping
    print("Sending ping...")
    socket.send_json({"command": "ping"})
    print(f"Received: {socket.recv_json()}")

    # 2. Set joint states
    print("\nMoving arm...")
    commands = [
        {"shoulder_pan": 1.0, "shoulder_lift": 0.5, "elbow_flex": -1.0},
        {"shoulder_pan": -1.0, "shoulder_lift": -0.5, "elbow_flex": 1.0},
        {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0},
    ]

    for cmd in commands:
        print(f"Sending set_joints_state: {cmd}")
        socket.send_json({"command": "set_joints_state", "joints": cmd})
        print(f"Received: {socket.recv_json()}")
        time.sleep(1)

    # 3. Read state
    print("\nReading state...")
    socket.send_json({"command": "read_state"})
    print(f"Received: {socket.recv_json()}")


if __name__ == "__main__":
    main()
