#!/usr/bin/env python3
"""
johnbot2 host-side controller

This script implements the OSC-based host controller used in:
"From Swarm to Individual: Emergent Individuality in Light-Mediated Robot Collectives"

It:
- receives left/right light sensor values from each robot via OSC,
- computes motor commands using the phototaxis mapping defined in the paper,
- sends motor commands back to each robot via OSC, and
- logs sensor/motor data at 24 fps to a CSV file for later analysis.
"""

from pythonosc import dispatcher, osc_server, udp_client
import threading
import time
import logging
import csv
import os
import signal
import sys
from datetime import datetime
import math

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

NUM_ROBOTS = 10

# Network configuration
SENSOR_BASE_PORT = 60000   # robots send /sensor to these ports (60000 + robot_id)
MOTOR_BASE_PORT = 61000    # host sends /motor to these ports (61000 + robot_id)
ROBOT_IP_TEMPLATE = "192.168.50.{}"  # filled with (50 + robot_id)

# Control law parameters (as in the paper)
MOTOR_MAX_OUTPUT = 200     # M_max in the paper
SIGMOID_ALPHA = 8          # α in the paper

# Logging configuration
LOG_DIR = "robot_logs"
FRAME_INTERVAL = 1.0 / 24.0   # 24 fps logging

# Optional: constant LED color (e.g., to reflect the "robot LED level" condition)
LED_ENABLED = False
LED_COLOR = (0, 0, 0)        # (R, G, B), all 0 = off

# -----------------------------------------------------------------------------
# Global state
# -----------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("johnbot2")

running = True

# Per-robot state (latest sensor/motor values and last update time)
robot_states = {
    i: {"sensors": (0.0, 0.0), "motors": (0, 0), "last_update": 0.0}
    for i in range(NUM_ROBOTS)
}
state_lock = threading.Lock()

# OSC motor clients (host -> robots)
motor_clients = []

# CSV logging
csv_file = None
csv_writer = None
csv_lock = threading.Lock()

# Buffer for latest per-robot data used when writing frames
robot_buffer = {}
last_frame_time = 0.0

# -----------------------------------------------------------------------------
# Control law (phototaxis mapping used in the paper)
# -----------------------------------------------------------------------------

def sharp_sigmoid(x: float, alpha: float) -> float:
    """
    Sigmoid-like contrast function used in the paper:

        σ_α(x) = x^α / (x^α + (1 - x)^α)

    with x in [0, 1] and α > 0.
    """
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0

    xn = x ** alpha
    yn = (1.0 - x) ** alpha
    return xn / (xn + yn)


def map_sensors_to_motors(left_sensor: float, right_sensor: float) -> tuple[int, int]:
    """
    Map raw light sensor readings (SL, SR) to motor commands (ML, MR)
    using the equations defined in the paper:

        r̂_L = (SR + 1) / (SL + SR + 2)
        r̂_R = (SL + 1) / (SL + SR + 2)

        ML = M_max * σ_α(r̂_L)
        MR = M_max * σ_α(r̂_R)

    where σ_α is the sharp sigmoid above.
    """
    SL = max(0.0, left_sensor)
    SR = max(0.0, right_sensor)

    denom = SL + SR + 2.0
    r_hat_L = (SR + 1.0) / denom
    r_hat_R = (SL + 1.0) / denom

    y_L = sharp_sigmoid(r_hat_L, SIGMOID_ALPHA)
    y_R = sharp_sigmoid(r_hat_R, SIGMOID_ALPHA)

    ML = int(round(MOTOR_MAX_OUTPUT * y_L))
    MR = int(round(MOTOR_MAX_OUTPUT * y_R))

    return ML, MR

# -----------------------------------------------------------------------------
# CSV logging
# -----------------------------------------------------------------------------

def setup_csv_logging() -> None:
    """
    Create a CSV file and writer.

    The CSV has:
        - one row per time frame (24 fps),
        - columns: timestamp, and for each robot:
          sensor_left, sensor_right, motor_left, motor_right.
    """
    global csv_file, csv_writer

    os.makedirs(LOG_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(LOG_DIR, f"johnbot2_{timestamp}.csv")

    csv_file = open(filename, "w", newline="")
    csv_writer = csv.writer(csv_file)

    header = ["timestamp"]
    for i in range(NUM_ROBOTS):
        header.extend(
            [
                f"robot{i}_sensor_left",
                f"robot{i}_sensor_right",
                f"robot{i}_motor_left",
                f"robot{i}_motor_right",
            ]
        )
    csv_writer.writerow(header)
    logger.info(f"CSV log file created: {filename}")


def log_data_to_buffer(robot_id: int,
                       left_sensor: float,
                       right_sensor: float,
                       left_motor: int,
                       right_motor: int) -> None:
    """
    Store the latest data for a robot in a buffer.
    The logging thread will periodically flush this buffer to CSV at 24 fps.
    """
    robot_buffer[robot_id] = {
        "timestamp": time.time(),
        "sensor_left": left_sensor,
        "sensor_right": right_sensor,
        "motor_left": left_motor,
        "motor_right": right_motor,
    }


def write_frame_to_csv() -> None:
    """
    Write one frame of data for all robots to the CSV file, at 24 fps.
    If a robot has not sent a recent update, zeros are written for that robot.
    """
    global last_frame_time

    if csv_writer is None or csv_file is None:
        return

    current_time = time.time()

    # Only write a new frame if enough time has passed
    if current_time - last_frame_time < FRAME_INTERVAL:
        return

    frame_time = last_frame_time + FRAME_INTERVAL
    while frame_time <= current_time:
        with csv_lock:
            row = [frame_time]
            for robot_id in range(NUM_ROBOTS):
                data = robot_buffer.get(robot_id)
                if data is None or (current_time - data["timestamp"] > 0.5):
                    # No recent data: write zeros
                    row.extend([0.0, 0.0, 0, 0])
                else:
                    row.extend(
                        [
                            data["sensor_left"],
                            data["sensor_right"],
                            data["motor_left"],
                            data["motor_right"],
                        ]
                    )

            csv_writer.writerow(row)

            # Flush occasionally to avoid data loss
            frame_index = int(frame_time * 24)
            if frame_index % 10 == 0:
                csv_file.flush()

        frame_time += FRAME_INTERVAL

    last_frame_time = frame_time - FRAME_INTERVAL


def csv_logging_thread() -> None:
    """
    Background thread that calls write_frame_to_csv() at 24 fps.
    """
    global last_frame_time
    last_frame_time = time.time()

    while running:
        write_frame_to_csv()
        time.sleep(FRAME_INTERVAL / 2.0)

# -----------------------------------------------------------------------------
# OSC handling
# -----------------------------------------------------------------------------

def osc_sensor_handler(robot_id: int, addr: str, *args) -> None:
    """
    OSC handler for /sensor messages.

    Expected message:
        /sensor <float:left_sensor> <float:right_sensor>
    """
    if len(args) != 2:
        logger.warning(
            f"Robot {robot_id}: invalid /sensor message, expected 2 values, got {len(args)}"
        )
        return

    try:
        left_sensor = float(args[0])
        right_sensor = float(args[1])
    except ValueError:
        logger.warning(
            f"Robot {robot_id}: could not parse sensor values: {args}"
        )
        return

    # Compute motor commands
    left_motor, right_motor = map_sensors_to_motors(left_sensor, right_sensor)

    with state_lock:
        robot_states[robot_id]["sensors"] = (left_sensor, right_sensor)
        robot_states[robot_id]["motors"] = (left_motor, right_motor)
        robot_states[robot_id]["last_update"] = time.time()

    # Log to buffer for CSV
    log_data_to_buffer(
        robot_id,
        left_sensor,
        right_sensor,
        left_motor,
        right_motor,
    )

    # Send commands to the robot
    if 0 <= robot_id < len(motor_clients):
        client = motor_clients[robot_id]
        try:
            client.send_message("/motor", [left_motor, right_motor])
            if LED_ENABLED:
                client.send_message("/LED", list(LED_COLOR))
        except Exception as e:
            logger.error(f"Robot {robot_id}: failed to send OSC command: {e}")
    else:
        logger.error(
            f"Robot {robot_id}: no motor client configured (len={len(motor_clients)})"
        )


def setup_sensor_servers():
    """
    Create one OSC UDP server per robot for incoming /sensor messages.
    """
    servers = []

    for robot_id in range(NUM_ROBOTS):
        port = SENSOR_BASE_PORT + robot_id

        disp = dispatcher.Dispatcher()
        from functools import partial

        handler = partial(osc_sensor_handler, robot_id)
        disp.map("/sensor", handler)

        server = osc_server.ThreadingOSCUDPServer(("0.0.0.0", port), disp)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        servers.append(server)
        logger.info(f"Listening for robot {robot_id} sensors on UDP port {port}")

    return servers


def setup_motor_clients():
    """
    Create one OSC UDP client per robot for outgoing /motor (and optional /LED) messages.
    """
    clients = []
    for robot_id in range(NUM_ROBOTS):
        ip = ROBOT_IP_TEMPLATE.format(50 + robot_id)
        port = MOTOR_BASE_PORT + robot_id
        client = udp_client.SimpleUDPClient(ip, port)
        clients.append(client)
        logger.info(f"Motor client for robot {robot_id}: IP={ip}, port={port}")
    return clients

# -----------------------------------------------------------------------------
# Monitoring and shutdown
# -----------------------------------------------------------------------------

def monitor_robot_states() -> None:
    """
    Periodically print a warning if a robot has not sent data for several seconds.
    """
    while running:
        now = time.time()
        with state_lock:
            for robot_id, state in robot_states.items():
                last_update = state["last_update"]
                if last_update > 0 and now - last_update > 5.0:
                    logger.warning(
                        f"Robot {robot_id} has not sent data for {now - last_update:.1f} seconds"
                    )
        time.sleep(1.0)


def send_stop_signals() -> None:
    """
    Send /motor [0, 0] (and /LED [0,0,0]) to all robots a few times to ensure they stop.
    """
    logger.info("Sending stop signals to all robots...")
    for robot_id, client in enumerate(motor_clients):
        try:
            for _ in range(3):
                client.send_message("/motor", [0, 0])
                if LED_ENABLED:
                    client.send_message("/LED", [0, 0, 0])
                time.sleep(0.01)
            logger.info(f"Stop signal sent to robot {robot_id}")
        except Exception as e:
            logger.error(f"Failed to send stop signal to robot {robot_id}: {e}")


def cleanup() -> None:
    """
    Clean shutdown: stop robots and close the CSV file.
    """
    global running

    if not running:
        return

    running = False
    logger.info("Shutting down controller...")

    try:
        send_stop_signals()
    except Exception:
        pass

    try:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
            logger.info("CSV log file closed")
    except Exception as e:
        logger.error(f"Error while closing CSV file: {e}")

    logger.info("Shutdown complete.")


def signal_handler(sig, frame) -> None:
    """
    Handle Ctrl+C (SIGINT) to perform a clean shutdown.
    """
    logger.info("SIGINT received, shutting down...")
    cleanup()
    sys.exit(0)

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    logger.info("Starting johnbot2 controller")
    logger.info(
        f"Phototaxis mapping: M_max={MOTOR_MAX_OUTPUT}, alpha={SIGMOID_ALPHA}, "
        f"{NUM_ROBOTS} robots"
    )

    # Set up CSV logging
    setup_csv_logging()

    # Set up OSC servers and clients
    servers = setup_sensor_servers()
    motor_clients = setup_motor_clients()

    # Start background threads
    monitor_thread = threading.Thread(target=monitor_robot_states, daemon=True)
    monitor_thread.start()

    csv_thread = threading.Thread(target=csv_logging_thread, daemon=True)
    csv_thread.start()

    logger.info("Controller is running. Press Ctrl+C to stop.")

    try:
        while running:
            time.sleep(0.5)
    finally:
        cleanup()