"""
Gripper Control for RB-Y1

Run this on the UPC or a machine with physical access to /dev/rby1_gripper.
Requires the robot to be powered on (48v) and tool flange voltage set to 12V.

Usage:
    python gripper_control.py --address 192.168.12.1:50051

The gripper uses 2 Dynamixel motors per hand (IDs 0 and 1).
Homing detects the full open/close range by stalling in each direction.
After homing, you can command positions as a normalized 0.0 (open) to 1.0 (closed) value.
"""

import rby1_sdk as rby
import numpy as np
import time
import threading
import argparse
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


class Gripper:
    MOTOR_IDS = [0, 1]
    HOMING_CURRENT = 0.5      # Amps applied during homing
    HOMING_TIMEOUT = 30       # Max seconds per direction
    HOMING_STALL_COUNT = 30   # Consecutive unchanged readings to declare stall
    CONTROL_TORQUE_LIMIT = 5  # Amps for position control mode
    LOOP_PERIOD = 0.1         # Seconds between position commands

    def __init__(self):
        self.bus = rby.DynamixelBus(rby.upc.GripperDeviceName)
        self.min_q = np.array([np.inf, np.inf])
        self.max_q = np.array([-np.inf, -np.inf])
        self.target_q = None
        self._running = False
        self._thread = None
        self._homed = False

    def connect(self):
        """Open the serial port to the gripper motors."""
        if not self.bus.open_port():
            logging.error("Failed to open gripper port")
            return False
        self.bus.set_baud_rate(2_000_000)
        self.bus.set_torque_constant([1, 1])
        return True

    def initialize(self):
        """Ping each motor and enable torque."""
        for dev_id in self.MOTOR_IDS:
            if not self.bus.ping(dev_id):
                logging.error(f"Dynamixel motor {dev_id} not responding")
                return False
            logging.info(f"Motor {dev_id} active")
        self.bus.group_sync_write_torque_enable(
            [(dev_id, 1) for dev_id in self.MOTOR_IDS]
        )
        logging.info("Gripper torque enabled")
        return True

    def _set_operating_mode(self, mode):
        """Switch operating mode (requires torque disable/enable cycle)."""
        self.bus.group_sync_write_torque_enable(
            [(dev_id, 0) for dev_id in self.MOTOR_IDS]
        )
        self.bus.group_sync_write_operating_mode(
            [(dev_id, mode) for dev_id in self.MOTOR_IDS]
        )
        self.bus.group_sync_write_torque_enable(
            [(dev_id, 1) for dev_id in self.MOTOR_IDS]
        )

    def homing(self):
        """
        Detect gripper range by driving to each limit.

        Applies current in one direction until stall, then the other.
        Records min/max encoder values to define the full range.
        """
        logging.info("Starting gripper homing...")
        self._set_operating_mode(rby.DynamixelBus.CurrentControlMode)

        q = np.array([0.0, 0.0])
        prev_q = np.array([0.0, 0.0])
        stall_count = 0

        for direction in range(2):
            sign = 1.0 if direction == 0 else -1.0
            label = "closing" if direction == 0 else "opening"
            logging.info(f"Homing: {label}...")
            stall_count = 0

            for _ in range(int(self.HOMING_TIMEOUT / self.LOOP_PERIOD)):
                self.bus.group_sync_write_send_torque(
                    [(dev_id, self.HOMING_CURRENT * sign) for dev_id in self.MOTOR_IDS]
                )
                rv = self.bus.group_fast_sync_read_encoder(self.MOTOR_IDS)
                if rv is not None:
                    for dev_id, enc in rv:
                        q[dev_id] = enc

                self.min_q = np.minimum(self.min_q, q)
                self.max_q = np.maximum(self.max_q, q)

                if np.array_equal(prev_q, q):
                    stall_count += 1
                else:
                    stall_count = 0
                prev_q = q.copy()

                if stall_count >= self.HOMING_STALL_COUNT:
                    break
                time.sleep(self.LOOP_PERIOD)

        # Stop current
        self.bus.group_sync_write_send_torque(
            [(dev_id, 0.0) for dev_id in self.MOTOR_IDS]
        )

        self._homed = True
        logging.info(f"Homing complete. Range: min={self.min_q}, max={self.max_q}")
        return True

    def set_position(self, normalized):
        """
        Set gripper position.

        Args:
            normalized: float 0.0 (fully open) to 1.0 (fully closed),
                        or np.array of shape (2,) for per-finger control.
        """
        if not self._homed:
            logging.error("Gripper not homed. Call homing() first.")
            return

        if isinstance(normalized, (int, float)):
            normalized = np.array([normalized, normalized])
        normalized = np.clip(normalized, 0.0, 1.0)

        # 0.0 = open (max_q), 1.0 = closed (min_q)
        self.target_q = (1 - normalized) * (self.max_q - self.min_q) + self.min_q

    def open(self):
        """Fully open the gripper."""
        self.set_position(0.0)

    def close(self):
        """Fully close the gripper."""
        self.set_position(1.0)

    def start(self):
        """Start the background control loop."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logging.info("Gripper control loop started")

    def stop(self):
        """Stop the background control loop."""
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        logging.info("Gripper control loop stopped")

    def _loop(self):
        """Background loop: sends position commands at fixed rate."""
        self._set_operating_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self.bus.group_sync_write_send_torque(
            [(dev_id, self.CONTROL_TORQUE_LIMIT) for dev_id in self.MOTOR_IDS]
        )
        while self._running:
            if self.target_q is not None:
                self.bus.group_sync_write_send_position(
                    [(dev_id, q) for dev_id, q in enumerate(self.target_q.tolist())]
                )
            time.sleep(self.LOOP_PERIOD)

    def read_position(self):
        """Read current encoder positions. Returns (2,) array or None."""
        rv = self.bus.group_fast_sync_read_encoder(self.MOTOR_IDS)
        if rv is None:
            return None
        q = np.zeros(2)
        for dev_id, enc in rv:
            q[dev_id] = enc
        return q

    def read_normalized_position(self):
        """Read current position as normalized 0.0 (open) to 1.0 (closed)."""
        if not self._homed:
            return None
        q = self.read_position()
        if q is None:
            return None
        range_q = self.max_q - self.min_q
        range_q[range_q == 0] = 1.0  # avoid division by zero
        return 1.0 - (q - self.min_q) / range_q


def setup_robot(address):
    """Connect to robot and power on tool flanges."""
    robot = rby.create_robot_a(address)
    if not robot.connect():
        logging.error(f"Failed to connect to {address}")
        return None

    if not robot.is_power_on("48v"):
        logging.info("Powering on 48v...")
        robot.power_on("48v")
        time.sleep(1)

    for arm in ["right", "left"]:
        if not robot.set_tool_flange_output_voltage(arm, 12):
            logging.error(f"Failed to set tool flange voltage for {arm}")
            return None
        logging.info(f"Tool flange {arm}: 12V")

    return robot


def interactive_demo(gripper):
    """Simple interactive loop for testing gripper."""
    print("\n--- Gripper Interactive Control ---")
    print("Commands: open, close, 0.0-1.0 (position), read, quit")

    while True:
        cmd = input("> ").strip().lower()
        if cmd == "quit":
            break
        elif cmd == "open":
            gripper.open()
            print("Opening...")
        elif cmd == "close":
            gripper.close()
            print("Closing...")
        elif cmd == "read":
            pos = gripper.read_normalized_position()
            print(f"Position: {pos}")
        else:
            try:
                val = float(cmd)
                gripper.set_position(val)
                print(f"Moving to {val:.2f}")
            except ValueError:
                print("Unknown command")


def main():
    parser = argparse.ArgumentParser(description="RB-Y1 Gripper Control")
    parser.add_argument("--address", type=str, default="192.168.12.1:50051",
                        help="Robot address (default: 192.168.12.1:50051)")
    parser.add_argument("--no-interactive", action="store_true",
                        help="Skip interactive mode, just home and open/close once")
    args = parser.parse_args()

    # 1. Connect to robot and power tool flanges
    robot = setup_robot(args.address)
    if robot is None:
        return

    # 2. Initialize gripper
    gripper = Gripper()
    if not gripper.connect():
        return
    if not gripper.initialize():
        return

    # 3. Home the gripper (find range)
    gripper.homing()

    # 4. Start control loop
    gripper.start()

    if args.no_interactive:
        # Demo: open -> close -> open
        print("Demo: open -> close -> open")
        gripper.open()
        time.sleep(2)
        gripper.close()
        time.sleep(2)
        gripper.open()
        time.sleep(2)
    else:
        interactive_demo(gripper)

    # Cleanup
    gripper.open()
    time.sleep(1)
    gripper.stop()
    logging.info("Done.")


if __name__ == "__main__":
    main()
