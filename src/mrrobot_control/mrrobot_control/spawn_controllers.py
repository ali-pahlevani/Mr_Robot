"""Bring the ros2_control controllers up, in order, and stay up to it.

    ros2 run mrrobot_control spawn_controllers joint_state_broadcaster base_controller ...

One process instead of a spawner per controller. controller_manager's
spawner asks the controller manager three things (loaded? load;
configure; activate) and gives up for good when the answer to `load` is
lost -- which it is, one start in ten: the first service calls after
wait_for_service race the DDS matching, the request goes through, the
reply does not, and the retry is told the controller is "already loaded"
(measured). Here every step is asked from the controller's CURRENT state,
so a lost reply just means asking again, and the controllers come up one
after the other rather than eight at once.
"""

import argparse
import sys
import time

import rclpy
import rclpy.utilities
from controller_manager_msgs.srv import (ConfigureController, ListControllers, LoadController,
                                         SwitchController)
from rclpy.node import Node


class Spawner(Node):
    def __init__(self, manager):
        super().__init__("mrRobot_spawner")
        self._list = self.create_client(ListControllers, f"{manager}/list_controllers")
        self._load = self.create_client(LoadController, f"{manager}/load_controller")
        self._configure = self.create_client(ConfigureController,
                                             f"{manager}/configure_controller")
        self._switch = self.create_client(SwitchController, f"{manager}/switch_controller")

    def call(self, client, request, timeout=5.0):
        """The response, or None when it did not come in time."""
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def states(self):
        res = self.call(self._list, ListControllers.Request())
        return None if res is None else {c.name: c.state for c in res.controller}

    def bring_up(self, name, deadline):
        """True once `name` is active: whatever state it is in, do the next thing."""
        while time.time() < deadline:
            states = self.states()
            if states is None:
                continue
            state = states.get(name)
            if state == "active":
                return True
            if state is None:
                req = LoadController.Request()
                req.name = name
                self.call(self._load, req)      # a lost reply shows up in the next listing
            elif state == "unconfigured":
                req = ConfigureController.Request()
                req.name = name
                self.call(self._configure, req)
            elif state == "inactive":
                req = SwitchController.Request()
                req.activate_controllers = [name]
                req.strictness = SwitchController.Request.STRICT
                req.activate_asap = True
                self.call(self._switch, req)
            else:
                self.get_logger().warn(f"{name} is {state}; waiting")
                time.sleep(1.0)
        return False


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("names", nargs="+", help="controllers, in the order to bring them up")
    parser.add_argument("-c", "--controller-manager", default="/controller_manager")
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="seconds for all of them, from the start")
    opts = parser.parse_args(rclpy.utilities.remove_ros_args(args or sys.argv)[1:])
    node = Spawner(opts.controller_manager)
    ok = True
    deadline = time.time() + opts.timeout
    try:
        for name in opts.names:
            if node.bring_up(name, deadline):
                node.get_logger().info(f"{name} active")
            else:
                node.get_logger().error(f"{name} did not come up")
                ok = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
