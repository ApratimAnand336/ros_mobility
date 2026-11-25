"""Shared ROS helpers: canonical topic names, QoS, geometry from parameters."""
import math

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from mobility_core import Geometry

# Canonical topics. Every domain uses exactly these; remap in launch, never in code.
CMD_VEL = "/cmd_vel"
STEER_MODE = "/mobility/steer_mode"
STEER_AIM = "/mobility/steer_aim"
SETPOINTS = "/mobility/wheel_setpoints"
JOINT_STATES = "/mobility/joint_states"
EFFORT_CMD = "/mobility/effort_cmd"
ESTOP = "/mobility/estop"
ARM_SRV = "/mobility/arm"
WHEEL_ODOM = "/mobility/wheel_odom"
DIAGNOSTICS = "/mobility/diagnostics"
# Remote execution of the 8 loops (Unity runs them inside its physics step):
CONTROLLER_CONFIG = "/mobility/controller_config"   # String, JSON: resolved gains
CONTROLLER_ENABLE = "/mobility/controller_enable"   # Bool: state == running

# Publishers of state-like topics (mode, e-stop) are latched so late joiners get the
# last value. Subscribers use plain RELIABLE/VOLATILE, which matches *any* publisher
# (a TRANSIENT_LOCAL subscriber would silently ignore e.g. `ros2 topic pub` -- not
# acceptable for an e-stop).
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
STATE_SUB = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)


def param(node, name, default):
    """Declare-if-needed and read. Works with automatically_declare_parameters_from_overrides."""
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def declare_geometry(node):
    """Read the shared geometry parameters (config/geometry.yaml) and build Geometry."""
    return Geometry.from_lists(
        param(node, "wheel_radius", 0.14985),
        list(param(node, "wheel_names", ["fl", "fr", "bl", "br"])),
        list(param(node, "wheel_x", [0.372365, 0.372365, -0.372365, -0.371204])),
        list(param(node, "wheel_y", [0.218, -0.218, 0.218, -0.218])),
        param(node, "steer_limit", 1.5708))


declare_geometry_safe = declare_geometry


def joint_state_to_dicts(msg: JointState):
    """-> (positions, velocities, efforts) dicts, skipping missing / NaN entries."""
    pos, vel, eff = {}, {}, {}
    for i, n in enumerate(msg.name):
        for arr, out in ((msg.position, pos), (msg.velocity, vel), (msg.effort, eff)):
            if i < len(arr) and math.isfinite(arr[i]):
                out[n] = arr[i]
    return pos, vel, eff


def make_joint_state(stamp, names, position=None, velocity=None, effort=None):
    msg = JointState()
    msg.header.stamp = stamp
    msg.name = list(names)
    n = len(names)
    msg.position = [float(x) for x in position] if position is not None else []
    msg.velocity = [float(x) for x in velocity] if velocity is not None else []
    msg.effort = [float(x) for x in effort] if effort is not None else []
    for arr in (msg.position, msg.velocity, msg.effort):
        assert len(arr) in (0, n)
    return msg


def run(node_cls, *args, **kwargs):
    """Standard main(): spin until Ctrl-C, call node.on_shutdown() if it exists."""
    import signal

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    # Keep the ROS context alive through Ctrl-C so on_shutdown() can still publish
    # zero commands; we shut the context down ourselves afterwards.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = node_cls(*args, **kwargs)
    # Without rclpy's handler, Ctrl-C only raises once spin() returns to Python. A node
    # that only wakes on messages (wheel_odometry) would hang at shutdown when its
    # publisher stops first, so wake every node periodically.
    node.create_timer(0.1, lambda: None)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise          # a real error; only swallow the race during shutdown
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # a 2nd Ctrl-C must not abort this
        if hasattr(node, "on_shutdown"):
            try:
                node.on_shutdown()
            except Exception as e:  # noqa: BLE001 - best effort on the way out
                print(f"[{node.get_name()}] shutdown hook failed: {e}")
        node.destroy_node()
        rclpy.try_shutdown()
