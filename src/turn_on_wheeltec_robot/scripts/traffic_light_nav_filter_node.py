#!/usr/bin/env python3
import argparse
from dataclasses import dataclass


GO_STATES = {"GO", "GO_HOLD"}
SLOW_STATES = {"SLOW"}


@dataclass
class GateConfig:
    allow_reverse: bool = False
    max_linear_speed: float = 0.12
    max_angular_speed: float = 0.80
    slow_linear_speed: float = 0.05
    slow_scale: float = 0.40
    nav_timeout_sec: float = 0.50
    state_timeout_sec: float = 1.00


@dataclass
class Command:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    def scaled(self, factor: float) -> "Command":
        return Command(
            linear_x=self.linear_x * factor,
            linear_y=self.linear_y * factor,
            linear_z=self.linear_z * factor,
            angular_x=self.angular_x * factor,
            angular_y=self.angular_y * factor,
            angular_z=self.angular_z * factor,
        )


def clamp_command(command: Command, config: GateConfig) -> Command:
    if command.linear_x < 0.0 and not config.allow_reverse:
        command = Command(
            linear_x=0.0,
            linear_y=command.linear_y,
            linear_z=command.linear_z,
            angular_x=command.angular_x,
            angular_y=command.angular_y,
            angular_z=0.0,
        )
    factor = 1.0
    if abs(command.linear_x) > config.max_linear_speed > 0.0:
        factor = min(factor, config.max_linear_speed / abs(command.linear_x))
    if abs(command.angular_z) > config.max_angular_speed > 0.0:
        factor = min(factor, config.max_angular_speed / abs(command.angular_z))
    return command.scaled(factor)


def filter_command(
    command: Command,
    state: str,
    nav_age_sec: float,
    state_age_sec: float,
    config: GateConfig,
) -> tuple[Command, str]:
    if nav_age_sec > config.nav_timeout_sec:
        return Command(), "NAV_TIMEOUT"
    if state_age_sec > config.state_timeout_sec:
        return Command(), "STATE_TIMEOUT"

    state = state.strip().upper()
    if state in GO_STATES:
        return clamp_command(command, config), state
    if state in SLOW_STATES:
        limited = clamp_command(command, config)
        factor = max(0.0, min(1.0, config.slow_scale))
        if abs(limited.linear_x) > config.slow_linear_speed > 0.0:
            factor = min(factor, config.slow_linear_speed / abs(limited.linear_x))
        # Scale steering with speed so Ackermann path curvature is preserved.
        return limited.scaled(factor), state
    return Command(), state or "EMPTY_STATE"


def run_self_test():
    config = GateConfig()
    nav = Command(linear_x=0.50, angular_z=1.00)

    go, reason = filter_command(nav, "GO", 0.0, 0.0, config)
    assert reason == "GO"
    assert abs(go.linear_x - 0.12) < 1e-9
    assert abs(go.angular_z - 0.24) < 1e-9

    slow, reason = filter_command(nav, "SLOW", 0.0, 0.0, config)
    assert reason == "SLOW"
    assert abs(slow.linear_x - 0.048) < 1e-9
    assert abs(slow.angular_z - 0.096) < 1e-9

    for state in ("STOP", "UNKNOWN", ""):
        stopped, _reason = filter_command(nav, state, 0.0, 0.0, config)
        assert stopped == Command()

    stale_nav, reason = filter_command(nav, "GO", 0.51, 0.0, config)
    assert stale_nav == Command() and reason == "NAV_TIMEOUT"
    stale_state, reason = filter_command(nav, "GO", 0.0, 1.01, config)
    assert stale_state == Command() and reason == "STATE_TIMEOUT"
    reverse, reason = filter_command(
        Command(linear_x=-0.05, angular_z=0.30), "GO", 0.0, 0.0, config
    )
    assert reverse == Command() and reason == "GO"
    print("traffic_light_nav_filter self-test OK")


def run_ros_node():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import String

    class TrafficLightNavFilterNode(Node):
        def __init__(self):
            super().__init__("traffic_light_nav_filter")
            self.declare_parameter("nav_cmd_topic", "/nav2/cmd_vel")
            self.declare_parameter("state_topic", "/traffic_light/state")
            self.declare_parameter("output_topic", "/cmd_vel")
            self.declare_parameter("publish_hz", 30.0)
            self.declare_parameter("allow_reverse", False)
            self.declare_parameter("max_linear_speed", 0.12)
            self.declare_parameter("max_angular_speed", 0.80)
            self.declare_parameter("slow_linear_speed", 0.05)
            self.declare_parameter("slow_scale", 0.40)
            self.declare_parameter("nav_timeout_sec", 0.50)
            self.declare_parameter("state_timeout_sec", 1.00)

            nav_cmd_topic = self.get_parameter("nav_cmd_topic").value
            state_topic = self.get_parameter("state_topic").value
            output_topic = self.get_parameter("output_topic").value
            publish_hz = float(self.get_parameter("publish_hz").value)
            self.config = GateConfig(
                allow_reverse=bool(self.get_parameter("allow_reverse").value),
                max_linear_speed=float(self.get_parameter("max_linear_speed").value),
                max_angular_speed=float(self.get_parameter("max_angular_speed").value),
                slow_linear_speed=float(
                    self.get_parameter("slow_linear_speed").value
                ),
                slow_scale=float(self.get_parameter("slow_scale").value),
                nav_timeout_sec=float(self.get_parameter("nav_timeout_sec").value),
                state_timeout_sec=float(
                    self.get_parameter("state_timeout_sec").value
                ),
            )

            self.last_nav = Command()
            self.last_state = "STOP"
            self.last_nav_time = None
            self.last_state_time = None
            self.last_reason = None

            self.publisher = self.create_publisher(Twist, output_topic, 10)
            self.create_subscription(Twist, nav_cmd_topic, self.on_nav_cmd, 10)
            self.create_subscription(String, state_topic, self.on_state, 10)
            self.create_timer(1.0 / max(1.0, publish_hz), self.on_timer)

            self.get_logger().info(
                f"navigation velocity filter: {nav_cmd_topic} + {state_topic} "
                f"-> {output_topic}; max={self.config.max_linear_speed:.3f}m/s, "
                f"slow={self.config.slow_linear_speed:.3f}m/s, "
                f"allow_reverse={self.config.allow_reverse}"
            )

        def on_nav_cmd(self, msg: Twist):
            self.last_nav = Command(
                linear_x=msg.linear.x,
                linear_y=msg.linear.y,
                linear_z=msg.linear.z,
                angular_x=msg.angular.x,
                angular_y=msg.angular.y,
                angular_z=msg.angular.z,
            )
            self.last_nav_time = self.get_clock().now()

        def on_state(self, msg: String):
            self.last_state = msg.data.strip().upper()
            self.last_state_time = self.get_clock().now()

        def age_sec(self, stamp) -> float:
            if stamp is None:
                return float("inf")
            return (self.get_clock().now() - stamp).nanoseconds / 1e9

        def publish_zero(self):
            self.publisher.publish(Twist())

        def on_timer(self):
            command, reason = filter_command(
                self.last_nav,
                self.last_state,
                self.age_sec(self.last_nav_time),
                self.age_sec(self.last_state_time),
                self.config,
            )

            msg = Twist()
            msg.linear.x = command.linear_x
            msg.linear.y = command.linear_y
            msg.linear.z = command.linear_z
            msg.angular.x = command.angular_x
            msg.angular.y = command.angular_y
            msg.angular.z = command.angular_z
            self.publisher.publish(msg)

            if reason != self.last_reason:
                self.get_logger().info(f"velocity gate state: {reason}")
                self.last_reason = reason

    rclpy.init()
    node = TrafficLightNavFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_zero()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args, _unknown = parser.parse_known_args()
    if args.self_test:
        run_self_test()
    else:
        run_ros_node()


if __name__ == "__main__":
    main()
