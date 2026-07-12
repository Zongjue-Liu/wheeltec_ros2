#!/usr/bin/env python3
import argparse
from dataclasses import dataclass


GO_STATES = {"GO", "GO_HOLD"}
SLOW_STATES = {"SLOW"}


@dataclass
class VelocityConfig:
    cruise_speed: float = 0.12
    slow_speed: float = 0.05
    accel_limit: float = 0.08
    decel_limit: float = 0.30
    state_timeout_sec: float = 1.5


class VelocityRamp:
    def __init__(self, config: VelocityConfig):
        self.config = config
        self.current_speed = 0.0
        self.last_state = "STOP"
        self.last_state_age = float("inf")

    def set_state(self, state: str):
        self.last_state = state.strip().upper()
        self.last_state_age = 0.0

    def step(self, dt: float) -> tuple[float, str, float]:
        self.last_state_age += max(0.0, dt)
        active_state = self.last_state
        if self.last_state_age > self.config.state_timeout_sec:
            active_state = "TIMEOUT"

        if active_state in GO_STATES:
            target = self.config.cruise_speed
        elif active_state in SLOW_STATES:
            target = self.config.slow_speed
        else:
            target = 0.0
        limit = (
            self.config.accel_limit
            if target > self.current_speed
            else self.config.decel_limit
        )
        max_delta = limit * max(0.0, dt)

        if self.current_speed < target:
            self.current_speed = min(target, self.current_speed + max_delta)
        elif self.current_speed > target:
            self.current_speed = max(target, self.current_speed - max_delta)

        if abs(self.current_speed) < 1e-4:
            self.current_speed = 0.0
        return self.current_speed, active_state, target


def run_self_test():
    config = VelocityConfig()
    ramp = VelocityRamp(config)
    dt = 0.05
    timeline = []

    for tick in range(120):
        t = tick * dt
        if t < 0.50:
            ramp.set_state("STOP")
        elif 0.50 <= t < 2.50:
            ramp.set_state("GO")
        elif 2.50 <= t < 4.00:
            ramp.set_state("SLOW")
        elif 4.00 <= t < 5.00:
            ramp.set_state("STOP")
        else:
            ramp.set_state("GO_HOLD")
        speed, state, target = ramp.step(dt)
        if tick % 10 == 0 or tick in (10, 50, 80):
            timeline.append((t, state, target, speed))

    print("t(s)  state     target  speed")
    for t, state, target, speed in timeline:
        print(f"{t:4.2f}  {state:<8}  {target:5.2f}  {speed:5.3f}")

    assert timeline[0][3] == 0.0
    assert any(speed > 0.0 for _, _, _, speed in timeline)
    assert all(speed <= config.cruise_speed + 1e-9 for _, _, _, speed in timeline)
    assert any(state == "SLOW" and speed <= config.slow_speed + 0.03 for _, state, _, speed in timeline)
    print("self-test OK")


def run_ros_node():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import String

    class TrafficLightVelocityNode(Node):
        def __init__(self):
            super().__init__("traffic_light_velocity_node")
            self.declare_parameter("state_topic", "/traffic_light/state")
            self.declare_parameter("output_topic", "/traffic_light/cmd_vel_test")
            self.declare_parameter("publish_hz", 20.0)
            self.declare_parameter("cruise_speed", 0.12)
            self.declare_parameter("slow_speed", 0.05)
            self.declare_parameter("accel_limit", 0.08)
            self.declare_parameter("decel_limit", 0.30)
            self.declare_parameter("state_timeout_sec", 1.5)

            state_topic = (
                self.get_parameter("state_topic").get_parameter_value().string_value
            )
            output_topic = (
                self.get_parameter("output_topic").get_parameter_value().string_value
            )
            publish_hz = (
                self.get_parameter("publish_hz").get_parameter_value().double_value
            )
            config = VelocityConfig(
                cruise_speed=self.get_parameter("cruise_speed")
                .get_parameter_value()
                .double_value,
                slow_speed=self.get_parameter("slow_speed")
                .get_parameter_value()
                .double_value,
                accel_limit=self.get_parameter("accel_limit")
                .get_parameter_value()
                .double_value,
                decel_limit=self.get_parameter("decel_limit")
                .get_parameter_value()
                .double_value,
                state_timeout_sec=self.get_parameter("state_timeout_sec")
                .get_parameter_value()
                .double_value,
            )

            self.ramp = VelocityRamp(config)
            self.last_time = self.get_clock().now()
            self.pub = self.create_publisher(Twist, output_topic, 10)
            self.sub = self.create_subscription(String, state_topic, self.on_state, 10)
            self.timer = self.create_timer(1.0 / max(1.0, publish_hz), self.on_timer)

            self.get_logger().info(
                "traffic light velocity control: "
                f"{state_topic} -> {output_topic}, cruise={config.cruise_speed:.3f}, "
                f"slow={config.slow_speed:.3f}, "
                f"accel={config.accel_limit:.3f}, decel={config.decel_limit:.3f}, "
                f"timeout={config.state_timeout_sec:.2f}s"
            )

        def on_state(self, msg: String):
            self.ramp.set_state(msg.data)

        def on_timer(self):
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now
            speed, active_state, _target = self.ramp.step(dt)

            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = 0.0
            self.pub.publish(cmd)

            self.get_logger().debug(
                f"state={active_state}, speed={speed:.3f} m/s"
            )

    rclpy.init()
    node = TrafficLightVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            stop = Twist()
            try:
                node.pub.publish(stop)
            except Exception:
                pass
        try:
            node.destroy_node()
        except (KeyboardInterrupt, Exception):
            pass
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run the velocity ramp self-test without ROS 2.",
    )
    args, _unknown = parser.parse_known_args()

    if args.self_test:
        run_self_test()
    else:
        run_ros_node()


if __name__ == "__main__":
    main()
