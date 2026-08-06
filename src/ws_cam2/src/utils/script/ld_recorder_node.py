#!/usr/bin/env python3

import csv
import signal
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from fo_msgs.msg import Cam2LD
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class LaneResultRecorder(Node):
    def __init__(self) -> None:
        super().__init__("ld_recorder_node")

        self.declare_parameter("input_topic", "/camera/lane_result")
        self.declare_parameter("output_dir", "~/lane_result_records")
        self.declare_parameter("basename", "")
        self.declare_parameter("image_width", 1600)
        self.declare_parameter("image_height", 900)
        self.declare_parameter("lane_x_min", 0.0)
        self.declare_parameter("lane_x_max", 30.0)
        self.declare_parameter("lane_x_samples", 100)
        self.declare_parameter("quality_threshold", 2)
        self.declare_parameter("max_plot_samples", 20000)
        self.declare_parameter("autosave_interval_sec", 5.0)

        self.input_topic = self._get_string_param("input_topic")
        output_dir = Path(self._get_string_param("output_dir")).expanduser()
        basename = self._get_string_param("basename")
        self.image_width = self._get_int_param("image_width")
        self.image_height = self._get_int_param("image_height")
        self.lane_x_min = self._get_float_param("lane_x_min")
        self.lane_x_max = self._get_float_param("lane_x_max")
        self.lane_x_samples = self._get_int_param("lane_x_samples")
        self.quality_threshold = self._get_int_param("quality_threshold")
        self.max_plot_samples = self._get_int_param("max_plot_samples")
        self.autosave_interval_sec = self._get_float_param("autosave_interval_sec")

        self._validate_params()

        output_dir.mkdir(parents=True, exist_ok=True)
        if not basename:
            basename = "lane_result_" + datetime.now().strftime("%Y%m%d_%H%M%S")

        self.csv_path = output_dir / f"{basename}.csv"
        self.image_path = output_dir / f"{basename}.png"
        self.csv_file = self.csv_path.open("w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ["timestamp", "l_a", "l_b", "l_c", "l_d", "r_a", "r_b", "r_c", "r_d"]
        )
        self.csv_file.flush()

        self.first_timestamp = None
        self.last_left_coeffs = None
        self.last_right_coeffs = None
        self.warned_zero_timestamp = False
        self.finalized = False
        self.sample_count = 0
        self.last_autosaved_sample_count = 0
        self.history = {
            "t": [],
            "l_a": [],
            "l_b": [],
            "l_c": [],
            "l_d": [],
            "r_a": [],
            "r_b": [],
            "r_c": [],
            "r_d": [],
        }

        self.subscription = self.create_subscription(
            Cam2LD, self.input_topic, self._lane_result_callback, 10
        )
        self.autosave_timer = None
        if self.autosave_interval_sec > 0.0:
            self.autosave_timer = self.create_timer(
                self.autosave_interval_sec, self._autosave_image
            )

        self.get_logger().info(
            "Recording lane results from '%s' to CSV '%s' and final image '%s'"
            % (self.input_topic, self.csv_path, self.image_path)
        )

    def _get_string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_int_param(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _get_float_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _validate_params(self) -> None:
        if not self.input_topic:
            self.get_logger().fatal("Parameter 'input_topic' is empty.")
            raise RuntimeError("input_topic is required")
        if self.image_width <= 0 or self.image_height <= 0:
            self.get_logger().fatal(
                f"Invalid image size: {self.image_width}x{self.image_height}"
            )
            raise RuntimeError("image_width and image_height must be greater than zero")
        if self.lane_x_samples < 2:
            self.get_logger().fatal(
                f"Invalid lane_x_samples: {self.lane_x_samples}"
            )
            raise RuntimeError("lane_x_samples must be at least 2")
        if self.lane_x_max <= self.lane_x_min:
            self.get_logger().fatal(
                f"Invalid lane x range: min={self.lane_x_min}, max={self.lane_x_max}"
            )
            raise RuntimeError("lane_x_max must be greater than lane_x_min")
        if self.max_plot_samples < 2:
            self.get_logger().fatal(
                f"Invalid max_plot_samples: {self.max_plot_samples}"
            )
            raise RuntimeError("max_plot_samples must be at least 2")
        if self.autosave_interval_sec < 0.0:
            self.get_logger().fatal(
                f"Invalid autosave_interval_sec: {self.autosave_interval_sec}"
            )
            raise RuntimeError("autosave_interval_sec must be zero or greater")

    def _lane_result_callback(self, msg: Cam2LD) -> None:
        timestamp = self._stamp_to_seconds(msg.header_ld.stamp)
        if timestamp == 0.0 and not self.warned_zero_timestamp:
            self.get_logger().warn("Received header_ld.stamp == 0.0")
            self.warned_zero_timestamp = True

        if self.first_timestamp is None:
            self.first_timestamp = timestamp
        relative_t = timestamp - self.first_timestamp

        left_valid = msg.la.lane_mark_quality >= self.quality_threshold
        right_valid = msg.ra.lane_mark_quality >= self.quality_threshold
        left_coeffs = (
            self._extract_left_coeffs(msg) if left_valid else self._nan_coeffs()
        )
        right_coeffs = (
            self._extract_right_coeffs(msg) if right_valid else self._nan_coeffs()
        )

        if left_valid:
            self.last_left_coeffs = left_coeffs
        if right_valid:
            self.last_right_coeffs = right_coeffs

        row = [timestamp, *left_coeffs, *right_coeffs]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        self.history["t"].append(relative_t)
        for key, value in zip(
            ["l_a", "l_b", "l_c", "l_d", "r_a", "r_b", "r_c", "r_d"],
            [*left_coeffs, *right_coeffs],
        ):
            self.history[key].append(value)

        self.sample_count += 1
        if self.sample_count == 1 or self.sample_count % 100 == 0:
            self.get_logger().info(
                "Recorded %d lane samples (latest timestamp=%.9f, left_valid=%s, right_valid=%s)"
                % (self.sample_count, timestamp, left_valid, right_valid)
            )

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _extract_left_coeffs(msg: Cam2LD) -> tuple[float, float, float, float]:
        return (
            float(msg.la.lane_mark_model_a),
            float(msg.lb.lane_mark_heading_angle),
            float(msg.la.lane_mark_position),
            float(msg.lb.lane_mark_model_da),
        )

    @staticmethod
    def _extract_right_coeffs(msg: Cam2LD) -> tuple[float, float, float, float]:
        return (
            float(msg.ra.lane_mark_model_a),
            float(msg.rb.lane_mark_heading_angle),
            float(msg.ra.lane_mark_position),
            float(msg.rb.lane_mark_model_da),
        )

    @staticmethod
    def _nan_coeffs() -> tuple[float, float, float, float]:
        nan = float("nan")
        return (nan, nan, nan, nan)

    def _finalize_outputs(self) -> None:
        if self.finalized:
            return
        self.finalized = True

        if self.autosave_timer is not None:
            self.autosave_timer.cancel()

        try:
            self._save_final_image()
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().error(f"Failed to save final image: {exc}")

        if hasattr(self, "csv_file") and self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None

    def _autosave_image(self) -> None:
        if self.history["t"] and self.sample_count != self.last_autosaved_sample_count:
            try:
                self._save_final_image(log_message=False)
                self.last_autosaved_sample_count = self.sample_count
            except Exception as exc:
                self.get_logger().error(f"Failed to autosave image: {exc}")

    def _save_final_image(self, log_message: bool = True) -> None:
        dpi = 100
        figsize = (self.image_width / dpi, self.image_height / dpi)
        fig = plt.figure(figsize=figsize, dpi=dpi, constrained_layout=True)

        if not self.history["t"]:
            ax = fig.add_subplot(1, 1, 1)
            ax.text(0.5, 0.5, "no data", ha="center", va="center", fontsize=18)
            ax.set_axis_off()
            fig.savefig(self.image_path)
            plt.close(fig)
            if log_message:
                self._log_info_if_ok(f"Saved no-data image: {self.image_path}")
            return

        grid = fig.add_gridspec(4, 2, width_ratios=[1.0, 1.45])
        lane_ax = fig.add_subplot(grid[:, 0])
        series_axes = [fig.add_subplot(grid[i, 1]) for i in range(4)]

        self._plot_lane_polynomials(lane_ax)
        self._plot_timeseries(series_axes)

        fig.savefig(self.image_path)
        plt.close(fig)
        if log_message:
            self._log_info_if_ok(
                "Saved final lane result image: %s (%d samples)"
                % (self.image_path, self.sample_count)
            )

    def _log_info_if_ok(self, message: str) -> None:
        if rclpy.ok():
            self.get_logger().info(message)

    def _plot_lane_polynomials(self, ax) -> None:
        x_values = np.linspace(self.lane_x_min, self.lane_x_max, self.lane_x_samples)

        if self.last_left_coeffs is not None:
            y_values = self._evaluate_lane(self.last_left_coeffs, x_values)
            ax.plot(y_values, x_values, label="left", color="tab:blue")

        if self.last_right_coeffs is not None:
            y_values = self._evaluate_lane(self.last_right_coeffs, x_values)
            ax.plot(y_values, x_values, label="right", color="tab:orange")

        if self.last_left_coeffs is None and self.last_right_coeffs is None:
            ax.text(
                0.5,
                0.5,
                "no valid lane",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )

        ax.set_title("Last Valid Lane Polynomial")
        ax.set_xlabel("y (m, + left)")
        ax.set_ylabel("x (m, + forward)")
        ax.invert_xaxis()
        ax.grid(True)
        ax.legend(loc="best")

    @staticmethod
    def _evaluate_lane(
        coeffs: tuple[float, float, float, float], x_values
    ) -> np.ndarray:
        a, b, c, d = coeffs
        return (a * x_values**3) + (b * x_values**2) + (c * x_values) + d

    def _plot_timeseries(self, axes) -> None:
        plot_history = self._downsample_history()
        t_values = plot_history["t"]
        series = [
            ("a", "l_a", "r_a"),
            ("b", "l_b", "r_b"),
            ("c", "l_c", "r_c"),
            ("d", "l_d", "r_d"),
        ]

        for ax, (title, left_key, right_key) in zip(axes, series):
            ax.plot(t_values, plot_history[left_key], label="left", color="tab:blue")
            ax.plot(
                t_values, plot_history[right_key], label="right", color="tab:orange"
            )
            ax.set_title(title)
            ax.set_ylabel(title)
            ax.grid(True)
            ax.relim()
            ax.autoscale_view()
            ax.legend(loc="best")

        axes[-1].set_xlabel("T (s)")

    def _downsample_history(self) -> dict[str, np.ndarray]:
        count = len(self.history["t"])
        if count > self.max_plot_samples:
            indices = np.linspace(0, count - 1, self.max_plot_samples, dtype=int)
        else:
            indices = np.arange(count)

        return {
            key: np.asarray(values, dtype=float)[indices]
            for key, values in self.history.items()
        }

    def destroy_node(self) -> bool:
        self._finalize_outputs()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    def finalize_then_shutdown(signum, frame) -> None:
        del signum, frame
        if node is not None:
            node._finalize_outputs()
        if rclpy.ok():
            rclpy.shutdown()

    try:
        node = LaneResultRecorder()
        signal.signal(signal.SIGTERM, finalize_then_shutdown)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
