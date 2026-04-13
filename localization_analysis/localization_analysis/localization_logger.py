#!/usr/bin/env python3
"""
Logs rtabmap localization quality metrics to a CSV file.

Subscribed topics:
  /rtabmap/info          (rtabmap_msgs/Info)
  /localization_pose     (geometry_msgs/PoseWithCovarianceStamped)

Parameters:
  camera_mode  (string, default 'single')  — label written to every CSV row
  output_path  (string, default '~/localization_log.csv')
  plan_path    (string, default '~/plan_log.csv')  — separate file for /plan poses

Key metrics recorded per rtabmap update:
  inliers         — visual feature inliers in the matched frame (higher = stronger match)
  matches         — total feature correspondences found
  inlier_ratio    — inliers / matches  (robustness of the match)
  hypothesis_ratio — probability of the best localization hypothesis (0‥1, higher = more certain)
  loop_closure_id  — >0 when a loop closure was accepted (landmark re-detection)
  wm_size          — working-memory node count
  detection_time_ms / total_time_ms — processing load
  pos_x, pos_y, yaw — pose estimated by rtabmap
  cov_xx, cov_yy, cov_yaw — diagonal of the 6×6 pose covariance (uncertainty)
  cov_pos_trace    — cov_xx + cov_yy  (scalar positional uncertainty summary)
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rtabmap_msgs.msg import Info


_SCRIPTS_DIR = os.path.join(
    os.path.expanduser('~'),
    'FEI', 'Mestrado', '1_quadrimestre', 'CienciaDeDados',
    'Projeto', 'FinalStruct', 'ros2_ws', 'src', 'localization_analysis',
    'data', 'logs',
)


def _next_run_dir() -> str:
    """Return the next available logger_csv_N folder inside the scripts directory."""
    n = 1
    while True:
        candidate = os.path.join(_SCRIPTS_DIR, f'logger_csv_{n}')
        if not os.path.exists(candidate):
            os.makedirs(candidate)
            return candidate
        n += 1


class LocalizationLogger(Node):
    def __init__(self):
        super().__init__('localization_logger')

        self.declare_parameter('camera_mode', 'single')

        run_dir = _next_run_dir()
        default_output = os.path.join(run_dir, 'localization_log.csv')
        default_plan   = os.path.join(run_dir, 'plan_log.csv')

        self.declare_parameter('output_path', default_output)
        self.declare_parameter('plan_path',   default_plan)

        self._camera_mode = self.get_parameter('camera_mode').get_parameter_value().string_value
        output_path = self.get_parameter('output_path').get_parameter_value().string_value
        plan_path   = self.get_parameter('plan_path').get_parameter_value().string_value

        self._latest_pose = None  # holds the most recent PoseWithCovarianceStamped
        self._plan_id     = 0     # increments each time a new plan arrives

        # --- plan CSV (one row per pose in the plan) ---
        self._plan_file   = open(plan_path, 'w', newline='')
        self._plan_writer = csv.DictWriter(self._plan_file,
                                           fieldnames=['plan_id', 'pose_index', 'x', 'y'])
        self._plan_writer.writeheader()
        self._plan_file.flush()

        self._file = open(output_path, 'w', newline='')
        self._writer = csv.DictWriter(self._file, fieldnames=[
            'timestamp_sec',
            'camera_mode',
            'node_id',
            'inliers',
            'matches',
            'inlier_ratio',
            'hypothesis_ratio',
            'loop_closure_id',
            'wm_size',
            'detection_time_ms',
            'total_time_ms',
            'pos_x',
            'pos_y',
            'yaw',
            'cov_xx',
            'cov_yy',
            'cov_yaw',
            'cov_pos_trace',
        ])
        self._writer.writeheader()
        self._file.flush()

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose',
            self._pose_cb,
            10,
        )
        self.create_subscription(
            Path,
            '/plan',
            self._plan_cb,
            10,
        )
        self.create_subscription(
            Info,
            '/info',
            self._info_cb,
            10,
        )

        self.get_logger().info(
            f'LocalizationLogger started — mode={self._camera_mode}, '
            f'output={output_path}, plan={plan_path}'
        )

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self._latest_pose = msg

    def _plan_cb(self, msg: Path):
        self._plan_id += 1
        for i, stamped_pose in enumerate(msg.poses):
            self._plan_writer.writerow({
                'plan_id':    self._plan_id,
                'pose_index': i,
                'x':          round(stamped_pose.pose.position.x, 4),
                'y':          round(stamped_pose.pose.position.y, 4),
            })
        self._plan_file.flush()
        self.get_logger().info(
            f'Plan #{self._plan_id} received — {len(msg.poses)} poses'
        )

    def _info_cb(self, msg: Info):
        # Build a dict from the parallel stats arrays
        stats = dict(zip(msg.stats_keys, msg.stats_values))

        # --- pose & covariance (use latest available pose) ---
        pos_x = pos_y = yaw = cov_xx = cov_yy = cov_yaw = float('nan')
        if self._latest_pose is not None:
            p = self._latest_pose.pose.pose
            pos_x = p.position.x
            pos_y = p.position.y
            q = p.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            # covariance: flat 36-element row-major 6×6 — x=0, y=7, yaw=35
            cov = self._latest_pose.pose.covariance
            cov_xx  = cov[0]
            cov_yy  = cov[7]
            cov_yaw = cov[35]

        inliers      = int(stats.get('Loop/Visual_inliers/', 0))
        matches      = int(stats.get('Loop/Visual_matches/', 0))
        inlier_ratio = stats.get('Loop/Visual_inliers_ratio/', float('nan'))

        row = {
            'timestamp_sec':    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'camera_mode':      self._camera_mode,
            'node_id':          msg.ref_id,
            'inliers':          inliers,
            'matches':          matches,
            'inlier_ratio':     round(inlier_ratio, 4),
            'hypothesis_ratio': round(stats.get('Loop/Hypothesis_ratio/', float('nan')), 4),
            'loop_closure_id':  msg.loop_closure_id,
            'wm_size':          int(stats.get('Memory/Working_memory_size/', len(msg.wm_state))),
            'detection_time_ms': round(stats.get('RtabmapROS/TimeRtabmap/ms', float('nan')), 2),
            'total_time_ms':    round(stats.get('RtabmapROS/TimeTotal/ms', float('nan')), 2),
            'pos_x':            round(pos_x, 4),
            'pos_y':            round(pos_y, 4),
            'yaw':              round(yaw, 4),
            'cov_xx':           round(cov_xx, 6),
            'cov_yy':           round(cov_yy, 6),
            'cov_yaw':          round(cov_yaw, 6),
            'cov_pos_trace':    round(cov_xx + cov_yy, 6),
        }
        self._writer.writerow(row)
        self._file.flush()

    def destroy_node(self):
        self._file.close()
        self._plan_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
