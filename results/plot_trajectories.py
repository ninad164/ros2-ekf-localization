import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def find_db3_file(folder):
    for f in os.listdir(folder):
        if f.endswith(".db3"):
            return os.path.join(folder, f)
    raise FileNotFoundError("No .db3 file found in bag folder")

def read_messages(db_path):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: {"name": row[1], "type": row[2]} for row in cursor.fetchall()}

    cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")
    rows = cursor.fetchall()

    conn.close()
    return topics, rows

def main():
    bag_dirs = [d for d in os.listdir(".") if os.path.isdir(d)]
    if not bag_dirs:
        raise FileNotFoundError("No rosbag folder found in current directory")

    bag_dirs.sort()
    bag_dir = bag_dirs[-1]
    db_path = find_db3_file(bag_dir)

    topics, rows = read_messages(db_path)

    odom_xy = []
    ekf_xy = []
    gps_xy = []

    for topic_id, timestamp, data in rows:
        topic_name = topics[topic_id]["name"]
        topic_type = topics[topic_id]["type"]
        msg_type = get_message(topic_type)
        msg = deserialize_message(data, msg_type)

        if topic_name == "/odom":
            odom_xy.append([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])

        elif topic_name == "/ekf/odom":
            ekf_xy.append([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y
            ])

        elif topic_name == "/fake_gps":
            gps_xy.append([
                msg.point.x,
                msg.point.y
            ])

    odom_xy = np.array(odom_xy) if odom_xy else np.empty((0, 2))
    ekf_xy = np.array(ekf_xy) if ekf_xy else np.empty((0, 2))
    gps_xy = np.array(gps_xy) if gps_xy else np.empty((0, 2))

    plt.figure(figsize=(8, 6))

    if len(odom_xy) > 0:
        plt.plot(odom_xy[:, 0], odom_xy[:, 1], label="Odometry", linewidth=2)

    if len(ekf_xy) > 0:
        plt.plot(ekf_xy[:, 0], ekf_xy[:, 1], label="EKF", linewidth=2)

    if len(gps_xy) > 0:
        plt.scatter(gps_xy[:, 0], gps_xy[:, 1], label="Fake GPS", s=8, alpha=0.6)

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory Comparison: Odometry vs Fake GPS vs EKF")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("trajectory_comparison.png", dpi=200)
    plt.show()

if __name__ == "__main__":
    main()