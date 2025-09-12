#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
import os

# Global list to store poses
poses = []
save_path = None

def callback(msg):
    for transform in msg.transforms:
        timestamp = transform.header.stamp.to_sec()

        # Translation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        # Rotation (quaternion)
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        poses.append((timestamp, tx, ty, tz, qx, qy, qz, qw))

        rospy.loginfo("TF: frame %s -> %s | t=%.6f pos=(%.3f, %.3f, %.3f)",
                      transform.header.frame_id,
                      transform.child_frame_id,
                      timestamp, tx, ty, tz)

def shutdown_hook():
    global save_path
    rospy.loginfo("Shutting down... Saving poses to %s", save_path)
    try:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, "w") as f:
            for (t, tx, ty, tz, qx, qy, qz, qw) in poses:
                f.write(f"{t:.6f} {tx:.6f} {ty:.6f} {tz:.6f} "
                        f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
        rospy.loginfo("Saved %d poses to %s", len(poses), save_path)
    except Exception as e:
        rospy.logerr("Failed to save poses: %s", str(e))

def listener():
    global save_path
    rospy.init_node('tf_listener', anonymous=True)

    # Read save path param
    save_path = rospy.get_param("~save_path",
                                "/tmp/poses.txt")  # default if not provided
    rospy.loginfo("Saving poses to: %s", save_path)

    rospy.Subscriber("/tf", TFMessage, callback)

    rospy.on_shutdown(shutdown_hook)
    rospy.spin()

if __name__ == '__main__':
    listener()
