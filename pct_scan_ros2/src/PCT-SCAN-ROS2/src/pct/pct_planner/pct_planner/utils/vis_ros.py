from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock

def traj2ros(traj_3d, frame_id="map", stamp=None):
    path = Path()
    path.header.stamp = stamp if stamp is not None else Clock().now().to_msg()
    path.header.frame_id = frame_id
    for i in range(traj_3d.shape[0]):
        pose = PoseStamped()
        pose.header.stamp = path.header.stamp
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x = float(traj_3d[i, 0])
        pose.pose.position.y = float(traj_3d[i, 1])
        pose.pose.position.z = float(traj_3d[i, 2])
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path
