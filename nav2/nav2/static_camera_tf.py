import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticCameraTF(Node):
    def __init__(self):
        super().__init__('static_camera_tf')
        self.broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = rclpy.time.Time().to_msg()  # time zero = valid for all time
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        # measure these by hand from your robot: camera position relative to base_link origin, in meters
        t.transform.translation.x = 0.0  # forward offset
        t.transform.translation.y = 0.0  # left/right offset
        t.transform.translation.z = 1.2  # height offset
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = StaticCameraTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
