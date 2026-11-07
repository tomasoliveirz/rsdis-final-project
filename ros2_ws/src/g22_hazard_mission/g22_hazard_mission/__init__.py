"""Mission nodes and the shared ROS process lifecycle."""


def run_node(node_type):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
