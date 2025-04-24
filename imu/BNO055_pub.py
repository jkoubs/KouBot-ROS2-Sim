import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import adafruit_bno055
import math

class BNO055Publisher(Node):
    def __init__(self):
        super().__init__('bno055_publisher')
        self.publisher_ = self.create_publisher(Imu, 'bno055/imu', 10)
        self.timer = self.create_timer(2, self.publish_sensor_data)  # 10Hz
        self.sensor = adafruit_bno055.BNO055_I2C(board.I2C())
        self.get_logger().info("BNO055 IMU Publisher Node has started")

    def publish_sensor_data(self):
        msg = Imu()

        # Orientation Data
        quat = self.sensor.quaternion
        if quat:
            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]

        # Angular Velocity (rad/s)
        gyro = self.sensor.gyro
        if gyro:
            msg.angular_velocity.x = math.radians(gyro[0])
            msg.angular_velocity.y = math.radians(gyro[1])
            msg.angular_velocity.z = math.radians(gyro[2])

        # Linear Acceleration (m/s^2)
        accel = self.sensor.linear_acceleration
        if accel:
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

        # Publish the IMU message
        self.publisher_.publish(msg)
        self.get_logger().info("Published IMU Data")

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Link: https://github.com/holmes24678/IMU_Node
