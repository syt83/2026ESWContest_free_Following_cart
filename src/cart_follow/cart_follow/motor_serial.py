import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
UNO_PORT = '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_754303331373510131B2-if00'
UNO_BAUD = 9600

class MotorSerial(Node):

    def __init__(self):
        super().__init__('motor_serial')
        self.ser = serial.Serial(UNO_PORT, UNO_BAUD, timeout=0)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.target_left = 0
        self.target_right = 0
        self.current_left = 0
        self.current_right = 0
        self.RAMP_STEP = 3
        self.last_cmd_time = time.monotonic()
        self.CMD_TIMEOUT = 0.4
        self.hard_stop = True
        self.create_subscription(Int32MultiArray, '/motor_cmd_pwm', self.motor_callback, 10)
        self.create_subscription(Bool, '/motor_hard_stop', self.hard_stop_callback, 10)
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('Motor serial started | HARD STOP DEFAULT ON | RAMP_STEP=3')

    def motor_callback(self, msg):
        if len(msg.data) < 2:
            return
        if self.hard_stop:
            return
        self.target_left = int(self.clamp(msg.data[0], -255, 255))
        self.target_right = int(self.clamp(msg.data[1], -255, 255))
        self.last_cmd_time = time.monotonic()

    def hard_stop_callback(self, msg):
        new_state = bool(msg.data)
        if new_state and (not self.hard_stop):
            self.get_logger().warn('MOTOR HARD STOP ACTIVATED')
        elif not new_state and self.hard_stop:
            self.get_logger().info('MOTOR HARD STOP RELEASED')
            self.target_left = 0
            self.target_right = 0
            self.current_left = 0
            self.current_right = 0
            self.last_cmd_time = time.monotonic()
        self.hard_stop = new_state
        if self.hard_stop:
            self.target_left = 0
            self.target_right = 0
            self.current_left = 0
            self.current_right = 0
            self.send_motor(0, 0)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def move_toward(self, current, target):
        if current < target:
            current += self.RAMP_STEP
            if current > target:
                current = target
        elif current > target:
            current -= self.RAMP_STEP
            if current < target:
                current = target
        return current

    def send_motor(self, left, right):
        command = f'{int(left)},{int(right)}\n'
        try:
            self.ser.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def update(self):
        now = time.monotonic()
        if self.hard_stop:
            self.target_left = 0
            self.target_right = 0
            self.current_left = 0
            self.current_right = 0
            self.send_motor(0, 0)
            return
        if now - self.last_cmd_time > self.CMD_TIMEOUT:
            self.target_left = 0
            self.target_right = 0
        self.current_left = self.move_toward(self.current_left, self.target_left)
        self.current_right = self.move_toward(self.current_right, self.target_right)
        self.send_motor(self.current_left, self.current_right)

    def destroy_node(self):
        try:
            if self.ser.is_open:
                self.send_motor(0, 0)
                time.sleep(0.05)
                self.send_motor(0, 0)
                time.sleep(0.05)
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
