#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
import math
import logging

# Constants
PROTOCOL_VERSION = 2

if PROTOCOL_VERSION == 1:
    ADDR_MX_TORQUE_ENABLE    = 24
    ADDR_MX_PRESENT_POSITION = 36
    ADDR_MX_GOAL_POSITION    = 30
    LEN_MX_GOAL_POSITION     = 2
elif PROTOCOL_VERSION == 2:
    ADDR_MX_TORQUE_ENABLE    = 64
    ADDR_MX_PRESENT_POSITION = 132
    ADDR_MX_GOAL_POSITION    = 116
    ADDR_MX_GOAL_VELOCITY    = 104
    ADDR_MX_PRESENT_VELOCITY = 128
    LEN_MX_GOAL_POSITION     = 4
    LEN_MX_PRESENT_POSITION  = 4
    LEN_MX_GOAL_VELOCITY     = 4
    LEN_MX_PRESENT_VELOCITY  = 4

class DxlComm(Node):
    def __init__(self, port="/dev/ttyUSB0", baudrate=None, baudnum=None, operating_mode=0):
        super().__init__('dxl_comm_node')
        self.port = port
        self.operating_mode = operating_mode

        if baudrate:
            self.baudrate = baudrate
        elif baudnum:
            self.baudrate = 20000000 / (baudnum + 1)
        else:
            self.baudrate = 57600

        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self.joint_ids = []
        self.joints = []
        self.total = 0

        if self.port_handler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().error("Failed to open the port")

        if self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().error("Failed to change the baudrate")

        if PROTOCOL_VERSION == 2:
            if self.operating_mode == 0:
                self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
            elif self.operating_mode == 1:
                self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)
        
        if self.operating_mode == 0:
            self.group_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
        elif self.operating_mode == 1:
            self.group_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_MX_GOAL_VELOCITY, LEN_MX_GOAL_VELOCITY)

        self.angle_subscriber = self.create_subscription(Int32, 'dxl/set_angle', self.send_angles_callback, 10)
        self.velocity_subscriber = self.create_subscription(Int32, 'dxl/set_velocity', self.send_velocity_callback, 10)
        self.enable_torque_service = self.create_service(SetBool, 'dxl/enable_torque', self.enable_torque_callback)
        self.disable_torque_service = self.create_service(SetBool, 'dxl/disable_torque', self.disable_torque_callback)

    def attach_joints(self, joints):
        for joint in joints:
            self.attach_joint(joint)

    def attach_joint(self, joint):
        self.joints.append(joint)
        self.joint_ids.append(joint.servo_id)
        joint._set_port_and_packet(self.port_handler, self.packet_handler)
        self.total += 1
        self.joint_ids.sort()

    def broadcast_ping(self):
        if PROTOCOL_VERSION == 2:
            detected_servos = []
            dxl_data_list, dxl_comm_result = self.packet_handler.broadcastPing(self.port_handler)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
            self.get_logger().info("Detected Dynamixels: ")
            for servo_id in dxl_data_list:
                detected_servos.append(servo_id)
                self.get_logger().info(f"[ID: {servo_id:03d}] model version: {dxl_data_list.get(servo_id)[0]} | firmware version: {dxl_data_list.get(servo_id)[1]}")
            if len(detected_servos) != self.total:
                self.get_logger().warn("Detected servos value different from total added joints. Check for connection issues.")
            return detected_servos
        else:
            self.get_logger().warn("Broadcast Ping only available for Protocol 2.0.")
            return False

    def send_angles_callback(self, msg):
        angle = msg.data
        self.send_angles({self.joints[0].servo_id: angle})

    def send_velocity_callback(self, msg):
        velocity = msg.data
        self.send_velocity({self.joints[0].servo_id: velocity})

    def enable_torque_callback(self, request, response):
        self.enable_torques(request.data)
        response.success = True
        return response

    def disable_torque_callback(self, request, response):
        self.disable_torques(request.data)
        response.success = True
        return response

    def send_angles(self, values=None):
        if values is None:
            ch_joints = [j for j in self.joints if j.changed]
            self._sync_write(servos=ch_joints)
            for i in ch_joints:
                i.changed = False
        else:
            if isinstance(values, dict):
                list_values = [values[i] for i in sorted(values)]
                if len(list_values) == len(self.joint_ids):
                    self._sync_write(values=list_values)
            else:
                self.get_logger().warn("Make sure the values parameter is a list of the same length of connected dynamixels.")

    def send_velocity(self, values=None):
        if values is None:
            ch_joints = [j for j in self.joints if j.changed]
            self._sync_write(servos=ch_joints)
            for i in ch_joints:
                i.changed = False
        else:
            if isinstance(values, dict):
                list_values = [values[i] for i in sorted(values)]
                if len(list_values) == len(self.joint_ids):
                    self._sync_write(values=list_values)
            else:
                self.get_logger().warn("Make sure the values parameter is a list of the same length of connected dynamixels.")

    def _sync_write(self, servos=None, values=None):
        if servos is not None:
            for s in servos:
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(s.goal_value)), DXL_HIBYTE(DXL_LOWORD(s.goal_value)), DXL_LOBYTE(DXL_HIWORD(s.goal_value)), DXL_HIBYTE(DXL_HIWORD(s.goal_value))]
                self.get_logger().info(f"Setting servo {s.servo_id} to goal position {param_goal_position}")
                dxl_add_param_result = self.group_sync_write.addParam(s.servo_id, param_goal_position)
                if not dxl_add_param_result:
                    self.get_logger().error(f"[ID: {s.servo_id:03d}] groupSyncWrite addParam failed")
        elif values is not None:
            for i, goal_value in enumerate(values):
                goal_value = self.set_goal_value(goal_value)
                servo_id = self.joint_ids[i]
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_value)), DXL_HIBYTE(DXL_LOWORD(goal_value)), DXL_LOBYTE(DXL_HIWORD(goal_value)), DXL_HIBYTE(DXL_HIWORD(goal_value))]
                self.get_logger().info(f"Setting servo {servo_id} to goal position {param_goal_position}")
                dxl_add_param_result = self.group_sync_write.addParam(servo_id, param_goal_position)
                if not dxl_add_param_result:
                    self.get_logger().error(f"[ID: {servo_id:03d}] groupSyncWrite addParam failed")
        dxl_comm_result = self.group_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
        else:
            self.get_logger().info("Sync write successfully transmitted")
        self.group_sync_write.clearParam()

    def get_angles(self):
        if PROTOCOL_VERSION == 2:
            servos_angles = self._sync_read(ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
            return servos_angles
        else:
            for joint in self.joints:
                joint.get_angle()

    def get_velocity(self):
        if PROTOCOL_VERSION == 2:
            servos_velocity = self._sync_read(ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)
            return servos_velocity
        else:
            for joint in self.joints:
                joint.get_velocity()

    def _sync_read(self, addr, info_len):
        if PROTOCOL_VERSION == 2:
            for servo_id in self.joint_ids:
                dxl_add_param_result = self.group_sync_read.addParam(servo_id)
                if not dxl_add_param_result:
                    self.get_logger().error(f"[ID: {servo_id:03d}] groupSyncRead addParam failed")
            dxl_comm_result = self.group_sync_read.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
            servos_data = []
            for servo_id in self.joint_ids:
                dxl_getdata_result = self.group_sync_read.isAvailable(servo_id, addr, info_len)
                if dxl_getdata_result:
                    value = self.group_sync_read.getData(servo_id, addr, info_len)
                    value = value & 0xFFFFFFFF  # Ensure value is interpreted as unsigned 32-bit
                    servos_data.append(value)
                else:
                    self.get_logger().error(f"[ID: {servo_id:03d}] groupSyncRead getData failed")
            self.group_sync_read.clearParam()
            return servos_data
        else:
            self.get_logger().warn("Sync Read only available for Protocol 2.0.")

    def enable_torques(self, servo_id):
        joint = next((j for j in self.joints if j.servo_id == servo_id), None)
        if joint:
            joint.enable_torque()

    def disable_torques(self, servo_id):
        joint = next((j for j in self.joints if j.servo_id == servo_id), None)
        if joint:
            joint.disable_torque()

    def set_goal_value(self, angle):
        return int(angle)

    def release(self):
        self.port_handler.closePort()

class Joint:
    def __init__(self, servo_id, goal_value=0, initial_position=0):
        self.servo_id = servo_id
        self.goal_value = goal_value
        self.initial_position = initial_position
        self.changed = False

        self.center_value = 2048

        self.logger = logging.getLogger(f'Joint_{self.servo_id}')
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)

    def _set_port_and_packet(self, port, packet):
        self.port_handler = port
        self.packet_handler = packet

    def set_goal_value(self, angle, radian=False):
        self.goal_angle = float(angle)
        if radian:
            self.goal_value = int(2048.0 * angle / math.pi) + self.center_value
        else:
            self.goal_value = int(2048.0 * angle / 180.0) + self.center_value
        self.changed = True

        if self.goal_value < 0 or self.goal_value > 4095:
            self.logger.warning("Goal value is out of range for Dynamixel servo. Make sure it's within the range 0-4095.")

    def set_goal_velocity(self, rpm):
        self.goal_velocity = float(rpm)
        self.goal_velocity = int(rpm / 0.229)
        self.changed = True

    def enable_torque(self):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.error(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.logger.error(f"{self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            self.logger.info(f"[ID: {self.servo_id:03d}] Torque enabled")

    def disable_torque(self):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.error(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.logger.error(f"{self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            self.logger.info(f"[ID: {self.servo_id:03d}] Torque disabled")

    def send_angle(self, angle, radian=False):
        self.set_goal_value(angle, radian=radian)
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_GOAL_POSITION, self.goal_value)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

    def get_angle(self, radian=False):
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.error(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.logger.error(f"{self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            dxl_present_position = dxl_present_position & 0xFFFFFFFF
            position = dxl_present_position - self.center_value
            if radian:
                angle = position * (2 * math.pi / 4096.0)
            else:
                angle = position * (360.0 / 4096.0)
            return angle

    def get_velocity(self):
        dxl_present_velocity, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.servo_id, ADDR_MX_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            self.logger.error(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.logger.error(f"{self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            dxl_present_velocity = dxl_present_velocity & 0xFFFFFFFF
            return dxl_present_velocity

def main(args=None):
    rclpy.init(args=args)
    node = DxlComm(operating_mode=0)  # Set operating mode here
    rclpy.spin(node)
    node.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
