import time
from pymavlink import mavutil, mavwp
import pymavlink.dialects.v20.all as dialect
import threading


class Connection:
    # The connection class is used to connect the vehicle,
    # further, more the class can also connect wit sitl for simulation and debug purposes.
    def __init__(self):
        self.connection = None
        self.device = None
        self.baud = None
        self.connection_string = None

    def init(self, device, baud):
        # To connect with telemetry/devices
        self.device = device
        self.baud = baud

    def init_sitl(self, connection_string):
        # to connect with software in the loop
        self.connection_string = connection_string

    def connect_telemetry(self):
        try:
            self.connection = mavutil.mavlink_connection(self.device, baud=self.baud)
            self.connection.wait_heartbeat()
            print("Heartbeat from system (system %u component %u)" % (
                self.connection.target_system, self.connection.target_component))
        except Exception as e:
            print("Error connecting to Flight Controller:", str(e))

    def connect_sitl(self):
        try:
            self.connection = mavutil.mavlink_connection(self.connection_string)
            self.connection.wait_heartbeat()
            print("Heartbeat from system (system %u component %u)" % (
                self.connection.target_system, self.connection.target_component))
        except Exception as e:
            print("Error connecting to Flight Controller:", str(e))

    def get_instance(self):
        return self.connection

    def close(self):
        if self.connection:
            self.connection.close()
            print("Connection closed.")


class FlightController:
    # Following class is used to perform the basic functions of vehicle
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.flight_modes = {
            0: 'STABILIZE',
            1: 'ACRO',
            2: 'ALT_HOLD',
            3: 'AUTO',
            4: 'GUIDED',
            5: 'LOITER',
            6: 'RTL',
            7: 'CIRCLE',
            8: 'LAND',
            9: 'DRIFT',
            10: 'SPORT',
            11: 'FLIP',
            12: 'AUTOTUNE',
            13: 'POSHOLD',
            14: 'BRAKE',
            15: 'THROW',
            16: 'AVOID_ADSB',
            17: 'GUIDED_NOGPS',
            18: 'SMART_RTL',
            19: 'FLOWHOLD',
            20: 'FOLLOW',
            21: 'ZIGZAG',
            22: 'SYSTEMID',
            23: 'AUTOROTATE',
            24: 'AUTO_RTL',
        }

    def turn_on_motor(self):
        # arm the motors
        self.vehicle.arducopter_arm()
        ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        if ack and ack.command == 400 and ack.result == 0:
            print("Turning on Motors")
        else:
            print("Failed to turn on Motors")

    def turn_off_motor(self):
        # disarm the motors
        self.vehicle.arducopter_disarm()
        self.vehicle.motors_disarmed_wait()
        print("Turning off Motors")

    def set_parameter_value(self, param_id, param_value):
        # get parameter type first
        _, param_type = self.get_parameter_value(param_id)
        time.sleep(0.1)
        # To set ardu-pilot parameters
        self.vehicle.mav.param_set_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            param_id.encode('utf-8'),
            param_value,
            param_type
        )


    def set_mode(self, mode):
        # function to set the vehicle mode (i-e guided auto rtl etc)
        mode_id = self.vehicle.mode_mapping()[mode]
        self.vehicle.set_mode_apm(mode_id)
        while True:
            ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("{} mode set successfully...".format(mode))
                break

    def get_mode(self):
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            mode = msg.custom_mode
            if mode in self.flight_modes:
                print(f"Current flight mode: {self.flight_modes[mode]}")
            else:
                print(f"Unknown flight mode: {mode}")



    def get_parameter_value(self, parameter_name):
        parameter_type = None
        # Request the parameter value
        self.vehicle.mav.param_request_read_send(
            self.vehicle.target_system, self.vehicle.target_component,
            parameter_name.encode('utf-8'),
            -1  # Request the value of all instances (use 0 for specific index)
        )
        message = self.vehicle.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        return message['param_value'], message['param_type']

    # To land the vehicle
    def land_vehicle(self):
        self.set_mode('LAND')
        print("Landing...")

    # To trigger Return to Launch (RTL)
    def rtl_vehicle(self):
        self.set_mode('RTL')
        print("Returning to Launch...")

    # Command the vehicle to take off
    def takeoff(self, altitude=5):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude
        )
        while True:
            ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Vehicle Takeoff successfully...")
                break


class GPS:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.latitude = None
        self.longitude = None
        self.rel_alt = None

    def gps_3d_location(self):
        self.vehicle.mav.request_data_stream_send(self.vehicle.target_system, self.vehicle.target_component,
                                                  mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1, 1)
        while True:
            msg = self.vehicle.recv_match(blocking=True)
            if msg:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    self.latitude = msg.lat / 1e7
                    self.longitude = msg.lon / 1e7
                    self.rel_alt = msg.relative_alt / 1000
                    print("GPS Coordinates (Global Position Int):")
                    print(f"Latitude: {msg.lat / 1e7} degrees")
                    print(f"Longitude: {msg.lon / 1e7} degrees")
                    print(f"Altitude: {msg.alt / 1000.0} meters")
                    print(f"Relative Altitude: {msg.relative_alt / 1000.0} meters")



class Compass:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.heading = None

    def get_heading(self):
        # Request compass data stream
        self.vehicle.mav.request_data_stream_send(self.vehicle.target_system, self.vehicle.target_component,
                                                  mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 1, 1)
        while True:
            msg = self.vehicle.recv_match(blocking=True)
            if msg:
                if msg.get_type() == 'VFR_HUD':
                    self.heading = msg.heading
                    print("Compass Heading (VFR HUD):")
                    print(f"Heading: {msg.heading} degrees")
                elif msg.get_type() == 'ATTITUDE':
                    self.heading = msg.yaw
                    # print("Compass Heading (Attitude):")
                    print(f"Heading: {msg.yaw} degrees")


class Waypoint:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.target_locations = None

    def init_target_locations(self, target_locations):
        self.target_locations = target_locations

        message = dialect.MAVLink_mission_count_message(target_system=self.vehicle.target_system,
                                                    target_component=self.vehicle.target_component,
                                                    count=len(target_locations) + 2,
                                                    mission_type=dialect.MAV_MISSION_TYPE_MISSION)
        self.vehicle.mav.send(message)
        # catch a message
        while True:
            message = self.vehicle.recv_match(blocking=True)
            # convert this message to dictionary
            message = message.to_dict()
            # check this message is MISSION_REQUEST
            if message["mavpackettype"] == dialect.MAVLink_mission_request_message.msgname:
                # check this request is for mission items
                if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION:
                    # get the sequence number of requested mission item
                    seq = message["seq"]
                    # create mission item int message
                    if seq == 0:
                        # create mission item int message that contains the home location (0th mission item)
                        message = dialect.MAVLink_mission_item_int_message(target_system=self.vehicle.target_system,
                                                                           target_component=self.vehicle.target_component,
                                                                           seq=seq,
                                                                           frame=dialect.MAV_FRAME_GLOBAL,
                                                                           command=dialect.MAV_CMD_NAV_WAYPOINT,
                                                                           current=0,
                                                                           autocontinue=0,
                                                                           param1=0,
                                                                           param2=0,
                                                                           param3=0,
                                                                           param4=0,
                                                                           x=0,
                                                                           y=0,
                                                                           z=0,
                                                                           mission_type=dialect.MAV_MISSION_TYPE_MISSION)

                    # send takeoff mission item
                    elif seq == 1:

                        # create mission item int message that contains the takeoff command
                        message = dialect.MAVLink_mission_item_int_message(target_system=self.vehicle.target_system,
                                                                           target_component=self.vehicle.target_component,
                                                                           seq=seq,
                                                                           frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                           command=dialect.MAV_CMD_NAV_TAKEOFF,
                                                                           current=0,
                                                                           autocontinue=0,
                                                                           param1=0,
                                                                           param2=0,
                                                                           param3=0,
                                                                           param4=0,
                                                                           x=0,
                                                                           y=0,
                                                                           z=target_locations[0][2],
                                                                           mission_type=dialect.MAV_MISSION_TYPE_MISSION)
                    # send target locations to the vehicle
                    else:
                        # create mission item int message that contains a target location
                        message = dialect.MAVLink_mission_item_int_message(target_system=self.vehicle.target_system,
                                                                           target_component=self.vehicle.target_component,
                                                                           seq=seq,
                                                                           frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                                           command=dialect.MAV_CMD_NAV_WAYPOINT,
                                                                           current=0,
                                                                           autocontinue=0,
                                                                           param1=0,
                                                                           param2=0,
                                                                           param3=0,
                                                                           param4=0,
                                                                           x=int(target_locations[seq - 2][0] * 1e7),
                                                                           y=int(target_locations[seq - 2][1] * 1e7),
                                                                           z=target_locations[seq - 2][2],
                                                                           mission_type=dialect.MAV_MISSION_TYPE_MISSION)
                    # send the mission item int message to the vehicle
                    self.vehicle.mav.send(message)
            # check this message is MISSION_ACK
            elif message["mavpackettype"] == dialect.MAVLink_mission_ack_message.msgname:
                # check this acknowledgement is for mission and it is accepted
                if message["mission_type"] == dialect.MAV_MISSION_TYPE_MISSION and \
                        message["type"] == dialect.MAV_MISSION_ACCEPTED:
                    # break the loop since the upload is successful
                    print("Mission upload is successful")
                    break


if __name__ == "__main__":
    try:
        connector = Connection()
        # device = 'com8'  # portself.vehicle.recv_match(type='PARAM_VALUE', blocking=True)
        # baud = 115200  # baud
        # connector.init(device, baud)
        # connector.connect_telemetry()
        connecting_string = 'tcp:127.0.0.1:5762'  # sitl connecting sting
        connector.init_sitl(connecting_string)
        connector.connect_sitl()
        flight_controller = FlightController(connector.get_instance())
        waypoint_mission = Waypoint(connector.get_instance())
        flight_controller.set_mode('GUIDED')
        flight_controller.turn_on_motor()
        flight_controller.takeoff()
        target_locations = ((-35.361297, 149.161120, 50.0),
                            (-35.360780, 149.167151, 50.0),
                            (-35.365115, 149.167647, 50.0),
                            (-35.364419, 149.161575, 50.0))
        waypoint_mission.init_target_locations(target_locations)
        flight_controller.set_mode('AUTO')

    except Exception as e:
        print("Exception", e)
