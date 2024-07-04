from pymavlink import mavutil
import time

def connect_to_pixhawk(connection_string):
    # Create a MAVLink connection to the Pixhawk
    master = mavutil.mavlink_connection(connection_string)

    # Wait for the first heartbeat 
    # This sets the system and component ID of remote system for the link
    master.wait_heartbeat(timeout=5)
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    # Request all parameters
    master.mav.param_request_list_send(master.target_system, master.target_component)

    # Receive all parameters
    while True:
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
        if message is None:
            break
        print("Parameter: %s = %f" % (message.param_id, message.param_value))

    # Request vehicle version information
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                 mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    version_msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=10)
    if version_msg:
        print("Autopilot Version: %d" % version_msg.flight_sw_version)

    return master

if __name__ == "__main__":
    # Replace with your connection string
    connection_string = '/dev/serial/by-id/usb-ArduPilot_Pixhawk4_220028000351333035363236-if00'
    print('Trying to connect to pixhawk')
    # Connect to the Pixhawk
    master = connect_to_pixhawk(connection_string)

    if master:
        print("Successfully connected to Pixhawk")
    else:
        print("Failed to connect to Pixhawk")
