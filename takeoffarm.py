from pymavlink import mavutil
import time
the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
the_connection.wait_heartbeat()

#SET_MODE
the_connection.set_mode('GUIDED')

#ARMING
the_connection.arducopter_arm()

#TAKEOFF
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

altitude = 0
target_altitude = 10  # Target takeoff altitude

# Wait for the drone to reach the target altitude
while altitude < target_altitude:
    msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        altitude = msg.alt / 1000.0  # Convert from millimeters to meters
        print(f"Current altitude: {altitude} meters")
        if altitude >= target_altitude:
            print("Target altitude reached.")
            break
    time.sleep(1)  # Check every second

#MOVEMENT
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
3527, 0, 0, 0, 10, 10, -10, 0, 0, 0, 0, 0))
msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
print(msg)