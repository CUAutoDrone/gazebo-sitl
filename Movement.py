import pymavlink
from pymavlink import mavutil
#import dronekit
#from dronekit import VehicleMode
# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
# Wait for the first heartbeat
# This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
#the_connection.mode = VehicleMode("GUIDED")
the_connection.arducopter_arm()
mode_id = the_connection.mode_mapping()['GUIDED']
the_connection.set_mode(mode_id)
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
# target_system: System ID of vehicle
# target_component: Component ID of flight controller or just 0
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
### LOCAL Frame
# SET_POSITION_TARGET_LOCAL_NED: Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
# MAV_FRAME_LOCAL_NED: NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
3527, 0, 0, 0, 10, 10, -10, 0, 0, 0, 0, 0))
#### GLOBAL Frame
# # SET_POSITION_TARGET_GLOBAL_INT: Set the vehicle\u2019s target position (in WGS84 coordinates), velocity, heading or turn rate.
# # MAV_FRAME_GLOBAL_RELATIVE_ALT: Global (WGS84) coordinate frame + altitude relative to the home position.
# the_connection.mav.send(mavutil.mavlink.MAVLink_SET_POSITION_TARGET_GLOBAL_INT(10,
# the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
# 0, 100, 100, -100, 10, 10, -10, 0, 0, 0, 0, 0))
while 1:
msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
print(msg)