# Connect to the Vehicle (in this case a simulator running on the same computer)
import dronekit
import dronekit_sitl
import time
from pymavlink import mavutil  # needed for command message definitions
from dronekit import connect, VehicleMode

import pickle
import argparse
import socket_server
from simple_pid import PID
import time

"""
pid = PID(1, 0.1, 0.05, setpoint=0) # instantiate our pid object

PV = 10 # create a dummy process variable. We will just manually change this to simulate a change in our system

CV = pid(PV) # everytime we call this, CV gets updated with a new value that gets fed to our system

CV = -53.906 # heres an example of the control variable being updated. Our process variable is 10 and our setpoint is 
0 so it’s driving negative to try to reach 0. 

CV = pid(pv) #update the CV again. We still haven’t changed the PV so the PID controller will attempt to drive it harder to 0.

CV = -65.75

PV = 0 # lets manually set the process variable to 0 to see how the CV responds.

CV = pid(pv) # update the control variable with the new process variable

CV = -81.98 # since our process variable is the same as our setpoint, I would expect this value to go to 0 or even positive.

# Here’s a simple loop to watch the control variable update. You can break out of the loop (ctrl-c)  periodically or 
use some other method to update the PV to whatever you want to see how it responds. I’ve tested this and it works, 
although the PID gains will effect how fast it responds. 

While True:
                CV = pid(PV)
                Print(CV)

After experimenting a bit with these, I’m wondering if the controller has an issue with 0 as a setpoint, 
or maybe even just going negative? There was a few time I noticed the CV was not changing.

I’ve re-run the tests with different gains to speed up the response: pid.Kp=1, pid.Ki = 1, and I changed the setpoint 
to 5 and a starting PV of 1. 

Let the loop run for a bit and you can see the cv increase. Then I broke out of the loop and set the PV to 6 which is 
slightly above our setpoint and the cv started to decrease as expected. 

For your question about why your control variables are always zero, have you confirmed that the process variable 
values being fed to the PID controllers are non-zero? Maybe just throw some print statements in to verify? I can’t 
help much with anything that says ‘self’, as I’ve always been confused by what that means.

I’m not sure about the update code below from the heater example. We don’t have an actual system or simulated system 
to test which is why I was manually changing the PV above.

Hopefully this gives you some ideas, but for me, I am able to see the PID library do its thing.

Let me know if you have any more questions.

"""


class Tracker:
    # in future, may expand args to fill the PID parameters currently hardcoded below.
    def __init__(self, addr, port, set_point=0, sample_time=0.5, output_limits=(None, None)):
        self._is_tracking = False
        self.my_socket = None
        self.x_displacement = 0
        self.y_displacement = 0
        self.addr = addr
        self.port = port
        self.set_point = set_point
        self.sample_time = sample_time
        self.output_limits = output_limits
        self.last_time = 0
        self.x_PID = PID(1, 0.1, 0.05, setpoint=self.set_point, sample_time=self.sample_time,
                         output_limits=self.output_limits)
        self.y_PID = PID(1, 0.1, 0.05, setpoint=self.set_point, sample_time=self.sample_time,
                         output_limits=self.output_limits)
        socket_server.bind_and_listen(self.addr, self.port, self.connect, self.disconnect, self.displacement_received)

    @property
    def is_tracking(self):
        return self._is_tracking

    @is_tracking.setter
    def is_tracking(self, new_value):
        # Enable or Disable both PID controllers
        # To disable the PID so that no new values are computed, set auto mode to False:
        # No new values will be computed when pid is called
        self._is_tracking = self.x_PID.auto_mode = self.y_PID.auto_mode = new_value
        # print("Tracking set to ", new_value)

    def displacement_received(self, __, message):
        """
        This method is called each time the socket_server script receives a message.
        The ppn_server script is configured to send them while tracking, if this endpoint accepts the connection.

        The first argument is a reference to the socket who delivered this message but it is not used here
        message: a dictionary containing keys 'header' and 'data'. The 'data' key is pickle-encoded.
        """
        values = pickle.loads(message['data'])
        # Currently 'values' is an (B, x, y) tuple; B is a boolean to indicate the tracking status
        self.is_tracking, self.x_displacement, self.y_displacement = values

        current_time = time.time()
        dt = current_time - self.last_time
        # print("Bar", dt, object_tracker.sample_time, object_tracker.is_tracking)
        # If object is being tracked, determine offsets and update PID control variables
        if self.is_tracking:
            # This loop needs to be called at regular intervals to update the PID controllers and transmit the velocity
            # commands without overloading comm. buffers. Assume 2 Hz (0.5 seconds per cycle for now)
            # This loop timing needs to consider the tracking FPS and if this will be run asynchronously

            # From the simple-PID documentation: The PID works best when it is updated at regular intervals. To
            # achieve this, set sample_time to the amount of time there should be between each update and then call
            # the PID every time in the program loop. A new output will only be calculated when sample_time seconds
            # has passed:

            # Update our process variables here. These are what we are trying to control, ie. bring them to 0.
            x_offset = self.x_displacement
            y_offset = self.y_displacement

            # Update our control variables generated from both PID controllers
            x_control_variable = self.x_PID(x_offset)
            y_control_variable = self.y_PID(y_offset)
            # print("Foo", dt, self.sample_time)
            if dt >= self.sample_time:
                print(dt, x_offset, y_offset, x_control_variable, y_control_variable)
                # Assemble a mavlink message giving RIGHT and DOWN velocity commands.
                # Note that down is negative and FORWARD component is ommitted
                # This function normally takes a duration. Should this be the update period? Less?
                # mavlink_msg = generate_MAVlink_RD_message(x_control_variable, y_control_variable)

                # print(current_time, x_control_variable, y_control_variable)

                # Send the message from the companion computer to the flight controller. This should be at regular
                # intervals and within our MAVLINK protocol timing requirements, ie. not too fast
                # mavlink_msg.transmit()

                # update last_time to reflect that of the most recently-issued message.
                send_frd_velocity(0, x_control_variable, 0 - y_control_variable, self.sample_time)
                self.last_time = time.time()

        # print("displacement: ", self.is_tracking, "received x=", self.x_displacement, "y=", self.y_displacement)

    def connect(self, client_socket):
        self.my_socket = client_socket

    def disconnect(self, __, arg):
        self.my_socket = None
        self.is_tracking = False


def arm_and_takeoff(target_altitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: "), vehicle.location.global_relative_frame.alt
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def send_frd_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    # No method "set_position_target_local_frd_encode"
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        # mavutil.mavlink.MAV_FRAME_BODY_FRD,  # frame
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    # for x in range(0, duration):
    vehicle.send_mavlink(msg)
    #     time.sleep(duration)


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x,  # X velocity in NED frame in m/s
        velocity_y,  # Y velocity in NED frame in m/s
        velocity_z,  # Z velocity in NED frame in m/s
        0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    # for x in range(0, duration):
    vehicle.send_mavlink(msg)
    #     time.sleep(duration)


vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

vehicle.home_location = vehicle.location.global_frame

arm_and_takeoff(20)

ap = argparse.ArgumentParser()
ap.add_argument("-p", "--port", required=False, type=int, default=14560,
                help="port for data offset receipt")
args = ap.parse_args()

Tracker('127.0.0.1', args.port)
