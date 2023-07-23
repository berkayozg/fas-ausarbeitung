#all imports
import carla
import random
import cv2
import numpy as np
import math
import time
import glob
import os
import sys

# connect to the sim
client = carla.Client('localhost', 2000)

# load custom map
world = client.load_world('Town04_Opt')

# unloading unnecessary objects from map
world.unload_map_layer(carla.MapLayer.Buildings)
world.unload_map_layer(carla.MapLayer.Props)
world.unload_map_layer(carla.MapLayer.Particles)
world.unload_map_layer(carla.MapLayer.Decals)
world.unload_map_layer(carla.MapLayer.Buildings)
world.unload_map_layer(carla.MapLayer.Foliage)
world.unload_map_layer(carla.MapLayer.ParkedVehicles)

# setting the best looking weather
weather = world.get_weather()
weather.fog_density = 1.0
weather.fog_distance = 10.0
weather.sun_altitude_angle = 10
world.set_weather(weather)

#little function for deleting actors (cars)
def delete_all(actor_list):
    for actor in actor_list:
        actor.destroy()


# Function for measuring the distance between cars
def calculate_distance(actor1, actor2):
    # Get the 3D locations of the two actors
    location1 = actor1.get_location()
    location2 = actor2.get_location()

    # Calculate the differences in x, y, and z coordinates between the two actors
    dx = location1.x - location2.x
    dy = location1.y - location2.y
    dz = location1.z - location2.z

    # Calculate the Euclidean distance between the two actors using the differences in coordinates
    distance = math.sqrt(dx**2 + dy**2 + dz**2)

    # Return the computed distance as the output of the function
    return distance


# Spawning cars in the middle of the road
# Create an empty list to store the spawned actors
actor_list = []

# Get the blueprint library for Tesla vehicles
my_vehicles = world.get_blueprint_library().filter('*tesla*')
# Define the spawn point for the first vehicle (Tesla)
spawn_point = carla.Transform(
    carla.Location(x=-9.890745, y=-211.247208, z=0.281942),
    carla.Rotation(pitch=0.0, yaw=89.775124, roll=0.000000)
)

# Attempt to spawn the first vehicle (Tesla) at the defined spawn point
vehicle = world.try_spawn_actor(my_vehicles[1], spawn_point)

# Add the spawned vehicle to the actor_list
actor_list.append(vehicle)

# Get the blueprint library for Audi vehicles
my_vehicles_2 = world.get_blueprint_library().filter('*audi*')

# Define the spawn point for the second vehicle (Audi)
spawn_point_2 = carla.Transform(
    carla.Location(x=-9.890745, y=-240.247208, z=0.281942),
    carla.Rotation(pitch=0.0, yaw=89.775124, roll=0.000000)
)

# Attempt to spawn the second vehicle (Audi) at the defined spawn point
vehicle_2 = world.try_spawn_actor(my_vehicles_2[1], spawn_point_2)
# Add the spawned vehicle to the actor_list
actor_list.append(vehicle_2)



# spawning cars with distance
actor_list = []
my_vehicles = world.get_blueprint_library().filter('*tesla*')
spawn_point = carla.Transform(carla.Location(x=-9.890745,y=-190.247208, z=0.281942),carla.Rotation(pitch=0.0, yaw=89.775124, roll=0.000000))
vehicle = world.try_spawn_actor(my_vehicles[1], spawn_point)
actor_list.append(vehicle)

my_vehicles_2 = world.get_blueprint_library().filter('*audi*')
spawn_point_2 = carla.Transform(carla.Location(x=-9.890745,y=-240.247208, z=0.281942),carla.Rotation(pitch=0.0, yaw=89.775124, roll=0.000000))
vehicle_2 = world.try_spawn_actor(my_vehicles_2[1], spawn_point_2)
actor_list.append(vehicle_2)

#deleting the actors (cars)
delete_all(actor_list)
    

# Class for our PID-Controller
class PIDController:
    def __init__(self, K_P, K_I, K_D, dt):
        # PID controller gains (proportional, integral, derivative)
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D

        # Time step (dt) for the controller
        self.dt = dt

        # Variables to store the error, integral, and previous error
        self.error = 0
        self.integral = 0
        self.previous_error = 0

    def update(self, setpoint, measured_value):
        # Calculate the current error as the difference between the setpoint and the measured value
        self.error = setpoint - measured_value

        # Accumulate the integral term over time
        self.integral += self.error * self.dt

        # Calculate the derivative term as the rate of change of the error
        derivative = (self.error - self.previous_error) / self.dt

        # Calculate the control signal (output) using the PID formula
        control_signal = (self.K_P * self.error) + (self.K_I * self.integral) + (self.K_D * derivative)

        # Update the previous error for the next iteration
        self.previous_error = self.error

        # Return the control signal as the output of the PID controller
        return control_signal



# Optimal values for our PID-Controller
K_P = 1.95
K_I = 0.15
K_D = 0.25
dt = 1.0 / 20.0

# Instance for our PID-Controller
pid_controller = PIDController(K_P, K_I, K_D, dt)

# Record the starting time
start = time.time()

# Select the ego vehicle and the stopping car from the actor_list
ego_vehicle = actor_list[1]
stopping_car = actor_list[0]

while True:
    # Get the map and the current waypoint of the ego vehicle
    map = world.get_map()
    current_waypoint = map.get_waypoint(ego_vehicle.get_location())
    left_lane = current_waypoint.get_left_lane()

    # Calculate the target yaw (heading) of the vehicle for the left lane
    target_yaw = math.radians(left_lane.transform.rotation.yaw)

    # Get the current yaw (heading) of the ego vehicle
    current_yaw = math.radians(ego_vehicle.get_transform().rotation.yaw)

    # Update the PID controller and calculate the steering value (steer) for the ego vehicle
    steer = pid_controller.update(target_yaw, current_yaw)

    # Get the velocity of the ego vehicle and calculate its speed in km/h
    velocity = ego_vehicle.get_velocity()
    speed_ms = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    speed_kmh = speed_ms * 3.6

    # Calculate the braking distance and emergency distance based on the speed
    breaking_distance = (speed_kmh / 10) * (speed_kmh / 10)
    emergency_distance = breaking_distance / 2

    # Calculate the distance between the ego vehicle and the stopping car
    distance = calculate_distance(ego_vehicle, stopping_car)

    # Create the vehicle control object to apply throttle, brake, and steering
    control = carla.VehicleControl(throttle=1, brake=0, steer=steer)

    # Apply the control signal based on the distance to the stopping car
    if distance < breaking_distance:
        ego_vehicle.apply_control(control)
    else:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=1, brake=0))

    # Wait for a short time to observe the maneuver (not recommended in real-time simulations)
    time.sleep(0.1)

    # Check if the elapsed time exceeds 4 seconds and stop the ego vehicle
    elapsed = time.time() - start
    if elapsed > 4:
        if speed_kmh > 0:
            ego_vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1.0, steer=0))
        break



# If the car in front of us is moving slower than us
# Record the starting time
start = time.time()
while True:
    # Select your car (ego vehicle) and the stopping car ahead of you
    ego_vehicle = actor_list[1]   # Audi TT
    stopping_car = actor_list[0]  # Tesla

    # Get the velocity of the ego vehicle and calculate its speed in km/h
    velocity = ego_vehicle.get_velocity()
    speed_ms = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    speed_kmh = speed_ms * 3.6

    # Calculate the braking distance and emergency distance based on the speed
    braking_distance = (speed_kmh / 10) * (speed_kmh / 10)
    emergency_distance = braking_distance / 2

    # Calculate the distance between the ego vehicle and the stopping car
    distance = calculate_distance(ego_vehicle, stopping_car)

    # Control the stopping car (Tesla) to move forward with some throttle
    stopping_car.apply_control(carla.VehicleControl(throttle=0.5, brake=0))

    if distance < braking_distance:
        # Warning: If the distance is less than the emergency_distance, initiate braking
        if distance < emergency_distance:
            ego_vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1.0))  # Perform emergency braking
    else:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=1, brake=0))  # Continue driving (apply throttle)

    # Calculate the elapsed time
    elapsed = time.time() - start

    # Check if the elapsed time exceeds 20 seconds and stop the simulation
    if elapsed > 20:
        break
    

    
# If the car in front of us is stopping
# Record the starting time
start = time.time()

# Flag to control whether to accelerate or brake
gas_flag = 1

while True:
    # Select your car (ego vehicle) and the stopping car ahead of you
    ego_vehicle = actor_list[1]
    stopping_car = actor_list[0]

    # Get the velocity of the ego vehicle and calculate its speed in km/h
    velocity = ego_vehicle.get_velocity()
    speed_ms = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    speed_kmh = speed_ms * 3.6

    # Calculate the braking distance and emergency distance based on the speed
    braking_distance = (speed_kmh / 10) * (speed_kmh / 10)
    emergency_distance = braking_distance / 2

    # Calculate the distance between the ego vehicle and the stopping car
    distance = calculate_distance(ego_vehicle, stopping_car)

    # Check if the gas_flag is set to 1 (accelerate) or 0 (brake)
    if gas_flag == 1:
        # Continue driving by applying throttle and not braking
        ego_vehicle.apply_control(carla.VehicleControl(throttle=1, brake=0))

    if gas_flag == 0:
        # Perform emergency braking (no throttle, full brake)
        ego_vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1))

    # If the distance to the stopping car is less than the braking_distance,
    # set the gas_flag to 0 to initiate braking (emergency stop)
    if distance < braking_distance:
        # Warning: If the distance is less than the emergency_distance + 7,
        # set gas_flag to 0 to initiate emergency braking.
        if distance < emergency_distance + 7:
            gas_flag = 0

    # Calculate the elapsed time
    elapsed = time.time() - start

    # Check if the elapsed time exceeds 10 seconds and stop the simulation
    if elapsed > 10:
        break