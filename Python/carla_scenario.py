#all imports
import carla
import random
import cv2
import numpy as np

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

# spawning a car
actor_list = []
my_vehicles = world.get_blueprint_library().filter('*tesla*')
spawn_point = carla.Transform(carla.Location(x=-16.890745,y=-211.247208, z=0.281942),carla.Rotation(pitch=0.0, yaw=89.775124, roll=0.000000))
vehicle = world.try_spawn_actor(my_vehicles[1], spawn_point)
actor_list.append(vehicle)

my_vehicles_2 = world.get_blueprint_library().filter('*audi*')
spawn_point_2 = carla.Transform(carla.Location(x=-16.890745,y=-240.247208, z=0.281942),carla.Rotation(pitch=0.0, yaw=89.775124, roll=0.000000))
vehicle_2 = world.try_spawn_actor(my_vehicles_2[1], spawn_point_2)
actor_list.append(vehicle_2)

# it is better to add all objects or vehicles to an array
actor_list = []
actor_list.append(vehicle)

# to control the vehicle: 
# https://pythonprogramming.net/control-camera-sensor-self-driving-autonomous-cars-carla-python/
vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))

def calculate_distance(actor1, actor2):
    location1 = actor1.get_location()
    location2 = actor2.get_location()

    dx = location1.x - location2.x
    dy = location1.y - location2.y
    dz = location1.z - location2.z

    distance = math.sqrt(dx**2 + dy**2 + dz**2)

    return distance
    
    
#Araba dururken
while True:
    # Select your car (ego vehicle) and the stopping car ahead of you
    ego_vehicle = actor_list[1]
    stopping_car = actor_list[0]  # Assuming the stopping car is the second vehicle in the list

    # Calculate the distance between the two vehicles
    distance = calculate_distance(ego_vehicle, stopping_car)
    if distance < 15:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1))
        break;
    else:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=0.7, brake=0))

#araba giderken
while True:
    # Select your car (ego vehicle) and the stopping car ahead of you
    ego_vehicle = actor_list[1]
    stopping_car = actor_list[0]  # Assuming the stopping car is the second vehicle in the list

    # Calculate the distance between the two vehicles
    distance = calculate_distance(ego_vehicle, stopping_car)
    stopping_car.apply_control(carla.VehicleControl(throttle=0.5, brake=0))
    if 7 < distance < 12:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.5))
        print('first: ' + str(distance))
    elif distance < 7:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1))
        print('sec: ' + str(distance))
    elif distance > 17:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=1, brake=0))
        print('third: ' + str(distance))
    else:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=0.7, brake=0))
        print('fourth: ' + str(distance))
    
    if distance <= 5:
        print('break: ' + str(distance))
        break
        
#manevra
#guzel
class PIDController:
    def __init__(self, K_P, K_I, K_D, dt):
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.dt = dt

        self.error = 0
        self.integral = 0
        self.previous_error = 0

    def update(self, setpoint, measured_value):
        self.error = setpoint - measured_value
        self.integral += self.error * self.dt
        derivative = (self.error - self.previous_error) / self.dt

        control_signal = (self.K_P * self.error) + (self.K_I * self.integral) + (self.K_D * derivative)

        self.previous_error = self.error

        return control_signal

# PID-Regler für die Lenkung konfigurieren
#K_P = 1.95
K_P = 1.95
#K_I = 0.05
K_I = 0.11
#K_D = 0.2
K_D = 0.25
dt = 1.0 / 20.0

pid_controller = PIDController(K_P, K_I, K_D, dt)

# Aktives Lenkmanöver ausführen
start = time.time()
ego_vehicle = actor_list[1]
stopping_car = actor_list[0]
while True:
    
    map = world.get_map()
    current_waypoint = map.get_waypoint(ego_vehicle.get_location())
    left_lane = current_waypoint.get_left_lane()
    
    # Ziel-Lenkwinkel berechnen
    target_yaw = math.radians(left_lane.transform.rotation.yaw)
    current_yaw = math.radians(ego_vehicle.get_transform().rotation.yaw)
    steer = pid_controller.update(target_yaw, current_yaw)
    
    #Berechnung der Geschwindigkeit
    velocity = ego_vehicle.get_velocity()
    speed_ms = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) # Umrechnung in km/h
    speed_kmh = speed_ms * 3.6
    
    #Berechnung des Bremswegs und der Notfallbremsung
    breaking_distance = (speed_kmh/10)*(speed_kmh/10)
    emergency_distance = breaking_distance / 2

    # Calculate the distance between the two vehicles
    distance = calculate_distance(ego_vehicle, stopping_car)

    # Lenkwinkel auf das Fahrzeug anwenden
    control = carla.VehicleControl(throttle=2, brake=0, steer=steer)
    
    if distance < breaking_distance:
        ego_vehicle.apply_control(control)
    else:
        ego_vehicle.apply_control(carla.VehicleControl(throttle=2, brake=0))
    
    
    # Für einige Sekunden warten, um das Manöver zu beobachten
    time.sleep(0.1)

    elapsed = time.time() - start
    if elapsed > 4:
        if speed_kmh > 0:
            ego_vehicle.apply_control(carla.VehicleControl(throttle=0, brake=10.0, steer=0))
        
        break
