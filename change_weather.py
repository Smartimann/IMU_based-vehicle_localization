import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import math



def change_weather(args, client): 

    fog_density = 2
    sun_azimuth_angle=-1.000000 
    sun_altitude_angle=45.000000
    precipitation=0.000000
    cloudiness=5.000000 
    wetness = 0


    world = client.get_world()
    weather = world.get_weather()
    daytime = args.daytime
    if (daytime == 'day'): 
        sun_azimuth_angle=-1.000000 
        sun_altitude_angle=45.000000
        print('set day')
    elif(daytime == 'night'): 
        sun_azimuth_angle=-1.000000 
        sun_altitude_angle=0.000000
        print('set night')

    weather_arg = args.weather

    if (weather_arg == 'clear'): 
        precipitation=0.000000
        cloudiness=5.000000         

        print('set clear')
    elif(weather_arg == 'light_fog'): 
        fog_density = 20
        precipitation=0.000000
        cloudiness=10.000000 
        

        print('set light fog')
    elif(weather_arg =='heavy_fog'): 
        fog_density = 100
        precipitation=0.000000
        cloudiness=30.000000

        print('set heavy fog')

    elif(weather_arg == 'light_rain'): 
        fog_density = 2
        precipitation=50.000000
        cloudiness=60.000000
        wetness=60


    elif(weather_arg == 'heavy_rain'): 
        fog_density = 2
        precipitation=100.000000
        cloudiness=100.000000
        wetness=100
    
    weather = carla.WeatherParameters(
        cloudiness=cloudiness, 
        precipitation=precipitation, 
        precipitation_deposits=0.000000, 
        wind_intensity=10.000000, 
        sun_azimuth_angle=sun_azimuth_angle, 
        sun_altitude_angle=sun_altitude_angle, 
        fog_density=fog_density, 
        fog_distance=0.750000, 
        fog_falloff=0.100000, 
        wetness=wetness, 
        scattering_intensity=1.000000, 
        mie_scattering_scale=0.030000, 
        rayleigh_scattering_scale=0.033100)
    world.set_weather(weather)
