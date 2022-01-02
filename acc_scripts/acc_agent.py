from __future__ import print_function
import time
import signal
import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import sys
import weakref
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue


try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

import random
import numpy as np
import carla
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.local_planner import RoadOption
from agents.navigation.behavior_types import Cautious, Aggressive, Normal

from agents.tools.misc import get_speed, positive, is_within_distance, compute_distance

from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
from misc import draw_waypoints
import collections

from acc import ACC
import matplotlib.pyplot as plt
import datetime
from queue import Queue
from queue import Empty
import csv


class AccAgent:
    def __init__(self, world, vehicle_name, spawn_point:carla.Location = carla.Location(0,0,0)):
        self.world = world
        self.vehicle_name = vehicle_name
        self.spawn_point = spawn_point
        self.player = None
        self.player_agent = None
        self.acc = ACC(self.world)

        self.control = carla.VehicleControl()

        self.sensor_list = []
        self.radar_list = []
        self.radar_queue = Queue()

        self.blueprint_library = world.get_blueprint_library()
        self.radar_bp = self.blueprint_library.find('sensor.other.radar')
        self.radar_bp.set_attribute('horizontal_fov', str(45))
        self.radar_bp.set_attribute('vertical_fov', str(10))
        self.radar_bp.set_attribute('range', str(100))

        self.role_name = "hero" #no rendering variable

        self.velocity = 0

        #INIT FUNCTIONS
        self.spawn()
        self.add_radar()

        pass

    def spawn(self):
        vehicle_bp =self.blueprint_library.find(self.vehicle_name)
        vehicle_bp.set_attribute('role_name', 'hero')
        self.player = self.world.spawn_actor(vehicle_bp,self.spawn_point)
        self.player_agent = BehaviorAgent(self.player)
        self.player_agent.ignore_vehicles(True)
        pass

    def add_radar(self):
        bound_x = 0.5 + self.player.bounding_box.extent.x
        bound_y = 0.5 + self.player.bounding_box.extent.y
        bound_z = 0.5 + self.player.bounding_box.extent.z

        radar = self.world.spawn_actor(self.radar_bp, carla.Transform(carla.Location(x=bound_x + 0.01, z=bound_z - 0.3),
                                                              carla.Rotation(pitch=5)), attach_to=self.player)
        radar_name = "radar{}".format(len(self.radar_list))
        radar.listen(lambda data: self.radar_callback(data, self.radar_queue, radar_name))
        self.radar_list.append(radar)
        pass

    def radar_callback(self,sensor_data: carla.RadarMeasurement, sensor_queue, sensor_name):
        #print("putting radar data", len(sensor_data))
        sensor_queue.put((sensor_data, sensor_data.frame, sensor_name))

    def update(self):
        #print("RADAR TICK: ", self.radar_bp.get_attribute('sensor_tick'))
        #print("Updating accegent")
        v3 = self.player.get_velocity()
        self.velocity = math.sqrt(v3.x ** 2 + v3.y ** 2 + v3.z ** 2)

        self.run_step()
        pass

    def run_step(self):
        self.control = self.player_agent.run_step()
        self.control.throttle = 0
        self.control.brake = 0

        s_frame = self.radar_queue.get(True, 1.0)
        radar_data = s_frame[0]
        current_rot = radar_data.transform.rotation

        self.acc.update(self.velocity, radar_data,current_rot)

        self.control.throttle = self.acc.control.throttle
        self.control.brake = self.control.brake

        self.player.apply_control(self.control)

        #print("V: {v:.3f} | Throttle: {th:.3f} | Brake: {br:.3f} | CTRL: {ctrl:.3f} | Gear{gr:d}".format(
        #    v=self.velocity,th=self.control.throttle,br=self.control.brake,ctrl=self.get_current_out(),gr=self.control.gear))
        pass

    def release(self):
        print("Releasing AccAgent")
        self.player.destroy()

        for sens in self.sensor_list:
            sens.destroy()
        for radar in self.radar_list:
            radar.destroy()

    def get_current_control(self):
        return self.control

    def get_current_out(self):
        return self.control.throttle + self.control.brake

    def __del__(self):
        print("Destroying AccAgent")

        for sens in self.sensor_list:
            sens.destroy()
        for radar in self.radar_list:
            radar.destroy()
