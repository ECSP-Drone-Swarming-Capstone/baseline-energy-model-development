import setup_path # Running with Python Version 3. .. .. tba
import airsim

import configparser
import os
import numpy as np
import swarm_controller as sc

from wind_model import WindModel
from DroneSwarmEnergyModel import DroneSwarmPowerModel 


class Main:
    
    # Controls Print Debug Statements 
    DEBUG = True

    def __init__(self, swarm_config_file_path):
        ''' 
            Initialize Airsim environment and gather configuration
            information.
        
        '''
        # Documents\AirSim\settings.json controls airsim simulation
        
        '''# Drone Configuration Information
        drone_swarm_config = configparser.ConfigParser()
        if os.path.exists(swarm_config_file_path):
            drone_swarm_config.read(swarm_config_file_path)
        else:
            print(f"Configuration file '{swarm_config_file_path}' does not exist.")'''

        # Setup Environment Parameters
        self.airsim_setup()

        # Environmental configuration information.
        # self.initial_position = np.array([])
        
        # Initialize Energy Consumption Model

        # Initialize Decision Making

        # Initialize Controller
        self.formation_controller = sc(self.swarm_drone_names, self.swarm_size)
        

    def airsim_setup(self):
        """
            Setup the Airsim Connection with Unreal Environment
            and arm drones/api control
        """
        # start drone client
        self.airsim_client = airsim.MultirotorClient()
        # Check connectivity
        self.airsim_client.confirmConnection()   
        # List of Drones in Simulation
        self.swarm_drone_names = self.airsim_client.listVehicles()
        self.swarm_size = len(self.swarm_drone_names)
        # Enable API/Arm drones 
        for drone in self.swarm_drone_names:
            self.airsim_client.enableApiControl(True, drone)
            self.airsim_client.armDisarm(True, drone)
    

    def main_loop(self):
        # Loop begins
        while True:
            