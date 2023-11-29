import profile
import setup_path
import airsim
import numpy as np
import time
import matplotlib.pyplot as plt
import geopy.distance
from math import pi, sqrt, pow, atan2, asin, sin, cos
from scipy.optimize import fsolve
from threading import Thread

# D:\Documents\AirSim contains settings.json folder


class DronePowerModel:
    
    # Controls print statements being used for debugging, 
    # set to Fasle to disable prints.
    DEBUG = True

    def __init__(self):
        self.assign_constants()
        # self.config_file_path = config_file
    
    # ########################## #
    # Energy Model of Quadcopter # 
    # ########################## #     

    ####################
    # Getter Functions #
    ####################

    def get_drone_velocity(self, drone):
        """
            Gets Drone Velocity in meters per second (m/s) 
            from the frame of the local NED frame
            of a given drone.
        """
        # gets the velocity at which the drones are traveling
        # print("Testing Dictionary", self.airsim_state[drone]['MultiRotorState'].kinematics_estimated)
        kinematics = self.airsim_state[drone]['MultiRotorState'].kinematics_estimated
        velocity_drone = [kinematics.linear_velocity.x_val, kinematics.linear_velocity.y_val, kinematics.linear_velocity.z_val]
        return np.array(velocity_drone) # TODO: Verify this is alright to use
    

    def get_velocity_air(self, drone):
        """
            Returns the air velocity (air speed) in meters per second (m/s),
            this is the difference between velocity drone travels at
            and velocity of the wind
        """
        # Velocity_Air = ||V_air|| = || V_ground (velocity drone is travelling) - V_wind ||
        velocity_ground = self.get_drone_velocity(drone)
        velocity_air = np.linalg.norm(velocity_ground - self.wind_vector) # TODO: mention to alain airspeed
        return velocity_air


    def get_net_thrust(self, drone):
        """ 
            computes net thrust of all 4 props, returns 
            net thrust in newtons -> kg * m/s^2 
        """
        rotor_states = self.airsim_state[drone]["RotorState"] #self.airsim_client.getRotorStates(drone)
        net_thrust = 0
        for rotor in rotor_states.rotors:
            net_thrust += rotor['thrust']
        # print("net_thrust", net_thrust)
        return net_thrust
    

    def get_thrust(self, drone):
        """ returns thrust of all four props in an array, units of newtons """
        rotor_states = self.airsim_state[drone]["RotorState"] # self.airsim_client.getRotorStates(drone)
        print("RotorStates", rotor_states.rotors)
        thrust = []
        for rotor in rotor_states.rotors:
            thrust.append(rotor['thrust'])
            # print("Thrust", thrust)
        return np.array(thrust)


    def get_angular_speed(self, drone):
        """ returns speed of all four props in an array as revolutions per minute (rpm) """
        rotor_states = self.airsim_state[drone]["RotorState"] # self.airsim_client.getRotorStates(drone)
        speed = []
        for rotor in rotor_states.rotors:
            speed.append(rotor['speed'])
            # print("angular speed", speed)
        return np.array(speed)


    def get_drone_orientation(self, drone):
        """ 
            Gets the Alpha and Beta Angles of the Drone 
            Returns an dictionary that contains radians and degrees
            each set is stored with roll, pitch, yaw
        """
        drone_pose = self.airsim_state[drone]["PoseWFNED"]#self.airsim_client.simGetObjectPose(drone)
        w = drone_pose.orientation.w_val
        x = drone_pose.orientation.x_val
        y = drone_pose.orientation.y_val
        z = drone_pose.orientation.z_val
        
        roll_x_rad, pitch_y_rad, yaw_z_rad = self.euler_from_quaternion(x, y, z, w) # radians
        roll_x_deg, pitch_y_deg, yaw_z_deg = np.rad2deg(roll_x_rad), np.rad2deg(pitch_y_rad), np.rad2deg(yaw_z_rad) # degrees
        return {"radians": [roll_x_rad, pitch_y_rad, yaw_z_rad], "degrees": [roll_x_deg, pitch_y_deg, yaw_z_deg]}
        
    
    def get_velocity_induced(self, drone):
        """ Caclulates Induced Velocity and returns it in units of meters per second. """
        # Calculate induced velocity when hovering
        if self.drone_flight_mode == "Hover":
            # velocity_induced = sqrt( Thrust / 2 * air_density * A (Total Area Covered By Propellers) )
            # A = M * pi * R^2 
            # M is nuber of propellers
            # R is radius of propellers
            thrust = self.get_net_thrust(drone)
            air_density = self.airsim_state[drone]["AirDensity"] #self.airsim_client.simGetGroundTruthEnvironment(drone).air_density # kg/m^3
            velocity_induced = sqrt( thrust / (2 * air_density * self.A) ) # meters per second (m/s)
            return velocity_induced
        elif self.drone_flight_mode == "Flight":
            # Induced velocity we can represent as the ground velocity of the drone while in flight.
            velocity_induced = self.get_drone_velocity(drone) # m/s
            return velocity_induced
            #thrust = self.get_net_thrust(drone) # N = Kg * m/s^2
            #prop_area = self.A # m^2
            #orientation = self.get_drone_orientation(drone)
            #alpha = orientation['radians'][1] # gets pitch aka angle of attack radians
            #beta = orientation['radians'][2] # gets yaw aka beta/steering angle randians
            #air_density = self.airsim_state[drone]["AirDensity"] # self.airsim_client.simGetGroundTruthEnvironment(drone).air_density
            #numerator = thrust/(2 * air_density * prop_area)
            #velocity_air = self.get_velocity_air(drone) # m/s
            #projection_jk = sqrt( ( pow(sin(alpha), 2) * pow(sin(beta), 2) - pow(sin(beta), 2) ) / (pow(sin(alpha), 2) * pow(sin(beta), 2) - 1) )
            #projection_ij = sqrt( ( pow(sin(alpha), 2) * pow(sin(beta), 2) - pow(sin(alpha), 2) ) / (pow(sin(alpha), 2) * pow(sin(beta), 2) - 1) )
            #projection_ik = ( cos(alpha) * cos(beta) ) / sqrt( pow(sin(alpha), 2) * pow(cos(beta), 2) + pow(cos(alpha), 2) )  
            #denom_term1 = (velocity_air * projection_jk)**2
            #denom_term2 = (velocity_air * projection_ik)**2
            #denom_term3_part1 = velocity_air * projection_ij
            #velocity_induced_hover = sqrt( thrust / (2 * air_density * self.A) ) # m/s # TODO: Assumption that  velocity induced would be a good enough approximation
            #velocity_induced_flight = numerator / sqrt( denom_term1 + denom_term2 + (denom_term3_part1 + velocity_induced_hover)**2 ) 
            #return velocity_induced_flight # m/s
        

    #def get_wind_heading(self):
    #    """ 
    #        Returns Direction relative to drone in the swarm 
    #    """        
    #    n,e,d = self.wind_vector
    #    wind_direction = atan2(e, n)
    #    orientation = self.get_drone_orientation(drone)
    #    beta = orientation['radians'][2] # gets yaw aka beta/steering/heading angle randians
        
        
    def get_parasitic_power(self):
        """ 
            Returns the power consumed in Watts given a particular formation and wind vector.
            The data was gathered through solid works drag simulations. (Direction should be relative
            to swarm).
        """
        wind_speed = np.linalg.norm(self.wind_vector)
        wind_direction = self.wind_direction # East+, North+, North-
        # Assigns sign of wind direction
        wind_velocity = wind_speed
        if wind_direction == "North-":
            wind_velocity = -1 * wind_speed
        
        if DronePowerModel.DEBUG:
            print("Parasitic Power (Flight Mode) Pt2: ", 
                    "\nWind Speed (m/s):", wind_speed,
                    "\nWind Direction (NED):", wind_direction,
                    "\nWind Velocity (m/s):", wind_velocity,
                    "\n Formation Type:", self.formation, "\n"
                    )
            
        if self.formation == "Vee":
            # Cubic estimation of power-x
            a_x , b_x, c_x, d_x = (0.04992735, -0.03668969, 4.85183825, 2.86950829)
            # Quadratic stimation for power-z
            a_z , b_z, c_z = (0.53830656, -3.10909162,  4.6096633)
            if wind_direction == "East+":
                return self.quadratic(wind_velocity, a_z , b_z, c_z)
            else:
                return self.cubic(wind_velocity, a_x , b_x, c_x, d_x)
        elif self.formation == "Echelon":
            # Cubic estimation of power-x
            a_x , b_x, c_x, d_x = (0.06068811, -0.06580198, 4.25524982, 1.32457763)
            # Quadratic stimation for power-z
            a_z , b_z, c_z = (0.74663132, -4.73078804, 6.92984925)
            if wind_direction == "East+":
                return self.quadratic(wind_velocity, a_z , b_z, c_z)
            else:
                return self.cubic(wind_velocity, a_x , b_x, c_x, d_x)
    

    def get_drag_force(self):
        """ 
            Returns the drag force consumed in Newtons given a particular formation and wind vector.
            The data was gathered through solid works drag simulations. (Direction should be relative
            to swarm).
        """
        wind_speed = np.linalg.norm(self.wind_vector)
        wind_direction = "Front" # Front, Left, Right we need a function that returns this
        # Assigns sign of wind direction
        wind_velocity = wind_speed
        if wind_direction == "Left":
            wind_velocity = -1 * wind_speed
            
        if self.formation == "Vee":
            # Cubic estimation of drag-x
            a_x , b_x, c_x, d_x = (0.00118268, -0.00188352, 0.71607042, 0.17407187)
            # Quadratic stimation for drag-z
            a_z , b_z, c_z = (0.0291959, 0.05643199, -0.07195437)
            if wind_direction == "Front":
                return self.quadratic(wind_velocity, a_z , b_z, c_z)
            else:
                return self.cubic(wind_velocity, a_x , b_x, c_x, d_x)
        elif self.formation == "Echelon":
            # Cubic estimation of drag-x
            a_x , b_x, c_x, d_x = (0.0018127, -0.00320426, 0.67314954, 0.07652908)
            # Quadratic stimation for drag-z
            a_z , b_z, c_z = (0.0405097677, -0.0111972537, -0.00000343289225)
            if wind_direction == "Front":
                return self.quadratic(wind_velocity, a_z , b_z, c_z)
            else:
                return self.cubic(wind_velocity, a_x , b_x, c_x, d_x)

    ####################
    # Setter Functions #
    ####################

    def set_operation_state(self, state):
        """ Assigns whether it is in "Hover" or "Flight" mode"""
        self.drone_flight_mode = state
        
        
    def set_wind_vector(self, wind_vector):
        """Specify a wind vector (N, E, D) NED Coordinate System"""
        self.wind_vector = wind_vector
     
        
    def set_formation(self, formation):
        """ Specify the name of the formation being used "Echelon", "Vee", etc """
        self.formation = formation
        
    def set_drones_in_swarm(self, drones_in_swarm):
        """ Assigns drones in swarm """
        self.drones_in_swarm = drones_in_swarm
        
    ###################################
    # Polynomial Regression Functions #
    ###################################

    def linear(self, x, a, b):
        return a*x + b
    
    
    def quadratic(self, x, a, b, c):
        return a*x**2 + b*x + c
    
    
    def cubic(self, x, a, b, c, d):
        return a*x**3+b*x**2+c*x + d

    ##################
    # Misc Functions #
    ##################

    def rpm_to_mps(self, rpm):
        """ Given an rpm will convert to meters per second """
        return self.R * ((2 * pi)/60) * rpm


    def euler_from_quaternion(self, x, y, z, w):
        """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians        


    def calculate_projection(self, drone):
        """
            Computes projection p_l of the specified drone.
        """
        orientation = self.get_drone_orientation(drone)
        alpha = orientation['radians'][1] # gets pitch aka angle of attack
        beta = orientation['radians'][2] # gets yaw aka beta/steering angle
        # Formula for projection is 
        # sqrt( (sin(alpha)^2 * sin(beta)^2 - sin(alpha)^2) / (sin(alpha)^2 * sin(beta)^2 - 1) )
        numerator = pow(sin(alpha), 2) * pow(sin(beta), 2) - pow(sin(alpha), 2)
        denominator = pow(sin(alpha), 2) * pow(sin(beta), 2) - 1
        quotient = numerator/denominator
        projection = sqrt(quotient)
        return projection


    def assign_constants(self):
        """ 
            Assigns all constants when called for the class to use. 
        """
        # Read a config file to get these constants
        self.M = 4 # Number of propeller blades
        self.R = 0.12 # Radius of propellers in meters (m)
        self.A = self.M * pi * pow(self.R, 2) # Area of the propeller m^2
        self.N = 2 # Number of blades per propeller        
        # c: blade chord width 
        self.c = 0.0157 # meters(m) ask alain for accurate value from cad model, fix
        # cd_blade: drag coefficient of blade
        self.cd_blade = 0.012 # unitless verify with Alain, currently selected a random approximation   
    
    #####################    
    # Power Computation #
    #####################

    def induced_power(self, drone):
        """ 
            Computes Induced Power: Power required to overcome
            the force of gravity to keep the aircraft in the air.
            Measured in Watts(W), returns power in W
        """
        if self.drone_flight_mode == "Hover":
            # Induced Power When hovering
            # Formula: P_induced = Thrust * Velocity_Induced
            net_rotor_thrust = self.get_net_thrust(drone)
            velocity_induced = self.get_velocity_induced(drone)
            induced_power = net_rotor_thrust * velocity_induced
            if DronePowerModel.DEBUG:
                print("Induced Power (Flight Mode):", drone,
                      "\nNet Thrust(N):", net_rotor_thrust,
                      "\nVelocity Induced (m/s):", velocity_induced,
                      "\nInduced Power (W):", induced_power, "\n"
                      )
            return induced_power
        elif self.drone_flight_mode == "Flight":
            # Induced Power When In Flight
            # Induced_Power = Thrust(N) * (Velocity_Air * Projection_Quad_Body + Velocity_Induced)
            # Watts (joules/second -> work/time -> Force * Displacemnt / Time) 
            # = Newtons(N) * (m/s * projection (unitless, angles in radians) * m/s)
            net_rotor_thrust = self.get_net_thrust(drone)   # Newtons Kg * m/s^2
            velocity_air = self.get_velocity_air(drone)     # m/s
            projection = self.calculate_projection(drone)   # unitless
            velocity_induced = self.get_velocity_induced(drone) # m/s
            induced_power = net_rotor_thrust * (velocity_air * projection + velocity_induced)
            if DronePowerModel.DEBUG:
                print("Induced Power (Flight Mode):", drone,
                      "\nNet Thrust(N):", net_rotor_thrust,
                      "\nAir Velocity(m/s):", velocity_air,
                      "\nProjetion (unitless):", projection,
                      "\nVelocity Induced (m/s):", velocity_induced,
                      "\nInduced Power (W):", induced_power, "\n"
                      )
            return induced_power
    

    def profile_power(self, drone):
        """ 
            Calculates Profile Power which is the power required to overcome
            the drag from the rotating propeller blades. Output of power is in Watts. 
        """
        air_density = self.airsim_state[drone]["AirDensity"] # self.airsim_client.simGetGroundTruthEnvironment(drone).air_density # kg/m^3
        if self.drone_flight_mode == "Hover":
            # Profile Power calculation for hover of all propellers
            # (N * c * cd_blade * air_density * R^4) / 8 * angular_speed^3
            avg_angular_velocity_rpm = np.mean(self.get_angular_speed(drone))
            avg_angular_velocity_mps = self.rpm_to_mps(avg_angular_velocity_rpm)
            avg_angular_speed_mps = abs(avg_angular_velocity_mps)
            profile_power = ( ( self.N * self.c * self.cd_blade * air_density * pow(self.R, 4) ) / ( 8 * pow(avg_angular_speed_mps, 3) ) )
            if DronePowerModel.DEBUG:
                print("Profile Power (Hover Mode):", drone, 
                      "\nAir Density (kg/m^3):", air_density,
                      "\nAverage Angular Speed (m/s):", avg_angular_speed_mps,
                      "\nNumber of Blades:", self.N,
                      "\nChord Length (m):", self.c,
                      "\nBlade Drag Coefficient:", self.cd_blade,
                      "\nBlade Radius (m):", self.R,
                      "\nProfile Power (W):", profile_power, "\n"
                      )
            return profile_power
        elif self.drone_flight_mode == "Flight":
            # C2 = (N * c * cd_blade * air_density * R^4) / 8*k^(3/2)
            # C3 = (N * c * cd_blade * air_density * R^2) / 8*k^(3/2)
            # k = Thrust/(angular speed)^2 # TODO K is calculated at an instant rather than through experiments
            # profile_power = C2*T^(3/2) + C3 * (Vair_jk^2 + Vair_ik^2) * T^(1/2)
            #     
            thrust = self.get_net_thrust(drone)
            avg_angular_velocity_rpm = np.mean(self.get_angular_speed(drone))
            avg_angular_velocity_mps = self.rpm_to_mps(avg_angular_velocity_rpm)
            avg_angular_speed_mps = abs(avg_angular_velocity_mps)
            k = thrust/(avg_angular_speed_mps**2)
            c2 = (self.N * self.c * self.cd_blade * air_density * self.R**4) / (8 * k**(3/2))
            c3 = (self.N * self.c * self.cd_blade * air_density * self.R**2) / (8 * k**(1/2))
            orientation = self.get_drone_orientation(drone)
            alpha = orientation['radians'][1] # gets pitch aka angle of attack radians
            beta = orientation['radians'][2] # gets yaw aka beta/steering angle randians
            projection_jk = sqrt( ( pow(sin(alpha), 2) * pow(sin(beta), 2) - pow(sin(beta), 2) ) / (pow(sin(alpha), 2) * pow(sin(beta), 2) - 1) )
            projection_ik = ( cos(alpha) * cos(beta) ) / sqrt( pow(sin(alpha), 2) * pow(cos(beta), 2) + pow(cos(alpha), 2) )  
            profile_power = c2 * thrust**(3/2) + c3 * (projection_jk**2 + projection_ik**2) * thrust**(1/2)
            if DronePowerModel.DEBUG:
                print("Profile Power (Flight Mode):", drone,
                      "\nAir Density (kg/m^3):", air_density,
                      "\nAverage Angular Speed (m/s):", avg_angular_speed_mps,
                      "\nNumber of Blades:", self.N,
                      "\nChord Length (m):", self.c,
                      "\nBlade Drag Coefficient:", self.cd_blade,
                      "\nBlade Radius (m):", self.R,
                      "\nAlpha Angle (rad):", alpha,
                      "\nBeta Angle (rad):", beta,
                      "\nProjection on jk-plane:", projection_jk,
                      "\nProjection on ik-plane:", projection_ik,
                      "\nk coefficient:", k,
                      "\nProfile Power (W):", profile_power, "\n"
                      )
            return profile_power
            
    
    def parasitic_power(self, mode, drone_name=""):
        """ 
            Parasitic Power Calculation is in Watts.
            Params:
            mode: refers to whetehr to perform calculation for entire "Swarm" or a "Single Drone"
            drone_name: if you select "Single Drone", give the name of the drone you want to calculate
            energy consumption
        """
        if mode == "Swarm":
            if self.drone_flight_mode == "Hover":
                # Formula: F_Drag * Air_velocity = 0.5 * Coef_drag * air_density * A_ref * Air_velocity^3
                coef_drag = 1.139 # TODO Follow up and verify this is good
                A_ref = 0.015 # Gathered from CAD model projection m^2
                net_parasitic_power = 0
                if DronePowerModel.DEBUG:
                    print("Parasitic Power (Swarm Hover Mode): ")
                for drone in self.drones_in_swarm:
                    velocity_air = self.get_velocity_air(drone)
                    air_density = self.airsim_state[drone]["AirDensity"]
                    parasitic_power = 0.5 * coef_drag * air_density * A_ref * pow(velocity_air, 3)
                    net_parasitic_power += parasitic_power
                    if DronePowerModel.DEBUG:
                        print("Drone:", drone,
                         "\nAir Velocity (m/s):", velocity_air, 
                        "\nParasitic Power of Drone (W):", parasitic_power
                        )
                if DronePowerModel.DEBUG:
                    print("Air Density (kg/m^3):", air_density,
                          "\ncoef_drag:", coef_drag,
                          "\nA_ref (m^2):", A_ref,
                          "\nNet Parasitic Power (W):", net_parasitic_power, "\n"
                          )
                return net_parasitic_power
            elif self.drone_flight_mode == "Flight":
                # Formula: F_Drag * Air_velocity = 0.5 * Coef_drag * air_density * A_ref * Air_velocity^3
                # We use a polynomial function that fits the data collected of the drone swarm to get the formula results above
                parasitic_power = self.get_parasitic_power()
                if DronePowerModel.DEBUG:
                    print("Parasitic Power (Swarm Flight Mode): ", 
                          "\nParasitic Power (W):", parasitic_power, "\n"
                          )
                return parasitic_power
        elif mode == "Single Drone":
            if self.drone_flight_mode == "Hover":
                # Formula: F_Drag * Air_velocity = 0.5 * Coef_drag * air_density * A_ref * Air_velocity^3
                coef_drag = 1.139 # TODO Follow up and verify this is good
                air_density = self.airsim_state[drone_name]["AirDensity"]
                A_ref = 0.015 # Gathered from CAD model projection m^2
                velocity_air = self.get_velocity_air(drone_name)
                parasitic_power = 0.5 * coef_drag * air_density * A_ref * pow(velocity_air, 3)
                if DronePowerModel.DEBUG:
                    print("Parasitic Power (Single Drone Hover Mode):",
                          "\nDrone:", drone_name,
                          "\nAir Velocity (m/s):", velocity_air,
                          "Air Density (kg/m^3):", air_density,
                          "\ncoef_drag:", coef_drag,
                          "\nA_ref (m^2):", A_ref,
                          "\nNet Parasitic Power (W):", parasitic_power, "\n"
                          )
                return parasitic_power
            elif self.drone_flight_mode == "Flight":
                # Formula: F_Drag * Air_velocity = 0.5 * Coef_drag * air_density * A_ref * Air_velocity^3
                # We use a polynomial function that fits the data collected of the drone swarm to get the formula results above
                # parasitic_power = self.get_parasitic_power()
                # if DronePowerModel.DEBUG:
                #    print("Parasitic Power (Swarm Flight Mode): ", 
                #          "\nParasitic Power (W):", parasitic_power, "\n"
                #          )
                # return parasitic_power
                # TODO Implement
                pass


    def drone_power_consumption_model(self, operation_mode, wind_vector, wind_direction, drone, airsim_state):
        """ 
            Compute Power consumption of a drone not in a swarm, this 
            should be called at the rate of once a second
        """
        self.set_operation_state(operation_mode)
        self.set_wind_vector(wind_vector) # Create a function to extract and return wind direction 
        self.wind_direction = wind_direction # Front, Left, Right
        self.airsim_state = airsim_state
        # This computes total_power(drone_i) = induced_power(drone_i) + profile_power(drone_i) for flight mode
        # Computes total_power(drone_i) = induced_power(drone_i) + profile_power(drone_i) + parasitic_power(drone_i) for flight mode
        # The result at this step should be summation of all power of drones in swarm 
        total_power = 0
        if self.drone_flight_mode == "Hover":
            total_power += self.induced_power(drone) + self.profile_power(drone) + self.parasitic_power("Single Drone", drone_name=drone)
            if DronePowerModel.DEBUG:
                print("Total Power (Mode Hover)(Single):", total_power, "Watts", "\n")
        elif self.drone_flight_mode == "Flight":
            total_power += self.induced_power(drone) + self.profile_power(drone) + self.parasitic_power("Single Drone", drone_name=drone)
            if DronePowerModel.DEBUG:
                print("Total Power (Mode Flight)(Single):", total_power, "Watts", "\n")
        return total_power
    
    
    def swarm_power_consumption_model(self, operation_mode, formation, wind_vector, wind_direction, drones_in_swarm, airsim_state):
        """ 
            Compute Power consumption of entire swarm, this 
            should be called at the rate of once a second
        """
        self.set_operation_state(operation_mode)
        self.set_wind_vector(wind_vector) # Create a function to extract and return wind direction 
        self.wind_direction = wind_direction # Front, Left, Right
        self.set_drones_in_swarm(drones_in_swarm)
        self.set_formation(formation)
        self.airsim_state = airsim_state
        # This computes total_power(drone_i) = induced_power(drone_i) + profile_power(drone_i) for flight mode
        # Computes total_power(drone_i) = induced_power(drone_i) + profile_power(drone_i) + parasitic_power(drone_i) for flight mode
        # The result at this step should be summation of all power of drones in swarm 
        total_power = 0
        if self.drone_flight_mode == "Hover":
            for drone in drones_in_swarm:
                total_power += (self.induced_power(drone) + self.profile_power(drone))
            total_power += self.parasitic_power("Swarm")
            if DronePowerModel.DEBUG:
                print("Total Power (Mode Hover)(Swarm):", total_power, "Watts", "\n")
        elif self.drone_flight_mode == "Flight":
            for drone in drones_in_swarm:
                total_power += (self.induced_power(drone) + self.profile_power(drone))
            total_power += self.parasitic_power("Swarm")
            if DronePowerModel.DEBUG:
                print("Total Power (Mode Flight)(Swarm):", total_power, "Watts", "\n")
        return total_power        


class QuadEnergyModel:
    """ Utilized to collect data on quadcopter dynamics """
    

    def __init__(self):
        # Drone Information
        self.drone_names_list = []        
        self.formation_size = 4
        # Setup Environment Parameters
        self.airsim_setup()
        self.initial_position = np.array([])
        self.wind_vector = (0,10,0) #(NED), East+ (0,1,0), North+ (1,0,0), North- (-1,0,0)
        self.wind_direction = "East+" 
    

    def airsim_setup(self):
        """
            Setup the airsim connection and initialize drones
        """
        # start drone client
        self.airsim_client = airsim.MultirotorClient()
        # Check connectivity

        self.airsim_client.confirmConnection()   
        for drone_id in range(1, self.formation_size+1):
            drone_name = "Drone" + str(drone_id)
            self.drone_names_list.append(drone_name)
            self.airsim_client.enableApiControl(True, drone_name)
            self.airsim_client.armDisarm(True, drone_name)
    
    # ################ #
    # Getter Functions #   
    # ################ #

    def get_absolute_poses_NED(self):
        """
            Gets the the global position of all the drones in 
            reference to an absolute global frame of reference.
            Returns a list of all drones ordered from Drone1 - DroneN
        """
        drone_poses_NED = []
        for drone_name in self.drone_names_list:
             # simgetobjectpose returns a pose in the world frame
            drone_pose_WFNED = self.airsim_client.simGetObjectPose(drone_name)
            drone_poses_NED.append( drone_pose_WFNED )
            
        # print("Test: Verifying printing of drones positions in global frame - ", drone_poses_NED)
        return drone_poses_NED
    

    def get_pose_position(self, pose):
        """
        Get the positional information from a pose as numpy array 
        """
        return np.array([pose.position.x_val, pose.position.y_val])
    
    
    def get_pose_altitude(self, pose):
        """
        Get the positional information from a pose as numpy array 
        """
        return np.array([pose.position.z_val])
        
    
    def get_current_drone_positions(self):
        """ Retruns the current drone positions in the NED global coord frame """
        pose_list = self.get_absolute_poses_NED()
        position_matrix = []
        for pose in pose_list:
            position = self.get_pose_position(pose)
            position_matrix.append(position)
        position_matrix = np.array(position_matrix)
        # print(position_matrix)
        return position_matrix
    
    
    def get_desired_position(self):
        desired_final_positions_K = np.array([  
                                      [10, 0], # Drone 1
                                      [0, 0],
                                      [0, 10],
                                      [10, 10]
                                    ])
        return desired_final_positions_K
    

    def get_desired_altitude_NED(self):
        """ Returns Desired NED Altitude """
        desired_altitude = np.array([  
                                -5,
                                -5,
                                -5,
                                -5
                            ])
        return desired_altitude


    def get_drone_altitude(self):
        pose_list = self.get_absolute_poses_NED()
        altitude_list = []
        for pose in pose_list:
            altitude = self.get_pose_altitude(pose)
            print(altitude)
            altitude_list.append(altitude[0])
        altitude_list = np.array(altitude_list)
        return altitude_list
            
    
    # ############### #
    # Data Collection #
    # ############### #

    def copy_drone_swarm_data(self):
        """ 
            Copies data of current drone swarm to be used by power model.
            This is to prevent blocking slow down in the control system.
            General Structure:
                drone_state_dict["drone"] = {
                                        "MultiRotorState": multirotor_state,
                                        "RotorState": rotor_state_info,
                                        "PoseWFNED": drone_pose_WFNED,
                                        "AirDensity": air_density
                                        }
        """
        multirotor_states = None
        rotor_states = None
        drone_pose_WFNED = None
        air_density = None        
        drone_state_dict = {}
        for drone in self.drone_names_list:
            multirotor_state = self.airsim_client.getMultirotorState(vehicle_name=drone)
            rotor_state_info = self.airsim_client.getRotorStates(vehicle_name=drone)
            drone_pose_WFNED = self.airsim_client.simGetObjectPose(drone)
            air_density = self.airsim_client.simGetGroundTruthEnvironment(drone).air_density
            drone_state_dict[drone] = {
                                        "MultiRotorState": multirotor_state,
                                        "RotorState": rotor_state_info,
                                        "PoseWFNED": drone_pose_WFNED,
                                        "AirDensity": air_density
                                        }
        return drone_state_dict            


    def drone_data_collection(self):
        """ Prints out data to the console of Drone1. """
        #drone_name = "Drone1"
        #multirotor_state = self.airsim_client.getMultirotorState(vehicle_name=drone_name)
        #rotor_state_info = self.airsim_client.getRotorStates(vehicle_name=drone_name)
        #drone_pose_WFNED = self.airsim_client.simGetObjectPose(drone_name)
        
        dpm = DronePowerModel()
        # This Power Calculation should be executed once a second
        copied_airsim_state_data = self.copy_drone_swarm_data()
        # State: Hover or Flight
        #power_calculation_thread = Thread(target=dpm.power_model, args=("Flight", "Vee", self.wind_vector,"East+", self.drone_names_list, copied_airsim_state_data))
        #power_calculation_thread.start()
        #power_usage = dpm.swarm_power_consumption_model("Hover", "Vee", self.wind_vector,"East+", self.drone_names_list, copied_airsim_state_data)
        power_usage =dpm.drone_power_consumption_model("Hover", self.wind_vector, "East+", "Drone1", copied_airsim_state_data)
        return power_usage
        #print("MultiRotor State Information:", multirotor_state)
        #print("************************************************\n")
        #print("Rotor State Information:", rotor_state_info, "\n")
        #print("**********************************************************")
        #print("WNED of drone", drone_pose_WFNED, "\n")
        #print("**********************************************************")
        # print("Drone Energy Usage in Watts", power_usage, "\n")
        #print("**********************************************************")
   
    # ################## #
    # Plotting Functions #
    # ################## #
    def update_plot(self, x, y1, y2):
        """
            Updates plots
        """
        # , self.position_data, self.power_data
        self.iteration_time_step.append(x)
        self.position_data.append(y1)
        self.power_data.append(y2)
        self.line1.set_data(self.iteration_time_step, self.position_data)
        self.line2.set_data(self.iteration_time_step, self.power_data)

        # Adjust limits
        self.position_ax1.relim()
        self.position_ax1.autoscale_view()
        self.power_ax2.relim()
        self.power_ax2.autoscale_view()

        self.fig.canvas.draw()
        plt.pause(0.1)
        self.fig.canvas.flush_events()

    # ############## #
    # Motion Control #
    # ############## #

    def drone_motion_control_alt(self, altitude_control_signal):
        """Assigns a set velocity for the drones to reach their desired destination. """
        #for index, drone_name in enumerate(self.drone_names_list):
        drone_name = "Drone1"
        self.airsim_client.moveByVelocityAsync(0, 0, altitude_control_signal[0], 0.1, vehicle_name=drone_name) # not working
        
        #worker_thread = Thread(target=self.drone_data_collection)
        #worker_thread.start()
        

    def takeoff_swarm(self):
        print(self.drone_names_list)
        for drone_name in self.drone_names_list:
            self.airsim_client.takeoffAsync(vehicle_name=drone_name).join()
       
            
    def drone_motion_control(self, tau_dot):
        """Assigns a set velocity for the drones to reach their desired destination. """
        for index, drone_name in enumerate(self.drone_names_list):
            drone_new_velocity = tau_dot[index] 
            xv = drone_new_velocity[0]
            yv = drone_new_velocity[1] 
            print("velocityL", xv, yv)
            self.airsim_client.moveByVelocityAsync(xv, yv, 0, 0.1, vehicle_name=drone_name) # not working


    def altitude_controller(self):
        # Plotting Setup
        plt.ion()
        plt.grid(True)
        #self.fig, (self.position_ax1, self.power_ax2) = plt.subplots(2,1)
        #self.iteration_time_step, self.position_data, self.power_data = [], [], []
        #self.line1, = self.position_ax1.plot([], [], 'r-', animated=True)
        #self.line2, = self.power_ax2.plot([], [], 'b-', animated=True)
        
        # Set labels and titles for each subplot
        #self.position_ax1.set_xlabel('Time (s)')
        #self.position_ax1.set_ylabel('Energy (Watts)')
        #self.position_ax1.set_title('Energy Output vs. Time')

        #self.power_ax2.set_xlabel('Time (s)')
        #self.power_ax2.set_ylabel('Position (meters)')
        #self.power_ax2.set_title('Position vs. Time')

        # Simulation and Control Parameters
        time_step = 0.1
        iteration = 0
        p_gain = 1
        d_gain = 1
        #self.takeoff_swarm()
        while True:
            # Wait for a time 
            time.sleep(time_step)
            desired_altitude = self.get_desired_altitude_NED()
            #current_drone_pose = self.get_current_drone_positions()
            current_drone_altitude = self.get_drone_altitude()
            print("current drone alti:", current_drone_altitude)
            # PID Altitude Controller 
            error = desired_altitude - current_drone_altitude # Whatever the size of the swarm desired should match
            print("Error", error)
            perror = p_gain * error
            derror = d_gain * (error/time_step)
            control_signal_altitude = perror + derror
            self.drone_motion_control_alt(control_signal_altitude)
            power = self.drone_data_collection()
            iteration = iteration + 1
            #self.update_plot(iteration, current_drone_altitude[0], power)
            
            #plt.clf()
            # Power
            plt.scatter(iteration, power, label='Power Consumption Hover (Watts)', color='red')
            #plt.scatter(5, desired_altitude[0], label='Desired Agent Altitude', color='blue')
            #plt.scatter(current_swarm_centroid[0], current_swarm_centroid[1], label='Current Centroid', color='green')
            #plt.scatter(reference_centroid[0], reference_centroid[1], label='Desired Centroid', color='black')
            #plt.xlim(-25, 25)  # Adjust the x-axis limits if needed
            #plt.ylim(-25, 25)  # Adjust the y-axis limits if needed
            plt.xlabel('Iteration')
            plt.ylabel('Power Consumption Watts')
            plt.title('Power Consumption')
            plt.grid(True)
            # Position
            #plt.scatter(5,  current_drone_altitude[0], label='Current Agent Altitude', color='red')
            #plt.scatter(5, desired_altitude[0], label='Desired Agent Altitude', color='blue')
            #plt.scatter(current_swarm_centroid[0], current_swarm_centroid[1], label='Current Centroid', color='green')
            #plt.scatter(reference_centroid[0], reference_centroid[1], label='Desired Centroid', color='black')
            #plt.xlim(-25, 25)  # Adjust the x-axis limits if needed
            #plt.ylim(-25, 25)  # Adjust the y-axis limits if needed
            #plt.xlabel('X Position')
            #plt.ylabel('Y Position')
            #plt.title('Agent Positions (Iteration {})'.format(iteration))
            #plt.grid(True)
            plt.draw()
            plt.pause(0.1)


if __name__ == "__main__":
    qem = QuadEnergyModel()
    qem.altitude_controller()