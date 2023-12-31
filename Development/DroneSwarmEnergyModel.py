from email.policy import default
import numpy as np
# import matplotlib.pyplot as plt
import geopy.distance
from math import pi, sqrt, pow, atan2, asin, sin, cos
import sympy as sp


class DroneSwarmPowerModel:
    # This class models the power consumption of a drone during flight and hover 
    # for single drones and drones in swarms. 
    # It includes methods for calculating various aspects of drone energy usage 
    # based on flight dynamics and environmental factors.
    
   
    DEBUG = True
    # A class variable that controls the output of debug statements. 
    # Set to False to disable debug print statements throughout the class.

    def __init__(self):
        # Constructor for the DronePowerModel class. 
        # It initializes the class by assigning constants used in power calculations.
        # These constants are set by the 'assign_constants' method.
        self.assign_constants()
        # self.config_file_path = config_file
    
    # ########################## #
    # Energy Model of Quadcopter # 
    # ########################## #     
    # This section contains methods related to the energy model of a quadcopter.

    ####################
    # Getter Functions #
    ####################
    # These functions are used to retrieve various parameters related to the drone, 
    # such as velocity, acceleration, etc.

    def get_drone_velocity(self, drone):
        """
        Retrieves the velocity of the drone in the local NED (North-East-Down) frame.
        This method returns the velocity in meters per second (m/s).
        :param drone: The drone object whose velocity is being queried.
        :return: A NumPy array representing the drone's velocity in the NED frame.
        """
        # gets the velocity at which the drones are traveling
        if DroneSwarmPowerModel.DEBUG:
            print("\nVerifying Drone Velocity", self.airsim_state[drone]['MultiRotorState'].kinematics_estimated)
        kinematics = self.airsim_state[drone]['MultiRotorState'].kinematics_estimated
        velocity_drone = [kinematics.linear_velocity.x_val, kinematics.linear_velocity.y_val, kinematics.linear_velocity.z_val]
        return np.array(velocity_drone)
    

    def get_air_speed(self, drone):
        """
        Calculates the air speed of the drone in meters per second (m/s).
        The air velocity is the difference between the ground velocity of the drone and the wind velocity.
        :param drone: The drone object for which air speed is being calculated.
        :return: The air velocity of the drone in m/s.
        """
        # Velocity_Air = ||V_air|| = || V_ground (velocity drone is travelling) - V_wind ||
        velocity_ground = self.get_drone_velocity(drone)
        velocity_air = np.linalg.norm(velocity_ground - self.wind_vector) # TODO: mention to alain airspeed
        return velocity_air


    def get_net_thrust(self, drone):
        """
        Calculates the net thrust generated by all four propellers of the drone.
        The thrust is computed in Newtons, a unit of force (kg * m/s^2).
        :param drone: The drone name for which net thrust is being calculated.
        :return: The total net thrust produced by the drone's propellers in Newtons.
        """
        rotor_states = self.airsim_state[drone]["RotorState"] 
        net_thrust = 0
        for rotor in rotor_states.rotors:
            net_thrust += rotor['thrust']
        if DroneSwarmPowerModel.DEBUG:
            print("\nNet_thrust:", net_thrust, "for Drone:", drone)
        return net_thrust
    

    def get_thrust_propellers(self, drone):
        """
        Retrieves the thrust generated by each of the four propellers of the drone.
        This method returns the thrust values in an array, with each value in Newtons.
        :param drone: The drone object for which thrust values are being retrieved.
        :return: A NumPy array containing the thrust values of each propeller in Newtons.
        """
        rotor_states = self.airsim_state[drone]["RotorState"] 
        thrust = []
        for rotor in rotor_states.rotors:
            thrust.append(rotor['thrust'])
        if DroneSwarmPowerModel.DEBUG:
            print("\nThrust:", thrust, "for Drone:", drone)
            print("RotorStates", rotor_states.rotors, "for Drone:", drone)
        return np.array(thrust)


    def get_angular_speed(self, drone):
        """
        Calculates the angular speed of each of the four propellers of the drone.
        The speed is measured in revolutions per minute (rpm).
        :param drone: The drone object for which angular speeds are being calculated.
        :return: A NumPy array containing the angular speeds of each propeller in rpm.
        """
        rotor_states = self.airsim_state[drone]["RotorState"] 
        speed = []
        for rotor in rotor_states.rotors:
            speed.append(rotor['speed'])
        if DroneSwarmPowerModel.DEBUG:
            print("\nAngular speed:", speed, "for Drone:", drone)
        return np.array(speed)


    def get_drone_orientation(self, drone):
        """
        Retrieves the orientation of the drone in terms of roll, pitch, and yaw.
        The orientation is provided in both radians and degrees. Pitch is the Alpha 
        angle and Yaw is the Beta angle
        :param drone: The drone name for which orientation is being calculated.
        :return: A dictionary containing roll, pitch, and yaw in both radians and degrees.
        """
        drone_pose = self.airsim_state[drone]["PoseWFNED"]
        w = drone_pose.orientation.w_val
        x = drone_pose.orientation.x_val
        y = drone_pose.orientation.y_val
        z = drone_pose.orientation.z_val
        
        roll_x_rad, pitch_y_rad, yaw_z_rad = self.euler_from_quaternion(x, y, z, w) # radians
        roll_x_deg, pitch_y_deg, yaw_z_deg = np.rad2deg(roll_x_rad), np.rad2deg(pitch_y_rad), np.rad2deg(yaw_z_rad) # degrees
        return {"radians": [roll_x_rad, pitch_y_rad, yaw_z_rad], "degrees": [roll_x_deg, pitch_y_deg, yaw_z_deg]}
    

    def calc_Vi(self, v_inf, alpha, T, omega, r, rho, A):
        """
        Induced velocity calculator for a single rotor
        Parameters
        ----------
        v_inf : float
            Airspeed [m/s]
        alpha : math.radians
            Blade angle of attack [deg] (the same value of drone angle of attack, for simplicity)
        T : float
            Rotor thrust [N]
        omega : float
            Rotational speed of rotor [rad/s]
        r :float
            Rotor radius [m]
        rho : float
            Air density [kg/m3]
        A : float
            Single rotor circle area depicted by the propeller rotation [m2]
        """
        v_i = sp.symbols('v_i')
        lambda_h = np.sqrt(T/(2*rho*A))
        mu = (v_inf*np.cos(alpha))/(omega*r)
        eq = v_i ** 4 + lambda_h * v_i ** 2 + 0.25 * (pow(mu,2) * pow(lambda_h,2) - pow(lambda_h,4)) - pow(lambda_h,4)
        roots = [x for x in sp.solve(eq,v_i) if x.is_real == True and x > 0]
        return roots[0] - v_inf * np.sin(alpha)
    

    def get_induced_velocity(self, drone):
        """
        Calculates the induced velocity of the drone, which differs based on its flight mode.
        Their are currently two modes 'Hover' and 'Flight' with each different calculations
        :param drone: The drone name for which induced velocity is being calculated.
        :return: The induced velocity of the drone in meters per second (m/s).
        """
        
        # Get Common Varaibles
        thrust = self.get_net_thrust(drone)
        air_density = self.airsim_state[drone]["AirDensity"] # kg/m^3
        # Calculate induced velocity when hovering
        if self.drone_flight_mode == "Hover":
            # velocity_induced = sqrt( Thrust / 2 * air_density * A (Total Area Covered By Propellers) )
            # A = M * pi * R^2 
            # M is nuber of propellers
            # R is radius of propellers
            velocity_induced = sqrt( thrust / (2 * air_density * self.A) ) # meters per second (m/s)
            return velocity_induced
        elif self.drone_flight_mode == "Flight":
            # Induced velocity we can represent as the ground velocity of the drone while in flight.
            air_speed = self.get_air_speed(drone)
            orientation = self.get_drone_orientation(drone)
            alpha = orientation['radians'][1] # gets pitch aka angle of attack radians
            mean_thrust = thrust/4
            rotor_speed = self.get_angular_speed(drone)
            mean_rotor_speed = np.mean(rotor_speed)
            mean_rotor_speed_rps = self.mps_to_radians_per_sec(mean_rotor_speed)
            velocity_induced = self.calc_Vi(air_speed, alpha, mean_thrust, mean_rotor_speed_rps, self.R, air_density, self.A) # m/s
            if DroneSwarmPowerModel.DEBUG:
                print("Induced Velocity Calculation:\n", 
                        "\nAir Speed (m/s):", air_speed,
                        "\nAlpha (radians):", alpha,
                        "\nMean Thrust:", mean_thrust,
                        "\nMean Rotor Speed (rps)", mean_rotor_speed_rps,
                        "\nVelocity Induced", velocity_induced,
                        )
            return velocity_induced
        
        
    def get_swarm_parasitic_power_consumption(self):
        """
        Computes the total power consumption in Watts given a particular formation and wind vector.
        The calculation takes into account various factors such as formation, wind, thrust, drag, and velocity.
        :return: The total power consumption of the swarm formation in Watts.
        """
        
        wind_speed = np.linalg.norm(self.wind_vector)
        wind_direction = self.wind_direction # N, E, S, W - Cardinal Directions
        # Assigns sign of wind direction
        wind_velocity = wind_speed
        if wind_direction == "W":
            wind_velocity = -1 * wind_speed
        elif wind_direction == "S":
            wind_velocity = 0 * wind_speed

        if DroneSwarmPowerModel.DEBUG:
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
            if wind_direction == "W" or wind_direction == "E":
                return self.cubic(wind_velocity, a_x , b_x, c_x, d_x)
            else:
                return self.quadratic(wind_velocity, a_z , b_z, c_z)
        elif self.formation == "Echelon":
            # Cubic estimation of power-x
            a_x , b_x, c_x, d_x = (0.06068811, -0.06580198, 4.25524982, 1.32457763)
            # Quadratic stimation for power-z
            a_z , b_z, c_z = (0.74663132, -4.73078804, 6.92984925)
            if wind_direction == "W" or wind_direction == "E":
                return self.cubic(wind_velocity, a_x , b_x, c_x, d_x)
            else:
                return self.quadratic(wind_velocity, a_z , b_z, c_z)
        elif self.formation == "Column":
            # Cubic estimation of power-x
            a_x , b_x, c_x, d_x = (0.04591221, -0.00451343,  1.36092396, -0.24269214)
            # Quadratic stimation for power-z
            a_z , b_z, c_z = (0.55367928, -2.25427749,  3.68723189)
            if wind_direction == "W" or wind_direction == "E":
                return self.cubic(wind_velocity, a_x , b_x, c_x, d_x)
            else:
                return self.quadratic(wind_velocity, a_z , b_z, c_z )
        elif self.formation == "Front":
            # Cubic estimation of power-x
            a_x , b_x, c_x, d_x = (0.03644207, -0.00831357,  1.32394737, -0.34071768)
            # Quadratic stimation for power-z
            a_z , b_z, c_z = (0.69623598, -4.36917437,  6.34302308)
            if wind_direction == "W" or wind_direction == "E":
                return self.cubic(wind_velocity, a_x , b_x, c_x, d_x)
            else:
                return self.quadratic(wind_velocity, a_z , b_z, c_z)
        elif self.formation == "Diamond":
            # Cubic estimation of power-x
            a_x , b_x, c_x, d_x = (0.02903825, -0.02000249,  0.05777605,  0.52897958)
            # Quadratic stimation for power-z
            a_z , b_z, c_z = (0.72696461, -3.91278085, 6.23165963)
            if wind_direction == "W" or wind_direction == "E":
                return self.cubic(wind_velocity, a_x , b_x, c_x, d_x)
            else:
                return self.quadratic(wind_velocity, a_z , b_z, c_z)


    ####################
    # Setter Functions #
    ####################

    def set_operation_state(self, state):
        """
        Sets the operational state of the drone, either 'Hover' or 'Flight'.
        This state affects various calculations within the class.
        :param state: The operation state to be set for the drone ('Hover' or 'Flight').
        """
        self.drone_flight_mode = state
        
        
    def set_wind_vector(self, wind_vector):
        """
        Sets the current wind vector that affects the drone and swarm. 
        The wind vector is specified in the NED (North-East-Down) coordinate system,
        this is the airsim world NED.
        :param wind_vector: The wind vector to be set, represented as a NumPy array or a list (N,E,D).
        """
        self.wind_vector = wind_vector
     
        
    def set_formation(self, formation):
        """
        Specifies the formation in which the drones are flying, such as 'Echelon', 'Vee', etc.
        The formation type may affect various calculations related to aerodynamics and power consumption.
        :param formation: A string representing the name of the formation.
        """
        self.formation = formation
        
    def set_drones_in_swarm(self, drones_in_swarm):
        """
        Assigns the number of drones that are part of the swarm.
        This information is used in calculations that depend on the number of drones in operation.
        :param drones_in_swarm: The number of drones in the swarm.
        """
        self.drones_in_swarm = drones_in_swarm
        
    ###################################
    # Polynomial Regression Functions #
    ###################################

    def linear(self, x, a, b):
        """
        A linear polynomial function used for various calculations within the class.
        :param x: The independent variable.
        :param a: The coefficient of x.
        :param b: The constant term.
        :return: The result of the linear polynomial a*x + b.
        """
        return a*x + b
    
    
    def quadratic(self, x, a, b, c):
        """
        A quadratic polynomial function used for various calculations within the class.
        :param x: The independent variable.
        :param a: The coefficient of x^2.
        :param b: The coefficient of x.
        :param c: The constant term.
        :return: The result of the quadratic polynomial a*x^2 + b*x + c.
        """
        return a*x**2 + b*x + c
    
    
    def cubic(self, x, a, b, c, d):
        """
        A cubic polynomial function used for various calculations within the class.
        :param x: The independent variable.
        :param a: The coefficient of x^3.
        :param b: The coefficient of x^2.
        :param c: The coefficient of x.
        :param d: The constant term.
        :return: The result of the cubic polynomial a*x^3 + b*x^2 + c*x + d.
        """
        return a*x**3+b*x**2+c*x + d

    ##################
    # Misc Functions #
    ##################

    def rpm_to_mps(self, rpm):
        """
        Converts revolutions per minute (rpm) to meters per second (m/s).
        This conversion is useful for translating propeller speed to linear velocity.
        :param rpm: The rotational speed in revolutions per minute.
        :return: The equivalent linear speed in meters per second.
        """
        return self.R * ((2 * pi)/60) * rpm
    

    def mps_to_radians_per_sec(self, mps):
        """
        Converts meters per second to radians per second
        """
        return (mps/self.R)


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
        Computes the projection \(p_l\) of the specified drone.
        This involves calculations based on the drone's pitch (angle of attack) and yaw (steering angle).
        :param drone: The drone name for which the projection is being calculated.
        :return: The calculated projection value.
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
        Assigns all constants used by the class. This method initializes 
        various parameters related to the drone's physical and aerodynamic 
        properties. Constants include the number of propeller blades,
        radius of propellers, area of the propeller, blade chord width,
        and drag coefficient of the blade.
        """
        # TODO: Read a config file to get these constants
        self.M = 4                              # Number of propeller blades
        self.R = 0.12                           # Radius of propellers in meters (m)
        self.A = self.M * pi * pow(self.R, 2)   # Area of the propeller m^2
        self.N = 2                              # Number of blades per propeller        
        self.c = 0.0157                         # blade chord width in meters(m) 
        self.cd_blade = 0.012                   # drag coefficient of blade, unitless   
    
    #####################    
    # Power Computation #
    #####################

    def induced_power(self, drone):
        """
        Computes the induced power of the drone, which is the 
        power required to overcome the force of gravity and 
        keep the aircraft airborne. The calculation differs 
        based on whether the drone is hovering or in flight.
        In 'Hover' mode, induced power is calculated as the 
        product of net rotor thrust and velocity induced.
        In 'Flight' mode, it considers additional factors 
        like air velocity and projection of the quad body.
        
        :param drone: The drone name for which induced power 
        is being calculated.
        
        :return: The induced power required for the drone's 
        current operation mode, measured in Watts (W).
        """
        if self.drone_flight_mode == "Hover":
            # Induced Power When hovering
            # Formula: P_induced = Thrust * Velocity_Induced
            net_rotor_thrust = self.get_net_thrust(drone)
            velocity_induced = self.get_induced_velocity(drone)
            induced_power = net_rotor_thrust * velocity_induced
            if DroneSwarmPowerModel.DEBUG:
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
            velocity_air = self.get_air_speed(drone)     # m/s
            projection = self.calculate_projection(drone)   # unitless
            velocity_induced = self.get_induced_velocity(drone) # m/s
            induced_power = net_rotor_thrust * (velocity_air * projection + velocity_induced)
            if DroneSwarmPowerModel.DEBUG:
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
        Calculates the profile power of the drone, which is the power consumed 
        due to the aerodynamic drag on the rotating propeller blades.
        The calculation is based on various factors including thrust, air density, 
        angular speed of propellers, and drone orientation.
        :param drone: The drone name for which profile power is being calculated.
        :return: The profile power of the drone in Watts (W).
        """
        air_density = self.airsim_state[drone]["AirDensity"] # kg/m^3
        if self.drone_flight_mode == "Hover":
            # Profile Power calculation for hover of all propellers
            # (N * c * cd_blade * air_density * R^4) / 8 * angular_speed^3
            avg_angular_velocity_rpm = np.mean(self.get_angular_speed(drone))
            avg_angular_velocity_mps = self.rpm_to_mps(avg_angular_velocity_rpm)
            avg_angular_speed_mps = abs(avg_angular_velocity_mps)
            profile_power = ( ( self.N * self.c * self.cd_blade * air_density * pow(self.R, 4) ) / ( 8 * pow(avg_angular_speed_mps, 3) ) )
            if DroneSwarmPowerModel.DEBUG:
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
            if DroneSwarmPowerModel.DEBUG:
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
        Calculates the parasitic power, which is the power consumed due to 
        aerodynamic drag force on the drone.This method can calculate the 
        parasitic power for either a single drone or the entire swarm. 
        Do not use this to calculate individual drone drag while in a swarm.
        :param mode: A string indicating whether to calculate for the entire 'Swarm' or a 'Single Drone'.
        :param drone_name: The name of the drone, required if 'mode' is set to 'Single Drone'.
        :return: The parasitic power consumption in Watts.
        """
        if mode == "Swarm":
            if self.drone_flight_mode == "Hover":
                # Formula: F_Drag * Air_velocity = 0.5 * Coef_drag * air_density * A_ref * Air_velocity^3
                coef_drag = 1.139 # TODO Follow up and verify this is good
                A_ref = 0.015 # Gathered from CAD model projection m^2
                net_parasitic_power = 0
                if DroneSwarmPowerModel.DEBUG:
                    print("Parasitic Power (Swarm Hover Mode): ")
                for drone in self.drones_in_swarm:
                    velocity_air = self.get_air_speed(drone)
                    air_density = self.airsim_state[drone]["AirDensity"]
                    parasitic_power = 0.5 * coef_drag * air_density * A_ref * pow(velocity_air, 3)
                    net_parasitic_power += parasitic_power
                    if DroneSwarmPowerModel.DEBUG:
                        print("Drone:", drone,
                         "\nAir Velocity (m/s):", velocity_air, 
                        "\nParasitic Power of Drone (W):", parasitic_power
                        )
                if DroneSwarmPowerModel.DEBUG:
                    print("Air Density (kg/m^3):", air_density,
                          "\ncoef_drag:", coef_drag,
                          "\nA_ref (m^2):", A_ref,
                          "\nNet Parasitic Power (W):", net_parasitic_power, "\n"
                          )
                return net_parasitic_power
            elif self.drone_flight_mode == "Flight":
                # Formula: F_Drag * Air_velocity = 0.5 * Coef_drag * air_density * A_ref * Air_velocity^3
                # We use a polynomial function that fits the data collected of the drone swarm to get the formula results above
                parasitic_power = self.get_swarm_parasitic_power_consumption()
                if DroneSwarmPowerModel.DEBUG:
                    print("Parasitic Power (Swarm Flight Mode): ", 
                          "\nParasitic Power (W):", parasitic_power, "\n"
                          )
                return parasitic_power
        elif mode == "Single Drone":
            if self.drone_flight_mode == "Hover" or self.drone_flight_mode == "Flight":
                # Formula: F_Drag * Air_velocity = 0.5 * Coef_drag * air_density * A_ref * Air_velocity^3
                coef_drag = 1.139 # TODO Follow up and verify this is good
                air_density = self.airsim_state[drone_name]["AirDensity"]
                A_ref = 0.015 # Gathered from CAD model projection m^2
                velocity_air = self.get_air_speed(drone_name)
                parasitic_power = 0.5 * coef_drag * air_density * A_ref * pow(velocity_air, 3)
                if DroneSwarmPowerModel.DEBUG:
                    print("Parasitic Power (Single Drone Hover/Flight Mode):",
                          "\nDrone:", drone_name,
                          "\nAir Velocity (m/s):", velocity_air,
                          "Air Density (kg/m^3):", air_density,
                          "\ncoef_drag:", coef_drag,
                          "\nA_ref (m^2):", A_ref,
                          "\nNet Parasitic Power (W):", parasitic_power, "\n"
                          )
                return parasitic_power


    def drone_power_consumption_model(self, operation_mode, wind_vector, wind_direction, drone, airsim_state):
        """
        Computes the power consumption of a single drone, ideally called at a rate of once per second.
        This method calculates the total power consumption based on various factors, 
        including the drone's operation mode, wind conditions, and its current state.

        :param operation_mode: The operation mode of the drone, such as 'Hover' or 'Flight'.
        :param wind_vector: The current wind vector, which affects the drone's power consumption.
        :param wind_direction: The direction of the wind, which can be 'Front', 'Left', or 'Right'.
        :param drone: The specific drone object for which power consumption is being calculated.
        :param airsim_state: The current state of the AirSim environment, which provides necessary data for the calculations.

        :return: The total power consumption of the specified drone in Watts.
        The total power is the sum of induced power, profile power, and parasitic power, which vary based 
        on whether the drone is in 'Hover' or 'Flight' mode.
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
            if DroneSwarmPowerModel.DEBUG:
                print("Total Power (Mode Hover)(Single):", total_power, "Watts", "\n")
        elif self.drone_flight_mode == "Flight":
            total_power += self.induced_power(drone) + self.profile_power(drone) + self.parasitic_power("Single Drone", drone_name=drone)
            if DroneSwarmPowerModel.DEBUG:
                print("Total Power (Mode Flight)(Single):", total_power, "Watts", "\n")
        return total_power
    
    
    def swarm_power_consumption_model(self, operation_mode, formation, wind_vector, wind_direction, drones_in_swarm, airsim_state):
        """
        Models the power consumption of an entire drone swarm.
        This method takes into account various factors like operation mode, formation, 
        wind conditions, and the number of drones. This method returns total Watts
        so it should be called at a rate of once per second.
        :param operation_mode: The operation mode of the drones ('Hover' or 'Flight').
        :param formation: The formation in which the drones are flying.
        :param wind_vector: The wind vector affecting the drones.
        :param wind_direction: The direction of the wind.
        :param drones_in_swarm: The number of drones in the swarm.
        :param airsim_state: The current state of the AirSim environment.
        :return: The total power consumption of the swarm in Watts.
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
            if DroneSwarmPowerModel.DEBUG:
                print("Total Power (Mode Hover)(Swarm):", total_power, "Watts", "\n")
        elif self.drone_flight_mode == "Flight":
            for drone in drones_in_swarm:
                total_power += (self.induced_power(drone) + self.profile_power(drone))
            total_power += self.parasitic_power("Swarm")
            if DroneSwarmPowerModel.DEBUG:
                print("Total Power (Mode Flight)(Swarm):", total_power, "Watts", "\n")
        return total_power