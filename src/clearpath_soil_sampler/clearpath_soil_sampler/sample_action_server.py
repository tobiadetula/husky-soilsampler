#!/usr/bin/env python3

import time
import csv
import os
from datetime import datetime
import statistics

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import your custom messages
from clearpath_soil_interfaces.action import Sample
from actuator_interfaces.msg import ActuatorState, ActuatorCommand
from sensor_msgs.msg import NavSatFix, Temperature
from std_msgs.msg import Float32, Float64, Bool

class SoilSampleServer(Node):
    def __init__(self):
        super().__init__('soil_sample_server')
        
        # Callback group allows the action server to run in parallel with topic subscribers
        self.cb_group = ReentrantCallbackGroup()
        
        # --- Action Server ---
        self._action_server = ActionServer(
            self,
            Sample,
            'take_soil_sample',
            self.execute_callback,
            callback_group=self.cb_group)
        
        # --- Publishers & Subscribers ---
        self.cmd_pub = self.create_publisher(ActuatorCommand, '/microros/actuator_control', 10)
        
        self.telemetry_sub = self.create_subscription(
            ActuatorState, '/microros/actuator_telemetry', self.telemetry_cb, 10, callback_group=self.cb_group)
            
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_cb, 10, callback_group=self.cb_group)

        # --- Ambient Environment Subscribers ---
        self.amb_temp_sub = self.create_subscription(
            Float32, '/microros/ambient_temperature', self.amb_temp_cb, 10, callback_group=self.cb_group)
        self.amb_hum_sub = self.create_subscription(
            Float32, '/microros/ambient_humidity', self.amb_hum_cb, 10, callback_group=self.cb_group)

        # --- TEROS 12 Subscribers ---
        self.ec_sub = self.create_subscription(
            Float64, '/teros12/sensor_0/ec', self.ec_cb, 10, callback_group=self.cb_group)
        self.vwc_sub = self.create_subscription(
            Float64, '/teros12/sensor_0/vwc', self.vwc_cb, 10, callback_group=self.cb_group)
        self.temp_sub = self.create_subscription(
            Temperature, '/teros12/sensor_0/temperature', self.temp_cb, 10, callback_group=self.cb_group)
        
        # --- Internal State Variables ---
        self.sample_attempt = 0  # Tracks how many samples have been attempted
        self.current_position = 0
        self.current_force = 0.0
        self.current_amps = 0.0
        self.safety_tripped = False
        
        self.current_lat = 0.0
        self.current_lon = 0.0
        
        self.current_ambient_temp = 0.0
        self.current_ambient_hum = 0.0

        # TEROS 12 state variables
        self.current_ec = 0.0
        self.current_vwc = 0.0
        self.current_temp = 0.0
        
        self._killed = False
        self._active_goal_handle = None
        self.create_subscription(Bool, '/mission/kill_switch', self._kill_cb, 10,
                                callback_group=self.cb_group)

        # --- Setup CSV Logging ---
        # We now use TWO files: one for the summary stats, one for the raw time-series plots
        self.csv_file = os.path.expanduser('~/soil_samples_log.csv')
        self.raw_csv_file = os.path.expanduser('~/soil_samples_raw.csv')
        self._init_csv()

        self.get_logger().info('Soil Sample Hardware Server is ready.')
        
    def _init_csv(self):
        """Creates the CSV files and writes exhaustive headers if they don't exist."""
        # 1. Main Summary Log
        if not os.path.isfile(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    'Attempt_ID', 'Timestamp', 'Latitude', 'Longitude', 
                    'Ambient_Temp_C', 'Ambient_Humidity_Pct',
                    'Position_Ticks', 'Force_Grams', 'Current_mA',
                    'VWC_Mean', 'VWC_Min', 'VWC_Max', 'VWC_Var', 'VWC_StdDev',
                    'Temp_Mean', 'Temp_Min', 'Temp_Max', 'Temp_Var', 'Temp_StdDev',
                    'EC_Mean', 'EC_Min', 'EC_Max', 'EC_Var', 'EC_StdDev'
                ])
                
        # 2. Raw Time-Series Log
        if not os.path.isfile(self.raw_csv_file):
            with open(self.raw_csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    'Attempt_ID', 'Time_Offset_s', 'VWC_Raw', 'Temp_C_Raw', 'EC_Raw'
                ])
                
    # --- Background Callbacks ---
    def telemetry_cb(self, msg):
        self.current_position = msg.position
        self.current_force = msg.applied_force
        self.current_amps = msg.motor_current
        self.safety_tripped = msg.safety_tripped

    def gps_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def amb_temp_cb(self, msg):
        self.current_ambient_temp = msg.data

    def amb_hum_cb(self, msg):
        self.current_ambient_hum = msg.data
    
    def ec_cb(self, msg):
        self.current_ec = msg.data

    def vwc_cb(self, msg):
        self.current_vwc = msg.data

    def temp_cb(self, msg):
        self.current_temp = msg.temperature
            
    def _kill_cb(self, msg):
        if msg.data and not self._killed:
            self._killed = True
            self.get_logger().warn('KILL SWITCH — aborting active sample!')
        elif not msg.data:
            self._killed = False
                   
    def execute_callback(self, goal_handle):
        self.sample_attempt += 1
        target_ticks = int(goal_handle.request.target_depth)
        
        self.get_logger().info(f'--- Attempt #{self.sample_attempt} ---')
        self.get_logger().info(f'Received sample request. Target Ticks: {target_ticks}')
        
        feedback_msg = Sample.Feedback()
        result = Sample.Result()
        self._active_goal_handle = goal_handle
        self._killed = False
        
        # --- STATE 1: INSERTION ---
        feedback_msg.current_state = "INSERTING"
        self.get_logger().info('Starting probe insertion...')
        
        cmd = ActuatorCommand()
        cmd.target_position = target_ticks
        cmd.use_auto_mode = True
        self.cmd_pub.publish(cmd)
        
        insertion_start_time = time.time()
        INSERTION_TIMEOUT_S = 60  
        
        while abs(self.current_position - target_ticks) > 10:
            if time.time() - insertion_start_time > INSERTION_TIMEOUT_S:
                self.get_logger().error('Insertion timeout — actuator never reached target.')
                feedback_msg.current_state = "ERROR: INSERTION_TIMEOUT"
                goal_handle.publish_feedback(feedback_msg)
                result.success = False
                result.message = "Insertion timed out."
                goal_handle.abort()
                return result
            
            if self.safety_tripped:
                self.get_logger().error('Safety limit breached! Hardware halted.')
                feedback_msg.current_state = "ERROR: SAFETY_TRIPPED"
                feedback_msg.current_depth = float(self.current_position)
                feedback_msg.current_draw = self.current_amps
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.1)
                
                result.success = False
                result.message = "Safety tripped during insertion."
                goal_handle.abort()
                return result
    
            if self._killed:
                self.get_logger().warn('Kill switch active — aborting sample!')
                feedback_msg.current_state = "ABORTED: KILL SWITCH"
                result.success = False
                result.message = "Aborted by kill switch."
                goal_handle.abort()
                return result

            feedback_msg.current_depth = float(self.current_position)
            feedback_msg.current_draw = self.current_amps
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)
            
        # --- STATE 2: DWELL & SENSE ---
        feedback_msg.current_state = "DWELLING"
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f'At target depth. Dwelling for {goal_handle.request.dwell_time}s...')
        
        ec_readings   = []
        vwc_readings  = []
        temp_readings = []
        raw_time_series = [] # Stores tuples: (time_offset, vwc, temp, ec)

        dwell_start = time.time()
        while time.time() - dwell_start < goal_handle.request.dwell_time:
            current_time = time.time()
            elapsed = current_time - dwell_start
            
            # Store raw exact measurements for the time-series log
            raw_time_series.append((
                elapsed,
                self.current_vwc,
                self.current_temp,
                self.current_ec
            ))
            
            # Append to lists for statistical calculations
            ec_readings.append(self.current_ec)
            vwc_readings.append(self.current_vwc)
            temp_readings.append(self.current_temp)

            feedback_msg.current_depth = float(self.current_position)
            feedback_msg.current_draw  = self.current_amps
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)  # Sample rate during dwell
            
            if self._killed:
                break

        # Compute Time Series Statistics
        def calc_stats(data_array):
            if len(data_array) > 1:
                return (
                    statistics.mean(data_array),
                    min(data_array),
                    max(data_array),
                    statistics.variance(data_array),
                    statistics.stdev(data_array)
                )
            elif len(data_array) == 1:
                return (data_array[0], data_array[0], data_array[0], 0.0, 0.0)
            return (0.0, 0.0, 0.0, 0.0, 0.0)

        vwc_stats = calc_stats(vwc_readings)
        temp_stats = calc_stats(temp_readings)
        ec_stats = calc_stats(ec_readings)
        
        self.get_logger().info(
            f'Dwell complete. {len(ec_readings)} samples captured. '
            f'VWC Mean:{vwc_stats[0]:.2f} (Var:{vwc_stats[3]:.4f})'
        )      
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # Write to the Raw Time-Series CSV
        with open(self.raw_csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            for reading in raw_time_series:
                writer.writerow([
                    self.sample_attempt,
                    f"{reading[0]:.3f}", # format seconds to 3 decimal places
                    reading[1], 
                    reading[2], 
                    reading[3]
                ])

        # Write to the Main Summary CSV
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.sample_attempt,
                timestamp, 
                self.current_lat, 
                self.current_lon, 
                self.current_ambient_temp,
                self.current_ambient_hum,
                self.current_position, 
                self.current_force, 
                self.current_amps,
                *vwc_stats,
                *temp_stats,
                *ec_stats
            ])
        self.get_logger().info(f'Data logged to {self.csv_file} and {self.raw_csv_file}')

        # --- STATE 3: RETRACTION ---
        feedback_msg.current_state = "RETRACTING"
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Retracting probe to home position...')
        
        cmd.target_position = 0
        cmd.use_auto_mode = True
        self.cmd_pub.publish(cmd)
        
        retraction_start_time = time.time()
        RETRACTION_TIMEOUT_S = 60 
        
        while abs(self.current_position) > 10:
            if time.time() - retraction_start_time > RETRACTION_TIMEOUT_S:
                self.get_logger().error('Retraction timeout — actuator never returned home.')
                break 
                
            if self._killed:
                break
                
            feedback_msg.current_depth = float(self.current_position)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)
            
        # --- COMPLETE ---
        goal_handle.succeed()
        
        # Return the Means and StdDevs to the client
        result.success = True
        result.vwc = float(vwc_stats[0])
        result.temperature = float(temp_stats[0])
        result.ec = float(ec_stats[0])
        result.vwc_stddev = float(vwc_stats[4])
        result.temperature_stddev = float(temp_stats[4])
        result.ec_stddev = float(ec_stats[4])
        result.message = "Sample collected and logged successfully."
        
        self.get_logger().info('Sample cycle complete. Waiting for next request...')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SoilSampleServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()