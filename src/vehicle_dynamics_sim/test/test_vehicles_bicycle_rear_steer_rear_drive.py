import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from test_vehicles import generate_test_description_for_config, TestVehicleVelocityTracking, TestVehicleVelocityTrackingShutdown


def generate_test_description(): return generate_test_description_for_config(
    'bicycle_rear_steer_rear_drive')
