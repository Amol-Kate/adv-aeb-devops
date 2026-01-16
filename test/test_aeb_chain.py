# test_aeb_chain.py

# Purpose:
# System-level unit test for the AEB decision chain.

#This test validates the core AEB logic:
# Pedestrian Distance + Vehicle Speed -> TTC -> Brake Decision

# Why this test exists:
# - Verifies decision correctness without ROS runtime
# - Catches regressions in safety logic early
# - Provides traceability for ISO 26262 & SOTIF

import math

# Safety threshold must match aeb_decision_node.py
TTC_THRESHOLD = 1.5  # seconds

def compute_ttc(distance, speed):
    
    # Compute Time-To-Collision (TTC).

    # Parameters:
    # distance : float
    #    Distance to pedestrian in meters
    # speed : float
    #    Vehicle speed in m/s

    # Returns:
    # float
    #    Time-To-Collision in seconds
  
    speed = max(speed, 0.1)  # mirror node logic
    return distance / speed

def should_brake(distance, speed):
  
    # Determine whether AEB braking should be applied.

    # Returns:
    #  bool
  
    ttc = compute_ttc(distance, speed)
    return ttc < TTC_THRESHOLD


# ----------------------------------------------------
# TEST CASES

def test_brake_applied_when_ttc_low():
  
    # Emergency braking should trigger when TTC is below threshold.
    
    distance = 5.0    # meters
    speed = 10.0      # m/s  → TTC = 0.5 s
    assert should_brake(distance, speed) is True


def test_no_brake_when_ttc_high():
    
    # No braking should occur when TTC is above threshold.
    
    distance = 20.0   # meters
    speed = 5.0       # m/s → TTC = 4.0 s
    assert should_brake(distance, speed) is False


def test_no_divide_by_zero():
    
    # Ensure zero or near-zero speed does not cause runtime error.
    
    distance = 10.0
    speed = 0.0
    ttc = compute_ttc(distance, speed)
    assert math.isfinite(ttc)
    assert ttc > 0


def test_boundary_condition():
    
    # Validate behavior exactly at TTC threshold.
   
    speed = 10.0
    distance = speed * TTC_THRESHOLD  # TTC == threshold
    assert should_brake(distance, speed) is False
