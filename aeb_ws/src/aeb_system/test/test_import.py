"""
Sanity test to verify that the aeb_system ROS 2 Python package
is correctly installed and all nodes are importable.
"""

def test_import_nodes():
    import aeb_system.camera_node
    import aeb_system.pedestrian_detector
    import aeb_system.aeb_decision_node
    import aeb_system.vehicle_control_node
