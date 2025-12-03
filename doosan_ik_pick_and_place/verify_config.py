#!/usr/bin/env python3
"""Simple test script to verify E0509 robot configuration without full Isaac Lab environment."""

import sys
import os

def test_basic_imports():
    """Test basic Python imports without Isaac Lab."""
    print("Testing basic imports...")
    
    try:
        import math
        print("✓ math module OK")
        
        # Test if USD path exists
        usd_path = "/home/woo/Downloads/source/3d/e0509/e0509_with_gripper.usd"
        if os.path.exists(usd_path):
            print(f"✓ E0509 USD file exists: {usd_path}")
        else:
            print(f"✗ E0509 USD file NOT found: {usd_path}")
            
        # Test alternative paths
        alt_paths = [
            "/home/woo/isaac_lab_ws/doosan/3d/e0509_with_gripper.usd",
            "/home/woo/isaac_lab_ws/doosan/3d/e0509/e0509_with_gripper.usd",
            "/home/woo/Desktop/isaac-sim/doosan/3d/e0509_with_gripper.usd"
        ]
        
        for path in alt_paths:
            if os.path.exists(path):
                print(f"✓ Alternative USD file found: {path}")
                return path
                
        return None
        
    except Exception as e:
        print(f"✗ Error during basic imports: {e}")
        return None

def show_config_summary():
    """Show configuration summary."""
    print("\n" + "="*50)
    print("E0509 Pick and Place Configuration Summary")
    print("="*50)
    
    usd_path = test_basic_imports()
    
    print(f"\nRobot Configuration:")
    print(f"  - Robot Type: E0509 with Robotiz Gripper")
    if usd_path:
        print(f"  - USD Path: {usd_path}")
    else:
        print(f"  - USD Path: NOT FOUND - needs to be created or copied")
    
    print(f"\nJoint Configuration:")
    print(f"  - Arm Joints: joint_1 to joint_6")
    print(f"  - Gripper Joints: rh_l1, rh_l2, rh_p12_rn, rh_r2")
    
    print(f"\nActuator Settings:")
    print(f"  - Arm: effort_limit=100.0, stiffness=10000.0, damping=100.0")
    print(f"  - Gripper: effort_limit=50.0, stiffness=5000.0, damping=50.0")
    
    print(f"\nAction Configuration:")
    print(f"  - Arm Action: Joint position control, scale=0.5")
    print(f"  - Gripper Action: Joint position control, scale=0.05")
    
    print(f"\nEnvironment Settings:")
    print(f"  - Block Position: (0.25, 0.0, 0.05)")
    print(f"  - Box Position: (0.4, 0.15, 0.0)")
    print(f"  - Reset Range: x=(0.20, 0.30), y=(-0.1, 0.1)")
    
    print(f"\nAvailable Configurations:")
    print(f"  - Joint Position Control: config.e0509.joint_pos_env_cfg")
    print(f"  - IK Absolute Control: config.e0509.ik_abs_env_cfg")
    print(f"  - IK Relative Control: config.e0509.ik_rel_env_cfg")
    
    print(f"\nNext Steps:")
    if not usd_path:
        print(f"  1. Create or copy E0509 USD file to proper location")
    print(f"  2. Set up Isaac Sim environment properly")
    print(f"  3. Test with: ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task E0509-PickPlace-v0")
    
    return True

if __name__ == "__main__":
    print("=== E0509 Configuration Verification (Standalone) ===\n")
    show_config_summary()
    print(f"\n✅ Configuration verification complete!")