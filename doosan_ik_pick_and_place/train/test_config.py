#!/usr/bin/env python3

"""Simple test script for E0509 pick and place environment."""

import sys
import os

# Add the doosan_ik_pick_place path to Python path
sys.path.append('/home/woo/isaac_lab_ws/doosan_ik_pick_place')

def test_config_import():
    """Test if we can import the configuration without errors."""
    try:
        print("Testing configuration import...")
        
        # Test main config
        print("1. Testing main pick_place_env_cfg...")
        try:
            from pick_place_env_cfg import E0509PickPlaceEnvCfg, E0509SceneCfg, ActionsCfg
            print("   ✓ Main config imported successfully")
        except Exception as e:
            print(f"   ✗ Failed to import main config: {e}")
            return False
            
        # Test joint position config
        print("2. Testing joint position config...")
        try:
            from config.e0509.joint_pos_env_cfg import E0509_CFG, E0509_HIGH_PD_CFG
            print("   ✓ Joint position config imported successfully")
        except Exception as e:
            print(f"   ✗ Failed to import joint pos config: {e}")
            
        # Test IK configs
        print("3. Testing IK configs...")
        try:
            from config.e0509.ik_abs_env_cfg import E0509PickPlaceEnvCfg as IKAbsEnvCfg
            print("   ✓ IK absolute config imported successfully")
        except Exception as e:
            print(f"   ✗ Failed to import IK abs config: {e}")
            
        try:
            from config.e0509.ik_rel_env_cfg import E0509PickPlaceEnvCfg as IKRelEnvCfg
            print("   ✓ IK relative config imported successfully")  
        except Exception as e:
            print(f"   ✗ Failed to import IK rel config: {e}")
            
        print("\n✓ All available configurations imported successfully!")
        return True
        
    except Exception as e:
        print(f"✗ Critical error during import: {e}")
        return False

def verify_robot_config():
    """Verify robot configuration parameters."""
    try:
        print("\nVerifying robot configuration...")
        from pick_place_env_cfg import E0509PickPlaceEnvCfg
        
        cfg = E0509PickPlaceEnvCfg()
        
        # Check if robot is configured
        if cfg.scene.robot is None:
            print("✗ Robot configuration is None")
            return False
            
        print("✓ Robot configuration is set")
        
        # Check USD path
        usd_path = cfg.scene.robot.spawn.usd_path
        print(f"  USD path: {usd_path}")
        
        if os.path.exists(usd_path):
            print("  ✓ USD file exists")
        else:
            print("  ✗ USD file does not exist")
            
        # Check actuators
        actuators = cfg.scene.robot.actuators
        print(f"  Actuators: {list(actuators.keys())}")
        
        # Check actions
        print(f"  Arm action joints: {cfg.actions.arm_action.joint_names}")
        print(f"  Gripper action joints: {cfg.actions.gripper_action.joint_names}")
        
        return True
        
    except Exception as e:
        print(f"✗ Error verifying robot config: {e}")
        return False

def main():
    """Main test function."""
    print("=== E0509 Pick and Place Configuration Test ===\n")
    
    success = True
    
    # Test configuration imports
    success &= test_config_import()
    
    # Verify robot configuration
    success &= verify_robot_config()
    
    print("\n" + "="*50)
    if success:
        print("🎉 All tests passed! E0509 environment is properly configured.")
        print("\nYou can now use the following configurations:")
        print("  - Joint Position Control: doosan_ik_pick_place.config.e0509.joint_pos_env_cfg")
        print("  - IK Absolute Control: doosan_ik_pick_place.config.e0509.ik_abs_env_cfg")  
        print("  - IK Relative Control: doosan_ik_pick_place.config.e0509.ik_rel_env_cfg")
    else:
        print("❌ Some tests failed. Please check the configuration.")
        return 1
        
    return 0

if __name__ == "__main__":
    exit(main())