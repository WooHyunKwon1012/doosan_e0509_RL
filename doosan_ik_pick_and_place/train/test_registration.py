#!/usr/bin/env python3

"""Simple test script to validate E0509 environment registration with Isaac Sim."""

import sys
import os

# Add Isaac Lab to Python path
isaac_lab_path = "/home/woo/isaac_lab_ws/IsaacLab/source"
sys.path.insert(0, isaac_lab_path)

# Add doosan_ik_pick_place to Python path  
doosan_path = "/home/woo/isaac_lab_ws/doosan_ik_pick_place"
sys.path.insert(0, doosan_path)

def test_environment_registration():
    """Test if E0509 environments are properly registered."""
    try:
        print("Testing environment registration...")
        
        # Import gym to check registrations
        import gymnasium as gym
        
        # Import our package to trigger registration
        import doosan_ik_pick_place
        
        # List all registered environments
        all_envs = list(gym.envs.registry.env_specs.keys())
        e0509_envs = [env for env in all_envs if "E0509" in env]
        
        print(f"Found {len(e0509_envs)} E0509 environments:")
        for env in e0509_envs:
            print(f"  ✓ {env}")
            
        if len(e0509_envs) > 0:
            print("✅ E0509 environments successfully registered!")
            return True
        else:
            print("❌ No E0509 environments found")
            return False
            
    except Exception as e:
        print(f"❌ Error during registration test: {e}")
        return False

def test_configuration_import():
    """Test if configurations can be imported."""
    try:
        print("\nTesting configuration imports...")
        
        from doosan_ik_pick_place.pick_place_env_cfg import E0509PickPlaceEnvCfg
        print("  ✓ Main environment config")
        
        from doosan_ik_pick_place.config.e0509.joint_pos_env_cfg import E0509_CFG
        print("  ✓ Joint position config")
        
        from doosan_ik_pick_place.config.e0509.ik_abs_env_cfg import E0509PickPlaceEnvCfg as IKAbsCfg
        print("  ✓ IK absolute config")
        
        from doosan_ik_pick_place.config.e0509.ik_rel_env_cfg import E0509PickPlaceEnvCfg as IKRelCfg 
        print("  ✓ IK relative config")
        
        print("✅ All configurations imported successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Configuration import failed: {e}")
        return False

def show_usage_instructions():
    """Show usage instructions."""
    print("\n" + "="*60)
    print("E0509 Environment Setup Complete!")
    print("="*60)
    
    print("\n🚀 Usage Instructions:")
    print("1. Set Isaac Sim path:")
    print("   export ISAAC_SIM_PATH='/isaac-sim'")
    
    print("\n2. Train with Joint Position Control:")
    print("   ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \\")
    print("     --task E0509-PickPlace-v0 --num_envs 64 --headless")
    
    print("\n3. Train with IK Absolute Control:")  
    print("   ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \\")
    print("     --task E0509-PickPlace-IK-Abs-v0 --num_envs 64 --headless")
    
    print("\n4. Train with IK Relative Control:")
    print("   ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \\")
    print("     --task E0509-PickPlace-IK-Rel-v0 --num_envs 64 --headless")
    
    print("\n📁 Configuration Files:")
    print(f"   • Main Config: {doosan_path}/pick_place_env_cfg.py")
    print(f"   • Joint Control: {doosan_path}/config/e0509/joint_pos_env_cfg.py")
    print(f"   • IK Absolute: {doosan_path}/config/e0509/ik_abs_env_cfg.py") 
    print(f"   • IK Relative: {doosan_path}/config/e0509/ik_rel_env_cfg.py")
    
    print("\n🤖 Robot Configuration:")
    print("   • Robot Type: DOOSAN E0509 with Robotiz Gripper")
    print("   • USD Path: /home/woo/Downloads/source/3d/e0509/e0509_with_gripper.usd")
    print("   • Arm Joints: joint_1 to joint_6")
    print("   • Gripper Joints: rh_l1, rh_l2, rh_p12_rn, rh_r2")

def main():
    """Main function."""
    print("=== E0509 Environment Registration Test ===")
    
    success = True
    success &= test_environment_registration()
    success &= test_configuration_import()
    
    if success:
        show_usage_instructions()
        return 0
    else:
        print("\n❌ Some tests failed. Please check the configuration.")
        return 1

if __name__ == "__main__":
    exit(main())