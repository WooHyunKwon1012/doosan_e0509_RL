#!/usr/bin/env python3

"""
Test Trained E0509 Robot Policy

This script loads and tests the trained PPO policy for DOOSAN E0509 pick and place task.
"""

import argparse
import os
import torch

def main():
    """Test the trained E0509 policy."""
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Test Trained E0509 Robot Policy")
    parser.add_argument("--model_path", type=str, 
                       default="/home/woo/isaac_lab_ws/doosan_ik_pick_place/logs/e0509_ppo_20251118_202602/final_model.pt",
                       help="Path to the trained model file")
    parser.add_argument("--task", type=str, default="E0509-PickPlace-v0", 
                       help="Name of the task environment")
    parser.add_argument("--headless", action="store_true", 
                       help="Run simulation in headless mode")
    parser.add_argument("--num_envs", type=int, default=1, 
                       help="Number of environments to test")
    parser.add_argument("--num_episodes", type=int, default=10,
                       help="Number of episodes to test")
    parser.add_argument("--render", action="store_true", 
                       help="Enable rendering (not headless)")
    
    args = parser.parse_args()
    
    # Disable rendering if headless
    if args.headless:
        args.render = False
    
    # Launch Isaac Sim app
    from isaaclab.app import AppLauncher
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app
    
    """Rest of the imports."""
    import gymnasium as gym
    import numpy as np
    
    # Import RSL-RL
    from rsl_rl.runners import OnPolicyRunner
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    
    # Import our custom environment
    import sys
    extensions_path = "/home/woo/isaac_lab_ws/IsaacLab/source/extensions"
    if extensions_path not in sys.path:
        sys.path.insert(0, extensions_path)
    
    import doosan_ik_pick_place
    
    print("🧪 DOOSAN E0509 Pick and Place Policy Testing")
    print("=" * 80)
    print(f"📋 Task: {args.task}")
    print(f"🤖 Model: {args.model_path}")
    print(f"🏢 Number of environments: {args.num_envs}")
    print(f"🎬 Episodes to test: {args.num_episodes}")
    print(f"🎮 Headless mode: {args.headless}")
    print("=" * 80)
    
    try:
        # Check if model file exists
        if not os.path.exists(args.model_path):
            raise FileNotFoundError(f"Model file not found: {args.model_path}")
        
        print(f"\n📂 Loading model from: {args.model_path}")
        
        # Create environment configuration
        print("\n🔧 Creating test environment...")
        from doosan_ik_pick_place.pick_place_env_cfg import E0509PickPlaceEnvCfg
        env_cfg = E0509PickPlaceEnvCfg()
        env_cfg.scene.num_envs = args.num_envs
        env_cfg.sim.device = "cuda:0"
        
        # Create environment directly
        from isaaclab.envs import ManagerBasedRLEnv
        env = ManagerBasedRLEnv(cfg=env_cfg)
        
        print("✅ Environment created successfully!")
        print(f"   - Number of environments: {env.num_envs}")
        print(f"   - Observation space: {env.observation_space}")
        print(f"   - Action space: {env.action_space}")
        print(f"   - Device: {env.device}")
        
        # Wrap environment for RSL-RL
        env = RslRlVecEnvWrapper(env)
        
        # Load the trained model
        print("\n🔄 Loading trained policy...")
        checkpoint = torch.load(args.model_path, map_location=env.device)
        
        # Get agent configuration
        from doosan_ik_pick_place.config.e0509.agents.rsl_rl_ppo_cfg import E0509PPORunnerCfg
        agent_cfg = E0509PPORunnerCfg()
        
        # Convert to dict format
        import dataclasses
        agent_cfg_dict = dataclasses.asdict(agent_cfg)
        
        # Create runner and load the model
        runner = OnPolicyRunner(env, agent_cfg_dict, device=env.device)
        runner.load(args.model_path)
        
        print("✅ Model loaded successfully!")
        
        # Test the policy
        print(f"\n🎯 Testing policy for {args.num_episodes} episodes...")
        print("-" * 80)
        
        total_rewards = []
        success_count = 0
        
        for episode in range(args.num_episodes):
            obs, info = env.reset()
            episode_reward = 0
            episode_length = 0
            done = False
            
            print(f"\n🎬 Episode {episode + 1}/{args.num_episodes}")
            
            while not done:
                # Get action from trained policy
                with torch.no_grad():
                    actions = runner.alg.act(obs)
                
                # Step environment (Isaac Lab returns 4 values)
                obs, reward, dones, info = env.step(actions)
                
                episode_reward += reward.mean().item()
                episode_length += 1
                
                # Check if done (dones is a tensor)
                done = dones.any().item() or episode_length >= 500
                
                # Check for success (if available in info)
                if episode_length % 50 == 0:
                    print(f"   Step {episode_length:3d} | Reward: {reward.mean().item():7.3f}")
            
            total_rewards.append(episode_reward)
            
            # Check success - look at final reward
            if episode_reward > 100.0:  # Success threshold based on reward accumulation
                success_count += 1
                print(f"   ✅ SUCCESS! Episode reward: {episode_reward:.3f}")
            else:
                print(f"   ❌ Failed. Episode reward: {episode_reward:.3f}")
            
            print(f"   📊 Episode reward: {episode_reward:.3f} | Length: {episode_length}")
        
        # Calculate statistics
        avg_reward = np.mean(total_rewards)
        std_reward = np.std(total_rewards)
        success_rate = (success_count / args.num_episodes) * 100
        
        print("\n" + "=" * 80)
        print("📈 TESTING RESULTS")
        print("=" * 80)
        print(f"🎯 Episodes tested: {args.num_episodes}")
        print(f"📊 Average reward: {avg_reward:.3f} ± {std_reward:.3f}")
        print(f"🏆 Success rate: {success_rate:.1f}% ({success_count}/{args.num_episodes})")
        print(f"📋 Reward range: [{min(total_rewards):.3f}, {max(total_rewards):.3f}]")
        print("=" * 80)
        
        if success_rate > 50:
            print("🎉 Great performance! The policy is working well!")
        elif success_rate > 20:
            print("👍 Decent performance! The policy shows some learning.")
        else:
            print("🤔 The policy needs more training or tuning.")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
        
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup
        if 'env' in locals():
            env.close()
        simulation_app.close()
        print("\n👋 Testing session ended!")

if __name__ == "__main__":
    main()