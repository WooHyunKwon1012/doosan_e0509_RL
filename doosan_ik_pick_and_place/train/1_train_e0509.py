#!/usr/bin/env python3

"""
Train DOOSAN E0509 Robot for Pick and Place Task using PPO

This script trains the DOOSAN E0509 robot to perform pick and place operations
using reinforcement learning with the PPO algorithm.
"""

import argparse
import os
from datetime import datetime

def main():
    """Train E0509 Pick and Place with RSL-RL PPO."""
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Train DOOSAN E0509 Robot Pick and Place")
    parser.add_argument("--task", type=str, default="E0509-PickPlace-v0", 
                       help="Name of the task environment")
    parser.add_argument("--headless", action="store_true", 
                       help="Run simulation in headless mode")
    parser.add_argument("--num_envs", type=int, default=1024, 
                       help="Number of parallel environments")
    parser.add_argument("--max_iterations", type=int, default=500,
                       help="Maximum training iterations")
    parser.add_argument("--seed", type=int, default=42,
                       help="Random seed for reproducibility")
    parser.add_argument("--log_dir", type=str, default="logs/rsl_rl",
                       help="Directory to save logs and models")
    
    args = parser.parse_args()
    
    # Launch Isaac Sim app
    from isaaclab.app import AppLauncher
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app
    
    """Rest of the imports."""
    import gymnasium as gym
    import torch
    
    # Import RSL-RL
    from rsl_rl.runners import OnPolicyRunner
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    
    # Import our custom environment
    import sys
    import os
    
    # Add the IsaacLab extensions path where doosan_ik_pick_place is installed
    extensions_path = "/home/woo/isaac_lab_ws/IsaacLab/source/extensions"
    if extensions_path not in sys.path:
        sys.path.insert(0, extensions_path)
    
    import doosan_ik_pick_place
    
    print("🚀 DOOSAN E0509 Pick and Place Training")
    print("=" * 80)
    print(f"📋 Task: {args.task}")
    print(f"🏢 Number of environments: {args.num_envs}")
    print(f"🔄 Max iterations: {args.max_iterations}")
    print(f"🎮 Headless mode: {args.headless}")
    print(f"🌱 Seed: {args.seed}")
    print("=" * 80)
    
    # Set random seed
    torch.manual_seed(args.seed)
    
    try:
        # Create environment configuration
        print("\n🔧 Creating training environment...")
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
        
        # Get agent configuration
        print("\n⚙️  Loading PPO configuration...")
        from doosan_ik_pick_place.config.e0509.agents.rsl_rl_ppo_cfg import E0509PPORunnerCfg
        agent_cfg = E0509PPORunnerCfg()
        
        # Override some parameters
        agent_cfg.max_iterations = args.max_iterations
        agent_cfg.num_steps_per_env = 96  # Steps per environment per iteration
        agent_cfg.save_interval = 25  # Save model every 25 iterations
        
        # Create log directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        experiment_name = f"e0509_ppo_{timestamp}"
        
        # Save to the doosan_ik_pick_place directory instead of IsaacLab
        project_dir = "/home/woo/isaac_lab_ws/doosan_ik_pick_place"
        log_dir = os.path.join(project_dir, "logs", experiment_name)
        os.makedirs(log_dir, exist_ok=True)
        
        print(f"📊 Experiment: {experiment_name}")
        print(f"💾 Logging to: {log_dir}")
        
        # Create RSL-RL runner
        print("\n🏃 Creating RSL-RL runner...")
        
        # Wrap environment for RSL-RL
        env = RslRlVecEnvWrapper(env)
        
        # Convert agent config to dictionary format for RSL-RL
        import dataclasses
        agent_cfg_dict = dataclasses.asdict(agent_cfg)
        
        # Create runner
        runner = OnPolicyRunner(env, agent_cfg_dict, log_dir=log_dir, device=env.device)
        print("✅ Runner created successfully!")
        
        # Print training info
        print("\n🎯 Training Configuration:")
        print(f"   - Algorithm: PPO (Proximal Policy Optimization)")
        print(f"   - Learning rate: {agent_cfg.algorithm.learning_rate}")
        print(f"   - Discount factor (γ): {agent_cfg.algorithm.gamma}")
        print(f"   - GAE lambda (λ): {agent_cfg.algorithm.lam}")
        print(f"   - Clip parameter: {agent_cfg.algorithm.clip_param}")
        print(f"   - Entropy coefficient: {agent_cfg.algorithm.entropy_coef}")
        print(f"   - Steps per env per iteration: {agent_cfg.num_steps_per_env}")
        print(f"   - Total samples per iteration: {args.num_envs * agent_cfg.num_steps_per_env}")
        
        # Start training
        print(f"\n🔥 Starting training...")
        print("💡 Tips:")
        print("   - Use tensorboard to monitor progress:")
        print(f"     tensorboard --logdir {log_dir}")
        print("   - Press Ctrl+C to stop training early")
        print("   - Model checkpoints will be saved automatically")
        print("-" * 80)
        
        # Training loop
        runner.learn(
            num_learning_iterations=args.max_iterations,
            init_at_random_ep_len=True
        )
        
        print("\n🎉 Training completed successfully!")
        
        # Save final model
        model_path = os.path.join(log_dir, "final_model.pt")
        runner.save(model_path)
        print(f"💾 Final model saved: {model_path}")
        
        # Export policy for deployment
        export_dir = os.path.join(log_dir, "exported")
        os.makedirs(export_dir, exist_ok=True)
        
        try:
            policy_path = os.path.join(export_dir, "policy.pt")
            # Try different attribute names for policy export
            if hasattr(runner.alg, 'actor_critic'):
                policy = runner.alg.actor_critic.actor
            elif hasattr(runner.alg, 'actor'):
                policy = runner.alg.actor
            else:
                print("⚠️  Could not find policy for export")
                policy = None
            
            if policy is not None:
                torch.jit.save(torch.jit.script(policy), policy_path)
                print(f"🚀 Policy exported: {policy_path}")
            else:
                print("⚠️  Policy export skipped")
        except Exception as e:
            print(f"⚠️  Policy export failed: {e}")
            print("💡 Model is still saved in final_model.pt")
        
        print("\n" + "=" * 80)
        print("🏆 TRAINING SUMMARY")
        print("=" * 80)
        print(f"🤖 Robot: DOOSAN E0509")
        print(f"🎯 Task: Pick and Place")
        print(f"🔄 Iterations completed: {args.max_iterations}")
        print(f"📊 Log directory: {log_dir}")
        print(f"💾 Model: {model_path}")
        print(f"🚀 Policy: {policy_path}")
        print("=" * 80)
        
        print("\n📈 Next Steps:")
        print("1. Check tensorboard logs for training progress")
        print("2. Test the trained policy with test script")
        print("3. Deploy the policy on real robot")
        
    except KeyboardInterrupt:
        print("\n⚠️  Training interrupted by user")
        if 'runner' in locals():
            model_path = os.path.join(log_dir, "interrupted_model.pt")
            runner.save(model_path)
            print(f"💾 Model saved before exit: {model_path}")
            
    except Exception as e:
        print(f"\n❌ Training failed: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup
        if 'env' in locals():
            env.close()
        simulation_app.close()
        print("\n👋 Training session ended!")

if __name__ == "__main__":
    main()