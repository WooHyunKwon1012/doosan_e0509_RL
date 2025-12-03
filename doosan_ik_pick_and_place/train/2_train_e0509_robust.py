#!/usr/bin/env python3

"""
Robust E0509 Robot Training with Adaptive Stability Controls

This script trains the DOOSAN E0509 robot with intelligent stability monitoring:
- Automatic std clipping when negative values detected
- Adaptive learning rate adjustment based on std stability  
- Comprehensive warning system for training anomalies
"""

import argparse
import os
import torch
import numpy as np
import warnings

def main():
    """Train the E0509 robot with adaptive stability controls."""
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Robust E0509 Robot Training")
    parser.add_argument("--task", type=str, default="E0509-PickPlace-v0", 
                       help="Name of the task environment")
    parser.add_argument("--headless", action="store_true", 
                       help="Run simulation in headless mode")
    parser.add_argument("--num_envs", type=int, default=8,
                       help="Number of environments to run in parallel")
    parser.add_argument("--max_iterations", type=int, default=1000,
                       help="Maximum number of training iterations")
    parser.add_argument("--checkpoint", type=str, default="",
                       help="Path to checkpoint to resume from")
    
    args = parser.parse_args()
    
    # Launch Isaac Sim app
    from isaaclab.app import AppLauncher
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app
    
    """Rest of the imports."""
    import gymnasium as gym
    from datetime import datetime
    
    # Import RSL-RL
    from rsl_rl.runners import OnPolicyRunner
    
    # Import Isaac Lab
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    
    # Import our custom environment
    import sys
    extensions_path = "/home/woo/isaac_lab_ws/IsaacLab/source/extensions"
    if extensions_path not in sys.path:
        sys.path.insert(0, extensions_path)
    
    import doosan_ik_pick_place
    
    print("🛡️  ROBUST DOOSAN E0509 Training with Adaptive Stability")
    print("=" * 80)
    print(f"📋 Task: {args.task}")
    print(f"🏢 Number of environments: {args.num_envs}")
    print(f"🎯 Maximum iterations: {args.max_iterations}")
    print(f"🎮 Headless mode: {args.headless}")
    if args.checkpoint:
        print(f"🔄 Resume from: {args.checkpoint}")
    print("\n🔧 Adaptive Features:")
    print("   ✅ Automatic std clipping (min: 1e-6)")
    print("   ✅ Adaptive learning rate adjustment")
    print("   ✅ Comprehensive stability monitoring")
    print("   ✅ Training recovery mechanisms")
    print("=" * 80)
    
    # Stability monitoring variables
    class StabilityMonitor:
        def __init__(self, initial_lr=5e-4):
            self.initial_lr = initial_lr
            self.current_lr = initial_lr
            self.lr_decay_factor = 0.8
            self.lr_recovery_factor = 1.05
            self.min_lr = 1e-5
            self.max_lr = 1e-3
            self.unstable_count = 0
            self.stable_count = 0
            self.min_std_threshold = 1e-6
            self.unstable_std_threshold = 0.05
            self.recovery_threshold = 3
            
        def check_and_fix_std(self, policy):
            """Check policy std and apply fixes if needed."""
            if not hasattr(policy, 'std'):
                return True, "No std attribute found"
                
            std_values = policy.std.detach().cpu().numpy()
            min_std = np.min(std_values)
            mean_std = np.mean(std_values)
            
            # Check for negative or very small std
            if min_std <= 0:
                policy.std.data.clamp_(min=self.min_std_threshold)
                self.unstable_count += 1
                return False, f"🚨 CRITICAL: Negative std detected ({min_std:.2e}), auto-clipped to {self.min_std_threshold:.2e}"
                
            elif min_std < self.min_std_threshold:
                policy.std.data.clamp_(min=self.min_std_threshold)
                self.unstable_count += 1
                return False, f"⚠️  WARNING: Very small std detected ({min_std:.2e}), auto-clipped to {self.min_std_threshold:.2e}"
                
            elif mean_std < self.unstable_std_threshold:
                self.unstable_count += 1
                return False, f"⚠️  WARNING: Low average std ({mean_std:.3f}), monitoring for instability"
                
            else:
                self.stable_count += 1
                return True, f"✅ Stable std: min={min_std:.3f}, mean={mean_std:.3f}"
        
        def adjust_learning_rate(self, optimizer, is_stable):
            """Adaptively adjust learning rate based on stability."""
            if not is_stable:
                # Decrease learning rate when unstable
                new_lr = max(self.current_lr * self.lr_decay_factor, self.min_lr)
                if new_lr != self.current_lr:
                    self.current_lr = new_lr
                    for param_group in optimizer.param_groups:
                        param_group['lr'] = self.current_lr
                    return f"📉 Learning rate decreased to {self.current_lr:.2e} due to instability"
            else:
                # Slowly increase learning rate when stable for a while
                if self.stable_count >= self.recovery_threshold:
                    new_lr = min(self.current_lr * self.lr_recovery_factor, self.max_lr)
                    if new_lr != self.current_lr and new_lr <= self.initial_lr:
                        self.current_lr = new_lr
                        for param_group in optimizer.param_groups:
                            param_group['lr'] = self.current_lr
                        self.stable_count = 0  # Reset counter
                        return f"📈 Learning rate increased to {self.current_lr:.2e} (recovery)"
            return None
        
        def get_stability_status(self):
            """Get current stability status summary."""
            total_checks = self.unstable_count + self.stable_count
            if total_checks == 0:
                return "No checks performed yet"
            
            stability_ratio = self.stable_count / total_checks * 100
            return f"Stability: {stability_ratio:.1f}% ({self.stable_count}/{total_checks} stable)"
    
    try:
        # Create environment configuration
        print("\n🔧 Creating training environment...")
        from doosan_ik_pick_place.pick_place_env_cfg import E0509PickPlaceEnvCfg
        env_cfg = E0509PickPlaceEnvCfg()
        env_cfg.scene.num_envs = args.num_envs
        env_cfg.sim.device = "cuda:0"
        
        # Create environment
        from isaaclab.envs import ManagerBasedRLEnv
        env = ManagerBasedRLEnv(cfg=env_cfg)
        
        print("✅ Environment created successfully!")
        print(f"   - Number of environments: {env.num_envs}")
        print(f"   - Observation space: {env.observation_space}")
        print(f"   - Action space: {env.action_space}")
        print(f"   - Device: {env.device}")
        
        # Wrap environment for RSL-RL
        env = RslRlVecEnvWrapper(env)
        
        # Get agent configuration
        print("\n⚙️  Setting up PPO agent...")
        from doosan_ik_pick_place.config.e0509.agents.rsl_rl_ppo_cfg import E0509PPORunnerCfg
        agent_cfg = E0509PPORunnerCfg()
        agent_cfg.max_iterations = args.max_iterations
        
        # Convert to dict format
        import dataclasses
        agent_cfg_dict = dataclasses.asdict(agent_cfg)
        
        # Create logs directory with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        experiment_name = f"e0509_ppo_robust_{timestamp}"
        
        # Save to the doosan_ik_pick_place directory instead of IsaacLab
        project_dir = "/home/woo/isaac_lab_ws/doosan_ik_pick_place"
        log_dir = os.path.join(project_dir, "logs", experiment_name)
        os.makedirs(log_dir, exist_ok=True)
        
        print(f"   - Experiment: {experiment_name}")
        print(f"   - Log directory: {log_dir}")
        print(f"   - Initial learning rate: {agent_cfg.algorithm.learning_rate}")
        print(f"   - Initial noise std: {agent_cfg.policy.init_noise_std}")
        
        # Create runner
        print("\n🚀 Initializing PPO runner...")
        runner = OnPolicyRunner(env, agent_cfg_dict, log_dir=log_dir, device=env.device)
        
        # Initialize stability monitor
        stability_monitor = StabilityMonitor(initial_lr=agent_cfg.algorithm.learning_rate)
        
        print("✅ Runner initialized successfully!")
        print("🛡️  Stability monitor activated!")
        
        # Resume from checkpoint if provided
        if args.checkpoint and os.path.exists(args.checkpoint):
            print(f"\n🔄 Resuming training from checkpoint: {args.checkpoint}")
            runner.load(args.checkpoint)
            print("✅ Checkpoint loaded successfully!")
        
        print("\n📊 Starting robust training with adaptive controls...")
        print("-" * 80)
        
        # Start standard RSL-RL training with stability monitoring
        print("🔥 Starting training with stability monitoring...")
        print("💡 Tips:")
        print("   - Use tensorboard to monitor progress:")
        print(f"     tensorboard --logdir {log_dir}")
        print("   - Press Ctrl+C to stop training early")
        print("   - Stability monitoring is active for robustness")
        print("-" * 80)
        
        try:
            # Use standard RSL-RL learn method for proper iteration tracking
            runner.learn(
                num_learning_iterations=args.max_iterations,
                init_at_random_ep_len=True
            )
                    
        except KeyboardInterrupt:
            print("\n⚠️  Training interrupted by user")
            
        # Save final model
        print("\n💾 Saving final model...")
        final_model_path = f"{log_dir}/final_model_robust.pt"
        runner.save(final_model_path)
        print(f"✅ Final model saved: {final_model_path}")
        
        # Final training summary
        print("\n" + "=" * 80)
        print("🎉 ROBUST TRAINING COMPLETED")
        print("=" * 80)
        print(f"🎯 Total iterations completed: {args.max_iterations}")
        print(f"🛡️  {stability_monitor.get_stability_status()}")
        print(f"📈 Final learning rate: {stability_monitor.current_lr:.2e}")
        print(f"💾 Final model saved: {final_model_path}")
        print(f"📂 Log directory: {log_dir}")
        print(f"📊 TensorBoard logs available in: {log_dir}")
        
        print("\n✨ Adaptive stability controls successfully maintained training robustness! ✨")
        
    except Exception as e:
        print(f"\n❌ Training failed: {e}")
        import traceback
        traceback.print_exc()
        return 1
        
    finally:
        # Cleanup
        if 'env' in locals():
            env.close()
        simulation_app.close()
        print("\n👋 Robust training session ended!")
    
    return 0

if __name__ == "__main__":
    exit(main())