# Single Arm ALOHA Control

This setup allows you to control only a single arm of the ALOHA robot instead of both arms. This is useful for:

- Testing with only one arm
- Scenarios where only one arm is needed
- Debugging single-arm policies
- Reduced computational requirements

## Key Changes from Dual-Arm Setup

### Action Space
- **Dual-arm**: `[left_arm_qpos(6), left_gripper(1), right_arm_qpos(6), right_gripper(1)]` = 14 dimensions
- **Single-arm**: `[arm_qpos(6), gripper(1)]` = 7 dimensions

### Observation Space
- **Dual-arm**: Both left and right arm joint positions, velocities, and efforts
- **Single-arm**: Only the selected arm's joint positions, velocities, and efforts
- **Images**: Uses only the relevant wrist camera (`cam_left_wrist` or `cam_right_wrist`)

### Robot Control
- Only initializes and controls one InterbotixManipulatorXS instance
- Only one Recorder for joint states
- Simplified gripper control

## Usage

### Method 1: Using Docker (Recommended)

**For left arm:**
```bash
export SERVER_ARGS="--env ALOHA --default_prompt='pick up the object'"
docker compose -f examples/aloha_real/single_arm_compose.yml up --build
```

**For right arm:**
```bash
export SERVER_ARGS="--env ALOHA --default_prompt='pick up the object'"
docker compose -f examples/aloha_real/single_arm_compose.yml up --build
# Then modify the command in the compose file to use --arm_side right
```

### Method 2: Manual Setup

**Terminal 1 - Start ROS:**
```bash
roslaunch aloha ros_nodes.launch
```

**Terminal 2 - Start Policy Server:**
```bash
uv run scripts/serve_policy.py --env ALOHA --default_prompt='pick up the object'
```

**Terminal 3 - Run Single Arm Client:**
```bash
# For left arm
python -m examples.aloha_real.single_arm_main --arm_side left

# For right arm  
python -m examples.aloha_real.single_arm_main --arm_side right

# With custom framerate
python -m examples.aloha_real.single_arm_main --arm_side left --max_hz 30
```

## Command Line Arguments

- `--arm_side`: Choose which arm to control ("left" or "right")
- `--host`: Server host (default: "0.0.0.0")
- `--port`: Server port (default: 8000)
- `--action_horizon`: Action horizon for policy (default: 25)
- `--max_hz`: Control loop frequency (default: 50.0)
- `--num_episodes`: Number of episodes to run (default: 1)
- `--max_episode_steps`: Maximum steps per episode (default: 1000)

## Important Notes

1. **Policy Compatibility**: The single-arm setup expects a policy trained for single-arm control. If using a dual-arm policy, you may need to:
   - Retrain the policy for single-arm
   - Modify the policy to handle the different action/observation spaces
   - Use only the relevant parts of a dual-arm policy

2. **Camera Setup**: Make sure your camera configuration in `third_party/aloha/aloha_scripts/realsense_publisher.py` includes the appropriate wrist camera for your chosen arm.

3. **Hardware**: Only the selected arm needs to be properly connected and calibrated.

4. **Reset Position**: The reset position is the same for both arms (using the first 6 joint positions from the dual-arm setup).

## Files Created

- `single_arm_env.py`: Single-arm environment class
- `single_arm_main.py`: Main script for single-arm control
- `single_arm_compose.yml`: Docker compose file for single-arm setup
- `SINGLE_ARM_README.md`: This documentation

## Troubleshooting

1. **Action dimension mismatch**: Ensure your policy expects 7-dimensional actions (6 joints + 1 gripper)
2. **Observation mismatch**: Check that the policy expects the single-arm observation space
3. **Camera issues**: Verify the correct wrist camera is being used for your selected arm
4. **Robot connection**: Make sure only the selected arm is properly connected to avoid conflicts
