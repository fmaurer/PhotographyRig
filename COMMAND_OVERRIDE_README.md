# Command Override System

## Overview

The command override system prevents multiple pan or tilt commands from stacking up and conflicting with each other. When a new command arrives, it automatically cancels any ongoing movement and starts the new one.

## How It Works

### 1. Movement Tracking
- Each motor (pan and tilt) has its own movement tracking
- The system maintains `current_pan_movement` and `current_tilt_movement` thread references
- A `movement_lock` ensures thread-safe operations

### 2. Command Override Process
When a new command arrives:

1. **Acquire lock** - Ensures only one command can be processed at a time
2. **Check ongoing movement** - If a movement thread is active, signal it to stop
3. **Wait for cleanup** - Brief timeout to allow the motor to stop gracefully
4. **Start new movement** - Create and start a new movement thread
5. **Release lock** - Allow other commands to be processed

### 3. Motor Driver Integration
Each motor driver class now includes:

- `should_stop` flag - Signals when movement should be interrupted
- `is_moving` flag - Tracks current movement state
- `stop_movement()` method - Gracefully stops ongoing movement

## Supported Commands

### Individual Commands
- `pan <steps>` - Pan motor to specified step position
- `tilt <steps>` - Tilt motor to specified step position

### Combined Commands
- `move <pan_steps> <tilt_steps>` - Move both motors simultaneously

## Benefits

1. **No Command Stacking** - New commands immediately override previous ones
2. **Predictable Behavior** - Motors always execute the most recent command
3. **Hardware Protection** - Prevents conflicting movements that could stress motors
4. **Responsive Control** - Real-time control without command queuing delays

## Testing

Use the provided test script to verify the override functionality:

```bash
python test_command_override.py
```

This script will:
- Send rapid pan/tilt commands
- Verify that new commands override previous ones
- Allow manual testing of commands

## Implementation Details

### WebSocket Server Changes
- Added movement tracking variables
- Implemented `cancel_and_start_pan()` and `cancel_and_start_tilt()` methods
- Updated all command handlers to use override functionality

### Motor Driver Changes
- Added movement state tracking
- Implemented graceful stop mechanism
- Added stop checking in step execution loops

### Thread Safety
- Uses `threading.Lock()` to prevent race conditions
- Proper thread cleanup and timeout handling
- Safe movement state management

## Troubleshooting

### Common Issues

1. **Motor doesn't stop immediately**
   - The system waits up to 100ms for graceful shutdown
   - This prevents motor stress and missed steps

2. **Commands seem delayed**
   - The lock ensures commands are processed sequentially
   - This prevents command conflicts and ensures reliability

3. **Motor behavior is erratic**
   - Check that both pan and tilt motors are properly connected
   - Verify GPIO pin configurations

### Debug Information
The system provides console output for:
- Movement start/stop events
- Stop command acknowledgments
- Thread lifecycle events

## Future Enhancements

Potential improvements could include:
- Command priority system
- Movement queuing with override options
- Emergency stop functionality
- Movement status reporting via WebSocket 