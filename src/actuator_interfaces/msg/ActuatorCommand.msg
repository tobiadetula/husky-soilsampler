# actuator_interfaces/msg/ActuatorCommand.msg

# Control Modes
uint8 MODE_STOP = 0
uint8 MODE_MANUAL_FORWARD = 1
uint8 MODE_MANUAL_REVERSE = 2
uint8 MODE_AUTO_TARGET = 3

uint8 control_mode           # Tells the Pico what to do
int32 target_position        # The tick to drive to (only used in auto mode)