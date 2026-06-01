# actuator_interfaces/msg/ActuatorState.msg

# Status Enums
uint8 STATUS_OK = 0
uint8 STATUS_WARN = 1
uint8 STATUS_ERROR = 2

# Core Data
int32 current_position       # Raw ticks from the Hall effect sensor
float32 applied_force        # Weight/Force from the load cell
float32 motor_current        # Current draw in Amps
float32 board_temperature    # Pico temperature in Celsius

# Hardware Health
uint8 system_status          # 0, 1, or 2 based on initialization and I2C watchdogs