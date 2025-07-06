# Livox LiDAR status message format.

Header header                    # ROS standard message header
string broadcast_code           # LiDAR broadcast code
uint8 handle                    # LiDAR handle
uint8 device_type              # Device type (0:Hub, 1:Mid40, 2:Tele, 3:Horizon, 6:Mid70, 7:Avia)
uint8 state                     # Current working state (0:Init, 1:Normal, 2:PowerSaving, 3:StandBy, 4:Error, 5:Unknown)
uint8 mode                      # Current working mode (1:Normal, 2:PowerSaving, 3:Standby)
uint8 feature                   # LiDAR feature (0:None, 1:RainFog)
uint8 return_mode              # Point cloud return mode (0:FirstReturn, 1:StrongestReturn, 2:DualReturn, 3:TripleReturn)
string ip                       # Device IP address
uint16 data_port               # Point cloud data UDP port
uint16 cmd_port                # Control command UDP port
uint16 sensor_port             # IMU data UDP port
uint8[4] firmware_version      # Firmware version
bool is_connected              # Connection status
bool is_sampling               # Sampling status
uint32 error_code              # Error code if any 