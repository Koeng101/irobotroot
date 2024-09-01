package irobotroot

// Packet represents the structure of a BLE packet for communication with the Root robot.
// It encapsulates all the necessary components of a packet as defined in the Root Robot
// Bluetooth Low Energy Protocol.
type Packet struct {
	Dev     byte     // Device number (1 byte) identifying the subsystem of the robot
	Cmd     byte     // Command number (1 byte) indicating the specific action or request
	ID      byte     // Packet ID (1 byte) used for tracking and matching responses
	Payload [16]byte // Payload (16 bytes) containing command-specific data
	CRC     byte     // Checksum (1 byte) for data integrity verification
}

// Device 0 - General

// GetVersionsPacket represents a request to get version information from a specific board.
type GetVersionsPacket struct {
	Board uint8 // Board identifier (0xA5 for main board, 0xC6 for color board)
}

// GetVersionsResponse contains the version information returned by the robot.
type GetVersionsResponse struct {
	Board    uint8 // Board identifier (0xA5 for main board, 0xC6 for color board)
	FWMaj    uint8 // Firmware version major number
	FWMin    uint8 // Firmware version minor number
	HWMaj    uint8 // Hardware version major number
	HWMin    uint8 // Hardware version minor number
	BootMaj  uint8 // Bootloader version major number
	BootMin  uint8 // Bootloader version minor number
	ProtoMaj uint8 // Protocol version major number
	ProtoMin uint8 // Protocol version minor number
	Patch    uint8 // Firmware version patch number
}

// SetNamePacket is used to set a new BLE advertising name for the robot.
type SetNamePacket struct {
	Name [16]byte // UTF-8 encoded string containing the new advertising name
}

// GetNameResponse contains the current BLE advertising name of the robot.
type GetNameResponse struct {
	Name [16]byte // UTF-8 encoded string containing the current advertising name
}

// EnableEventsPacket is used to enable BLE notification for events by devices on the robot.
type EnableEventsPacket struct {
	DevicesBitfield [16]byte // 128-bit bitfield representing devices to enable (1 = enable, 0 = no change)
}

// DisableEventsPacket is used to disable BLE notification for events by devices on the robot.
type DisableEventsPacket struct {
	DevicesBitfield [16]byte // 128-bit bitfield representing devices to disable (1 = disable, 0 = no change)
}

// GetEnabledEventsResponse contains a bitfield of the currently enabled devices.
type GetEnabledEventsResponse struct {
	DevicesBitfield [16]byte // 128-bit bitfield representing enabled devices (1 = enabled, 0 = disabled)
}

// GetSerialNumberResponse contains the product serial number.
type GetSerialNumberResponse struct {
	SerialNumber [12]byte // UTF-8 encoded string containing the product serial number
}

// GetSKUResponse contains the product SKU.
type GetSKUResponse struct {
	SKU [16]byte // UTF-8 encoded, null-terminated string containing the product SKU
}

// StopProjectEvent represents an event indicating that the running project should be stopped.
type StopProjectEvent struct {
	Timestamp uint32 // Timestamp in milliseconds
}

// Device 1 - Motors

// SetLeftAndRightMotorSpeedPacket is used to set the linear velocity for both motors.
type SetLeftAndRightMotorSpeedPacket struct {
	LeftSpeed  int32 // Left motor speed in mm/s (-100 to 100)
	RightSpeed int32 // Right motor speed in mm/s (-100 to 100)
}

// SetLeftMotorSpeedPacket is used to set the linear velocity for the left motor only.
type SetLeftMotorSpeedPacket struct {
	LeftSpeed int32 // Left motor speed in mm/s (-100 to 100)
}

// SetRightMotorSpeedPacket is used to set the linear velocity for the right motor only.
type SetRightMotorSpeedPacket struct {
	RightSpeed int32 // Right motor speed in mm/s (-100 to 100)
}

// DriveDistancePacket is used to command the robot to drive a set distance in a straight line.
type DriveDistancePacket struct {
	Distance int32 // Distance to drive in mm
}

// RotateAnglePacket is used to command the robot to rotate in place by a set angle.
type RotateAnglePacket struct {
	Angle int32 // Angle to rotate in decidegrees (1/10 of degrees)
}

// SetGravityCompensationPacket is used to set the amount of correction used during vertical driving.
type SetGravityCompensationPacket struct {
	Active uint8  // Activation state (0 = off, 1 = on, 2 = enabled when marker is down)
	Amount uint16 // Compensation amount in decipercent (0-1000)
}

// GetPositionResponse contains the estimated robot location and orientation.
type GetPositionResponse struct {
	Timestamp uint32 // Timestamp in milliseconds
	X         int32  // X coordinate in millimeters
	Y         int32  // Y coordinate in millimeters
	Heading   int16  // Orientation in decidegrees (0-3599)
}

// NavigateToPositionPacket is used to command the robot to navigate to a coordinate location.
type NavigateToPositionPacket struct {
	X       int32 // X coordinate in millimeters
	Y       int32 // Y coordinate in millimeters
	Heading int16 // Final orientation in decidegrees (0-3599, or -1 for robot to choose)
}

// DriveArcPacket is used to command the robot to drive the length of an arc.
type DriveArcPacket struct {
	Angle  int32 // Angle of the arc in decidegrees
	Radius int32 // Radius of the arc in millimeters
}

// MotorStallEvent represents an event indicating that a motor has stalled.
type MotorStallEvent struct {
	Timestamp uint32 // Timestamp in milliseconds
	Motor     uint8  // Stalled motor (0 = left, 1 = right, 2 = marker/eraser)
	Cause     uint8  // Cause of stall (0-5, representing different stall reasons)
}

// Device 2 - Marker/Eraser

// SetMarkerErasePositionPacket is used to set the position of the marker/eraser actuator.
type SetMarkerErasePositionPacket struct {
	Position uint8 // Position (0 = both up, 1 = marker down, 2 = eraser down)
}

// Device 3 - LED Lights

// SetLEDAnimationPacket is used to set LED cross animation type and color.
type SetLEDAnimationPacket struct {
	State uint8 // LED state (0 = off, 1 = on, 2 = blink, 3 = spin)
	Red   uint8 // Red LED brightness (0-255)
	Green uint8 // Green LED brightness (0-255)
	Blue  uint8 // Blue LED brightness (0-255)
}

// Device 4 - Color Sensor

// GetColorSensorDataPacket is used to request color sensor data.
type GetColorSensorDataPacket struct {
	Bank     uint8 // Sensor bank (0-3, representing groups of 8 sensors)
	Lighting uint8 // LED illumination during measurement (0-4)
	Format   uint8 // Data format (0 = 12-bit ADC counts, 1 = millivolts)
}

// GetColorSensorDataResponse contains the requested color sensor data.
type GetColorSensorDataResponse struct {
	Data [8]uint16 // Eight 16-bit color sensor values
}

// ColorSensorEvent represents an event indicating a change in detected color.
type ColorSensorEvent struct {
	Colors [32]uint8 // 32 4-bit identified color values
}

// Device 5 - Sound

// PlayNotePacket is used to play a frequency from the robot's buzzer.
type PlayNotePacket struct {
	Frequency uint32 // Frequency of note in Hz
	Duration  uint16 // Duration of note in milliseconds
}

// SayPhrasePacket is used to speak a text string in robot language.
type SayPhrasePacket struct {
	Phrase [16]byte // UTF-8 encoded string with text to speak
}

// PlaySweepPacket is used to sweep linearly between two frequencies over a specified duration.
type PlaySweepPacket struct {
	StartFrequency uint32 // Start frequency in milli-Hertz
	EndFrequency   uint32 // End frequency in milli-Hertz
	Duration       uint16 // Time to play sweep in milliseconds
	Attack         uint8  // Time to ramp volume from zero to maximum in milliseconds
	Release        uint8  // Time to ramp volume from maximum to zero in milliseconds
	Volume         uint8  // Volume from 0 (silent) to 255 (max)
	ModulationType uint8  // Type of modulation (0-3)
	ModulationRate uint8  // Modulation rate in Hertz
	Append         uint8  // Non-zero to append to currently playing sweep
}

// Device 11 - IR Proximity

// GetIRProximityValuesWithTimestampResponse contains values from the IR proximity sensors.
type GetIRProximityValuesWithTimestampResponse struct {
	Timestamp uint32    // Timestamp in milliseconds
	Sensor    [6]uint16 // Six 16-bit IR proximity sensor values
}

// GetPackedIRProximityValuesAndStatesResponse contains packed values from all IR proximity sensors.
type GetPackedIRProximityValuesAndStatesResponse struct {
	Timestamp uint32   // Timestamp in milliseconds
	State     uint8    // Bitflag for state of triggered/untriggered sensors
	SensorMSB [7]uint8 // High eight bits of seven reflective IR sensor values
	SensorLSN [4]uint8 // Low four bits of seven reflective IR sensor values (packed)
}

// SetEventThresholdsPacket is used to set thresholds and shared hysteresis for IR sensors.
type SetEventThresholdsPacket struct {
	Hysteresis uint16    // Hysteresis in counts
	Sensor     [7]uint16 // Seven 16-bit IR sensor threshold values
}

// GetEventThresholdsResponse contains the current thresholds and shared hysteresis for IR sensors.
type GetEventThresholdsResponse struct {
	Hysteresis uint16    // Hysteresis in counts
	Sensor     [7]uint16 // Seven 16-bit IR sensor threshold values
}

// IRProximityEvent represents an event indicating a change in IR proximity sensor readings.
type IRProximityEvent struct {
	Timestamp uint32   // Timestamp in milliseconds
	State     uint8    // Bitflag for state of triggered/untriggered sensors
	SensorMSB [7]uint8 // High eight bits of seven reflective IR sensor values
	SensorLSN [4]uint8 // Low four bits of seven reflective IR sensor values (packed)
}

// Device 12 - Bumpers

// BumperEvent represents an event indicating a change in bumper state.
type BumperEvent struct {
	Timestamp uint32 // Timestamp in milliseconds
	State     uint8  // New bumper state (0x00 = none, 0x40 = right, 0x80 = left, 0xC0 = both)
}

// Device 13 - Light Sensors

// GetLightValuesResponse contains values from the ambient light sensors.
type GetLightValuesResponse struct {
	Timestamp uint32 // Timestamp in milliseconds
	Left      uint16 // Left eye ambient light level in decipercent (0-1000)
	Right     uint16 // Right eye ambient light level in decipercent (0-1000)
}

// LightEvent represents an event indicating a change in ambient light levels.
type LightEvent struct {
	Timestamp uint32 // Timestamp in milliseconds
	State     uint8  // New ambient light state (4-7)
	Left      uint16 // Left eye ambient light level in decipercent (0-1000)
	Right     uint16 // Right eye ambient light level in decipercent (0-1000)
}

// Device 14 - Battery

// GetBatteryLevelResponse contains information about the battery level.
type GetBatteryLevelResponse struct {
	Timestamp uint32 // Timestamp in milliseconds
	Voltage   uint16 // Battery voltage in millivolts
	Percent   uint8  // Battery percent (0-100)
}

// BatteryLevelEvent represents an event indicating a change in battery level.
type BatteryLevelEvent struct {
	Timestamp uint32 // Timestamp in milliseconds
	Voltage   uint16 // Battery voltage in millivolts
	Percent   uint8  // Battery percent (0-100)
}

// Device 16 - Accelerometer

// GetAccelerometerResponse contains accelerometer data.
type GetAccelerometerResponse struct {
	Timestamp uint32 // Timestamp in milliseconds
	X         int16  // X axis acceleration in milli-g
	Y         int16  // Y axis acceleration in milli-g
	Z         int16  // Z axis acceleration in milli-g
}

// Device 17 - Touch Sensors

// TouchSensorEvent represents an event indicating a change in touch sensor state.
type TouchSensorEvent struct {
	Timestamp uint32 // Timestamp in milliseconds
	State     uint8  // Bitfield for the state of touch sensors (upper 4 bits)
}

// Device 19 - Docking Sensors

// GetDockingValuesResponse contains values from the docking sensors.
type GetDockingValuesResponse struct {
	Timestamp uint32   // Timestamp in milliseconds
	Contacts  uint8    // Docking contacts state (0 = disconnected, 1 = connected)
	IRSensor  [3]uint8 // Three IR sensor readings
}

// DockingSensorEvent represents an event indicating a change in docking sensor state.
type DockingSensorEvent struct {
	Timestamp uint32   // Timestamp in milliseconds
	Contacts  uint8    // Docking contacts state (0 = disconnected, 1 = connected)
	IRSensor  [3]uint8 // Three IR sensor readings
}

// Device 20 - Cliff Sensor

// CliffEvent represents an event indicating a change in cliff sensor reading.
type CliffEvent struct {
	Timestamp uint32 // Timestamp in milliseconds
	Cliff     uint8  // Cliff state (0 = no cliff, other values indicate cliff)
	Sensor    uint16 // Current cliff sensor value in millivolts
	Threshold uint16 // Current cliff sensor threshold in millivolts
}

// Device 100 - Connectivity

// GetIPv4AddressesResponse contains IPv4 addresses of the robot's network interfaces.
type GetIPv4AddressesResponse struct {
	Wlan0 uint32 // IPv4 address assigned to wlan0
	Wlan1 uint32 // IPv4 address assigned to wlan1
	Usb0  uint32 // IPv4 address assigned to usb0
}

// IPv4ChangeEvent represents an event indicating a change in IPv4 address for any
// of the robot's network interfaces.
type IPv4ChangeEvent struct {
	Wlan0 uint32 // New IPv4 address assigned to wlan0
	Wlan1 uint32 // New IPv4 address assigned to wlan1
	Usb0  uint32 // New IPv4 address assigned to usb0
}

// EasyUpdateEvent represents an event providing feedback during an easy update process.
// It includes information about the current stage of the update and its progress.
type EasyUpdateEvent struct {
	Timestamp uint32 // Timestamp in milliseconds
	Stage     uint8  // Character indicating the current stage of the update ('d' for downloading, 'i' for installing)
	Percent   int8   // Percentage of current stage completed (0-100), or -1 if there is an error
}
