/*
Package irobotroot provides a Go interface for interacting with iRobot's Root robot using Bluetooth Low Energy (BLE).

This package implements the Root Robot Bluetooth Low Energy Protocol as documented by iRobot Education:
https://github.com/iRobotEducation/root-robot-ble-protocol

The Root robot is an educational robot designed to teach coding basics to students of various ages and skill levels.
This package allows developers to communicate with Root robots using any BLE-capable hardware, enabling custom
applications and integrations.

BLE Characteristics and Services:

The Root Robot uses three main BLE services:

 1. Root Identifier service (UUID: 48c5d828-ac2a-442d-97a3-0c9822b04979):
    An empty service used to uniquely identify Root Robots during device scanning.

 2. Device Information service (UUID: 0000180a-0000-1000-8000-00805f9b34fb):
    Provides static information about the Root Robot, including serial number, firmware version, hardware version,
    manufacturer name, and robot state.

 3. UART service (UUID: 6e400001-b5a3-f393-e0a9-e50e24dcca9e):
    Used for bi-directional communication with the robot. It contains two characteristics:
    - RX characteristic (UUID: 6e400002-b5a3-f393-e0a9-e50e24dcca9e): For sending commands to the robot.
    - TX characteristic (UUID: 6e400003-b5a3-f393-e0a9-e50e24dcca9e): For receiving responses and events from the robot.

Packet Structure:

Communication with the Root robot is done through 20-byte packets with the following structure:
- Device (1 byte): Identifies the subsystem of the robot (e.g., motors, sensors).
- Command (1 byte): Specifies the action or query for the device.
- Packet ID (1 byte): Used for packet identification and matching responses to requests.
- Payload (16 bytes): Contains command-specific data.
- Checksum (1 byte): CRC-8 checksum for data integrity verification.

Devices:

The Root robot is organized into several logical devices, each representing a subsystem:
- Device 0: General robot operations
- Device 1: Motors
- Device 2: Marker/Eraser
- Device 3: LED Lights
- Device 4: Color Sensor
- Device 5: Sound
- Device 11: IR Proximity
- Device 12: Bumpers
- Device 13: Light Sensors
- Device 14: Battery
- Device 16: Accelerometer
- Device 17: Touch Sensors
- Device 19: Docking Sensors
- Device 20: Cliff Sensor
- Device 100: Connectivity

Each device implements a set of commands for control and data retrieval.

Event Processing System:

The package implements an event-driven system using Go's concurrency features:
  - A main event listener goroutine processes incoming packets from the robot.
  - Each event type (e.g., bumper events, light sensor events) has a dedicated channel.
  - The main listener routes events to their respective channels based on the device and command.
  - Users of this package can consume events from these channels concurrently, allowing for
    modular and responsive handling of robot state changes and sensor data.

This system allows for efficient, concurrent processing of various robot events while
maintaining the simplicity and safety of Go's channel-based communication.

For detailed usage instructions and API documentation, refer to the function and type
descriptions within this package.
*/
package irobotroot

import (
	"encoding/binary"
	"encoding/hex"
	"errors"
	"log"
	"math"
	"time"

	"tinygo.org/x/bluetooth"
)

var (
	// UUID for the Root Identifier service
	rootServiceUUID, _ = ParseUUID("48c5d828-ac2a-442d-97a3-0c9822b04979")
	// UUID for the UART service RX characteristic
	rootCharUUID, _ = ParseUUID("486f72ea-c76f-4f9e-9032-8f598a9d9f3a")
	adapter         = bluetooth.DefaultAdapter
)

// ParseUUID converts a UUID string to a [16]byte array
// This is used to parse the UUIDs for BLE services and characteristics
func ParseUUID(s string) ([16]byte, error) {
	s = s[0:8] + s[9:13] + s[14:18] + s[19:23] + s[24:]
	b, err := hex.DecodeString(s)
	if err != nil {
		return [16]byte{}, err
	}
	var uuid [16]byte
	copy(uuid[:], b)
	return uuid, nil
}

// BleConnection interface defines the method for sending packets over BLE
type BleConnection interface {
	SendPacket(Packet) (Packet, error)
}

// Robot is a struct containing all the information for a iRobot rt1 robot.
type Robot struct {
	device              *bluetooth.Device
	char                *bluetooth.DeviceCharacteristic
	Conn                BleConnection
	incID               uint8                   // Incremental ID counter
	Events              chan Packet             // General event channel
	StopProjectEvents   chan StopProjectEvent   // Events for Device 0 - Stop Project
	ColorSensorEvents   chan ColorSensorEvent   // Events for Device 4 - Color Sensor
	BumperEvents        chan BumperEvent        // Events for Device 12 - Bumpers
	LightEvents         chan LightEvent         // Events for Device 13 - Light Sensors
	BatteryLevelEvents  chan BatteryLevelEvent  // Events for Device 14 - Battery
	IRProximityEvents   chan IRProximityEvent   // Events for Device 11 - IR Proximity
	TouchSensorEvents   chan TouchSensorEvent   // Events for Device 17 - Touch Sensors
	DockingSensorEvents chan DockingSensorEvent // Events for Device 19 - Docking Sensors
	CliffEvents         chan CliffEvent         // Events for Device 20 - Cliff Sensor
	IPv4ChangeEvents    chan IPv4ChangeEvent    // Events for Device 100 - Connectivity (IPv4 Change)
	EasyUpdateEvents    chan EasyUpdateEvent    // Events for Device 100 - Connectivity (Easy Update)
	MotorStallEvents    chan MotorStallEvent    // Events for Device 1 - Motors (Motor Stall)
}

// RootRobot interface defines the methods for interacting with the Root robot.
type RootRobot interface {
	// Event handling methods
	StartEventListener(bufferSize int)
	StopEventListener()

	// Device 0 - General
	GetVersions(GetVersionsPacket) (GetVersionsResponse, error)
	SetName(SetNamePacket) error
	GetName() (GetNameResponse, error)
	StopAndReset() error
	Disconnect() error
	EnableEvents(EnableEventsPacket) error
	DisableEvents(DisableEventsPacket) error
	GetEnabledEvents() (GetEnabledEventsResponse, error)
	GetSerialNumber() (GetSerialNumberResponse, error)
	GetSKU() (GetSKUResponse, error)

	// Device 1 - Motors
	SetLeftAndRightMotorSpeed(SetLeftAndRightMotorSpeedPacket) error
	SetLeftMotorSpeed(SetLeftMotorSpeedPacket) error
	SetRightMotorSpeed(SetRightMotorSpeedPacket) error
	DriveDistance(DriveDistancePacket) error
	RotateAngle(RotateAnglePacket) error
	SetGravityCompensation(SetGravityCompensationPacket) error
	ResetPosition() error
	GetPosition() (GetPositionResponse, error)
	NavigateToPosition(NavigateToPositionPacket) error
	Dock() error
	Undock() error
	DriveArc(DriveArcPacket) error

	// Device 2 - Marker/Eraser
	SetMarkerErasePosition(SetMarkerErasePositionPacket) error

	// Device 3 - LED Lights
	SetLEDAnimation(SetLEDAnimationPacket) error

	// Device 4 - Color Sensor
	GetColorSensorData(GetColorSensorDataPacket) (GetColorSensorDataResponse, error)

	// Device 5 - Sound
	PlayNote(PlayNotePacket) error
	StopSound() error
	SayPhrase(SayPhrasePacket) error
	PlaySweep(PlaySweepPacket) error

	// Device 11 - IR Proximity
	GetIRProximityValuesWithTimestamp() (GetIRProximityValuesWithTimestampResponse, error)
	GetPackedIRProximityValuesAndStates() (GetPackedIRProximityValuesAndStatesResponse, error)
	SetEventThresholds(SetEventThresholdsPacket) error
	GetEventThresholds() (GetEventThresholdsResponse, error)

	// Device 13 - Light Sensors
	GetLightValues() (GetLightValuesResponse, error)

	// Device 14 - Battery
	GetBatteryLevel() (GetBatteryLevelResponse, error)

	// Device 16 - Accelerometer
	GetAccelerometer() (GetAccelerometerResponse, error)

	// Device 19 - Docking Sensors
	GetDockingValues() (GetDockingValuesResponse, error)

	// Device 100 - Connectivity
	GetIPv4Addresses() (GetIPv4AddressesResponse, error)
	RequestEasyUpdate() error
}

func ConnectToRootRobot() (*Robot, error) {
	// Enable BLE interface
	err := adapter.Enable()
	if err != nil {
		return nil, err
	}

	connectionChannel := make(chan *Robot, 1)
	errorChannel := make(chan error, 1)

	// Start scanning for devices
	println("Scanning for Root robot...")
	err = adapter.Scan(func(adapter *bluetooth.Adapter, device bluetooth.ScanResult) {
		if device.LocalName() == "ROOT" {
			println("Found Root robot:", device.Address.String())
			adapter.StopScan()

			go func() {
				// Connect to the device
				connectedDevice, err := adapter.Connect(device.Address, bluetooth.ConnectionParams{})
				if err != nil {
					errorChannel <- err
					return
				}

				// Discover services
				services, err := connectedDevice.DiscoverServices([]bluetooth.UUID{bluetooth.NewUUID(rootServiceUUID)})
				if err != nil || len(services) == 0 {
					errorChannel <- errors.New("error discovering services")
					return
				}

				// Get the characteristic
				chars, err := services[0].DiscoverCharacteristics([]bluetooth.UUID{bluetooth.NewUUID(rootCharUUID)})
				if err != nil || len(chars) == 0 {
					errorChannel <- errors.New("error discovering characteristics")
					return
				}

				// Create a new Robot instance
				robot := &Robot{
					device: &connectedDevice,
					char:   &chars[0],
					Events: make(chan Packet, 100),
				}

				// Enable notifications
				err = robot.char.EnableNotifications(func(buf []byte) {
					packet := parsePacket(buf)
					robot.Events <- packet
				})
				if err != nil {
					errorChannel <- errors.New("error enabling notifications")
					return
				}

				// Start the event listener
				robot.StartEventListener(100)

				println("Connected to Root robot!")
				connectionChannel <- robot
			}()
		}
	})

	if err != nil {
		return nil, err
	}

	// Wait for connection, error, or timeout
	select {
	case robot := <-connectionChannel:
		return robot, nil
	case err := <-errorChannel:
		return nil, err
	case <-time.After(30 * time.Second):
		adapter.StopScan()
		return nil, errors.New("connection timeout")
	}
}

func parsePacket(data []byte) Packet {
	if len(data) != 20 {
		return Packet{} // Return an empty packet if the data is invalid
	}

	return Packet{
		Dev:     data[0],
		Cmd:     data[1],
		ID:      data[2],
		Payload: [16]byte(data[3:19]),
		CRC:     data[19],
	}
}

// getNextID increments and returns the next packet ID
func (r *Robot) getNextID() uint8 {
	r.incID = (r.incID + 1) & 0xFF
	return r.incID
}

/*
Device 0 - General
This device handles general robot operations and information retrieval.
*/

// GetVersions requests software and hardware version numbers from the robot.
func (r *Robot) GetVersions(request GetVersionsPacket) (GetVersionsResponse, error) {
	packet := Packet{
		Dev:     0,
		Cmd:     0,
		ID:      r.getNextID(),
		Payload: [16]byte{request.Board},
	}

	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetVersionsResponse{}, err
	}

	return GetVersionsResponse{
		Board:    response.Payload[0],
		FWMaj:    response.Payload[1],
		FWMin:    response.Payload[2],
		HWMaj:    response.Payload[3],
		HWMin:    response.Payload[4],
		BootMaj:  response.Payload[5],
		BootMin:  response.Payload[6],
		ProtoMaj: response.Payload[7],
		ProtoMin: response.Payload[8],
		Patch:    response.Payload[9],
	}, nil
}

// SetName sets a new BLE advertising name for the robot.
func (r *Robot) SetName(request SetNamePacket) error {
	packet := Packet{
		Dev:     0,
		Cmd:     1,
		ID:      r.getNextID(),
		Payload: request.Name,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// GetName requests the current BLE advertising name of the robot.
func (r *Robot) GetName() (GetNameResponse, error) {
	packet := Packet{
		Dev: 0,
		Cmd: 2,
		ID:  r.getNextID(),
	}

	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetNameResponse{}, err
	}

	return GetNameResponse{Name: response.Payload}, nil
}

// StopAndReset immediately stops the robot and cancels any pending actions.
func (r *Robot) StopAndReset() error {
	packet := Packet{
		Dev: 0,
		Cmd: 3,
		ID:  r.getNextID(),
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// Disconnect instructs the robot to immediately terminate the BLE connection.
func (r *Robot) Disconnect() error {
	packet := Packet{
		Dev: 0,
		Cmd: 6,
		ID:  r.getNextID(),
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// EnableEvents enables BLE notification for events by devices on the robot.
func (r *Robot) EnableEvents(request EnableEventsPacket) error {
	packet := Packet{
		Dev:     0,
		Cmd:     7,
		ID:      r.getNextID(),
		Payload: request.DevicesBitfield,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// DisableEvents disables BLE notification for events by devices on the robot.
func (r *Robot) DisableEvents(request DisableEventsPacket) error {
	packet := Packet{
		Dev:     0,
		Cmd:     9,
		ID:      r.getNextID(),
		Payload: request.DevicesBitfield,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// GetEnabledEvents requests a response packet containing a bitfield of the enabled devices.
func (r *Robot) GetEnabledEvents() (GetEnabledEventsResponse, error) {
	packet := Packet{
		Dev: 0,
		Cmd: 11,
		ID:  r.getNextID(),
	}

	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetEnabledEventsResponse{}, err
	}

	return GetEnabledEventsResponse{DevicesBitfield: response.Payload}, nil
}

// GetSerialNumber requests the product serial number.
func (r *Robot) GetSerialNumber() (GetSerialNumberResponse, error) {
	packet := Packet{
		Dev: 0,
		Cmd: 14,
		ID:  r.getNextID(),
	}

	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetSerialNumberResponse{}, err
	}

	var serialNumber [12]byte
	copy(serialNumber[:], response.Payload[:12])
	return GetSerialNumberResponse{SerialNumber: serialNumber}, nil
}

// GetSKU requests the product SKU.
func (r *Robot) GetSKU() (GetSKUResponse, error) {
	packet := Packet{
		Dev: 0,
		Cmd: 15,
		ID:  r.getNextID(),
	}

	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetSKUResponse{}, err
	}

	return GetSKUResponse{SKU: response.Payload}, nil
}

/*
Device 1 - Motors
This device controls the robot's movement and positioning.
*/

// SetLeftAndRightMotorSpeed sets the linear velocity for both motors of the robot.
// leftSpeed and rightSpeed are in units of mm/s, with positive values for forward movement
// and negative values for backward movement. Valid range is -100 to 100 mm/s.
func (r *Robot) SetLeftAndRightMotorSpeed(request SetLeftAndRightMotorSpeedPacket) error {
	if math.Abs(float64(request.LeftSpeed)) > 100 || math.Abs(float64(request.RightSpeed)) > 100 {
		return errors.New("speed must be between -100 and 100 mm/s")
	}

	var payload [16]byte
	binary.BigEndian.PutUint32(payload[0:4], uint32(request.LeftSpeed))
	binary.BigEndian.PutUint32(payload[4:8], uint32(request.RightSpeed))

	packet := Packet{
		Dev:     1,
		Cmd:     4,
		ID:      r.getNextID(),
		Payload: payload,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// SetLeftMotorSpeed sets the linear velocity for the left motor only.
// speed is in units of mm/s, with positive values for forward movement
// and negative values for backward movement. Valid range is -100 to 100 mm/s.
func (r *Robot) SetLeftMotorSpeed(request SetLeftMotorSpeedPacket) error {
	if math.Abs(float64(request.LeftSpeed)) > 100 {
		return errors.New("speed must be between -100 and 100 mm/s")
	}

	var payload [16]byte
	binary.BigEndian.PutUint32(payload[0:4], uint32(request.LeftSpeed))

	packet := Packet{
		Dev:     1,
		Cmd:     6,
		ID:      r.getNextID(),
		Payload: payload,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// SetRightMotorSpeed sets the linear velocity for the right motor only.
// speed is in units of mm/s, with positive values for forward movement
// and negative values for backward movement. Valid range is -100 to 100 mm/s.
func (r *Robot) SetRightMotorSpeed(request SetRightMotorSpeedPacket) error {
	if math.Abs(float64(request.RightSpeed)) > 100 {
		return errors.New("speed must be between -100 and 100 mm/s")
	}

	var payload [16]byte
	binary.BigEndian.PutUint32(payload[0:4], uint32(request.RightSpeed))

	packet := Packet{
		Dev:     1,
		Cmd:     7,
		ID:      r.getNextID(),
		Payload: payload,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// DriveDistance instructs the robot to drive a set distance in a straight line.
// distance is in units of mm, with positive values for forward movement
// and negative values for backward movement.
func (r *Robot) DriveDistance(request DriveDistancePacket) error {
	var payload [16]byte
	binary.BigEndian.PutUint32(payload[0:4], uint32(request.Distance))

	packet := Packet{
		Dev:     1,
		Cmd:     8,
		ID:      r.getNextID(),
		Payload: payload,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// RotateAngle instructs the robot to rotate in place by a set angle.
// angle is in units of decidegrees (1/10 of degrees), with positive values for clockwise rotation
// and negative values for counterclockwise rotation.
func (r *Robot) RotateAngle(request RotateAnglePacket) error {
	var payload [16]byte
	binary.BigEndian.PutUint32(payload[0:4], uint32(request.Angle))

	packet := Packet{
		Dev:     1,
		Cmd:     12,
		ID:      r.getNextID(),
		Payload: payload,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// SetGravityCompensation sets the amount of correction used during vertical driving
// and when gravity compensation is active.
// active: 0 (Always off), 1 (Always on), 2 (Enabled when marker is down)
// amount: Compensation amount in decipercent (0-1000, representing 0-100%)
func (r *Robot) SetGravityCompensation(request SetGravityCompensationPacket) error {
	if request.Amount > 1000 {
		return errors.New("amount must be between 0 and 1000")
	}

	var payload [16]byte
	payload[0] = request.Active
	binary.BigEndian.PutUint16(payload[1:3], request.Amount)

	packet := Packet{
		Dev:     1,
		Cmd:     13,
		ID:      r.getNextID(),
		Payload: payload,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// ResetPosition resets the estimated robot location to (0, 0) and orientation to 90 degrees of yaw.
func (r *Robot) ResetPosition() error {
	packet := Packet{
		Dev: 1,
		Cmd: 15,
		ID:  r.getNextID(),
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// GetPosition requests the current estimated robot location and orientation.
func (r *Robot) GetPosition() (GetPositionResponse, error) {
	packet := Packet{
		Dev: 1,
		Cmd: 16,
		ID:  r.getNextID(),
	}

	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetPositionResponse{}, err
	}

	return GetPositionResponse{
		Timestamp: binary.BigEndian.Uint32(response.Payload[0:4]),
		X:         int32(binary.BigEndian.Uint32(response.Payload[4:8])),
		Y:         int32(binary.BigEndian.Uint32(response.Payload[8:12])),
		Heading:   int16(binary.BigEndian.Uint16(response.Payload[12:14])),
	}, nil
}

// NavigateToPosition instructs the robot to navigate to a coordinate location with an optional end orientation.
// x, y: Target coordinates in millimeters
// heading: Final orientation in decidegrees (0-3599), or -1 to allow robot to choose
// Note: This command is only supported with protocol version 1.4 or greater.
func (r *Robot) NavigateToPosition(request NavigateToPositionPacket) error {
	var payload [16]byte

	binary.BigEndian.PutUint32(payload[0:4], uint32(request.X))
	binary.BigEndian.PutUint32(payload[4:8], uint32(request.Y))
	binary.BigEndian.PutUint16(payload[8:10], uint16(request.Heading))

	packet := Packet{
		Dev:     1,
		Cmd:     17,
		ID:      r.getNextID(),
		Payload: payload,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// Dock triggers a docking action.
// Note: This command is only supported on Create 3 robots with protocol version 1.5 or greater.
func (r *Robot) Dock() error {
	packet := Packet{
		Dev: 1,
		Cmd: 19,
		ID:  r.getNextID(),
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// Undock triggers an undocking action.
// Note: This command is only supported on Create 3 robots with protocol version 1.5 or greater.
func (r *Robot) Undock() error {
	packet := Packet{
		Dev: 1,
		Cmd: 20,
		ID:  r.getNextID(),
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// DriveArc instructs the robot to drive the length of an arc defined by a set angle and radius.
// angle: Arc angle in decidegrees, positive for clockwise, negative for counterclockwise
// radius: Arc radius in millimeters, positive for rotation point to the right, negative for left
func (r *Robot) DriveArc(request DriveArcPacket) error {
	var payload [16]byte
	binary.BigEndian.PutUint32(payload[0:4], uint32(request.Angle))
	binary.BigEndian.PutUint32(payload[4:8], uint32(request.Radius))

	packet := Packet{
		Dev:     1,
		Cmd:     27,
		ID:      r.getNextID(),
		Payload: payload,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

/*
Device 2 - Marker/Eraser
*/

// SetMarkerErasePosition sets the position of the marker/eraser actuator.
// position: 0 (Marker Up/Eraser Up), 1 (Marker Down/Eraser Up), or 2 (Marker Up/Eraser Down)
func (r *Robot) SetMarkerErasePosition(request SetMarkerErasePositionPacket) error {
	packet := Packet{
		Dev:     2,
		Cmd:     0,
		ID:      r.getNextID(),
		Payload: [16]byte{request.Position},
	}
	_, err := r.Conn.SendPacket(packet)
	return err
}

/*
Device 3 - LED Lights
*/

// SetLEDAnimation sets the LED cross animation type and color.
// state: 0 (Off), 1 (On), 2 (Blink Animation), or 3 (Spin Animation)
// red, green, blue: Brightness levels for each LED channel (0-255)
func (r *Robot) SetLEDAnimation(request SetLEDAnimationPacket) error {
	packet := Packet{
		Dev:     3,
		Cmd:     2,
		ID:      r.getNextID(),
		Payload: [16]byte{request.State, request.Red, request.Green, request.Blue},
	}
	_, err := r.Conn.SendPacket(packet)
	return err
}

/*
Device 4 - Color Sensor
*/

// GetColorSensorData requests color sensor data.
// bank: 0-3 (sensor bank to read from)
// lighting: 0 (Off), 1 (Red), 2 (Green), 3 (Blue), or 4 (All)
// format: 0 (12-bit ADC counts) or 1 (millivolts)
func (r *Robot) GetColorSensorData(request GetColorSensorDataPacket) (GetColorSensorDataResponse, error) {
	packet := Packet{
		Dev:     4,
		Cmd:     1,
		ID:      r.getNextID(),
		Payload: [16]byte{request.Bank, request.Lighting, request.Format},
	}
	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetColorSensorDataResponse{}, err
	}
	var data [8]uint16
	for i := 0; i < 8; i++ {
		data[i] = binary.BigEndian.Uint16(response.Payload[i*2 : i*2+2])
	}
	return GetColorSensorDataResponse{Data: data}, nil
}

/*
Device 5 - Sound
*/

// PlayNote plays a frequency from the robot's buzzer.
// frequency: Frequency of note in Hz
// duration: Duration of note in milliseconds
func (r *Robot) PlayNote(request PlayNotePacket) error {
	var payload [16]byte
	binary.BigEndian.PutUint32(payload[0:4], request.Frequency)
	binary.BigEndian.PutUint16(payload[4:6], request.Duration)
	packet := Packet{
		Dev:     5,
		Cmd:     0,
		ID:      r.getNextID(),
		Payload: payload,
	}
	_, err := r.Conn.SendPacket(packet)
	return err
}

// StopSound immediately stops playing any sound.
func (r *Robot) StopSound() error {
	packet := Packet{
		Dev: 5,
		Cmd: 1,
		ID:  r.getNextID(),
	}
	_, err := r.Conn.SendPacket(packet)
	return err
}

// SayPhrase speaks a text string in robot language. Note: you can only say
// a max 16 byte phrase.
// phrase: UTF-8 encoded string to speak.
func (r *Robot) SayPhrase(request SayPhrasePacket) error {
	packet := Packet{
		Dev:     5,
		Cmd:     4,
		ID:      r.getNextID(),
		Payload: request.Phrase,
	}

	_, err := r.Conn.SendPacket(packet)
	return err
}

// PlaySweep sweeps linearly between two frequencies over a specified duration.
// startFreq, endFreq: Start and end frequencies in milli-Hertz
// duration: Time to play sweep in milliseconds
// attack, release: Time to ramp volume from/to zero in milliseconds
// volume: Volume from 0 (silent) to 255 (max)
// modulationType: 0 (Disabled), 1 (Volume), 2 (Pulse Width), or 3 (Frequency)
// modulationRate: Modulation rate in Hertz
// append: If non-zero, the Sweep will start after the currently playing Sweep is finished
func (r *Robot) PlaySweep(request PlaySweepPacket) error {
	var payload [16]byte
	binary.BigEndian.PutUint32(payload[0:4], request.StartFrequency)
	binary.BigEndian.PutUint32(payload[4:8], request.EndFrequency)
	binary.BigEndian.PutUint16(payload[8:10], request.Duration)
	payload[10] = request.Attack
	payload[11] = request.Release
	payload[12] = request.Volume
	payload[13] = request.ModulationType
	payload[14] = request.ModulationRate
	payload[15] = request.Append
	packet := Packet{
		Dev:     5,
		Cmd:     5,
		ID:      r.getNextID(),
		Payload: payload,
	}
	_, err := r.Conn.SendPacket(packet)
	return err
}

/*
Device 11 - IR Proximity
*/

// GetIRProximityValuesWithTimestamp requests values from the leftmost six IR proximity sensors.
func (r *Robot) GetIRProximityValuesWithTimestamp() (GetIRProximityValuesWithTimestampResponse, error) {
	packet := Packet{
		Dev: 11,
		Cmd: 1,
		ID:  r.getNextID(),
	}
	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetIRProximityValuesWithTimestampResponse{}, err
	}
	result := GetIRProximityValuesWithTimestampResponse{
		Timestamp: binary.BigEndian.Uint32(response.Payload[0:4]),
	}
	for i := 0; i < 6; i++ {
		result.Sensor[i] = binary.BigEndian.Uint16(response.Payload[4+i*2 : 6+i*2])
	}
	return result, nil
}

// GetPackedIRProximityValuesAndStates requests values from all seven IR proximity sensors.
func (r *Robot) GetPackedIRProximityValuesAndStates() (GetPackedIRProximityValuesAndStatesResponse, error) {
	packet := Packet{
		Dev: 11,
		Cmd: 2,
		ID:  r.getNextID(),
	}
	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetPackedIRProximityValuesAndStatesResponse{}, err
	}
	result := GetPackedIRProximityValuesAndStatesResponse{
		Timestamp: binary.BigEndian.Uint32(response.Payload[0:4]),
		State:     response.Payload[4],
	}
	copy(result.SensorMSB[:], response.Payload[5:12])
	copy(result.SensorLSN[:], response.Payload[12:16])
	return result, nil
}

// SetEventThresholds sets thresholds and shared hysteresis for IR sensors.
// hysteresis: Hysteresis in counts
// sensors: Array of 7 threshold values for each sensor (0-4095)
func (r *Robot) SetEventThresholds(request SetEventThresholdsPacket) error {
	var payload [16]byte
	binary.BigEndian.PutUint16(payload[0:2], request.Hysteresis)
	for i, sensor := range request.Sensor {
		binary.BigEndian.PutUint16(payload[2+i*2:4+i*2], sensor)
	}
	packet := Packet{
		Dev:     11,
		Cmd:     3,
		ID:      r.getNextID(),
		Payload: payload,
	}
	_, err := r.Conn.SendPacket(packet)
	return err
}

// GetEventThresholds requests the current thresholds and shared hysteresis for IR sensors.
func (r *Robot) GetEventThresholds() (GetEventThresholdsResponse, error) {
	packet := Packet{
		Dev: 11,
		Cmd: 4,
		ID:  r.getNextID(),
	}
	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetEventThresholdsResponse{}, err
	}
	result := GetEventThresholdsResponse{
		Hysteresis: binary.BigEndian.Uint16(response.Payload[0:2]),
	}
	for i := 0; i < 7; i++ {
		result.Sensor[i] = binary.BigEndian.Uint16(response.Payload[2+i*2 : 4+i*2])
	}
	return result, nil
}

/*
Device 13 - Light Sensors
*/

// GetLightValues requests values from the ambient light sensors.
func (r *Robot) GetLightValues() (GetLightValuesResponse, error) {
	packet := Packet{
		Dev: 13,
		Cmd: 1,
		ID:  r.getNextID(),
	}
	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetLightValuesResponse{}, err
	}
	return GetLightValuesResponse{
		Timestamp: binary.BigEndian.Uint32(response.Payload[0:4]),
		Left:      binary.BigEndian.Uint16(response.Payload[4:6]),
		Right:     binary.BigEndian.Uint16(response.Payload[6:8]),
	}, nil
}

/*
Device 14 - Battery
*/

// GetBatteryLevel requests the current battery level.
func (r *Robot) GetBatteryLevel() (GetBatteryLevelResponse, error) {
	packet := Packet{
		Dev: 14,
		Cmd: 1,
		ID:  r.getNextID(),
	}
	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetBatteryLevelResponse{}, err
	}
	return GetBatteryLevelResponse{
		Timestamp: binary.BigEndian.Uint32(response.Payload[0:4]),
		Voltage:   binary.BigEndian.Uint16(response.Payload[4:6]),
		Percent:   response.Payload[6],
	}, nil
}

/*
Device 16 - Accelerometer
*/

// GetAccelerometer requests accelerometer data.
func (r *Robot) GetAccelerometer() (GetAccelerometerResponse, error) {
	packet := Packet{
		Dev: 16,
		Cmd: 1,
		ID:  r.getNextID(),
	}
	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetAccelerometerResponse{}, err
	}
	return GetAccelerometerResponse{
		Timestamp: binary.BigEndian.Uint32(response.Payload[0:4]),
		X:         int16(binary.BigEndian.Uint16(response.Payload[4:6])),
		Y:         int16(binary.BigEndian.Uint16(response.Payload[6:8])),
		Z:         int16(binary.BigEndian.Uint16(response.Payload[8:10])),
	}, nil
}

/*
Device 19 - Docking Sensors
*/

// GetDockingValues requests values from the docking sensors.
func (r *Robot) GetDockingValues() (GetDockingValuesResponse, error) {
	packet := Packet{
		Dev: 19,
		Cmd: 1,
		ID:  r.getNextID(),
	}
	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetDockingValuesResponse{}, err
	}
	result := GetDockingValuesResponse{
		Timestamp: binary.BigEndian.Uint32(response.Payload[0:4]),
		Contacts:  response.Payload[4],
	}
	copy(result.IRSensor[:], response.Payload[5:8])
	return result, nil
}

/*
Device 100 - Connectivity
*/
// GetIPv4Addresses requests the IPv4 addresses of all of the robot's interfaces.
func (r *Robot) GetIPv4Addresses() (GetIPv4AddressesResponse, error) {
	packet := Packet{
		Dev: 100,
		Cmd: 1,
		ID:  r.getNextID(),
	}
	response, err := r.Conn.SendPacket(packet)
	if err != nil {
		return GetIPv4AddressesResponse{}, err
	}
	return GetIPv4AddressesResponse{
		Wlan0: binary.BigEndian.Uint32(response.Payload[0:4]),
		Wlan1: binary.BigEndian.Uint32(response.Payload[4:8]),
		Usb0:  binary.BigEndian.Uint32(response.Payload[8:12]),
	}, nil
}

// RequestEasyUpdate requests the robot to download and install the most recent firmware update.
func (r *Robot) RequestEasyUpdate() error {
	packet := Packet{
		Dev: 100,
		Cmd: 2,
		ID:  r.getNextID(),
	}
	_, err := r.Conn.SendPacket(packet)
	return err
}

/*
Event Processing in the Root Robot System

The event processing system for the Root robot is designed to efficiently handle
asynchronous events from various robot subsystems using Go's concurrency model with
channels. This approach perfectly aligns with the event-driven nature of robot
operations, where multiple sensors and systems can generate events independently
and simultaneously.

How it works:
1. The StartEventListener function initializes buffered channels for each type of event.
   The buffer size allows for temporary storage of events, preventing immediate blocking
   if the consumer is momentarily busy.

2. A single goroutine (processEvents) is started to handle all incoming events. This
   centralized processing ensures that events are handled sequentially, avoiding race
   conditions and simplifying synchronization.

3. The processEvents function continuously reads from the main Events channel and
   routes each event to its specific channel based on the device (Dev) and command (Cmd)
   in the packet.

4. Each specific event channel (e.g., BumperEvents, LightEvents) can be independently
   consumed by different parts of the application, allowing for modular and concurrent
   handling of different event types.

Importance of Channel Consumers (Sinks):
It's crucial to note that for each channel created in StartEventListener, there must
be a corresponding consumer or "sink" in the application. If any channel lacks a
consumer, it can lead to a deadlock situation. This occurs because once a channel's
buffer is full, any attempt to send more data to it will block until space is available.
If no part of the application is reading from a channel, it will eventually fill up,
causing the processEvents goroutine to block indefinitely when trying to send to that
channel. This, in turn, would prevent processing of any further events, effectively
deadlocking the entire event system. Therefore, when implementing a system using this
event model, developers must ensure that every event channel has at least one consumer,
even if it's just a goroutine that reads and discards events from channels that aren't
actively used.

The StopEventListener function provides a clean way to shut down the event processing
system by closing all channels, which will cause the processEvents goroutine to
terminate once all pending events have been processed.
*/

// StartEventListener initializes the event processing system for the Root robot.
// It creates buffered channels for various event types and starts a goroutine
// to process incoming events.
//
// The bufferSize parameter determines the capacity of each event channel.
// A larger buffer size can help prevent blocking in high-event scenarios,
// but may increase memory usage.
//
// This function should be called before any event-generating operations
// are performed on the robot. Ensure that consumers are set up for each
// event channel to prevent potential deadlocks.
//
// Example usage:
//
//	robot.StartEventListener(100)
//	// Set up event consumers here
//	go processColorSensorEvents(robot.ColorSensorEvents)
//	go processBumperEvents(robot.BumperEvents)
//	// ... set up other event consumers as needed
func (r *Robot) StartEventListener(bufferSize int) {
	r.Events = make(chan Packet, bufferSize)
	r.StopProjectEvents = make(chan StopProjectEvent, bufferSize)
	r.ColorSensorEvents = make(chan ColorSensorEvent, bufferSize)
	r.BumperEvents = make(chan BumperEvent, bufferSize)
	r.LightEvents = make(chan LightEvent, bufferSize)
	r.BatteryLevelEvents = make(chan BatteryLevelEvent, bufferSize)
	r.IRProximityEvents = make(chan IRProximityEvent, bufferSize)
	r.TouchSensorEvents = make(chan TouchSensorEvent, bufferSize)
	r.DockingSensorEvents = make(chan DockingSensorEvent, bufferSize)
	r.CliffEvents = make(chan CliffEvent, bufferSize)
	r.IPv4ChangeEvents = make(chan IPv4ChangeEvent, bufferSize)
	r.EasyUpdateEvents = make(chan EasyUpdateEvent, bufferSize)
	r.MotorStallEvents = make(chan MotorStallEvent, bufferSize)

	go r.processEvents()
}

func (r *Robot) processEvents() {
	for packet := range r.Events {
		switch packet.Dev {
		case 0:
			if packet.Cmd == 4 {
				r.StopProjectEvents <- StopProjectEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
				}
			}
		case 4:
			if packet.Cmd == 2 {
				var colors [32]uint8
				for i := 0; i < 32; i++ {
					colors[i] = packet.Payload[i/2] >> (4 * (i % 2)) & 0x0F
				}
				r.ColorSensorEvents <- ColorSensorEvent{Colors: colors}
			}
		case 12:
			if packet.Cmd == 0 {
				r.BumperEvents <- BumperEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
					State:     packet.Payload[4],
				}
			}
		case 13:
			if packet.Cmd == 0 {
				r.LightEvents <- LightEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
					State:     packet.Payload[4],
					Left:      binary.BigEndian.Uint16(packet.Payload[5:7]),
					Right:     binary.BigEndian.Uint16(packet.Payload[7:9]),
				}
			}
		case 14:
			if packet.Cmd == 0 {
				r.BatteryLevelEvents <- BatteryLevelEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
					Voltage:   binary.BigEndian.Uint16(packet.Payload[4:6]),
					Percent:   packet.Payload[6],
				}
			}
		case 11:
			if packet.Cmd == 0 {
				event := IRProximityEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
					State:     packet.Payload[4],
				}
				copy(event.SensorMSB[:], packet.Payload[5:12])
				copy(event.SensorLSN[:], packet.Payload[12:16])
				r.IRProximityEvents <- event
			}
		case 17:
			if packet.Cmd == 0 {
				r.TouchSensorEvents <- TouchSensorEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
					State:     packet.Payload[4] >> 4,
				}
			}
		case 19:
			if packet.Cmd == 0 {
				event := DockingSensorEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
					Contacts:  packet.Payload[4],
				}
				copy(event.IRSensor[:], packet.Payload[5:8])
				r.DockingSensorEvents <- event
			}
		case 20:
			if packet.Cmd == 0 {
				r.CliffEvents <- CliffEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
					Cliff:     packet.Payload[4],
					Sensor:    binary.BigEndian.Uint16(packet.Payload[5:7]),
					Threshold: binary.BigEndian.Uint16(packet.Payload[7:9]),
				}
			}
		case 100:
			if packet.Cmd == 0 {
				r.IPv4ChangeEvents <- IPv4ChangeEvent{
					Wlan0: binary.BigEndian.Uint32(packet.Payload[0:4]),
					Wlan1: binary.BigEndian.Uint32(packet.Payload[4:8]),
					Usb0:  binary.BigEndian.Uint32(packet.Payload[8:12]),
				}
			} else if packet.Cmd == 3 {
				r.EasyUpdateEvents <- EasyUpdateEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
					Stage:     packet.Payload[4],
					Percent:   int8(packet.Payload[5]),
				}
			}
		case 1:
			if packet.Cmd == 29 {
				r.MotorStallEvents <- MotorStallEvent{
					Timestamp: binary.BigEndian.Uint32(packet.Payload[0:4]),
					Motor:     packet.Payload[4],
					Cause:     packet.Payload[5],
				}
			}
		default:
			log.Printf("Unknown packet received: Dev: %d, Cmd: %d, ID: %d, Payload: %v, CRC: %d\n",
				packet.Dev, packet.Cmd, packet.ID, packet.Payload, packet.CRC)
		}
	}
}

// StopEventListener shuts down the event processing system for the Root robot.
// It closes the main event channel and all specific event channels.
//
// This function should be called when you're done using the robot or
// before reinitializing the event system. It ensures proper cleanup
// and allows the event processing goroutine to terminate.
//
// After calling this function, no more events will be processed.
// If you need to restart event processing, call StartEventListener again.
//
// Example usage:
//
//	// When you're done using the robot:
//	robot.StopEventListener()
func (r *Robot) StopEventListener() {
	close(r.Events) // Close the main event channel
	// Close all specific event channels
	close(r.StopProjectEvents)
	close(r.ColorSensorEvents)
	close(r.BumperEvents)
	close(r.LightEvents)
	close(r.BatteryLevelEvents)
	close(r.IRProximityEvents)
	close(r.TouchSensorEvents)
	close(r.DockingSensorEvents)
	close(r.CliffEvents)
	close(r.IPv4ChangeEvents)
	close(r.EasyUpdateEvents)
	close(r.MotorStallEvents)
}
