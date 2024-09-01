# irobotroot

[![GoDoc](https://godoc.org/github.com/koeng101/irobotroot?status.svg)](https://godoc.org/github.com/koeng101/irobotroot)
[![Go Report Card](https://goreportcard.com/badge/github.com/koeng101/irobotroot)](https://goreportcard.com/report/github.com/koeng101/irobotroot)

`irobotroot` is a Go package that provides an interface for interacting with iRobot's Root robot using Bluetooth Low Energy (BLE). This package implements the Root Robot Bluetooth Low Energy Protocol, allowing developers to communicate with Root robots using any BLE-capable hardware.

Everything here is based off of the [iRobot Root BLE documentation](https://github.com/iRobotEducation/root-robot-ble-protocol?tab=readme-ov-file#device-2---markereraser).

## Features

- Full implementation of the Root Robot BLE Protocol
- Event-driven system for handling robot events and sensor data
- Concurrent processing of various robot subsystems
- Support for all Root robot devices (motors, sensors, lights, etc.)

## Installation

To install the package, use `go get`:
```
go get github.com/koeng101/irobotroot
```

## Usage

Here's a basic example of how to use the package:

```go
## Usage

Here's a basic example of how to use the package:

```go
package main

import (
    "fmt"
    "github.com/koeng101/irobotroot"
)

func main() {
    // Connect to the Root robot
    robot, err := irobotroot.ConnectToRootRobot()
    if err != nil {
        fmt.Println("Error connecting to Root robot:", err)
        return
    }
    defer robot.Disconnect()
    
    // Start the event listener
    robot.StartEventListener(100)
    defer robot.StopEventListener()
    
    // Drive forward for 100mm
    packet, err := robot.DriveDistance(100)
    if err != nil {
        fmt.Println("Error:", err)
        return
    }
    
    // Print the response packet
    fmt.Printf("Response: %+v\n", packet)
}
```

This example demonstrates how to:
1. Connect to a Root robot using `ConnectToRootRobot()`
2. Start the event listener
3. Send a command to the robot (in this case, to drive forward)
4. Handle the response
5. Properly disconnect and stop the event listener when done

For more detailed usage examples and API documentation, please refer to the [GoDoc](https://godoc.org/github.com/koeng101/irobotroot).
