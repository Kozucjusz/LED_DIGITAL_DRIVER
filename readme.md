## 1. Introduction
The controller enables precise adjustment of LED brightness and color using digital potentiometers, powered by a stable current source. The entire system is managed by a microcontroller programmed in C within the VS Code environment, using the HAL and custom control libraries.

In addition, the project implements **1‑Wire** communication from an STM32 microcontroller via the **DS28E18** communication bridge. This solution allows control data to be transmitted easily using a single signal line, which significantly simplifies the connection topology and reduces the number of required microcontroller pins. As a result, it is possible to create simple and uncomplicated designs in which all communication is based on just a single transmission line and ground.

<a href="Schematics/LED_DIGITAL_DRIVER_SCH.png"><img src="Schematics/LED_DIGITAL_DRIVER_SCH.png" width=75% height=75%  ></a>

## 2. PCB Description and Key Components
The PCB was designed using proven integrated circuits and discrete components.
Key components include:
- **AD5282** - a digital potentiometer enabling precise adjustment of operating current parameters and colors.
- **LT3080** - linear voltage regulators providing high power-supply stability, operating as current sources.
- **DS28E18** - a communication converter/bridge that enables flexible connections and simplifies the system topology.
- Power resistors used to limit and stabilize currents in the LED circuits.

The PCB was designed as a small, double-sided board, which facilitates manufacturing and testing.

<a href="Schematics/pcb2.jpg"><img src="Schematics/pcb2.jpg" width=22% height=22% ></a>
<a href="Schematics/pcb1.jpg"><img src="Schematics/pcb1.jpg" width=21% height=21% ></a>
<a href="Schematics/operation.gif"><img src="Schematics/operation.gif" width=22% height=22% ></a>

## 3. Functioning
The system is controlled by an **STM32** microcontroller, for which the firmware was developed in **C**.
The project was created in the **Visual Studio Code** environment, using extensions supporting **STM32** devices and the **STM32CubeMX** configurator.

Communication with the **DS28E18** bridge is handled via the **1-Wire** bus, operating in Single Wire (Half-Duplex) mode supported by the **STM32**. The bridge acts as a translator between the **1-Wire** interface and the I²C bus, to which the digital potentiometers are connected.

The operating sequence is as follows:
1. The microcontroller initializes the 1-Wire communication and configures the DS28E18 bridge.
2. A sequence of I²C commands defining the potentiometer values is sent to the bridge.
3. The sequence is stored in the internal memory of the bridge.
4. When triggered by the STM32, the DS28E18 autonomously reproduces this sequence on the I²C bus, enabling precise control of the LED operating parameters.

This approach significantly simplifies the wiring (a single data line plus ground) while maintaining flexibility in controlling multiple elements from the microcontroller level.

## 5. Library
The KM_lib.h* header files contain all definitions of useful functions as well as the project configuration. This approach significantly simplifies transferring the libraries between projects and adapting them to specific requirements.
- KMlib_1wire.h - allows selection of the appropriate USART module and provides functions for full 1-Wire protocol communication.
- KMlib_DS28E18.h - configuration of the desired protocol on the bridge output (I²C or SPI) along with its settings. At present, the library supports only the I²C protocol. The most important functions include:
    - DS28E18_ROMID - retrieves the device address. The full address will not be read unless the DS28E18_INIT function has been called at least once beforehand.
    - DS28E18_INIT - on the first call, retrieves and sets the full device address. On the second call, allows configuration of the GPIO control register.
    - DS28E18_STATUS - retrieves device information and clears the POR flag.
    - DS28E18_CONFIG - sets the output protocol of the bridge.
    - DS28E18_WRITE_SEQ - writes a byte sequence to the device memory and executes it.
- KMlib_AD5282.h - defines all available potentiometer configuration options using simple, descriptive names.

## 6. Future Project Development
Planned directions for further development include:
- Implementation of advanced lighting effects (fade, smooth transitions).
- Addition of a wireless communication module (Bluetooth Low Energy or Wi-Fi).
- Integration of a display and color selection using a color map.
