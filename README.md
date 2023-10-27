# bicycle-safety-monitor
# CicloViário - Bicycle Safety Monitor Project

## Introduction
CicloViário is an Arduino-based project designed to enhance bicycle safety by monitoring various parameters during a cyclist's journey. It uses an ESP32 with 30 pins, an MPU6050 accelerometer, a VL53L0X laser distance sensor, a u-blox 6M NEO GPS module, and an SD card module to collect and store data related to bicycle safety.

## Project Overview
The CicloViário project includes the following features:
- Monitoring lateral distances using two VL53L0X sensors to detect nearby obstacles.
- Measuring variations and vibrations using the MPU6050 accelerometer.
- Tracking the cyclist's route using the u-blox 6M NEO GPS module.
- Saving collected data to an SD card for later analysis.

## Hardware Requirements
- ESP32 with 30 pins
- MPU6050 accelerometer
- VL53L0X laser distance sensors (x2)
- u-blox 6M NEO GPS module
- SD card module

### LED Indicators
- A red LED indicates sensor errors.
- A yellow LED signals the waiting state before data collection.
- A green LED remains on during data collection.

### Redundancy and Synchronization
In version 2.0, the internal RTC of the ESP32 is synchronized with the GPS clock, providing redundancy in case of GPS signal loss. Users can also utilize a mobile app as a GPS Tracker for parallel data collection, enabling later synchronization with the device's CSV data.

## Dependencies
This project relies on the following libraries:
- [Adafruit VL53L0X Library](https://github.com/adafruit/Adafruit_VL53L0X)
- [Adafruit MPU6050 Library](https://github.com/adafruit/Adafruit_MPU6050)
- [TinyGPS++ Library](https://github.com/mikalhart/TinyGPSPlus)
- [SD Card Library](https://www.arduino.cc/en/Reference/SD)

Make sure to install these libraries in your Arduino IDE before uploading the code.

## Usage
1. Connect the hardware components as per the provided schematic (insert a schematic image if available).
2. Open the Arduino IDE, import the required libraries, and load the CicloViário code.
3. Customize the code to suit your specific requirements, such as data collection intervals, data format, and SD card file names.
4. Upload the code to your ESP32 board.
5. Power on the system and observe the data collection process.
6. Collected data will be saved to the SD card in the specified format.

## Version History

- **Version 2.0 (Current Version)**
  - Improved stability and reliability.
  - Data collection interval reduced to 100 milliseconds for more precise monitoring.
  - Added internal RTC synchronization with the GPS clock for redundancy in case of GPS signal loss.
  - Users can utilize a mobile app as a GPS Tracker for parallel data collection, enabling later synchronization with the device's CSV data.
  - Added LEDs for system status: red for errors, yellow for waiting, and green for data collection.

- **Version 1.0 (Initial Release)**
  - Initial release of the CicloViário project.

## Contributing
Contributions to this project are welcome. Feel free to fork the repository, make changes, and submit pull requests for improvements or bug fixes.

## License
This project is licensed under the [GNU General Public License, version 3.0](LICENSE.md). See the [LICENSE.md](LICENSE.md) file for details.

## Contact
For questions or further information, please contact Jairo Ivo at jairoivo.brito@det.ufc.br.
