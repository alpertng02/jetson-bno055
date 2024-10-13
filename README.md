# Jetson-BNO055 

## Overview
This project is the Jetson port of the [Bosch BNO055 sensor driver](https://github.com/boschsensortec/BNO055_SensorAPI) to provide orientation and motion sensing capabilities.

## Features
- 9-axis sensor fusion
- Real-time orientation data
- Easy integration with Jetson devices

## Requirements
- C++17 or higher
- CMake (3.2 or higher)
- Make or Ninja

## Installation
1. Clone the repository with submodules:
    ```sh
    git clone --recurse-submodules https://github.com/yourusername/jetson-bno055.git
    ```
2. Navigate to the project directory:
    ```sh
    cd jetson-bno055
    ```
3. Create build directory:
    ```sh
    mkdir build && cd build
    ```
4. Build: 
    ```sh
    cmake ..
    make
    ```

## Usage
1. Connect the BNO055 sensor to your Jetson device through GPIO 3 and 5.
2. Run the example:
    ```sh
    ./bno055_test
    ```

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributing
Contributions are welcome! Please open an issue or submit a pull request.

## Contact
For any questions or suggestions, please contact alpert.guven@gmail.com.

