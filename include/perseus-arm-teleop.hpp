#pragma once

#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <cstdint>

class ST3215ServoReader 
{
public:
    /**
     * @brief Constructs a new ST3215ServoReader
     * @param port Serial port path (e.g., "/dev/ttyUSB0")
     * @param baud_rate Baud rate for serial communication
     */
    ST3215ServoReader(const std::string& port, unsigned int baud_rate);
    
    /**
     * @brief Destructor ensures serial port is properly closed
     */
    ~ST3215ServoReader();

    /**
     * @brief Reads the current position of a specified servo
     * @param servo_id ID of the servo to read from
     * @return Current position value (0-4095)
     * @throws std::runtime_error if communication fails
     */
    uint16_t readPosition(uint8_t servo_id);

private:
    /**
     * @brief Creates a read command packet according to ST3215 protocol
     * @param id Servo ID
     * @param address Memory address to read from
     * @param size Number of bytes to read
     * @return Vector containing the complete command packet
     */
    std::vector<uint8_t> _createReadCommand(uint8_t id, uint8_t address, uint8_t size);

    boost::asio::io_service _io_service;
    boost::asio::serial_port _serial_port;
};