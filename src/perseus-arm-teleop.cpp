#include "perseus-arm-teleop.hpp"
#include <stdexcept>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace boost::asio;

ST3215ServoReader::ST3215ServoReader(const std::string& port, unsigned int baud_rate)
    : _io_service(), _serial_port(_io_service)
{
    try {
        _serial_port.open(port);
        _serial_port.set_option(serial_port::baud_rate(baud_rate));
        _serial_port.set_option(serial_port::character_size(8));
        _serial_port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        _serial_port.set_option(serial_port::parity(serial_port::parity::none));
        _serial_port.set_option(serial_port::flow_control(serial_port::flow_control::none));
    }
    catch (const boost::system::system_error& e) {
        throw std::runtime_error(std::string("Failed to open serial port: ") + e.what());
    }
}

ST3215ServoReader::~ST3215ServoReader() 
{
    try {
        if (_serial_port.is_open()) 
        {
            _serial_port.close();
        }
    }
    catch (...) {
        // Ignore errors in destructor
    }
}

uint16_t ST3215ServoReader::readPosition(uint8_t servo_id) 
{
    try {
        // Create read position command packet
        std::vector<uint8_t> command = _createReadCommand(servo_id, 0x38, 2); // 0x38 (56) is position register
        
        // Send command
        boost::system::error_code write_ec;
        size_t written = boost::asio::write(_serial_port, buffer(command), write_ec);
        if (write_ec) {
            throw std::runtime_error(std::string("Write error: ") + write_ec.message());
        }
        if (written != command.size()) {
            throw std::runtime_error("Failed to write complete command");
        }
        
        // Small delay to ensure servo has time to respond
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Read response header first
        std::vector<uint8_t> header(4);
        size_t total_read = 0;
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::milliseconds(100);

        // Read header
        while (total_read < 4) {
            boost::system::error_code read_ec;
            size_t bytes = _serial_port.read_some(buffer(header.data() + total_read, 4 - total_read), read_ec);
            
            if (read_ec) {
                throw std::runtime_error(std::string("Header read error: ") + read_ec.message());
            }
            
            if (bytes > 0) {
                total_read += bytes;
            }
            
            if (std::chrono::steady_clock::now() - start_time > timeout) {
                throw std::runtime_error("Timeout waiting for header");
            }
            
            if (bytes == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        
        // Validate header
        std::string header_error;
        if (header[0] != 0xFF || header[1] != 0xFF) {
            header_error = "Invalid header markers";
        } else if (header[2] != servo_id) {
            header_error = "Mismatched servo ID";
        } else if (header[3] < 4) {
            header_error = "Invalid length";
        }
        
        if (!header_error.empty()) {
            std::stringstream ss;
            ss << "Header error (" << header_error << "): ";
            for (int i = 0; i < 4; i++) {
                ss << std::hex << std::setw(2) << std::setfill('0') 
                   << static_cast<int>(header[i]) << " ";
            }
            throw std::runtime_error(ss.str());
        }

        // Read remaining bytes based on length
        std::vector<uint8_t> data(header[3]);
        total_read = 0;
        start_time = std::chrono::steady_clock::now();

        while (total_read < header[3]) {
            boost::system::error_code read_ec;
            size_t bytes = _serial_port.read_some(buffer(data.data() + total_read, header[3] - total_read), read_ec);
            
            if (read_ec) {
                throw std::runtime_error(std::string("Data read error: ") + read_ec.message());
            }
            
            if (bytes > 0) {
                total_read += bytes;
            }
            
            if (std::chrono::steady_clock::now() - start_time > timeout) {
                throw std::runtime_error("Timeout waiting for data");
            }
            
            if (bytes == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        // Check for servo errors
        if (data[0] != 0x00) {
            std::string error = "Servo errors:";
            if (data[0] & 0x01) error += " Input Voltage";
            if (data[0] & 0x02) error += " Angle Limit";
            if (data[0] & 0x04) error += " Overheating";
            if (data[0] & 0x08) error += " Range";
            if (data[0] & 0x10) error += " Checksum";
            if (data[0] & 0x20) error += " Overload";
            if (data[0] & 0x40) error += " Instruction";
            throw std::runtime_error(error);
        }

        // Position is in little-endian format in the next two bytes
        return static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::string("Error reading servo ") + 
                               std::to_string(static_cast<int>(servo_id)) + 
                               ": " + e.what());
    }
}

std::vector<uint8_t> ST3215ServoReader::_createReadCommand(uint8_t id, uint8_t address, uint8_t size) 
{
    std::vector<uint8_t> command = {
        0xFF, 0xFF,    // Header
        id,            // Servo ID
        0x04,          // Length (always 4 for read command)
        0x02,          // READ instruction
        address,       // Starting address
        size,          // Number of bytes to read
        0x00           // Checksum (to be calculated)
    };
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (size_t i = 2; i < command.size() - 1; i++) 
    {
        checksum += command[i];
    }
    command[command.size() - 1] = ~checksum;

    return command;
}