#include "perseus-arm-teleop.hpp"
#include <iostream>
#include <thread>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <ncurses.h>
#include <csignal>
#include <atomic>

static std::atomic<bool> running(true);

void signalHandler(int signum)
{
    running = false;
}

// Find available serial ports
std::vector<std::string> findSerialPorts()
{
    std::vector<std::string> ports;
    const std::filesystem::path dev_path("/dev");

    for (const auto &entry : std::filesystem::directory_iterator(dev_path))
    {
        std::string filename = entry.path().filename().string();
        if (filename.find("ttyUSB") != std::string::npos ||
            filename.find("ttyACM") != std::string::npos)
        {
            ports.push_back(entry.path().string());
        }
    }

    std::sort(ports.begin(), ports.end());
    return ports;
}

// Let user select a port
std::string selectSerialPort(const std::vector<std::string> &ports)
{
    if (ports.empty())
    {
        throw std::runtime_error("No serial ports found");
    }

    std::cout << "Available serial ports:" << std::endl;
    for (size_t i = 0; i < ports.size(); ++i)
    {
        std::cout << i + 1 << ": " << ports[i] << std::endl;
    }

    size_t selection = 0;
    while (selection < 1 || selection > ports.size())
    {
        std::cout << "Select port (1-" << ports.size() << "): ";
        if (!(std::cin >> selection))
        {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            selection = 0;
        }
    }

    return ports[selection - 1];
}

// Create a progress bar string
std::string createProgressBar(uint16_t value)
{
    // Clamp value to 0-4095
    value = std::min(value, static_cast<uint16_t>(4095));

    const size_t barLength = 20;
    size_t filledLength = static_cast<size_t>((static_cast<double>(value) / 4095.0) * barLength);

    std::string bar = "[";
    bar.append(filledLength, '#');
    bar.append(barLength - filledLength, ' ');
    bar.append("]");

    return bar;
}

// Display servo values in ncurses window
void displayServoValues(WINDOW *win, const std::vector<uint16_t> &positions,
                        const std::vector<std::string> &errors)
{
    werase(win);

    // Display header
    mvwprintw(win, 0, 0, "Perseus Arm Servo Positions (0-4095)");
    mvwprintw(win, 1, 0, "--------------------------------"); // Changed to standard ASCII dashes

    // Display each servo's position or error
    for (size_t i = 0; i < 6; ++i)
    {
        int row = i + 2;
        if (errors[i].empty())
        {
            uint16_t position = std::min(positions[i], static_cast<uint16_t>(4095));
            std::string progressBar = createProgressBar(position);
            mvwprintw(win, row, 0, "Servo %d: %4u %s",
                      static_cast<int>(i + 1), position, progressBar.c_str());
        }
        else
        {
            mvwprintw(win, row, 0, "Servo %d: Error: %s",
                      static_cast<int>(i + 1), errors[i].c_str());
        }
    }

    mvwprintw(win, 9, 0, "Press Ctrl+C to exit");
    wrefresh(win);
}

int main(int argc, char *argv[])
{
    try
    {
        // Set up signal handling
        signal(SIGINT, signalHandler);

        // Get port path
        std::string port_path;
        if (argc > 1)
        {
            port_path = argv[1];
        }
        else
        {
            auto available_ports = findSerialPorts();
            port_path = selectSerialPort(available_ports);
        }

        std::cout << "Using serial port: " << port_path << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Initialize ncurses
        WINDOW *win = initscr();
        cbreak();
        noecho();
        curs_set(0);
        nodelay(win, TRUE);
        keypad(win, TRUE);

        // Initialize servo reader and data storage
        ST3215ServoReader reader(port_path, 1000000);
        std::vector<uint16_t> positions(6, 0);
        std::vector<std::string> errors(6);

        // Main loop
        while (running)
        {
            // Read all servo positions
            for (uint8_t i = 0; i < 6; ++i)
            {
                try
                {
                    positions[i] = reader.readPosition(i + 1);
                    errors[i].clear();
                }
                catch (const std::exception &e)
                {
                    errors[i] = e.what();
                }
            }

            // Update display
            displayServoValues(win, positions, errors);

            // Delay to prevent overwhelming servos
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Clean up
        endwin();
        std::cout << "Program terminated by user." << std::endl;
        return 0;
    }
    catch (const std::exception &e)
    {
        if (isendwin() == FALSE)
        {
            endwin();
        }
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}