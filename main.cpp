#include "perseus-arm-teleop.hpp"
#include <iostream>
#include <thread>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <ncurses.h>
#include <csignal>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>

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

// Let user select ports for both arms
std::pair<std::string, std::string> selectSerialPorts(const std::vector<std::string> &ports)
{
    if (ports.empty())
    {
        throw std::runtime_error("No serial ports found");
    }

    std::cout << "\nSelect ports for Perseus arms control\n";
    std::cout << "=====================================\n";
    std::cout << "Available serial ports:\n";
    for (size_t i = 0; i < ports.size(); ++i)
    {
        std::cout << i + 1 << ": " << ports[i] << std::endl;
    }
    std::cout << "0: Rescan for ports\n\n";

    std::string port1, port2;
    size_t selection1 = 0, selection2 = 0;

    // Select first arm port
    while (true)
    {
        std::cout << "Select port for ARM 1 (0 to rescan, 1-" << ports.size() << " to select): ";
        if (!(std::cin >> selection1))
        {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (selection1 == 0)
        {
            return selectSerialPorts(findSerialPorts()); // Rescan and restart
        }

        if (selection1 > 0 && selection1 <= ports.size())
        {
            port1 = ports[selection1 - 1];
            break;
        }

        std::cout << "Invalid selection. Please try again.\n";
    }

    // Display confirmation and proceed to second arm
    std::cout << "\nARM 1 will use: " << port1 << "\n\n";

    // Select second arm port
    while (true)
    {
        std::cout << "Select port for ARM 2 (0 to rescan, 1-" << ports.size() << " to select): ";
        if (!(std::cin >> selection2))
        {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (selection2 == 0)
        {
            return selectSerialPorts(findSerialPorts()); // Rescan and restart
        }

        if (selection2 > 0 && selection2 <= ports.size() && selection2 != selection1)
        {
            port2 = ports[selection2 - 1];
            break;
        }

        if (selection2 == selection1)
        {
            std::cout << "Cannot use the same port for both arms. Please select a different port.\n";
        }
        else
        {
            std::cout << "Invalid selection. Please try again.\n";
        }
    }

    std::cout << "\nARM 2 will use: " << port2 << "\n";
    std::cout << "\nPress Enter to continue...";
    std::cin.ignore(10000, '\n');
    std::cin.get();

    return {port1, port2};
}

// Create a colored progress bar string
void displayProgressBar(WINDOW *win, int y, int x, uint16_t current, uint16_t min, uint16_t max)
{
    // Clamp values to 0-4095
    current = std::min(current, static_cast<uint16_t>(4095));
    min = std::min(min, static_cast<uint16_t>(4095));
    max = std::min(max, static_cast<uint16_t>(4095));

    const size_t barLength = 40;

    // Calculate positions
    size_t currentPos = static_cast<size_t>((static_cast<double>(current) / 4095.0) * barLength);
    size_t minPos = static_cast<size_t>((static_cast<double>(min) / 4095.0) * barLength);
    size_t maxPos = static_cast<size_t>((static_cast<double>(max) / 4095.0) * barLength);

    // Print opening bracket
    mvwaddch(win, y, x, '[');
    x++;

    // Print bar with colors
    for (size_t i = 0; i < barLength; i++)
    {
        if (has_colors())
        {
            if (i == minPos)
            {
                wattron(win, COLOR_PAIR(1)); // Blue for min
                waddch(win, '#');
                wattroff(win, COLOR_PAIR(1));
            }
            else if (i == maxPos)
            {
                wattron(win, COLOR_PAIR(2)); // Green for max
                waddch(win, '#');
                wattroff(win, COLOR_PAIR(2));
            }
            else if (i < currentPos)
            {
                if (i < minPos)
                {
                    wattron(win, COLOR_PAIR(3) | A_DIM); // Dimmed white for positions before min
                    waddch(win, '#');
                    wattroff(win, COLOR_PAIR(3) | A_DIM);
                }
                else
                {
                    wattron(win, COLOR_PAIR(3)); // Bright white for current valid position
                    waddch(win, '#');
                    wattroff(win, COLOR_PAIR(3));
                }
            }
            else
            {
                waddch(win, ' ');
            }
        }
        else
        {
            // For non-color displays, still show all positions but with different characters
            if (i < currentPos)
            {
                waddch(win, (i < minPos) ? '.' : '#');
            }
            else
            {
                waddch(win, ' ');
            }
        }
    }

    // Print closing bracket
    waddch(win, ']');
}

// Structure to hold servo data including min/max values
struct ServoData
{
    uint16_t current = 0;
    uint16_t min = 4095;
    uint16_t max = 0;
    std::string error;
};

std::string getWorkingDirectory()
{
    return std::filesystem::current_path().string();
}

// Display servo values in ncurses window for both arms
// Display servo values in ncurses window for both arms
void displayServoValues(WINDOW *win,
    const std::vector<ServoData> &arm1_data,
    const std::vector<ServoData> &arm2_data)
{
werase(win);

// Display header
mvwprintw(win, 0, 0, "Perseus Arms Servo Positions (0-4095)");
mvwprintw(win, 1, 0, "--------------------------------------------------------");

// Column headers
mvwprintw(win, 2, 2, "Servo    Current    Min      Max      Range");
mvwprintw(win, 3, 0, "--------------------------------------------------------");

// Display first arm's servos
mvwprintw(win, 4, 0, "Arm 1:");
for (size_t i = 0; i < 6; ++i)
{
int row = i + 5;
const auto &servo = arm1_data[i];

if (servo.error.empty())
{
mvwprintw(win, row, 2, "%-8d %8u  %8u  %8u  ",
   static_cast<int>(i + 1),
   servo.current,
   servo.min,
   servo.max);
displayProgressBar(win, row, 42, servo.current, servo.min, servo.max);
}
else
{
mvwprintw(win, row, 2, "%d: Error: %s",
   static_cast<int>(i + 1),
   servo.error.c_str());
}
}

mvwprintw(win, 11, 0, "--------------------------------------------------------");

// Display second arm's servos
mvwprintw(win, 12, 0, "Arm 2:");
for (size_t i = 0; i < 6; ++i)
{
int row = i + 13;
const auto &servo = arm2_data[i];

if (servo.error.empty())
{
mvwprintw(win, row, 2, "%-8d %8u  %8u  %8u  ",
   static_cast<int>(i + 1),
   servo.current,
   servo.min,
   servo.max);
displayProgressBar(win, row, 42, servo.current, servo.min, servo.max);
}
else
{
mvwprintw(win, row, 2, "%-8d Error: %s",
   static_cast<int>(i + 1),
   servo.error.c_str());
}
}

mvwprintw(win, 19, 0, "--------------------------------------------------------");

// Add instructions and working directory
mvwprintw(win, 20, 0, "Instructions:");
mvwprintw(win, 21, 0, "1. Move both arms through their full range of motion");
mvwprintw(win, 22, 0, "2. Press 's' to save calibration when done");
mvwprintw(win, 23, 0, "3. Press Ctrl+C to exit");
mvwprintw(win, 24, 0, "Save directory: %s", getWorkingDirectory().c_str());

wrefresh(win);
}


void exportCalibrationData(const std::vector<ServoData> &arm1_data,
                           const std::vector<ServoData> &arm2_data,
                           const std::string &port1,
                           const std::string &port2)
{
    YAML::Node config;

    // Add metadata
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");

    config["timestamp"] = ss.str();
    config["arm1_port"] = port1;
    config["arm2_port"] = port2;

    // Add calibration data for arm 1
    YAML::Node arm1_node;
    for (size_t i = 0; i < arm1_data.size(); ++i)
    {
        YAML::Node servo;
        servo["id"] = i + 1;
        servo["min"] = arm1_data[i].min;
        servo["max"] = arm1_data[i].max;
        arm1_node["servos"].push_back(servo);
    }
    config["arm1"] = arm1_node;

    // Add calibration data for arm 2
    YAML::Node arm2_node;
    for (size_t i = 0; i < arm2_data.size(); ++i)
    {
        YAML::Node servo;
        servo["id"] = i + 1;
        servo["min"] = arm2_data[i].min;
        servo["max"] = arm2_data[i].max;
        arm2_node["servos"].push_back(servo);
    }
    config["arm2"] = arm2_node;

    // Create filename with timestamp
    std::string filename = ss.str() + "_perseus_arm_calibration.yaml";

    // Save to file
    std::ofstream fout(filename);
    fout << config;

    std::cout << "\nCalibration data exported to: " << filename << std::endl;
}

int main(int argc, char *argv[])
{
    try
    {
        // Set up signal handling
        signal(SIGINT, signalHandler);

        // Get port paths
        std::string port_path1, port_path2;
        if (argc > 2)
        {
            port_path1 = argv[1];
            port_path2 = argv[2];
        }
        else
        {
            auto available_ports = findSerialPorts();
            auto [p1, p2] = selectSerialPorts(available_ports);
            port_path1 = p1;
            port_path2 = p2;
        }

        std::cout << "Using serial ports:\nArm 1: " << port_path1
                  << "\nArm 2: " << port_path2 << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Initialize ncurses
        WINDOW *win = initscr();
        cbreak();
        noecho();
        curs_set(0);
        nodelay(win, TRUE);
        keypad(win, TRUE);

        // Initialize colors if terminal supports them
        if (has_colors())
        {
            start_color();
            init_pair(1, COLOR_BLUE, COLOR_BLACK);  // For min value
            init_pair(2, COLOR_GREEN, COLOR_BLACK); // For max value
            init_pair(3, COLOR_WHITE, COLOR_BLACK); // For current value
        }

        // Initialize servo readers and data storage for both arms
        ST3215ServoReader reader1(port_path1, 1000000);
        ST3215ServoReader reader2(port_path2, 1000000);
        std::vector<ServoData> arm1_data(6);
        std::vector<ServoData> arm2_data(6);

        // Main loop
        while (running)
        {
            // Read all servo positions from first arm
            for (uint8_t i = 0; i < 6; ++i)
            {
                try
                {
                    auto& servo = arm1_data[i];
                    servo.current = reader1.readPosition(i + 1);
                    servo.min = std::min(servo.min, servo.current);
                    servo.max = std::max(servo.max, servo.current);
                    servo.error.clear();
                }
                catch (const std::exception& e)
                {
                    arm1_data[i].error = e.what();
                }
            }

            // Read all servo positions from second arm
            for (uint8_t i = 0; i < 6; ++i)
            {
                try
                {
                    auto& servo = arm2_data[i];
                    servo.current = reader2.readPosition(i + 1);
                    servo.min = std::min(servo.min, servo.current);
                    servo.max = std::max(servo.max, servo.current);
                    servo.error.clear();
                }
                catch (const std::exception& e)
                {
                    arm2_data[i].error = e.what();
                }
            }

            // Update display with both arms' data
            displayServoValues(win, arm1_data, arm2_data);

            // Handle keyboard input for saving
            int ch = wgetch(win);
            if (ch == 's' || ch == 'S')
            {
                mvwprintw(win, 25, 0, "Saving calibration data...");
                wrefresh(win);
                
                try 
                {
                    exportCalibrationData(arm1_data, arm2_data, port_path1, port_path2);
                    mvwprintw(win, 25, 0, "                                                                        ");
                    mvwprintw(win, 25, 0, "Calibration data saved successfully! Press any key to continue");
                    wrefresh(win);
                    
                    // Wait for any key
                    nodelay(win, FALSE);  // Switch to blocking mode temporarily
                    wgetch(win);
                    nodelay(win, TRUE);   // Switch back to non-blocking
                    
                    // Clear status line
                    mvwprintw(win, 25, 0, "                                                                        ");
                    wrefresh(win);
                }
                catch (const std::exception& e)
                {
                    mvwprintw(win, 25, 0, "                                                                        ");
                    mvwprintw(win, 25, 0, "Error saving calibration: %s", e.what());
                    wrefresh(win);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    
                    // Clear error message
                    mvwprintw(win, 25, 0, "                                                                        ");
                    wrefresh(win);
                }
            }

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