#include <vector>
#include <regex>
#include <fstream>
#include "../Point.h"


std::vector<Point> parse_ini_file(const std::string& filepath) {
    std::ifstream file(filepath);
    if(!file.is_open()) {
        throw std::runtime_error("bad file");
    }
    std::string line;
    std::regex fiducial_regex(R"(fiducial\d+)");
    std::vector<Point> fiducials;
    std::map<std::string, std::string> current_section;

    std::string current_section_name;
    Point current_fiducial = {0, 0, 0};

    while (std::getline(file, line)) {
        line.erase(0, line.find_first_not_of(" \t\r\n"));  // Trim left
        line.erase(line.find_last_not_of(" \t\r\n") + 1);  // Trim right
        if (line.empty() || line[0] == '#')
            continue;

        if (line[0] == '[' && line.back() == ']') {
            // Save previous fiducial before moving to next section
            if (std::regex_match(current_section_name, fiducial_regex)) {
                fiducials.push_back(current_fiducial);
            }
            current_section_name = line.substr(1, line.size() - 2);
            current_fiducial = {0, 0, 0};
        } else {
            std::istringstream iss(line);
            std::string key, value;
            if (std::getline(iss, key, '=') && std::getline(iss, value)) {
                if (current_section_name.find("fiducial") == 0) {
                    if (key == "x") current_fiducial.x = std::stod(value);
                    else if (key == "y") current_fiducial.y = std::stod(value);
                    else if (key == "z") current_fiducial.z = std::stod(value);
                }
            }
        }
    }
    // Final push if the last section was a fiducial
    if (std::regex_match(current_section_name, fiducial_regex)) {
        fiducials.push_back(current_fiducial);
    }
    return fiducials;
}
