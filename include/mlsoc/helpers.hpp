#ifndef SIMAAI__HELPERS_HPP_
#define SIMAAI__HELPERS_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>

#include <nlohmann/json.hpp>

inline bool
parse_json_from_file(const std::string config_file_path, nlohmann::json &json)
{
    try {
        std::ifstream input_file(config_file_path.c_str());
        if (!input_file) 
            throw std::runtime_error(std::string("Error opening file ") + config_file_path );
        std::ostringstream string_stream;
        string_stream << input_file.rdbuf();

        json = nlohmann::json::parse(string_stream.str());

        return true;

    } catch (std::exception & ex) {
        std::cerr << "Unable to parse config file: " << ex.what() << std::endl;
        return false;
    }
}

bool check_if_file_exists(std::string path){
    if(std::filesystem::exists(path)) return true;

    return false;
}

std::string get_hardware_target()
{
    std::string build_path_1 = "/etc/build";
    std::string build_path_2 = "/etc/buildinfo";
    std::ifstream build_file;
    if (check_if_file_exists(build_path_1)){
        std::cout << "Found build path: " << build_path_1 << std::endl;
        build_file.open(build_path_1);
    } else if(check_if_file_exists(build_path_2)){
        std::cout << "Found build path: " << build_path_2 << std::endl;
        build_file.open(build_path_2);
    } else {
        std::cerr << "Could not find a build file in /etc" << std::endl;
        return "unknown";
    }

    if (!build_file.is_open()) {
        std::cerr << "Could not open build file in /etc" << std::endl;
        return "unknown";
    }

    std::string line;
    while (std::getline(build_file, line)) {
        // Trim leading spaces
        line.erase(0, line.find_first_not_of(" \t"));
        
        if (line.rfind("MACHINE", 0) == 0) {
            size_t eq_pos = line.find('=');
            if (eq_pos != std::string::npos) {
                std::string value = line.substr(eq_pos + 1);
                
                // Trim spaces
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);

                return value;
            }
        }
    }

    return "unknown";
}



std::string resolve_package_resource_path(
    const std::string & relative_path_template,
    const std::string & hardware_target,
    const std::string & package_share_dir)
{
    // Replace "{hardware_target}" in the path template if it exists
    std::string resolved_relative_path = relative_path_template;
    const std::string placeholder = "{hardware_target}";
    size_t pos = resolved_relative_path.find(placeholder);
    if (pos != std::string::npos) {
        resolved_relative_path.replace(pos, placeholder.length(), hardware_target);
    }

    std::filesystem::path path_candidate(resolved_relative_path);

    // If the path is absolute, return as is
    if (path_candidate.is_absolute()) {
        return path_candidate.string();
    }

    // Otherwise, construct the absolute path using package_share_dir
    std::filesystem::path absolute_path = std::filesystem::path(package_share_dir) / resolved_relative_path;

    // Validate existence
    if (!std::filesystem::exists(absolute_path)) {
        throw std::runtime_error("File not found: " + absolute_path.string());
    }

    return absolute_path.string();
}

std::string get_ros_project_root() {
    const char* env_p = std::getenv("VDP_ROS_PACKAGE_DIR");
    if (!env_p) {
        throw std::runtime_error("VDP_ROS_PACKAGE_DIR environment variable is not set");
    }

    std::string path(env_p);
    if (!path.empty() && path.back() != '/') {
        path += '/';
    }

    return path;
}

#endif // SIMAAI__HELPERS_HPP_

