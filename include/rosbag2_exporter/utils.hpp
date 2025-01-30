#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "rosbag2_exporter/handlers/base_handler.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace rosbag2_exporter {

std::string get_cam_name(const std::string& path) {
    const std::string prefix = "/sensor/camera/";
    size_t start = path.find(prefix);
    if (start == std::string::npos) {
        return ""; // Prefix not found
    }
    
    // Move past "/sensor/camera/"
    start += prefix.length(); 
    
    size_t end = path.find('/', start);
    return path.substr(start, end - start);
}

// Helper function to convert builtin_interfaces::msg::Time to nanoseconds
int64_t toNanoseconds(const builtin_interfaces::msg::Time& timestamp) {
    return static_cast<int64_t>(timestamp.sec) * 1'000'000'000 + timestamp.nanosec;
}

// Function to find the closest index in a sorted vector for a given timestamp
size_t find_closest_timestamp(const std::vector<DataMeta>& target_vector, 
                              const builtin_interfaces::msg::Time& timestamp, 
                              size_t& last_index) {
                                
    int64_t timestamp_ns = toNanoseconds(timestamp);

    // Start from the last known index
    size_t closest_index = last_index;
    int64_t min_diff = std::numeric_limits<int64_t>::max();

    // Iterate forward through the sorted vector
    for (size_t i = last_index; i < target_vector.size(); ++i) {
        int64_t target_ns = toNanoseconds(target_vector[i].timestamp);
        int64_t diff = std::abs(target_ns - timestamp_ns);

        // Update closest index and minimum difference
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = i;
        } else {
            // Since the vector is sorted, stop when differences start increasing
            break;
        }
    }

    // Update the last index for the next call
    last_index = closest_index;
    return closest_index;
}

} // namespace rosbag2_exporter

#endif // UTILS_HPP
