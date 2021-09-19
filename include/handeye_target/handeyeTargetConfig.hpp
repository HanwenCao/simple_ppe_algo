#pragma once
#include <string>


struct handeyeTargetConfig{
    static const int num_marker_x = 5;                                  // Number of markers along X axis
    static const int num_marker_y = 5;                                  // Number of markers along Y axis
    static const int marker_pixel_size = 200;                           // Marker size in pixels
    static const int marker_pixel_separation = 0;                      // Marker separation distance in pixels
    static const int border_bits = 1;                                   // Margin of boarder in bits
    double marker_measured_size = 0.053;                                // Printed marker size in real world
    double marker_measured_separation = 0.0053;                         // Printed marker separation distance in real world
    std::string dictionary_id_str;                                      // Aruco dictionary id with string type
};