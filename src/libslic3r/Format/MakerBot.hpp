#ifndef slic3r_Format_MakerBot_hpp_
#define slic3r_Format_MakerBot_hpp_

#include <string>
#include <vector>
#include <boost/optional.hpp>

namespace Slic3r {

struct MakerBotConfig {
    boost::optional<int>  tool0_temp;
    boost::optional<int>  tool1_temp;
    boost::optional<int>  bed_temp;
    double bed_width  = 0.0;  // bed X dimension in mm (for centering coordinates)
    double bed_depth  = 0.0;  // bed Y dimension in mm (for centering coordinates)
};

// Convert a G-code file to MakerBot .makerbot format (ZIP containing meta.json + print.jsontoolpath).
// Returns true on success, false on failure with error_message set.
bool export_makerbot(const std::string &gcode_path, const std::string &makerbot_path,
                     std::string &error_message, const MakerBotConfig &cfg = MakerBotConfig());

} // namespace Slic3r

#endif // slic3r_Format_MakerBot_hpp_
