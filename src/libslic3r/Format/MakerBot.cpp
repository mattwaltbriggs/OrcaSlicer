#include "MakerBot.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <set>
#include <nlohmann/json.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/log/trivial.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "libslic3r/Zipper.hpp"

using json = nlohmann::json;

namespace Slic3r {

namespace {

struct AxisState {
    double x = 0.0, y = 0.0, z = 0.0, a = 0.0, feedrate = 0.0;
};

struct PrinterSettings {
    int    tool0_temp = 0;
    int    tool1_temp = 0;
    int    bed_temp   = 0;
    bool   heat_bed   = false;
    double time       = 0.0;
    size_t command_count = 0;
    double x_min = 1e9, x_max = -1e9;
    double y_min = 1e9, y_max = -1e9;
    double z_min = 1e9, z_max = -1e9;
    int    num_z_layers = 0;
    std::set<double> z_heights;
    double total_extrusion_0 = 0.0;
    double total_extrusion_1 = 0.0;
};

static std::string make_uuid()
{
    return boost::uuids::to_string(boost::uuids::random_generator()());
}

static json make_empty_metadata()
{
    return json::object();
}

static json make_empty_tags()
{
    return json::array();
}

static json make_move_metadata(bool relative_xy)
{
    return json{{"relative", {
        {"a", true},
        {"b", true},
        {"x", relative_xy},
        {"y", relative_xy},
        {"z", false}
    }}};
}

static json make_change_toolhead_command(int index)
{
    return json{{"command", {
        {"function", "change_toolhead"},
        {"metadata", make_empty_metadata()},
        {"parameters", {
            {"index", index}
        }},
        {"tags", make_empty_tags()}
    }}};
}

static json make_change_toolhead_command(int index, double x, double y)
{
    return json{{"command", {
        {"function", "change_toolhead"},
        {"metadata", make_empty_metadata()},
        {"parameters", {
            {"index", index},
            {"x", x},
            {"y", y}
        }},
        {"tags", make_empty_tags()}
    }}};
}

static json make_move_command(const AxisState &axis, const AxisState &prev, const std::string &tag = "")
{
    json cmd = {{"command", {
        {"function", "move"},
        {"parameters", {
            {"x", axis.x},
            {"y", axis.y},
            {"z", axis.z},
            {"a", axis.a},
            {"b", 0.0},
            {"feedrate", axis.feedrate}
        }},
        {"metadata", make_move_metadata(false)},
        {"tags", tag.empty() ? json::array() : json::array({tag})}
    }}};
    return cmd;
}

static json make_temperature_command(int index, int temperature)
{
    return json{{"command", {
        {"function", "set_toolhead_temperature"},
        {"metadata", make_empty_metadata()},
        {"parameters", {
            {"index", index},
            {"temperature", temperature}
        }},
        {"tags", make_empty_tags()}
    }}};
}

static json make_wait_for_temperature_command(int index)
{
    return json{{"command", {
        {"function", "wait_for_temperature"},
        {"metadata", make_empty_metadata()},
        {"parameters", {{"index", index}}},
        {"tags", make_empty_tags()}
    }}};
}

static json make_toggle_fan_command(int index, bool value)
{
    return json{{"command", {
        {"function", "toggle_fan"},
        {"metadata", make_empty_metadata()},
        {"parameters", {
            {"index", index},
            {"value", value}
        }},
        {"tags", make_empty_tags()}
    }}};
}

static json make_fan_duty_command(int index, double duty)
{
    return json{{"command", {
        {"function", "fan_duty"},
        {"metadata", make_empty_metadata()},
        {"parameters", {
            {"index", index},
            {"value", duty}
        }},
        {"tags", make_empty_tags()}
    }}};
}

static json make_comment_command(const std::string &text)
{
    return json{{"command", {
        {"function", "comment"},
        {"metadata", make_empty_metadata()},
        {"parameters", {{"comment", text}}},
        {"tags", make_empty_tags()}
    }}};
}

static json make_delay_command(int seconds)
{
    return json{{"command", {
        {"function", "delay"},
        {"metadata", make_empty_metadata()},
        {"parameters", {{"seconds", seconds}}},
        {"tags", make_empty_tags()}
    }}};
}

static double compute_time(const AxisState &prev, const AxisState &cur)
{
    double dist;
    if (cur.x == prev.x && cur.y == prev.y && cur.z == prev.z && cur.a != prev.a) {
        dist = std::abs(cur.a - prev.a);
    } else {
        dist = std::sqrt((cur.x - prev.x) * (cur.x - prev.x) +
                         (cur.y - prev.y) * (cur.y - prev.y) +
                         (cur.z - prev.z) * (cur.z - prev.z));
    }
    if (cur.feedrate <= 0.0)
        return 0.0;
    return dist / cur.feedrate;
}

static std::string tag_from_gcode_comment(const std::string &comment)
{
    if (comment.find("WALL-OUTER") != std::string::npos || comment.find("External perimeter") != std::string::npos)
        return "WALL_OUTER_0";
    if (comment.find("WALL-INNER") != std::string::npos || comment.find("Perimeter") != std::string::npos)
        return "WALL_INNER_0";
    if (comment.find("FILL") != std::string::npos || comment.find("Internal infill") != std::string::npos || comment.find("Solid infill") != std::string::npos)
        return "FILL_0";
    if (comment.find("TOP SURFACE") != std::string::npos || comment.find("Top solid infill") != std::string::npos)
        return "TOP_SURFACE_0";
    if (comment.find("SUPPORT") != std::string::npos && comment.find("INTERFACE") != std::string::npos)
        return "SUPPORT_INTERFACE_0";
    if (comment.find("SUPPORT") != std::string::npos)
        return "SUPPORT_0";
    if (comment.find("SKIRT") != std::string::npos)
        return "SKIRT_0";
    if (comment.find("PRIME TOWER") != std::string::npos)
        return "PRIME_TOWER_0";
    return "";
}

} // anonymous namespace

bool export_makerbot(const std::string &gcode_path, const std::string &makerbot_path,
                     std::string &error_message, const MakerBotConfig &cfg)
{
    std::ifstream gcode_file(gcode_path);
    if (!gcode_file.is_open()) {
        error_message = "Failed to open G-code file: " + gcode_path;
        return false;
    }

    std::vector<json> commands;
    AxisState axis;
    AxisState prev_axis;
    PrinterSettings settings;
    std::string current_tag;
    bool relative_e = false;
    double absolute_e = 0.0;
    int current_tool = 0;
    bool past_first_layer = false;
    double last_x = 0.0, last_y = 0.0, last_z = 0.0;

    std::string line;
    while (std::getline(gcode_file, line)) {
        size_t semi = line.find(';');
        std::string comment;
        if (semi != std::string::npos) {
            comment = line.substr(semi + 1);
            line = line.substr(0, semi);
        }

        boost::trim(line);
        boost::trim(comment);

        // Extract settings from comments (always, even before first layer)
        if (settings.bed_temp == 0) {
            auto btpos = comment.find("first_layer_bed_temperature = ");
            if (btpos != std::string::npos) {
                settings.bed_temp = std::stoi(boost::trim_copy(comment.substr(btpos + 30)));
                if (settings.bed_temp > 0) settings.heat_bed = true;
            } else {
                auto btpos2 = comment.find("bed_temperature = ");
                if (btpos2 != std::string::npos) {
                    settings.bed_temp = std::stoi(boost::trim_copy(comment.substr(btpos2 + 18)));
                    if (settings.bed_temp > 0) settings.heat_bed = true;
                }
            }
        }

        // Track position from skipped gcode (before tokens are parsed)
        if (!past_first_layer && line.size() >= 2 && line[0] == 'G' && (line[1] == '0' || line[1] == '1')) {
            for (size_t i = 2; i < line.size(); ++i) {
                if (line[i] == 'X' || line[i] == 'Y' || line[i] == 'Z') {
                    char axis_char = line[i];
                    size_t start = i + 1;
                    while (start < line.size() && line[start] != ' ') ++start;
                    double val = std::stod(line.substr(i + 1, start - i - 1));
                    if (axis_char == 'X') last_x = val;
                    else if (axis_char == 'Y') last_y = val;
                    else if (axis_char == 'Z') last_z = val;
                    i = start - 1;
                }
            }
        }

        // Process mode-setting commands even before first layer (M83/M82 affect E interpretation)
        if (!past_first_layer && line.size() >= 2) {
            if (line[0] == 'M' && line[1] == '8') {
                if (line == "M83") { relative_e = true; continue; }
                if (line == "M82") { relative_e = false; continue; }
            }
            if (line.find("G92") == 0) {
                std::vector<std::string> pre_tokens;
                boost::split(pre_tokens, line, boost::is_any_of(" "), boost::token_compress_on);
                for (auto &t : pre_tokens) if (!t.empty() && t[0] == 'E') absolute_e = 0.0;
                continue;
            }
        }

        // Skip all gcode before first LAYER_CHANGE
        // (Prusa gcode has bed leveling at Z=40, purge lines, clearance moves etc)
        if (!past_first_layer) {
            if (comment.find("LAYER_CHANGE") != std::string::npos) {
                past_first_layer = true;
                axis.x = last_x;
                axis.y = last_y;
                axis.z = last_z > 0.0 && last_z < 10.0 ? last_z : 0.2;
                commands.push_back(make_comment_command(boost::trim_copy(comment)));
            }
            continue;
        }

        if (!comment.empty() && comment[0] == ' ') {
            std::string tag = tag_from_gcode_comment(comment);
            if (!tag.empty())
                current_tag = tag;
            auto layer_pos = comment.find("LAYER_CHANGE");
            if (layer_pos != std::string::npos || comment.find("LAYER") != std::string::npos) {
                commands.push_back(make_comment_command(boost::trim_copy(comment)));
            }
        }

        // Skip end-of-print commands
        if (line.find("M84") == 0 || line.find("M400") == 0)
            continue;

        if (line.empty())
            continue;

        std::vector<std::string> tokens;
        boost::split(tokens, line, boost::is_any_of(" "), boost::token_compress_on);
        if (tokens.empty())
            continue;

        const std::string &cmd = tokens[0];

        if (cmd == "G0" || cmd == "G1") {
            if (tokens.size() == 2 && tokens[1][0] == 'F') {
                axis.feedrate = std::stod(tokens[1].substr(1)) / 60.0;
            } else {
                prev_axis = axis;
                double new_e = absolute_e;
                bool has_e = false;
                for (size_t i = 1; i < tokens.size(); ++i) {
                    const std::string &tok = tokens[i];
                    if (tok.empty()) continue;
                    char c = tok[0];
                    if (c == 'E') {
                        new_e = std::stod(tok.substr(1));
                        has_e = true;
                    } else if (c == 'X')
                        axis.x = std::stod(tok.substr(1));
                    else if (c == 'Y')
                        axis.y = std::stod(tok.substr(1));
                    else if (c == 'Z')
                        axis.z = std::stod(tok.substr(1));
                    else if (c == 'F')
                        axis.feedrate = std::stod(tok.substr(1)) / 60.0;
                }

                if (has_e) {
                    if (relative_e) {
                        axis.a = new_e;
                        absolute_e += new_e;
                    } else {
                        axis.a = new_e - absolute_e;
                        absolute_e = new_e;
                    }
                } else {
                    axis.a = 0.0;
                }

                // Skip end-gcode moves with excessive Z (e.g. G1 Z25.55 head-lift)
                if (axis.z > 10.0 && !has_e)
                    continue;

                if (axis.x < settings.x_min) settings.x_min = axis.x;
                if (axis.x > settings.x_max) settings.x_max = axis.x;
                if (axis.y < settings.y_min) settings.y_min = axis.y;
                if (axis.y > settings.y_max) settings.y_max = axis.y;
                if (axis.z < settings.z_min) settings.z_min = axis.z;
                if (axis.z > settings.z_max) settings.z_max = axis.z;
                if (axis.z > 0.01)
                    settings.z_heights.insert(std::round(axis.z * 100.0) / 100.0);

                if (has_e && axis.a > 0.0) {
                    if (current_tool == 0)
                        settings.total_extrusion_0 += axis.a;
                    else
                        settings.total_extrusion_1 += axis.a;
                }

                bool is_retract = (axis.a < 0.0 && axis.x == prev_axis.x && axis.y == prev_axis.y);
                bool is_restart = (axis.a > 0.0 && axis.x == prev_axis.x && axis.y == prev_axis.y);
                std::string tag;
                if (is_retract)
                    tag = "Retract";
                else if (is_restart)
                    tag = "Restart";
                else if (axis.a == 0.0)
                    tag = "Travel Move";
                else if (!current_tag.empty())
                    tag = current_tag;

                commands.push_back(make_move_command(axis, prev_axis, tag));
                settings.time += compute_time(prev_axis, axis);
            }
        } else if (cmd == "G92") {
            for (size_t i = 1; i < tokens.size(); ++i) {
                const std::string &tok = tokens[i];
                if (tok.empty()) continue;
                if (tok[0] == 'E') {
                    absolute_e = 0.0;
                }
            }
        } else if (cmd == "M82") {
            relative_e = false;
        } else if (cmd == "M83") {
            relative_e = true;
        } else if (cmd == "M104" || cmd == "M109") {
            int tool_index = current_tool;
            int temperature = 0;
            for (size_t i = 1; i < tokens.size(); ++i) {
                const std::string &tok = tokens[i];
                if (tok.empty()) continue;
                if (tok[0] == 'T')
                    tool_index = std::stoi(tok.substr(1));
                else if (tok[0] == 'S')
                    temperature = std::stoi(tok.substr(1));
            }
            if (tool_index == 0 && settings.tool0_temp == 0)
                settings.tool0_temp = temperature;
            else if (tool_index == 1 && settings.tool1_temp == 0)
                settings.tool1_temp = temperature;
        } else if (cmd == "M106") {
            int fan_idx = 0;
            double fan_duty_value = 1.0;
            for (size_t i = 1; i < tokens.size(); ++i) {
                const std::string &tok = tokens[i];
                if (tok.empty()) continue;
                if (tok[0] == 'P')
                    fan_idx = std::stoi(tok.substr(1));
                else if (tok[0] == 'S')
                    fan_duty_value = std::stod(tok.substr(1)) / 255.0;
            }
            commands.push_back(make_toggle_fan_command(fan_idx, true));
            commands.push_back(make_fan_duty_command(fan_idx, fan_duty_value));
        } else if (cmd == "M107") {
            int fan_idx = 0;
            for (size_t i = 1; i < tokens.size(); ++i) {
                const std::string &tok = tokens[i];
                if (tok.empty()) continue;
                if (tok[0] == 'P')
                    fan_idx = std::stoi(tok.substr(1));
            }
            commands.push_back(make_toggle_fan_command(fan_idx, false));
        } else if (cmd == "M140") {
            for (size_t i = 1; i < tokens.size(); ++i) {
                const std::string &tok = tokens[i];
                if (!tok.empty() && tok[0] == 'S') {
                    settings.bed_temp = std::stoi(tok.substr(1));
                    settings.heat_bed = true;
                }
            }
        } else if (cmd == "M190") {
            for (size_t i = 1; i < tokens.size(); ++i) {
                const std::string &tok = tokens[i];
                if (!tok.empty() && tok[0] == 'S') {
                    settings.bed_temp = std::stoi(tok.substr(1));
                    settings.heat_bed = true;
                }
            }
        } else if (cmd == "T0" || cmd == "T1") {
            current_tool = std::stoi(cmd.substr(1));
            commands.push_back(make_change_toolhead_command(current_tool));
        }
    }

    // Trim everything after the last extrusion move (removes end gcode: head-lift, fan off, etc.)
    {
        size_t last_extrusion = 0;
        for (size_t i = 0; i < commands.size(); ++i) {
            auto &c = commands[i]["command"];
            if (c.value("function", "") == "move" && c["parameters"].value("a", 0.0) > 0.0)
                last_extrusion = i;
        }
        if (last_extrusion + 1 < commands.size())
            commands.resize(last_extrusion + 1);
    }

    // Recompute bounding box from kept commands only
    settings.x_min = settings.y_min = settings.z_min = 1e9;
    settings.x_max = settings.y_max = settings.z_max = -1e9;
    settings.z_heights.clear();
    for (auto &cmd : commands) {
        auto &c = cmd["command"];
        if (c.value("function", "") == "move") {
            auto &p = c["parameters"];
            double x = p["x"].get<double>(), y = p["y"].get<double>(), z = p["z"].get<double>();
            if (x < settings.x_min) settings.x_min = x;
            if (x > settings.x_max) settings.x_max = x;
            if (y < settings.y_min) settings.y_min = y;
            if (y > settings.y_max) settings.y_max = y;
            if (z < settings.z_min) settings.z_min = z;
            if (z > settings.z_max) settings.z_max = z;
            if (z > 0.01) settings.z_heights.insert(std::round(z * 100.0) / 100.0);
        }
    }

    if (cfg.tool0_temp)
        settings.tool0_temp = *cfg.tool0_temp;
    if (cfg.tool1_temp)
        settings.tool1_temp = *cfg.tool1_temp;
    if (cfg.bed_temp) {
        settings.bed_temp = *cfg.bed_temp;
        settings.heat_bed = true;
    }

    settings.num_z_layers = settings.z_heights.size();

    if (settings.tool0_temp == 0) settings.tool0_temp = 210;
    if (settings.tool1_temp == 0) settings.tool1_temp = 160;

    // Center coordinates on the build plate (MakerBot uses center-origin, OrcaSlicer uses corner-origin).
    // Center on the model's bounding box, not the bed center.
    const double x_offset = (settings.x_min + settings.x_max) / 2.0;
    const double y_offset = (settings.y_min + settings.y_max) / 2.0;
    const double x_limit = (cfg.bed_width > 0.0) ? cfg.bed_width / 2.0 - 5.0 : 71.0;
    const double y_limit = (cfg.bed_depth > 0.0) ? cfg.bed_depth / 2.0 - 5.0 : 90.0;

    // Offset all move commands and toolhead commands
    for (auto &cmd : commands) {
        auto &c = cmd["command"];
        std::string func = c.value("function", "");
        if (func == "move") {
            auto &params = c["parameters"];
            auto &meta = c["metadata"];
            json rel = c["metadata"].contains("relative") ? c["metadata"]["relative"] : json::object();
            bool rel_x = rel.value("x", false);
            bool rel_y = rel.value("y", false);
            double x = params["x"].get<double>();
            double y = params["y"].get<double>();
            if (!rel_x) x -= x_offset;
            if (!rel_y) y -= y_offset;
            if (x < -x_limit) x = -x_limit;
            if (x >  x_limit) x =  x_limit;
            if (y < -y_limit) y = -y_limit;
            if (y >  y_limit) y =  y_limit;
            params["x"] = x;
            params["y"] = y;
        } else if (func == "change_toolhead") {
            auto &params = c["parameters"];
            if (params.contains("x")) {
                double x = params["x"].get<double>() - x_offset;
                if (x < -x_limit) x = -x_limit;
                if (x >  x_limit) x =  x_limit;
                params["x"] = x;
            }
            if (params.contains("y")) {
                double y = params["y"].get<double>() - y_offset;
                if (y < -y_limit) y = -y_limit;
                if (y >  y_limit) y =  y_limit;
                params["y"] = y;
            }
        }
    }

    // Offset bounding box
    settings.x_min -= x_offset; settings.x_max -= x_offset;
    settings.y_min -= y_offset; settings.y_max -= y_offset;
    if (settings.x_min < -x_limit) settings.x_min = -x_limit;
    if (settings.x_max >  x_limit) settings.x_max =  x_limit;
    if (settings.y_min < -y_limit) settings.y_min = -y_limit;
    if (settings.y_max >  y_limit) settings.y_max =  y_limit;

    {
        std::vector<json> startup;
        startup.push_back(make_temperature_command(1, settings.tool1_temp));
        startup.push_back(make_toggle_fan_command(1, true));
        startup.push_back(make_fan_duty_command(1, 1.0));
        startup.push_back(make_temperature_command(0, settings.tool0_temp));
        startup.push_back(make_change_toolhead_command(0, 50.0, -80.0));
        startup.push_back(make_wait_for_temperature_command(0));
        startup.push_back(make_delay_command(5));
        startup.push_back(make_toggle_fan_command(1, false));
        commands.insert(commands.begin(), startup.begin(), startup.end());
    }

    settings.command_count = commands.size();

    json meta = {
        {"version", "3.0.0"},
        {"bot_type", "fire_e"},
        {"material", "pla"},
        {"materials", {"pla"}},
        {"tool_type", "mk14_e"},
        {"tool_types", {"mk14_e", "mk14_s"}},
        {"extruder_temperature", settings.tool0_temp},
        {"extruder_temperatures", {settings.tool0_temp, settings.tool1_temp}},
        {"platform_temperature", settings.heat_bed ? settings.bed_temp : 0},
        {"build_plane_temperature", 0},
        {"chamber_temperature", 0},
        {"duration_s", std::round(settings.time * 10.0) / 10.0},
        {"commanded_duration_s", std::round(settings.time * 10.0) / 10.0},
        {"total_commands", settings.command_count},
        {"extrusion_distance_mm", settings.total_extrusion_0 + settings.total_extrusion_1},
        {"extrusion_distances_mm", {settings.total_extrusion_0, settings.total_extrusion_1}},
        {"extrusion_mass_g", 0.0},
        {"extrusion_masses_g", {0.0, 0.0}},
        {"model_counts", json::array({{ {"count", 1}, {"name", "default"} }})},
        {"preferences", {
            {"bounding_box", nullptr},
            {"overrides", json::object()},
            {"print_mode", "balanced"}
        }},
        {"uuid", make_uuid()}
    };

    std::string meta_str = meta.dump(4);
    std::string toolpath_str = json(commands).dump();

    try {
        Zipper zipper(makerbot_path);
        zipper.add_entry("meta.json", meta_str.data(), meta_str.size());
        zipper.add_entry("print.jsontoolpath", toolpath_str.data(), toolpath_str.size());
    } catch (const std::exception &e) {
        error_message = "Failed to create .makerbot file: " + std::string(e.what());
        return false;
    }

    BOOST_LOG_TRIVIAL(info) << "MakerBot export completed: " << makerbot_path
                            << " (" << settings.command_count << " commands, "
                            << static_cast<int>(std::ceil(settings.time)) << "s estimated)";

    return true;
}

} // namespace Slic3r
