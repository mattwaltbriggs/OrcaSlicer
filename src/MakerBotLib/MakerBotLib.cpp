// MakerBotLib - Standalone .makerbot file exporter
// Compiles in seconds with no libslic3r/boost dependencies.
// Usage: c++ -shared -o libMakerBotLib.dylib -I../../deps_src/nlohmann -I../../deps_src/miniz MakerBotLib.cpp ../../deps_src/miniz/miniz.c

#include "MakerBotLib.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <set>
#include <vector>
#include <string>
#include <random>
#include <cstdint>

#include "nlohmann/json.hpp"

#define MINIZ_NO_ZLIB_COMPATIBLE_NAMES
#include "miniz.h"

using json = nlohmann::json;

static std::string trim_copy(const std::string &s)
{
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

static void trim_inplace(std::string &s)
{
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) { s.clear(); return; }
    s = s.substr(start);
    size_t end = s.find_last_not_of(" \t\r\n");
    s = s.substr(0, end + 1);
}

static std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> tokens;
    std::istringstream iss(s);
    std::string tok;
    while (std::getline(iss, tok, delim))
        if (!tok.empty())
            tokens.push_back(tok);
    return tokens;
}

static std::string make_uuid()
{
    static std::mt19937 rng(std::random_device{}());
    static std::uniform_int_distribution<uint32_t> dist(0, 255);
    uint8_t bytes[16];
    for (auto &b : bytes) b = (uint8_t)dist(rng);
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    char buf[37];
    snprintf(buf, sizeof(buf),
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        bytes[0], bytes[1], bytes[2], bytes[3],
        bytes[4], bytes[5], bytes[6], bytes[7],
        bytes[8], bytes[9], bytes[10], bytes[11],
        bytes[12], bytes[13], bytes[14], bytes[15]);
    return buf;
}

namespace {

struct AxisState {
    double x = 0.0, y = 0.0, z = 0.0, a = 0.0, feedrate = 0.0;
};

struct PrinterSettings {
    int    tool0_temp = 0;
    int    tool1_temp = 0;
    int    bed_temp   = 0;
    int    chamber_temp = 0;
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

static json make_command(const std::string &func, json params = json::object(), json tags = json::array())
{
    json cmd;
    cmd["function"] = func;
    cmd["metadata"] = json::object();
    cmd["parameters"] = std::move(params);
    cmd["tags"] = std::move(tags);
    json result;
    result["command"] = std::move(cmd);
    return result;
}

static json move_meta(bool relative_xy)
{
    json m;
    json rel;
    rel["a"] = true; rel["b"] = true;
    rel["x"] = relative_xy; rel["y"] = relative_xy; rel["z"] = false;
    m["relative"] = std::move(rel);
    return m;
}

static json cmd_change_toolhead(int index)
{
    json p;
    p["index"] = index;
    return make_command("change_toolhead", std::move(p));
}

static json cmd_change_toolhead(int index, double x, double y)
{
    json p;
    p["index"] = index; p["x"] = x; p["y"] = y;
    return make_command("change_toolhead", std::move(p));
}

static json cmd_move(const AxisState &axis, const AxisState &prev, const std::string &tag = "")
{
    json p;
    p["x"] = axis.x; p["y"] = axis.y; p["z"] = axis.z;
    p["a"] = axis.a; p["b"] = 0.0; p["feedrate"] = axis.feedrate;
    json tags = tag.empty() ? json::array() : json::array({tag});
    json cmd;
    cmd["function"] = "move";
    cmd["metadata"] = move_meta(false);
    cmd["parameters"] = std::move(p);
    cmd["tags"] = std::move(tags);
    json result;
    result["command"] = std::move(cmd);
    return result;
}

static json cmd_temperature(int index, int temp)
{
    json p;
    p["index"] = index; p["temperature"] = temp;
    return make_command("set_toolhead_temperature", std::move(p));
}

static json cmd_wait_temp(int index)
{
    json p;
    p["index"] = index;
    return make_command("wait_for_temperature", std::move(p));
}

static json cmd_fan(int index, bool on)
{
    json p;
    p["index"] = index; p["value"] = on;
    return make_command("toggle_fan", std::move(p));
}

static json cmd_fan_duty(int index, double duty)
{
    json p;
    p["index"] = index; p["value"] = duty;
    return make_command("fan_duty", std::move(p));
}

static json cmd_comment(const std::string &text)
{
    json p;
    p["comment"] = text;
    return make_command("comment", std::move(p));
}

static json cmd_delay(int seconds)
{
    json p;
    p["seconds"] = seconds;
    return make_command("delay", std::move(p));
}

static double compute_time(const AxisState &prev, const AxisState &cur)
{
    double dist;
    if (cur.x == prev.x && cur.y == prev.y && cur.z == prev.z && cur.a != prev.a)
        dist = std::abs(cur.a - prev.a);
    else
        dist = std::sqrt((cur.x-prev.x)*(cur.x-prev.x) + (cur.y-prev.y)*(cur.y-prev.y) + (cur.z-prev.z)*(cur.z-prev.z));
    return (cur.feedrate <= 0.0) ? 0.0 : dist / cur.feedrate;
}

static std::string tag_from_comment(const std::string &c)
{
    if (c.find("WALL-OUTER") != std::string::npos || c.find("External perimeter") != std::string::npos) return "WALL_OUTER_0";
    if (c.find("WALL-INNER") != std::string::npos || c.find("Perimeter") != std::string::npos) return "WALL_INNER_0";
    if (c.find("FILL") != std::string::npos || c.find("Internal infill") != std::string::npos || c.find("Solid infill") != std::string::npos) return "FILL_0";
    if (c.find("TOP SURFACE") != std::string::npos || c.find("Top solid infill") != std::string::npos) return "TOP_SURFACE_0";
    if (c.find("SUPPORT") != std::string::npos && c.find("INTERFACE") != std::string::npos) return "SUPPORT_INTERFACE_0";
    if (c.find("SUPPORT") != std::string::npos) return "SUPPORT_0";
    if (c.find("SKIRT") != std::string::npos) return "SKIRT_0";
    if (c.find("PRIME TOWER") != std::string::npos) return "PRIME_TOWER_0";
    return "";
}

static int makerbot_export_impl(const char *gcode_path, const char *makerbot_path,
                                char *error_message, MakerBotConfig_C *cfg)
{
    std::ifstream gcode_file(gcode_path);
    if (!gcode_file.is_open()) {
        snprintf(error_message, 256, "Failed to open G-code file: %s", gcode_path);
        return 1;
    }

    std::vector<json> commands;
    AxisState axis, prev_axis;
    PrinterSettings settings;
    std::string current_tag;
    bool relative_e = false;
    double absolute_e = 0.0;
    int current_tool = 0;
    bool past_first_layer = false;  // Skip Prusa startup (bed leveling, purge, Z=40 etc)
    double last_x = 0.0, last_y = 0.0, last_z = 0.0;  // Track position from skipped gcode

    std::string line;
    while (std::getline(gcode_file, line)) {
        size_t semi = line.find(';');
        std::string comment;
        if (semi != std::string::npos) {
            comment = line.substr(semi + 1);
            line = line.substr(0, semi);
        }
        trim_inplace(line);
        trim_inplace(comment);

        // Extract settings from gcode comments (before skipping)
        if (settings.bed_temp == 0) {
            auto btpos = comment.find("first_layer_bed_temperature = ");
            if (btpos != std::string::npos) {
                settings.bed_temp = std::stoi(trim_copy(comment.substr(btpos + 30)));
                if (settings.bed_temp > 0) settings.heat_bed = true;
            } else {
                auto btpos2 = comment.find("bed_temperature = ");
                if (btpos2 != std::string::npos) {
                    settings.bed_temp = std::stoi(trim_copy(comment.substr(btpos2 + 18)));
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
                auto pre_tokens = split(line, ' ');
                for (auto &t : pre_tokens) if (!t.empty() && t[0] == 'E') absolute_e = 0.0;
                continue;
            }
            // Extract chamber temperature from M141/M191 in start gcode
            if (line[0] == 'M' && line.size() >= 3 && (line.substr(0, 3) == "M141" || line.substr(0, 3) == "M191")) {
                auto pre_tokens = split(line, ' ');
                for (auto &t : pre_tokens)
                    if (!t.empty() && t[0] == 'S') {
                        int temp = std::stoi(t.substr(1));
                        if (temp > settings.chamber_temp)
                            settings.chamber_temp = temp;
                    }
                continue;
            }
        }

        // Detect first LAYER_CHANGE - skip all gcode commands before it
        // (Prusa gcode has bed leveling, Z=40 clearance, purge lines etc before first layer)
        if (!past_first_layer) {
            if (comment.find("LAYER_CHANGE") != std::string::npos) {
                past_first_layer = true;
                axis.x = last_x;
                axis.y = last_y;
                axis.z = last_z > 0.0 && last_z < 10.0 ? last_z : 0.2;
                commands.push_back(cmd_comment(trim_copy(comment)));
            }
            continue;  // Skip everything before first layer
        }

        if (!comment.empty() && comment[0] == ' ') {
            std::string tag = tag_from_comment(comment);
            if (!tag.empty()) current_tag = tag;
            if (comment.find("LAYER_CHANGE") != std::string::npos || comment.find("LAYER") != std::string::npos)
                commands.push_back(cmd_comment(trim_copy(comment)));
        }

        // Skip end-of-print gcode (M84, M400, etc.)
        if (line.find("M84") == 0 || line.find("M400") == 0)
            continue;

        if (line.empty()) continue;

        auto tokens = split(line, ' ');
        if (tokens.empty()) continue;
        const std::string &cmd = tokens[0];

        if (cmd == "G0" || cmd == "G1") {
            if (tokens.size() == 2 && tokens[1][0] == 'F') {
                axis.feedrate = std::stod(tokens[1].substr(1)) / 60.0;
            } else {
                prev_axis = axis;
                double new_e = absolute_e;
                bool has_e = false;
                for (size_t i = 1; i < tokens.size(); ++i) {
                    char c = tokens[i][0];
                    if (c == 'E') { new_e = std::stod(tokens[i].substr(1)); has_e = true; }
                    else if (c == 'X') axis.x = std::stod(tokens[i].substr(1));
                    else if (c == 'Y') axis.y = std::stod(tokens[i].substr(1));
                    else if (c == 'Z') axis.z = std::stod(tokens[i].substr(1));
                    else if (c == 'F') axis.feedrate = std::stod(tokens[i].substr(1)) / 60.0;
                }
                if (has_e) {
                    if (relative_e) { axis.a = new_e; absolute_e += new_e; }
                    else { axis.a = new_e - absolute_e; absolute_e = new_e; }
                } else axis.a = 0.0;

                // Skip end-gcode moves with excessive Z (e.g. G1 Z25.55 head-lift at end)
                if (axis.z > 10.0 && !has_e)
                    continue;

                if (axis.x < settings.x_min) settings.x_min = axis.x;
                if (axis.x > settings.x_max) settings.x_max = axis.x;
                if (axis.y < settings.y_min) settings.y_min = axis.y;
                if (axis.y > settings.y_max) settings.y_max = axis.y;
                if (axis.z < settings.z_min) settings.z_min = axis.z;
                if (axis.z > settings.z_max) settings.z_max = axis.z;
                if (axis.z > 0.01) settings.z_heights.insert(std::round(axis.z * 100.0) / 100.0);
                if (has_e && axis.a > 0.0) {
                    if (current_tool == 0) settings.total_extrusion_0 += axis.a;
                    else settings.total_extrusion_1 += axis.a;
                }

                bool is_retract = (axis.a < 0.0 && axis.x == prev_axis.x && axis.y == prev_axis.y);
                bool is_restart = (axis.a > 0.0 && axis.x == prev_axis.x && axis.y == prev_axis.y);
                std::string tag;
                if (is_retract) tag = "Retract";
                else if (is_restart) tag = "Restart";
                else if (axis.a == 0.0) tag = "Travel Move";
                else if (!current_tag.empty()) tag = current_tag;

                commands.push_back(cmd_move(axis, prev_axis, tag));
                settings.time += compute_time(prev_axis, axis);
            }
        } else if (cmd == "G92") {
            for (auto &t : tokens) if (t[0] == 'E') absolute_e = 0.0;
        } else if (cmd == "M82") {
            relative_e = false;
        } else if (cmd == "M83") {
            relative_e = true;
        } else if (cmd == "M104" || cmd == "M109") {
            int tool_index = current_tool, temperature = 0;
            for (auto &t : tokens) {
                if (t[0] == 'T') tool_index = std::stoi(t.substr(1));
                else if (t[0] == 'S') temperature = std::stoi(t.substr(1));
            }
            if (tool_index == 0 && settings.tool0_temp == 0) settings.tool0_temp = temperature;
            else if (tool_index == 1 && settings.tool1_temp == 0) settings.tool1_temp = temperature;
        } else if (cmd == "M106") {
            int fan_idx = 0; double duty = 1.0;
            for (auto &t : tokens) {
                if (t[0] == 'P') fan_idx = std::stoi(t.substr(1));
                else if (t[0] == 'S') duty = std::stod(t.substr(1)) / 255.0;
            }
            commands.push_back(cmd_fan(fan_idx, true));
            commands.push_back(cmd_fan_duty(fan_idx, duty));
        } else if (cmd == "M107") {
            int fan_idx = 0;
            for (auto &t : tokens) if (t[0] == 'P') fan_idx = std::stoi(t.substr(1));
            commands.push_back(cmd_fan(fan_idx, false));
        } else if (cmd == "M140" || cmd == "M190") {
            for (auto &t : tokens)
                if (!t.empty() && t[0] == 'S') settings.bed_temp = std::stoi(t.substr(1));
        } else if (cmd == "M141" || cmd == "M191") {
            for (auto &t : tokens)
                if (!t.empty() && t[0] == 'S') {
                    int temp = std::stoi(t.substr(1));
                    if (temp > settings.chamber_temp)
                        settings.chamber_temp = temp;
                }
        } else if (cmd == "T0" || cmd == "T1") {
            current_tool = std::stoi(cmd.substr(1));
            commands.push_back(cmd_change_toolhead(current_tool));
        }
    }

    if (!past_first_layer) {
        snprintf(error_message, 256, "No LAYER_CHANGE found in gcode - cannot convert");
        return 1;
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

    if (cfg) {
        if (cfg->tool0_temp > 0) settings.tool0_temp = cfg->tool0_temp;
        if (cfg->tool1_temp > 0) settings.tool1_temp = cfg->tool1_temp;
        if (cfg->bed_temp > 0) { settings.bed_temp = cfg->bed_temp; settings.heat_bed = true; }
    }

    settings.num_z_layers = (int)settings.z_heights.size();
    if (settings.tool0_temp == 0) settings.tool0_temp = 210;
    if (settings.tool1_temp == 0) settings.tool1_temp = 160;

    // Center the model's bounding box on the Method bed (centered coordinate system)
    // The Method bed goes from -bed_width/2 to +bed_width/2 in X, -bed_depth/2 to +bed_depth/2 in Y
    // OrcaSlicer gcode uses corner-origin (0,0) coordinates
    double x_offset = (settings.x_min + settings.x_max) / 2.0;
    double y_offset = (settings.y_min + settings.y_max) / 2.0;
    double x_limit = (cfg && cfg->bed_width > 0.0) ? cfg->bed_width / 2.0 - 5.0 : 71.0;
    double y_limit = (cfg && cfg->bed_depth > 0.0) ? cfg->bed_depth / 2.0 - 5.0 : 90.0;

    for (auto &cmd : commands) {
        auto &c = cmd["command"];
        std::string fn = c.value("function", "");
        if (fn == "move") {
            auto &p = c["parameters"];
            auto &meta = c["metadata"];
            json rel = meta.contains("relative") ? meta["relative"] : json::object();
            bool rel_x = rel.value("x", false);
            bool rel_y = rel.value("y", false);
            // Only center absolute coordinates — relative deltas must not be offset
            double x = p["x"].get<double>();
            double y = p["y"].get<double>();
            if (!rel_x) x -= x_offset;
            if (!rel_y) y -= y_offset;
            if (x < -x_limit) x = -x_limit;
            if (x >  x_limit) x =  x_limit;
            if (y < -y_limit) y = -y_limit;
            if (y >  y_limit) y =  y_limit;
            p["x"] = x;
            p["y"] = y;
        } else if (fn == "change_toolhead") {
            auto &p = c["parameters"];
            if (p.contains("x")) {
                double x = p["x"].get<double>() - x_offset;
                if (x < -x_limit) x = -x_limit;
                if (x >  x_limit) x =  x_limit;
                p["x"] = x;
            }
            if (p.contains("y")) {
                double y = p["y"].get<double>() - y_offset;
                if (y < -y_limit) y = -y_limit;
                if (y >  y_limit) y =  y_limit;
                p["y"] = y;
            }
        }
    }
    settings.x_min -= x_offset; settings.x_max -= x_offset;
    settings.y_min -= y_offset; settings.y_max -= y_offset;
    // Clamp bounding box to valid range too
    if (settings.x_min < -x_limit) settings.x_min = -x_limit;
    if (settings.x_max >  x_limit) settings.x_max =  x_limit;
    if (settings.y_min < -y_limit) settings.y_min = -y_limit;
    if (settings.y_max >  y_limit) settings.y_max =  y_limit;

    {
        // Match reference startup: tool1 idle heat, tool0 active, change_toolhead, wait, delay
        std::vector<json> startup;
        startup.push_back(cmd_temperature(1, 160));  // Tool 1 idle temperature
        startup.push_back(cmd_fan(1, true));
        startup.push_back(cmd_fan_duty(1, 1.0));
        startup.push_back(cmd_temperature(0, settings.tool0_temp));
        startup.push_back(cmd_change_toolhead(0, 50.0, -80.0));
        startup.push_back(cmd_wait_temp(0));
        startup.push_back(cmd_delay(5));
        startup.push_back(cmd_fan(1, false));
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
        {"build_plane_temperature", settings.heat_bed ? settings.bed_temp : 0},
        {"chamber_temperature", settings.chamber_temp},
        {"duration_s", std::round(settings.time * 10.0) / 10.0},
        {"commanded_duration_s", std::round(settings.time * 10.0) / 10.0},
        {"total_commands", settings.command_count},
        {"extrusion_distance_mm", settings.total_extrusion_0 + settings.total_extrusion_1},
        {"extrusion_distances_mm", {settings.total_extrusion_0, settings.total_extrusion_1}},
        {"extrusion_mass_g", 0.0},
        {"extrusion_masses_g", {0.0, 0.0}},
        {"model_counts", json::array({{{"count", 1}, {"name", "default"}}})},
        {"preferences", {{"bounding_box", nullptr}, {"overrides", json::object()}, {"print_mode", "balanced"}}},
        {"uuid", make_uuid()}
    };

    std::string meta_str = meta.dump(4);
    std::string toolpath_str = json(commands).dump();

    // Write zip using miniz directly
    mz_zip_archive zip;
    memset(&zip, 0, sizeof(zip));
    if (!mz_zip_writer_init_file(&zip, makerbot_path, 0)) {
        snprintf(error_message, 256, "Failed to create zip file: %s", makerbot_path);
        return 1;
    }
    if (!mz_zip_writer_add_mem(&zip, "meta.json", meta_str.data(), meta_str.size(), MZ_NO_COMPRESSION) ||
        !mz_zip_writer_add_mem(&zip, "print.jsontoolpath", toolpath_str.data(), toolpath_str.size(), MZ_NO_COMPRESSION)) {
        snprintf(error_message, 256, "Failed to add entries to zip file");
        mz_zip_writer_finalize_archive(&zip);
        mz_zip_writer_end(&zip);
        return 1;
    }
    if (!mz_zip_writer_finalize_archive(&zip)) {
        snprintf(error_message, 256, "Failed to finalize zip file");
        mz_zip_writer_end(&zip);
        return 1;
    }
    mz_zip_writer_end(&zip);

    fprintf(stderr, "MakerBotLib: export completed: %s (%zu commands, %ds estimated)\n",
            makerbot_path, settings.command_count, (int)std::ceil(settings.time));
    return 0;
}

} // anonymous namespace

extern "C" {

int makerbot_export(const char *gcode_path, const char *makerbot_path,
                    char *error_message, MakerBotConfig_C *cfg)
{
    try {
        return makerbot_export_impl(gcode_path, makerbot_path, error_message, cfg);
    } catch (const std::exception &e) {
        snprintf(error_message, 256, "Exception: %s", e.what());
        return 1;
    } catch (...) {
        snprintf(error_message, 256, "Unknown exception during MakerBot export");
        return 1;
    }
}

} // extern "C"
