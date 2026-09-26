#ifndef MAKERBOT_LIB_H
#define MAKERBOT_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int    tool0_temp;
    int    tool1_temp;
    int    bed_temp;
    double bed_width;
    double bed_depth;
} MakerBotConfig_C;

// Returns 0 on success, 1 on failure. error_message is filled on failure (must be >= 256 bytes).
int makerbot_export(const char *gcode_path, const char *makerbot_path,
                    char *error_message, MakerBotConfig_C *cfg);

#ifdef __cplusplus
}
#endif

#endif // MAKERBOT_LIB_H
