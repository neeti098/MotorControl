#ifndef FEATURE_CONFIG_H
#define FEATURE_CONFIG_H

/* Compile-time feature switches (1 = built, 0 = compiled out).
 *
 * Kept separate from app_config.h on purpose:
 *   app_config.h     -> VALUES     (pins, periods, buffer sizes, credentials).
 *                       The Firmware Topology editor reads and writes the APP_* macros there.
 *   feature_config.h -> PRESENCE   (which optional subsystems are compiled in at all).
 *
 * Example:
 *   #define FEATURE_OTA        0
 *   #define FEATURE_TELEMETRY  1
 */

#endif /* FEATURE_CONFIG_H */
