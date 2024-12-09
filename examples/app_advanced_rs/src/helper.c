#include "param.h"
#include "log.h"

uint8_t app_param1;

uint8_t app_log1;
float app_log2;

/**
 * Parameters for use in rust
 */
PARAM_GROUP_START(app)
PARAM_ADD_CORE(PARAM_UINT8, param1, &app_param1)
PARAM_GROUP_STOP(app)

/**
 * Logging for use in rust
 */
LOG_GROUP_START(app)
LOG_ADD(LOG_UINT8, log1, &app_log1)
LOG_ADD(LOG_FLOAT, log2, &app_log2)
LOG_GROUP_STOP(app)