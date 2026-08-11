#pragma once

#if defined(__GNUC__)
#define EDGEZ_API __attribute__((visibility("default")))
#else
#define EDGEZ_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize and run the complete EdgeZ device application. */
EDGEZ_API void edgez_app_run(void);

#ifdef __cplusplus
}
#endif
