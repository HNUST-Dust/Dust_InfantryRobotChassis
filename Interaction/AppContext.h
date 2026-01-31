#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Application context / service locator for C entry points (e.g., CubeMX FreeRTOS start task)
void App_Init(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Robot;

// C++ accessors
Robot& App_Robot(void);

#endif
