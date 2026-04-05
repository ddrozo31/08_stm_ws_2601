/**
 * @file    esc_app.h
 * @brief   ESC application layer — UART-controlled motor state machine.
 *
 * Sits on top of custom_foc.c (motor control) and esc_comm.c (UART).
 * Implements the ESC state machine for RPi5 command/telemetry.
 * Active only when BUILD_ESC is defined (Release configuration).
 */
#ifndef ESC_APP_H
#define ESC_APP_H

#ifdef BUILD_ESC

#include <stdint.h>

/* ESC application states (sent in telemetry byte) */
typedef enum {
  ESC_BOOT         = 0,
  ESC_WAIT_NEUTRAL = 1,
  ESC_READY        = 2,
  ESC_FORWARD      = 3,
  ESC_BRAKE        = 4,
  ESC_REVERSE      = 5,
  ESC_FAULT        = 6,
} ESC_State_t;

/** Initialize ESC application layer (call after CFOC_Init in main.c). */
void ESC_APP_Init(void);

/** 1 kHz tick — call from SysTick after CFOC_MediumFrequencyTask(). */
void ESC_APP_Tick(void);

#endif /* BUILD_ESC */
#endif /* ESC_APP_H */
