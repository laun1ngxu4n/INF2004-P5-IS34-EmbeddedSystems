/*
 * Onboard status LED (NeoPixel on GP18, PIO) and buzzer (GP22, PWM)
 * helpers for the bring-up tests. Robo Pico: 2 RGB LEDs on GP18; the
 * buzzer has a mute switch on the board - check it if you hear nothing.
 */
#ifndef NOTIFY_HW_H
#define NOTIFY_HW_H

#include <stdint.h>
#include <stdbool.h>

#define NEOPIXEL_PIN    18u
#define NEOPIXEL_COUNT  2u
#define BUZZER_PIN      22u

bool neopixel_init(void);
/* Same colour on every LED. Non-blocking while the 8-word FIFO has room. */
void neopixel_set_all(uint8_t r, uint8_t g, uint8_t b);

void buzzer_init(void);
void buzzer_tone(uint32_t hz);   /* 0 = off */

#endif /* NOTIFY_HW_H */
