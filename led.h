#ifndef LED_H
#define LED_H


void update_ledPick(void);
void update_FrontLed(void);

void set_ledPick(void);
void set_frontLed(bool state);
void set_all_led(bool state);

void led_start(void);

#endif /* LED_H */
