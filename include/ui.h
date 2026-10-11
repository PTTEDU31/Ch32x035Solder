#ifndef UI_H
#define UI_H

/* Screen + knob handling. ui_poll() redraws only fields that changed. */
void ui_init(void);
void ui_poll(void);

#endif
