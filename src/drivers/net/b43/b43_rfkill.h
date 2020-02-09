#ifndef B43_RFKILL_H_
#define B43_RFKILL_H_

struct b43_wldev;

void b43_rfkill_poll();

bool b43_is_hw_radio_enabled(struct b43_wldev *dev);

#endif /* B43_RFKILL_H_ */
