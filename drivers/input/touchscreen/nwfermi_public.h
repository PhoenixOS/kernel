#ifndef _NWFERMI_PUBLIC_H
#define _NWFERMI_PUBLIC_H

#define FERMI_TOUCH_DOWN  1
#define FERMI_TOUCH       2
#define FERMI_TOUCH_UP    3
#define FERMI_TOUCH_RIGHT 4
#define FERMI_TOUCH_HOVER 5

struct fermi_touch_t {
	int state;
	int x;
	int y;
	unsigned char id;
};

struct fermi_touch_report_t {
	struct fermi_touch_t touch[2];
	unsigned char count;
};

#endif
