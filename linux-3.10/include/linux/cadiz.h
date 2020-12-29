#ifndef CADIZ_H
#define CADIZ_H

#define MAX_CADIZ_REGS 100

typedef struct cadiz_regs {
    int len;
    int regs[MAX_CADIZ_REGS][2];
}cadiz_settings;

#define CADIZ_IOCTL_SET_REGISTERS _IOW('s', 1, cadiz_settings)
#define CADIZ_IOCTL_GET_REGISTERS _IOWR('s', 2, cadiz_settings)

enum {
	CADIZ = 0x1,
	GSENSOR = 0x2,
	ECOMPASS = 0x4,
	LIGHTSENSOR = 0x8,
	UNKNOWN = 0x10
};

int cadiz_power_control(int owner, bool enable);

#endif
