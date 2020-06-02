#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "Qmodel.h"

#define SYS_ID_SELF		42
#define COMP_ID_SELF		MAV_COMP_ID_LOG
#define SYS_ID_PF4FLOW		81
#define COMP_ID_PF4FLOW		50
#define REFRESH_PX4		72
#define PX4DEVICE		"/dev/ttyACM0"
int serial_open(char *path);
int serial_write(int fd, uint8_t *buffer, int len);
int serial_read(int fd, uint8_t *buffer, int len);
bool configure_px4flow(int fd);
int readDataPX4 (int fd, struct Quad *quad);

