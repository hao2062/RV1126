#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>

#define MOTOR_IOC_MAGIC       'M'
#define MOTOR_IOC_ZOOM        _IOW(MOTOR_IOC_MAGIC, 0x01, int)
#define MOTOR_IOC_ZOOM1       _IOW(MOTOR_IOC_MAGIC, 0x02, int)

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <zoom2|zoom3> <pos>\n", argv[0]);
        return 1;
    }

    const char *motor_name = argv[1];
    int pos = atoi(argv[2]);

    int cmd;
    if (strcmp(motor_name, "zoom2") == 0) {
        cmd = MOTOR_IOC_ZOOM;
    } else if (strcmp(motor_name, "zoom3") == 0) {
        cmd = MOTOR_IOC_ZOOM1;
    } else {
        fprintf(stderr, "Unknown motor: %s (should be zoom2 or zoom3)\n", motor_name);
        return 1;
    }

    int fd = open("/dev/ms41908_2", O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    if (ioctl(fd, cmd, pos) < 0) {
        perror("ioctl");
        close(fd);
        return 1;
    }

    printf("Set %s to position %d success.\n", motor_name, pos);
    close(fd);
    return 0;
}
