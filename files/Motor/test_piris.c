/***************************************************************
Author       : hao2062
Date         : 2025-08-01 16:03:49
LastEditors  : hao2062 | 894357340@qq.com
LastEditTime : 2025-08-01 16:09:27
FilePath     : /sdk/kernel/linux_drivers/ms41908/test_piris.c
Description  : 
Copyright (c) 2025 by 894357340@qq.com, All Rights Reserved. 
***************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>

#define MOTOR_IOC_MAGIC       'M'
#define MOTOR_IOC_PIRIS       _IOW(MOTOR_IOC_MAGIC, 0x03, int)

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <piris> <pos>\n", argv[0]);
        return 1;
    }

    const char *motor_name = argv[1];
    int pos = atoi(argv[2]);

    int cmd;
    if (strcmp(motor_name, "piris") == 0) {
        cmd = MOTOR_IOC_PIRIS;
    } else {
        fprintf(stderr, "Unknown motor: %s (should be piris)\n", motor_name);
        return 1;
    }

    int fd = open("/dev/ms41908_3", O_RDWR);
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
