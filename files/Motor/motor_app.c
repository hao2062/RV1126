#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define AF_SOCKET_PATH "/tmp/af_control.sock"

// 命令类型定义
typedef enum {
    CMD_SET_PIRIS = 1,    // 设置光圈位置
    CMD_SET_ZOOM,         // 设置变焦位置
    CMD_GET_ZOOM,         // 获取当前变焦位置
    CMD_ONESHOT_FOCUS,    // 触发自动对焦
    CMD_EXIT              // 退出服务
} af_cmd_type_t;

// 命令结构体
typedef struct {
    af_cmd_type_t cmd;    // 命令类型
    int param1;           // 参数1（如位置）
    int param2;           // 参数2（备用）
    char result[256];     // 执行结果
} af_cmd_t;

// 发送命令到服务端
int send_command(af_cmd_t *cmd) {
    int sock_fd;
    struct sockaddr_un addr;

    // 创建 socket
    sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock_fd == -1) {
        perror("socket");
        return -1;
    }

    // 连接服务端
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, AF_SOCKET_PATH);

    if (connect(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("connect");
        close(sock_fd);
        return -1;
    }

    // 发送命令
    if (send(sock_fd, cmd, sizeof(*cmd), 0) != sizeof(*cmd)) {
        perror("send");
        close(sock_fd);
        return -1;
    }

    // 接收结果
    if (recv(sock_fd, cmd, sizeof(*cmd), 0) != sizeof(*cmd)) {
        perror("recv");
        close(sock_fd);
        return -1;
    }

    close(sock_fd);
    return 0;
}

// 显示帮助信息
void show_help() {
    printf("\n=== Motor Control Commands ===\n");
    printf("piris <position>    - Set piris position\n");
    printf("zoom <position>     - Set zoom position\n");
    printf("get_zoom            - Get current zoom position\n");
    printf("focus               - Trigger oneshot focus\n");
    printf("exit                - Exit program\n");
    printf("==============================\n\n");
}

int main() {
    af_cmd_t cmd;
    char input[256];
    char command[64];
    int param;

    printf("Motor Control Application Starting...\n");
    show_help();

    while (1) {
        printf("motor_app ");
        fflush(stdout);

        // 获取用户输入
        if (!fgets(input, sizeof(input), stdin)) {
            break;
        }

        // 移除换行符
        input[strcspn(input, "\n")] = 0;

        // 解析命令
        if (sscanf(input, "%s %d", command, &param) >= 1) {
            if (strcmp(command, "piris") == 0) {
                cmd.cmd = CMD_SET_PIRIS;
                cmd.param1 = param;
                send_command(&cmd);
                printf("Result: %s\n", cmd.result);
            } else if (strcmp(command, "zoom") == 0) {
                cmd.cmd = CMD_SET_ZOOM;
                cmd.param1 = param;
                send_command(&cmd);
                printf("Result: %s\n", cmd.result);
            } else if (strcmp(command, "get_zoom") == 0) {
                cmd.cmd = CMD_GET_ZOOM;
                send_command(&cmd);
                printf("Result: %s\n", cmd.result);
            } else if (strcmp(command, "focus") == 0) {
                cmd.cmd = CMD_ONESHOT_FOCUS;
                send_command(&cmd);
                printf("Result: %s\n", cmd.result);
            } else if (strcmp(command, "exit") == 0) {
                cmd.cmd = CMD_EXIT;
                send_command(&cmd);
                printf("Result: %s\n", cmd.result);
                break;
            } else {
                printf("Unknown command: %s\n", command);
                show_help();
            }
        } else {
            printf("Invalid input. Type 'help' for available commands.\n");
        }
    }

    printf("Motor Control Application Exiting...\n");
    return 0;
}