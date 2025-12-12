/***************************************************************
Author       : hao2062
Date         : 2025-06-12 14:13:25
LastEditors  : hao2062 | 894357340@qq.com
LastEditTime : 2025-11-07 10:56:50
FilePath     : /sdk/external/rkmedia/examples/common/af.c
Description  : 
Copyright (c) 2025 by 894357340@qq.com, All Rights Reserved. 
***************************************************************/
/* examples/common/af.c */
#include "af.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/wait.h>

#include "rk_aiq_types_af_algo.h"
#include "rk_aiq_types_af_algo_int.h"
#include "rk_aiq_user_api_af.h"
#include "rk_aiq_user_api_sysctl.h"
#include "rk_aiq_user_api_awb.h"
#include "rk_aiq_uapi_awb_int.h"
#include "rk_aiq_uapi_af_int.h"
#include "rk_aiq_user_api_imgproc.h"
#include <sys/ioctl.h>

// include ae 库
#include "rk_aiq_user_api_ae.h"

// socket 通信库
#include <sys/socket.h>
#include <sys/un.h>
#include <pthread.h>
#include <string.h>

#define AF_SOCKET_PATH "/tmp/af_control.sock"

static int s_server_fd = -1;
static volatile bool s_running_app = true;
// socket 通信库

// AF 线程
// static pthread_t s_tid_af;
static volatile bool s_running  = false;
static const rk_aiq_sys_ctx_t *s_ctx = NULL;

// 服务器端命令和结果
typedef enum {
    CMD_SET_PIRIS = 1,    // 设置光圈位置
    CMD_SET_ZOOM,         // 设置变焦位置
    CMD_GET_ZOOM,         // 获取当前变焦位置
    CMD_ONESHOT_FOCUS,    // 触发自动对焦
    CMD_EXIT              // 退出服务
} af_cmd_type_t;

typedef struct {
    af_cmd_type_t cmd;    // 命令类型
    int param1;           // 参数1（如位置）
    int param2;           // 参数2（备用）
    char result[256];     // 执行结果
} af_cmd_t;
// 服务器端命令和结果

// // /* Focus 电机参数 */
// #define CODE_MIN         (-467)
// #define CODE_MAX         (2632)
// #define STEP_COARSE      (150)
// #define STEP_FINE        (5)

// // TimeDot Night
// // [0.0000 0.0300 0.0300 0.0300 0.0300 0.0300 0.0300 0.0300 0.0300 0.0300 ]

// // 不同 Zoom Pos 下的 PIrisDot 
// // 每张表一整条路线（和 XML 对齐） 
// static const int p1[10]  = { 10, 10, 228, 228, 228, 228, 228, 406, 406, 406 };     // 31 5  0     10 228 406
// static const int p2[10]  = { 9, 9, 114, 114, 114, 114, 114, 161, 161, 161 };       // 33 23 7     8  28  181 
// static const int p3[10]  = { 9, 9, 114, 114, 114, 114, 114, 161, 161, 161 };       // 32 11 8     9  114 161
// static const int p4[10]  = { 10, 10, 114, 114, 114, 114, 114, 138, 138, 138 };     // 31 11 9     10 114 138
// static const int p5[10]  = { 12, 12, 56, 56, 56, 56, 56, 128, 128, 128 };          // 30 17 10    12 56  128
// static const int p6[10]  = { 10, 10, 45, 45, 45, 45, 45, 114, 114, 114 };          // 31 19 11    10 45  114
// static const int p7[10]  = { 10, 10, 50, 50, 50, 50, 50, 114, 114, 114 };          // 31 18 11    10 50  114
// static const int p8[10]  = { 10, 10, 50, 50, 50, 50, 50, 114, 114, 114 };          // 31 18 11    10 50  114
// static const int p9[10]  = { 10, 10, 71, 71, 71, 71, 71, 114, 114, 114 };          // 31 15 11    10 71  114
// static const int p10[10] = { 10, 10, 90, 90, 90, 90, 90, 114, 114, 114 };          // 31 13 11    10 90  114 
// static const int p11[10] = { 16, 16, 80, 80, 80, 80, 80, 114, 114, 114 };          // 28 14 11    16 80  114
// static const int p12[10] = { 16, 16, 80, 80, 80, 80, 80, 114, 114, 114 };          // 28 14 11    16 80  114
// static const int p13[10] = { 16, 16, 90, 90, 90, 90, 90, 114, 114, 114 };          // 28 13 11    16 90  114
// static const int p14[10] = { 17, 17, 90, 90, 90, 90, 90, 114, 114, 114 };          // 27 13 11    17 90  114
// static const int p15[10] = { 17, 17, 80, 80, 80, 80, 80, 114, 114, 114 };          // 27 14 11    17 80  114

// static const float g1[10]  = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 16.0000, 16.0000, 32.0000, 64.00000 };
// static const float g2[10]  = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 16.0000, 16.0000, 32.0000, 64.00000 };
// static const float g3[10]  = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 16.0000, 16.0000, 32.0000, 64.00000 };
// static const float g4[10]  = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 16.0000, 16.0000, 32.0000, 64.00000 };
// static const float g5[10]  = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 16.0000, 16.0000, 32.0000, 64.00000 };
// static const float g6[10]  = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 16.0000, 16.0000, 32.0000, 64.00000 };
// static const float g7[10]  = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 16.0000, 16.0000, 32.0000, 64.00000 };
// static const float g8[10]  = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 16.0000, 16.0000, 32.0000, 64.00000 };
// static const float g9[10]  = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 32.0000, 32.0000, 32.0000, 64.00000 };
// static const float g10[10] = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 32.0000, 32.0000, 32.0000, 64.00000 };
// static const float g11[10] = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 32.0000, 32.0000, 32.0000, 64.00000 };
// static const float g12[10] = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 32.0000, 32.0000, 32.0000, 64.00000 };
// static const float g13[10] = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 32.0000, 32.0000, 32.0000, 64.00000 };
// static const float g14[10] = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 32.0000, 32.0000, 32.0000, 64.00000 };
// static const float g15[10] = { 1.0000, 1.0000, 1.0000, 4.0000, 8.0000, 16.0000, 32.0000, 32.0000, 32.0000, 64.00000 };

// static const float t1[10]  = { 0.0000, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042 };
// static const float t2[10]  = { 0.0000, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060 };
// static const float t3[10]  = { 0.0000, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060, 0.0060 };
// static const float t4[10]  = { 0.0000, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042 };
// static const float t5[10]  = { 0.0000, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045 };
// static const float t6[10]  = { 0.0000, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042, 0.0042 };
// static const float t7[10]  = { 0.0000, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045 };
// static const float t8[10]  = { 0.0000, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045, 0.0045 };
// static const float t9[10]  = { 0.0000, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050 };
// static const float t10[10] = { 0.0000, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050, 0.0050 };
// static const float t11[10] = { 0.0000, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040 };
// static const float t12[10] = { 0.0000, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040 };
// static const float t13[10] = { 0.0000, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040  };
// static const float t14[10] = { 0.0000, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040  };
// static const float t15[10] = { 0.0000, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040, 0.0040  };

// typedef struct {
//     int zoom_min;   // 区间下边界（含）
//     int zoom_max;   // 区间上边界（含）
//     const int* piris;   // 指向上面一张表
//     const float* gain;
//     const float* time;
// } ZoomPIrisRoute;   // 

// 按你的“Zoom Step -> 最佳Step”/倍率段划分，把区间和表对齐
// static const ZoomPIrisRoute kZoom2Piris[] = {
//     {0, 584, p1, g1, t1},   // 1X 附近
//     { 584, 881, p2, g2, t2},   // 2X
//     {  881, 1103, p3, g3, t3},   // 3X
//     {   1103,  1314, p4, g4, t4},   // 4X
//     {    1314,  1480, p5, g5, t5},   // 5X
//     {     1480,  1578, p6, g6, t6},   // 6X 
//     {      1578,  1639, p7, g7, t7},   // 7X
//     {       1639,  1680, p8, g8, t8},   // 8X
//     {        1680,  1705, p9, g9, t9},   // 9X
//     {         1705,  1721, p10, g10, t10},   // 10X
//     {          1721,  1732, p11, g11, t11},   // 11X
//     {           1732,  1740, p12, g12, t12},   // 12X
//     {            1740,  1746, p13, g13, t13},   // 13X
//     {             1746,  1751, p14, g14, t14},   // 14X
//     {              1751,  1755, p15, g15, t15},   // 15X
// };

// static const int kZoom2PirisCnt = sizeof(kZoom2Piris)/sizeof(kZoom2Piris[0]);

// #define ZOOM_HYSTERESIS  8   // 放抖：步进抖动小于 8 时不切表

// Function1: 设置 Zoom 变焦
static int set_zoom_position(int position) {

    if (position < 0 || position > 1755) { // 假设变焦位置范围是 0-1755
        printf("Error: Invalid zoom position %d (valid range: 0-1755)", position);
        return -1;
    }

    XCamReturn ret = rk_aiq_uapi_setOpZoomPosition(s_ctx, position);
    if (ret != XCAM_RETURN_NO_ERROR) {
        printf("Error: Failed to set zoom position %d (ret=%d)", position, ret);
        return -1;
    }

    rk_aiq_uapi_endOpZoomChange(s_ctx);
    
    printf("Zoom position set to %d successfully\n", position);
    return 0;
}

// Function2: 触发一次 Focus 变焦
static void set_focus_once( ) {
    int pos=0;
    XCamReturn ret;
    rk_aiq_uapi_getOpZoomPosition(s_ctx, &pos);
    ret = rk_aiq_uapi_setOpZoomPosition(s_ctx, pos+1);
    ret = rk_aiq_uapi_setOpZoomPosition(s_ctx, pos);
    if (ret != XCAM_RETURN_NO_ERROR) {
        printf("Error: Failed to set focus position\n");
        // return -1;
    }
    rk_aiq_uapi_endOpZoomChange(s_ctx);
}

void start_af_service() {
    struct sockaddr_un addr;

    // 创建 socket
    s_server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (s_server_fd == -1) {
        perror("socket");
        exit(1);
    }

    // 删除旧的 socket 文件
    unlink(AF_SOCKET_PATH);

    // 绑定地址
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, AF_SOCKET_PATH);

    if (bind(s_server_fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("bind");
        close(s_server_fd);
        exit(1);
    }

    // 监听连接
    if (listen(s_server_fd, 5) == -1) {
        perror("listen");
        close(s_server_fd);
        exit(1);
    }

    printf("AF service started on %s\n", AF_SOCKET_PATH);
}

void* command_thread() {
    struct sockaddr_un addr;
    int client_fd;
    socklen_t addr_len;
    af_cmd_t cmd;
    // int ret_zoom;

    printf("=== Function: command_thread started ===\n");

    while (s_running_app) {
        // Socket 监听
        addr_len = sizeof(addr);
        client_fd = accept(s_server_fd, (struct sockaddr*)&addr, &addr_len);
        if (client_fd == -1) {
            continue;
        }

        // 接收命令
        ssize_t bytes = recv(client_fd, &cmd, sizeof(cmd), 0);
        if (bytes == sizeof(cmd)) {
            printf("Received command\n");
            
            // 命令解析与执行
            switch (cmd.cmd) {
                // 设置光圈位置
                case CMD_SET_PIRIS:     
                    printf("Setting piris position to %d\n", cmd.param1);
                    snprintf(cmd.result, sizeof(cmd.result), "Piris position set to %d", cmd.param1);
                    break;
                
                // 设置变焦位置
                case CMD_SET_ZOOM:
                    set_zoom_position(cmd.param1);
                    printf("Setting zoom position to %d\n", cmd.param1);
                    snprintf(cmd.result, sizeof(cmd.result), "Zoom position set to %d", cmd.param1);
                    break;
                
                // 获取当前变焦位置
                case CMD_GET_ZOOM:
                    printf("Getting current zoom position\n");
                    snprintf(cmd.result, sizeof(cmd.result), "Current zoom position: %d", 500); // 示例值
                    break;

                // 
                case CMD_ONESHOT_FOCUS:
                    printf("Triggering oneshot focus\n");
                    set_focus_once();
                    snprintf(cmd.result, sizeof(cmd.result), "Oneshot focus triggered");
                    break;

                case CMD_EXIT:
                    printf("Exiting AF service\n");
                    s_running_app = false;
                    snprintf(cmd.result, sizeof(cmd.result), "AF service exiting");
                    break;

                default:
                    snprintf(cmd.result, sizeof(cmd.result), "Unknown command");
                    break;
            }

            // 发送结果
            send(client_fd, &cmd, sizeof(cmd), 0);
        }

        close(client_fd);
    }

    close(s_server_fd);
    unlink(AF_SOCKET_PATH);
    return NULL;
}

// static const ZoomPIrisRoute* pick_piris_table_by_zoom(int cur_zoom)
// {
//     // return &kZoom2Piris[0];
//     // 遍历 KZoom2Piris，找命中的区间
//     for (int i = 0; i < kZoom2PirisCnt; i++) {
//         if (cur_zoom >= kZoom2Piris[i].zoom_min && cur_zoom <= kZoom2Piris[i].zoom_max){
//             printf("=== pick table %d for zoom pos %d ===\n", i, cur_zoom);
//             return &kZoom2Piris[i];
//         }
//     }
    
//     return &kZoom2Piris[0];   // 没找到
// }

/* -------- 后台线程主体 -------- */
// static void* af_thread(void *arg)
// {
//     const rk_aiq_sys_ctx_t *ctx = (const rk_aiq_sys_ctx_t *)arg;
//     if (!ctx) return NULL;
    
//     // Uapi_LinAeRouteAttr_t linAeNightRouteAttr;
//     Uapi_LinAeRouteAttr_t night;
//     Uapi_LinAeRouteAttr_t day;

//     printf("=== AF thread started ===\n");

//     // int i = 0;

//     sleep(10);   /* 等待 5000 ms */

//     // ****** Set 一次 Zoom Pos ******
//     // XCamReturn ret = XCAM_RETURN_NO_ERROR;
//     // ret = rk_aiq_uapi_setOpZoomPosition(ctx, 1748);
    
//     // if (ret != XCAM_RETURN_NO_ERROR) {
//     //     printf("=== ret = %d ===\n", ret);
//     // }

//     // rk_aiq_uapi_endOpZoomChange(ctx);

//     // ******* 遍历 1x -> 15x 变焦位置，演示变焦和 AE 切表 ******
//     XCamReturn ret = XCAM_RETURN_NO_ERROR;

//     for (int i = 0; i < kZoom2PirisCnt; i++) {
        
//         // Set Zoom Pos
//         ret = rk_aiq_uapi_setOpZoomPosition(ctx, kZoom2Piris[i].zoom_min + 2);
//         printf("=== Set Zoom Position to %d ===\n", kZoom2Piris[i].zoom_min + 2);
//         if (ret != XCAM_RETURN_NO_ERROR) {
//             printf("=== ret = %d ===\n", ret);  
//         }

//         // ****** 获取 Day 和 Night 下的 PIris Gain Time 参数 ******
//         rk_aiq_user_api_ae_getLinAeNightRouteAttr(ctx, &night);
//         rk_aiq_user_api_ae_getLinAeDayRouteAttr(ctx, &day);
    
//         // array_size 校验，和 PIRIS_DOT_LEN 对齐
//         int n = night.array_size;
//         // if (n > PIRIS_DOT_LEN) n = PIRIS_DOT_LEN;

//         // ****** 获取当前 Zoom Pos ******
//         int pos = kZoom2Piris[i].zoom_min + 2;
        
//         // 选择对应表
//         const ZoomPIrisRoute* tbl = pick_piris_table_by_zoom(pos);

//         // 若表没变化则不下发，避免频繁 set

//         // ****** 写 Night Route ******
//         for (int k = 0; k < n; ++k) {
//             night.PIrisGainDot[k] = tbl->piris[k];       // 若你的类型是 double，则写 (double)tbl[i]
//             night.GainDot[k] = tbl->gain[k];
//             night.TimeDot[k] = tbl->time[k];
//         }

//         // ****** 写 Day Route ******
//         for (int k = 0; k < n; ++k) {
//             day.PIrisGainDot[k] = tbl->piris[k];       // 若你的类型是 double，则写 (double)tbl[i]
//             day.GainDot[k] = tbl->gain[k];
//             day.TimeDot[k] = tbl->time[k];
//         }

//         ret = rk_aiq_user_api_ae_setLinAeNightRouteAttr(ctx, night);
//         ret = rk_aiq_user_api_ae_setLinAeDayRouteAttr(ctx, day);

//         // usleep(1000 * 1000);  // 200ms，按需要调
        
//         // ****** 结束变焦，并对焦 ******
//         rk_aiq_uapi_endOpZoomChange(ctx);
        
//         sleep(5);   /* 等待 2000 ms */

//         // ****** 打印 Zoom 位置 ******
//         rk_aiq_uapi_getOpZoomPosition(ctx, &pos);
//         printf("=== [AE] zoom=%d ===\n", pos);
        
//         // ****** 打印 Night Route 下的 PIris Gain Time 参数 ******
//         for (int j = 0; j < night.array_size; j++) {
//             printf("PIrisGainDot_Night[%d] = %d\n", j, night.PIrisGainDot[j]);
//         }

//         for (int j = 0; j < night.array_size; j++) {
//             printf("GainDot_Night[%d] = %f\n", j, night.GainDot[j]);
//         }

//         for (int j = 0; j < night.array_size; j++) {
//             printf("TimeDot_Night[%d] = %f\n", j, night.TimeDot[j]);
//         }
        
//         // ****** 打印 Day Route 下的 PIris Gain Time 参数 ******
//         for (int j = 0; j < day.array_size; j++) {
//             printf("PIrisGainDot_Day[%d] = %d\n", j, day.PIrisGainDot[j]);
//         }

//         for (int j = 0; j < day.array_size; j++) {
//             printf("GainDot_Day[%d] = %f\n", j, day.GainDot[j]);
//         }

//         for (int j = 0; j < day.array_size; j++) {
//             printf("TimeDot_Day[%d] = %f\n", j, day.TimeDot[j]);
//         }

//         sleep(5);
//     }

//     // ****** 遍历 0-1755 变焦位置，演示变焦和 AE 切表 ******
//     // while (1) {

//     //   i += 100;
        
//     //   if(i > 1755) break;

//     //   XCamReturn ret = XCAM_RETURN_NO_ERROR;

//     //   printf("=== Set Zoom Position ===\n");
//     //   ret = rk_aiq_uapi_setOpZoomPosition(ctx, i); // 设置变焦
//     //   if (ret != XCAM_RETURN_NO_ERROR) {
//     //       printf("=== ret = %d ===\n", ret);
//     //   }

//     //   printf("sleep 2s\n");
//     //   sleep(2);   

//     //   printf("=== end Zoom Position ===\n");
//     //   rk_aiq_uapi_endOpZoomChange(ctx);             // 结束变焦

//     // //   printf("=== Set Focus Position ===\n");
//     // //   rk_aiq_uapi_oneshotFocus(ctx);                // 调用一次对焦

//     //   printf("sleep 10s\n");
//     //   sleep(8);  
      
//     //   // sleep(1);
//     // }

//     printf("=== Set Zoom Position ===\n");
//     // rk_aiq_uapi_setOpZoomPosition(ctx, 0); // 设置变焦位置为 500

//     return NULL;
// }

// /* -------- 后台线程主体 -------- */
// static void* ae_thread(void *arg)
// {
//     const rk_aiq_sys_ctx_t *ctx = (const rk_aiq_sys_ctx_t *)arg;
//     if (!ctx) return NULL;

//     int count = 0;

//     int pos = 0;

//     Uapi_LinAeRouteAttr_t linAeDayRouteAttr;
//     Uapi_LinAeRouteAttr_t linAeNightRouteAttr;
//     // int *p = pLinAeDayRouteAttr.Params.PIrisDot;
//     // int  n = pLinAeDayRouteAttr.Params.PIrisDot_len; 

//     printf("=== AE thread started ===\n");

//     while (1) {

//         printf("=== AE thread running %d ===\n", count);
        
//         count++;

//         rk_aiq_uapi_getOpZoomPosition(ctx, &pos); // 设置变焦

//         printf("=== Zoom pos = %d ===\n", pos);

//         rk_aiq_user_api_ae_getLinAeDayRouteAttr(ctx, &linAeDayRouteAttr);
//         rk_aiq_user_api_ae_getLinAeNightRouteAttr(ctx, &linAeNightRouteAttr);
        
//         printf("array_size_day = %d\n", linAeDayRouteAttr.array_size);
//         printf("array_size_night = %d\n", linAeNightRouteAttr.array_size);

        
//         for (int i = 0; i < linAeDayRouteAttr.array_size; i++) {
//             printf("PIrisGainDot_Day[%d] = %d\n", i, linAeDayRouteAttr.PIrisGainDot[i]);
//         }

//        for (int i = 0; i < linAeNightRouteAttr.array_size; i++) {
//             printf("PIrisGainDot_Night[%d] = %d\n", i, linAeNightRouteAttr.PIrisGainDot[i]);
//         }

//         if (count == 5) {
//             linAeDayRouteAttr.PIrisGainDot[0] = 32;
//             linAeDayRouteAttr.PIrisGainDot[1] = 32;
//             rk_aiq_user_api_ae_setLinAeDayRouteAttr(ctx, linAeDayRouteAttr);
//             printf("=== AE setLinAeDayRouteAttr done ===\n");

//             linAeNightRouteAttr.PIrisGainDot[0] = 8;
//             linAeNightRouteAttr.PIrisGainDot[1] = 8;
//             rk_aiq_user_api_ae_setLinAeNightRouteAttr(ctx, linAeNightRouteAttr);
//             printf("=== AE setLinAeNightRouteAttr done ===\n");
//         }
        
//         sleep(5);

//         printf("=== AE thread exit ===\n");
//     }


//     return NULL;
// }

// static void* ae_thread(void *arg)
// {
//     const rk_aiq_sys_ctx_t *ctx = (const rk_aiq_sys_ctx_t *)arg;
//     if (!ctx) return NULL;

//     int last_zoom = 0;
//     const ZoomPIrisRoute* last_tbl = NULL;
    
//     // Uapi_LinAeRouteAttr_t linAeNightRouteAttr;
//     Uapi_LinAeRouteAttr_t night;
//     Uapi_LinAeRouteAttr_t day;

//     printf("=== AE thread started ===\n");

//     while (1) {
//         int pos = 0;
//         rk_aiq_uapi_getOpZoomPosition(ctx, &pos);
        
//         // ****** 打印 Night Route 下的 PIris Gain Time 参数 ******
//         rk_aiq_user_api_ae_getLinAeNightRouteAttr(ctx, &night);
       
//         printf("Zoom Pos = %d\n", pos);
        
//         for (int i = 0; i < night.array_size; i++) {
//             printf("PIrisGainDot_Night[%d] = %d\n", i, night.PIrisGainDot[i]);
//         }

//         for (int i = 0; i < night.array_size; i++) {
//             printf("GainDot_Night[%d] = %f\n", i, night.GainDot[i]);
//         }

//         for (int i = 0; i < night.array_size; i++) {
//             printf("TimeDot_Night[%d] = %f\n", i, night.TimeDot[i]);
//         }
        
//         // if (rk_aiq_user_api_ae_getLinAeNightRouteAttr(ctx, &night) != XCAM_RETURN_NO_ERROR) {
//         //     usleep(100*1000);
//         //     continue;
//         // }

//         // ****** 打印 Day Route 下的 PIris Gain Time 参数 ******
//         rk_aiq_user_api_ae_getLinAeDayRouteAttr(ctx, &day);
        
//         for (int i = 0; i < day.array_size; i++) {
//             printf("PIrisGainDot_Day[%d] = %d\n", i, day.PIrisGainDot[i]);
//         }

//         for (int i = 0; i < day.array_size; i++) {
//             printf("GainDot_Day[%d] = %f\n", i, day.GainDot[i]);
//         }

//         for (int i = 0; i < day.array_size; i++) {
//             printf("TimeDot_Day[%d] = %f\n", i, day.TimeDot[i]);
//         }
    
//         // array_size 校验，和 PIRIS_DOT_LEN 对齐
//         int n = night.array_size;
//         // if (n > PIRIS_DOT_LEN) n = PIRIS_DOT_LEN;

//         // 选择对应表
//         const ZoomPIrisRoute* tbl = pick_piris_table_by_zoom(pos, last_zoom, last_tbl);

//         // 若表没变化则不下发，避免频繁 set
//         if (tbl != last_tbl) {
//             // ****** 写 Night Route ******
//             for (int i = 0; i < n; ++i) {
//                 night.PIrisGainDot[i] = tbl->piris[i];       // 若你的类型是 double，则写 (double)tbl[i]
//                 night.GainDot[i] = tbl->gain[i];
//                 night.TimeDot[i] = tbl->time[i];
//             }

//             // ****** 写 Day Route ******
//             for (int i = 0; i < n; ++i) {
//                 day.PIrisGainDot[i] = tbl->piris[i];       // 若你的类型是 double，则写 (double)tbl[i]
//                 day.GainDot[i] = tbl->gain[i];
//                 day.TimeDot[i] = tbl->time[i];
//             }

//             XCamReturn ret = rk_aiq_user_api_ae_setLinAeNightRouteAttr(ctx, night);
//             ret = rk_aiq_user_api_ae_setLinAeDayRouteAttr(ctx, day);
            
//             if (ret == XCAM_RETURN_NO_ERROR) {
//                 printf("[AE] zoom=%d ", pos);
//                 last_tbl = tbl;
//                 last_zoom = pos;
//             } else {
//                 printf("[AE] set NightRoute failed, ret=%d\n", ret);
//             }
//         }

//         usleep(1000 * 1000);  // 200ms，按需要调
//     }
    
//     return NULL;
// }

/* --------- 对外 API --------- */
int AF_Start(const rk_aiq_sys_ctx_t *ctx)
{
    if (!ctx) return -EINVAL;
    if (s_running) return 0;

    // 打开 AF 线程
    printf("=== AF_Start: ctx ==== \n");
    s_ctx = ctx;
    // if (pthread_create(&s_tid_af, NULL, af_thread, (void*)ctx) != 0)
        // return -errno;
    
    // pthread_detach(s_tid_af);    /* 不需要 join */

    // 启动 socket 监听
    start_af_service();
    pthread_t tid;
    pthread_create(&tid, NULL, command_thread, NULL);
    pthread_detach(tid);

    printf("=== AF thread created ==== \n");
    // // 打开 AE 线程
    // printf("=== AE_Start: ctx ==== \n");
    // if (pthread_create(&s_tid_ae, NULL, ae_thread, (void*)ctx) != 0)
    //     return -errno;

    // pthread_detach(s_tid_ae);    /* 不需要 join */

    s_running = true;
    return 0;
}


void AF_Stop(void)
{
    /* 最小版啥都不做；完整版可用 flag + pthread_join */
}

bool AF_IsRunning(void)
{
    return s_running;
}
