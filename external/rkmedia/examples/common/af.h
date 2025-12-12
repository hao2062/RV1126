/* examples/common/af.h */
#ifndef _RK_AF_H_
#define _RK_AF_H_

#include <stdbool.h>
#include "rk_aiq_user_api_sysctl.h"   /* 用到 rk_aiq_sys_ctx_t */

#ifdef __cplusplus
extern "C" {
#endif

/* 启动 AF 后台线程；ctx 由外部提供。返回 0=OK，<0=错误码 */
int  AF_Start(const rk_aiq_sys_ctx_t *ctx);

/* 请求线程退出并回收资源（最小版本里其实啥也没做） */
void AF_Stop(void);

/* 查询线程是否在跑（可选） */
bool AF_IsRunning(void);

#ifdef __cplusplus
}
#endif
#endif /* _RK_AF_H_ */
