/**
  ******************************************************************************
  * @file    Service/link_service.h
  * @brief   链路服务：帧分发 + 心跳监控 + 应答发送
  *
  * 本阶段（M0-M2.5）实现：
  *   - PING        -> PONG（含 uptime / fault / config）
  *   - SET_CONFIG  -> ACK（写 Flash）
  *   - GET_CONFIG  -> CONFIG
  *   - MOVE/SERVO/TTS/TOF_CFG/CLEAR_FAULT -> NAK(未实现)
  *   - 心跳超时监控
  *
  * 后续阶段（M3+）在 LinkSvc_Dispatch 里挂上真正的执行器。
  ******************************************************************************
  */
#ifndef __LINK_SERVICE_H
#define __LINK_SERVICE_H

#include <stdint.h>
#include "link_proto.h"

/* ---------- 初始化 ---------- */
void     LinkSvc_Init(void);

/* ---------- 主循环任务 ---------- */
void     LinkSvc_Poll(uint32_t now_ms);            /* 收字节 -> 解析 -> 分发 */
void     LinkSvc_HeartbeatTask(uint32_t now_ms);   /* 心跳超时 -> 置故障 */
void     LinkSvc_TaskMotion(void);                 /* 上报 MOTION_DONE / 热保护 */

/* ---------- 状态查询 ---------- */
uint8_t  LinkSvc_GetFault(void);
uint8_t  LinkSvc_IsCommLost(void);
uint8_t  LinkSvc_HasEverCommed(void);   /* 上电后是否收到过至少一帧有效帧 */
uint32_t LinkSvc_GetLastFrameMs(void);
uint32_t LinkSvc_GetGoodFrameCount(void);
uint32_t LinkSvc_GetBadFrameCount(void);

/* ---------- 双链路状态（A7Z = 0，蓝牙 = 1）---------- */
uint8_t  LinkSvc_GetOwner(void);              /* 当前控制权拥有者，0xFF = 无人 */
uint8_t  LinkSvc_IsPortAlive(uint8_t src);
uint8_t  LinkSvc_IsBtEnabled(void);           /* 蓝牙端口是否已初始化 */

/* ---------- 故障 ---------- */
void     LinkSvc_ClearFault(void);

/* ---------- 故障（供其它模块设置，按优先级合并）---------- */
void     LinkSvc_SetFault(uint8_t code);
void     LinkSvc_ClearFaultCode(uint8_t code);   /* 只清指定码，别的不动 */

/* ---------- 应答发送（供其它模块复用）---------- */
void     LinkSvc_SendAck(uint16_t seq, uint8_t status);
void     LinkSvc_SendNak(uint16_t seq, uint8_t status, uint8_t detail);
void     LinkSvc_SendFaultEvent(uint8_t code);

/* ---------- 主动事件（MOTION_DONE / EDGE / OBSTACLE / TOF_DATA）---------- */
void     LinkSvc_SendEvent(uint8_t op, const uint8_t *payload, uint8_t len);

/* ---------- 统一运动入口（TOF 自动回退也走这里，方向换算只有一处）---------- */
int      LinkSvc_RequestMove(uint8_t dir, uint16_t hz, int32_t steps);
uint8_t  LinkSvc_GetLastDir(void);

#endif /* __LINK_SERVICE_H */
