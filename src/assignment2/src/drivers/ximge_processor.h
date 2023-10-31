// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XIMGE_PROCESSOR_H
#define XIMGE_PROCESSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "ximge_processor_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Axi_cpu_BaseAddress;
} XImge_processor_Config;
#endif

typedef struct {
    u64 Axi_cpu_BaseAddress;
    u32 IsReady;
} XImge_processor;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XImge_processor_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XImge_processor_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XImge_processor_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XImge_processor_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XImge_processor_Initialize(XImge_processor *InstancePtr, u16 DeviceId);
XImge_processor_Config* XImge_processor_LookupConfig(u16 DeviceId);
int XImge_processor_CfgInitialize(XImge_processor *InstancePtr, XImge_processor_Config *ConfigPtr);
#else
int XImge_processor_Initialize(XImge_processor *InstancePtr, const char* InstanceName);
int XImge_processor_Release(XImge_processor *InstancePtr);
#endif

void XImge_processor_Start(XImge_processor *InstancePtr);
u32 XImge_processor_IsDone(XImge_processor *InstancePtr);
u32 XImge_processor_IsIdle(XImge_processor *InstancePtr);
u32 XImge_processor_IsReady(XImge_processor *InstancePtr);
void XImge_processor_EnableAutoRestart(XImge_processor *InstancePtr);
void XImge_processor_DisableAutoRestart(XImge_processor *InstancePtr);

u32 XImge_processor_Get_in_r_BaseAddress(XImge_processor *InstancePtr);
u32 XImge_processor_Get_in_r_HighAddress(XImge_processor *InstancePtr);
u32 XImge_processor_Get_in_r_TotalBytes(XImge_processor *InstancePtr);
u32 XImge_processor_Get_in_r_BitWidth(XImge_processor *InstancePtr);
u32 XImge_processor_Get_in_r_Depth(XImge_processor *InstancePtr);
u32 XImge_processor_Write_in_r_Words(XImge_processor *InstancePtr, int offset, word_type *data, int length);
u32 XImge_processor_Read_in_r_Words(XImge_processor *InstancePtr, int offset, word_type *data, int length);
u32 XImge_processor_Write_in_r_Bytes(XImge_processor *InstancePtr, int offset, char *data, int length);
u32 XImge_processor_Read_in_r_Bytes(XImge_processor *InstancePtr, int offset, char *data, int length);
u32 XImge_processor_Get_out_r_BaseAddress(XImge_processor *InstancePtr);
u32 XImge_processor_Get_out_r_HighAddress(XImge_processor *InstancePtr);
u32 XImge_processor_Get_out_r_TotalBytes(XImge_processor *InstancePtr);
u32 XImge_processor_Get_out_r_BitWidth(XImge_processor *InstancePtr);
u32 XImge_processor_Get_out_r_Depth(XImge_processor *InstancePtr);
u32 XImge_processor_Write_out_r_Words(XImge_processor *InstancePtr, int offset, word_type *data, int length);
u32 XImge_processor_Read_out_r_Words(XImge_processor *InstancePtr, int offset, word_type *data, int length);
u32 XImge_processor_Write_out_r_Bytes(XImge_processor *InstancePtr, int offset, char *data, int length);
u32 XImge_processor_Read_out_r_Bytes(XImge_processor *InstancePtr, int offset, char *data, int length);

void XImge_processor_InterruptGlobalEnable(XImge_processor *InstancePtr);
void XImge_processor_InterruptGlobalDisable(XImge_processor *InstancePtr);
void XImge_processor_InterruptEnable(XImge_processor *InstancePtr, u32 Mask);
void XImge_processor_InterruptDisable(XImge_processor *InstancePtr, u32 Mask);
void XImge_processor_InterruptClear(XImge_processor *InstancePtr, u32 Mask);
u32 XImge_processor_InterruptGetEnabled(XImge_processor *InstancePtr);
u32 XImge_processor_InterruptGetStatus(XImge_processor *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
