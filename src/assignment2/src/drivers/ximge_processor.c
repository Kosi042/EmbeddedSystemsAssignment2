// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "ximge_processor.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XImge_processor_CfgInitialize(XImge_processor *InstancePtr, XImge_processor_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Axi_cpu_BaseAddress = ConfigPtr->Axi_cpu_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XImge_processor_Start(XImge_processor *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImge_processor_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_AP_CTRL) & 0x80;
    XImge_processor_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_AP_CTRL, Data | 0x01);
}

u32 XImge_processor_IsDone(XImge_processor *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImge_processor_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XImge_processor_IsIdle(XImge_processor *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImge_processor_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XImge_processor_IsReady(XImge_processor *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImge_processor_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XImge_processor_EnableAutoRestart(XImge_processor *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImge_processor_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_AP_CTRL, 0x80);
}

void XImge_processor_DisableAutoRestart(XImge_processor *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImge_processor_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_AP_CTRL, 0);
}

u32 XImge_processor_Get_in_r_BaseAddress(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE);
}

u32 XImge_processor_Get_in_r_HighAddress(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_HIGH);
}

u32 XImge_processor_Get_in_r_TotalBytes(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE + 1);
}

u32 XImge_processor_Get_in_r_BitWidth(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XIMGE_PROCESSOR_AXI_CPU_WIDTH_IN_R;
}

u32 XImge_processor_Get_in_r_Depth(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XIMGE_PROCESSOR_AXI_CPU_DEPTH_IN_R;
}

u32 XImge_processor_Write_in_r_Words(XImge_processor *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XImge_processor_Read_in_r_Words(XImge_processor *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE + (offset + i)*4);
    }
    return length;
}

u32 XImge_processor_Write_in_r_Bytes(XImge_processor *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XImge_processor_Read_in_r_Bytes(XImge_processor *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_IN_R_BASE + offset + i);
    }
    return length;
}

u32 XImge_processor_Get_out_r_BaseAddress(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE);
}

u32 XImge_processor_Get_out_r_HighAddress(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_HIGH);
}

u32 XImge_processor_Get_out_r_TotalBytes(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE + 1);
}

u32 XImge_processor_Get_out_r_BitWidth(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XIMGE_PROCESSOR_AXI_CPU_WIDTH_OUT_R;
}

u32 XImge_processor_Get_out_r_Depth(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XIMGE_PROCESSOR_AXI_CPU_DEPTH_OUT_R;
}

u32 XImge_processor_Write_out_r_Words(XImge_processor *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XImge_processor_Read_out_r_Words(XImge_processor *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE + (offset + i)*4);
    }
    return length;
}

u32 XImge_processor_Write_out_r_Bytes(XImge_processor *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XImge_processor_Read_out_r_Bytes(XImge_processor *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_HIGH - XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Axi_cpu_BaseAddress + XIMGE_PROCESSOR_AXI_CPU_ADDR_OUT_R_BASE + offset + i);
    }
    return length;
}

void XImge_processor_InterruptGlobalEnable(XImge_processor *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImge_processor_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_GIE, 1);
}

void XImge_processor_InterruptGlobalDisable(XImge_processor *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImge_processor_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_GIE, 0);
}

void XImge_processor_InterruptEnable(XImge_processor *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XImge_processor_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_IER);
    XImge_processor_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_IER, Register | Mask);
}

void XImge_processor_InterruptDisable(XImge_processor *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XImge_processor_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_IER);
    XImge_processor_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_IER, Register & (~Mask));
}

void XImge_processor_InterruptClear(XImge_processor *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImge_processor_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_ISR, Mask);
}

u32 XImge_processor_InterruptGetEnabled(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XImge_processor_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_IER);
}

u32 XImge_processor_InterruptGetStatus(XImge_processor *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XImge_processor_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XIMGE_PROCESSOR_AXI_CPU_ADDR_ISR);
}

