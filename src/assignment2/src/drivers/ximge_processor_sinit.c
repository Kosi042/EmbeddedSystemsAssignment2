// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "ximge_processor.h"

extern XImge_processor_Config XImge_processor_ConfigTable[];

XImge_processor_Config *XImge_processor_LookupConfig(u16 DeviceId) {
	XImge_processor_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XIMGE_PROCESSOR_NUM_INSTANCES; Index++) {
		if (XImge_processor_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XImge_processor_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XImge_processor_Initialize(XImge_processor *InstancePtr, u16 DeviceId) {
	XImge_processor_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XImge_processor_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XImge_processor_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

