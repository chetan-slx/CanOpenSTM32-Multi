/*
 * CO_app_STM32.h
 *
 *  Created on: Aug 7, 2022
 *      Author: hamed
 */

#ifndef CANOPENSTM32_CO_APP_STM32_H_
#define CANOPENSTM32_CO_APP_STM32_H_

#include "CANopen.h"
#include "main.h"

#include "stdbool.h"

#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE)
#include "CO_storageBlank.h"  /* pulls in storage/CO_storage.h */
#endif

/* CANHandle : Pass in the CAN Handle to this function and it wil be used for all CAN Communications. It can be FDCan or CAN
 * and CANOpenSTM32 Driver will take of care of handling that
 * HWInitFunction : Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init
 * timerHandle : Pass in the timer that is going to be used for generating 1ms interrupt for tmrThread function,
 * please note that CANOpenSTM32 Library will override HAL_TIM_PeriodElapsedCallback function, if you also need this function
 * in your codes, please take required steps

 */

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	bool timerEn;
	uint32_t timerCount;
} CO_timer;


typedef struct {
    uint8_t
        desiredNodeID; /*This is the Node ID that you ask the CANOpen stack to assign to your device, although it might not always
	 * be the final NodeID, after calling canopen_app_init() you should check ActiveNodeID of CANopenNodeSTM32 structure for assigned Node ID.
	 */
    uint8_t activeNodeID; /* Assigned Node ID */
    uint16_t baudrate;     /* This is the baudrate you've set in your CubeMX Configuration */
    TIM_HandleTypeDef*
        timerHandle; /*Pass in the timer that is going to be used for generating 1ms interrupt for tmrThread function,
	 * please note that CANOpenSTM32 Library will override HAL_TIM_PeriodElapsedCallback function, if you also need this function in your codes, please take required steps
	 */

    /* Pass in the CAN Handle to this function and it wil be used for all CAN Communications. It can be FDCan or CAN
	 * and CANOpenSTM32 Driver will take of care of handling that*/
#ifdef CO_STM32_FDCAN_Driver
    FDCAN_HandleTypeDef* CANHandle;
#else
    CAN_HandleTypeDef* CANHandle;
#endif

    void (*HWInitFunction)(); /* Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init */

    /* ---- Per-instance OD pointers (set before calling canopen_app_init) - */
    OD_t*   od;                     /* per-instance OD pointer */
    void*   od_persist_comm;        /* opaque pointer — type differs per node */
    size_t  od_persist_comm_size;   /* sizeof(OD_PERSIST_COMM_t) for that node */

    CO_config_t co_config;  /**< Populated by OD_can_node_X_INIT_CONFIG() before canopen_app_init() */

    uint8_t outStatusLEDGreen; // This will be updated by the stack - Use them for the LED management
    uint8_t outStatusLEDRed;   // This will be updated by the stack - Use them for the LED management
    CO_t* canOpenStack;

    /* ---- Per-instance timing (private - managed by canopen_app_process) -- */
    uint32_t time_old;
    uint32_t time_current;

    /* ---- Per-instance storage objects (private) -------------------------- */
#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE)
    CO_storage_t       storage;
    CO_storage_entry_t storageEntries[1];
    uint32_t           storageInitError;
#endif

    CO_timer co_timer;

} CANopenNodeSTM32;


/**
 * Allocate CANopen stack objects and trigger first communication reset.
 * Call once per node at startup.
 */
int  canopen_app_init(CANopenNodeSTM32* canopenSTM32);

/**
 * Reset CAN peripheral and all CANopen communication objects.
 * Called internally by canopen_app_process on CO_RESET_COMM.
 * May also be called directly if a hard reset is needed.
 */
int  canopen_app_resetCommunication(CANopenNodeSTM32* canopenSTM32);

/**
 * Non-time-critical processing. Call from while(1) for every node.
 * Handles NMT state machine, SDO, emergency, and reset requests.
 */
void canopen_app_process(CANopenNodeSTM32* canopenSTM32);

/**
 * Time-critical 1 ms ISR handler. Call from HAL_TIM_PeriodElapsedCallback
 * for the timer associated with this node.
 * Processes SYNC, RPDO, TPDO.
 */
void canopen_app_interrupt(CANopenNodeSTM32* canopenSTM32);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CANOPENSTM32_CO_APP_STM32_H_ */
