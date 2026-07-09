/*
 * CANopen main program file.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        main_generic.c
 * @author      Hamed Jafarzadeh 	2022
 * 				Janez Paternoster	2021
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "CO_app_STM32.h"
#include "CANopen.h"
#include "main.h"
#include <stdio.h>
#include <inttypes.h>

#include "CO_storageBlank.h"

/* Printf function of CanOpen app */
#define log_printf(macropar_message, ...) printf(macropar_message, ##__VA_ARGS__)

/* default values for CO_CANopenInit() */
#define NMT_CONTROL                                                                                                    \
    CO_NMT_STARTUP_TO_OPERATIONAL                                                                                      \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME        500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK        false
#define OD_STATUS_BITS       NULL

#define CO_TIMER_THR         10


/* ===========================================================================
 * canopen_app_init
 * Allocates CO_t objects and triggers a communication reset.
 * Safe to call for node1 and node2 independently.
 * =========================================================================*/
/* This function will basically setup the CANopen node */
int
canopen_app_init(CANopenNodeSTM32* canopenSTM32) {

	CO_t* CO = NULL;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    /* Initialise storage entry to point at THIS node's persist-comm block.
     * Cannot be a static initializer because od_persist_comm is runtime.   */
	canopenSTM32->storageEntries[0].addr = canopenSTM32->od_persist_comm;
	canopenSTM32->storageEntries[0].len  = canopenSTM32->od_persist_comm_size;
    canopenSTM32->storageEntries[0].subIndexOD = 2;
    canopenSTM32->storageEntries[0].attr       = CO_storage_cmd | CO_storage_restore;
    canopenSTM32->storageEntries[0].addrNV     = NULL;
    uint8_t storageEntriesCount = 1U;
    canopenSTM32->storageInitError = 0U;
#endif

    /* Allocate memory for this node's CANopen objects */
    CO_config_t* config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    /* co_config was populated by OD_can_node_X_INIT_CONFIG() in main.c
       before canopen_app_init() was called. Pass it directly. */
    config_ptr = &canopenSTM32->co_config;
#endif /* CO_MULTIPLE_OD */

    uint32_t heapMemoryUsed;
    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf("Error: Can't allocate memory\n");
        return 1;
    } else {
        log_printf("Allocated %" PRIu32 " bytes for CANopen objects\n", heapMemoryUsed);
    }

    canopenSTM32->canOpenStack = CO;

#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE)
    CO_ReturnError_t storErr = CO_storageBlank_init(
        &canopenSTM32->storage,
        CO->CANmodule,
        OD_find(canopenSTM32->od, 0x1010),  /* OD_ENTRY_H1010 - per instance */
        OD_find(canopenSTM32->od, 0x1011),  /* OD_ENTRY_H1011 - per instance */
        canopenSTM32->storageEntries,
        storageEntriesCount,
        &canopenSTM32->storageInitError);

    if (storErr != CO_ERROR_NO && storErr != CO_ERROR_DATA_CORRUPT) {
        log_printf("Error: Storage %d\n", canopenSTM32->storageInitError);
        return 2;
    }
#endif

    return canopen_app_resetCommunication(canopenSTM32);
}

/* ===========================================================================
 * canopen_app_resetCommunication
 * Reinitialises CAN hardware and all CANopen protocol objects for one node.
 * =========================================================================*/
int
canopen_app_resetCommunication(CANopenNodeSTM32* canopenSTM32) {
    log_printf("CANopenNode - Reset communication...\n");

    CO_t* CO = canopenSTM32->canOpenStack;

    CO->CANmodule->CANnormal = false;

    CO_CANsetConfigurationMode((void*)canopenSTM32);
    CO_CANmodule_disable(CO->CANmodule);

    CO_ReturnError_t err;

    err = CO_CANinit(CO, canopenSTM32, 0);
    if (err != CO_ERROR_NO) {
        log_printf("Error: CAN initialization failed: %d\n", err);
        return 1;
    }

    /* LSS address sourced from THIS node's OD persist-comm */
    CO_LSS_address_t lssAddress = { 0 };
    {
        OD_entry_t* h1018 = OD_find(canopenSTM32->od, 0x1018);
        if (h1018 != NULL) {
            OD_get_u32(h1018, 1, &lssAddress.identity.vendorID,       true);
            OD_get_u32(h1018, 2, &lssAddress.identity.productCode,    true);
            OD_get_u32(h1018, 3, &lssAddress.identity.revisionNumber, true);
            OD_get_u32(h1018, 4, &lssAddress.identity.serialNumber,   true);
        }
    }
    err = CO_LSSinit(CO, &lssAddress, &canopenSTM32->desiredNodeID, &canopenSTM32->baudrate);
    if (err != CO_ERROR_NO) {
        log_printf("Error: LSS slave initialization failed: %d\n", err);
        return 2;
    }

    canopenSTM32->activeNodeID = canopenSTM32->desiredNodeID;
    uint32_t errInfo = 0;

    /* Use THIS node's OD pointer - not the global OD */
    err = CO_CANopenInit(CO,
                         NULL,
                         NULL,
                         canopenSTM32->od,       /* <-- per-instance OD */
                         OD_STATUS_BITS,
                         NMT_CONTROL,
                         FIRST_HB_TIME,
                         SDO_SRV_TIMEOUT_TIME,
                         SDO_CLI_TIMEOUT_TIME,
                         SDO_CLI_BLOCK,
                         canopenSTM32->activeNodeID,
                         &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        if (err == CO_ERROR_OD_PARAMETERS) {
            log_printf("Error: Object Dictionary entry 0x%" PRIx32 "\n", errInfo);
        } else {
            log_printf("Error: CANopen initialization failed: %d\n", err);
        }
        return 3;
    }

    err = CO_CANopenInitPDO(CO, CO->em, canopenSTM32->od, canopenSTM32->activeNodeID, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        if (err == CO_ERROR_OD_PARAMETERS) {
            log_printf("Error: Object Dictionary entry 0x%" PRIx32 "\n", errInfo);
        } else {
            log_printf("Error: PDO initialization failed: %d\n", err);
        }
        return 4;
    }

    HAL_TIM_Base_Start_IT(canopenSTM32->timerHandle);

    if (!CO->nodeIdUnconfigured) {
#if ((CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE)
        if (canopenSTM32->storageInitError != 0U) {
            CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE,
                           canopenSTM32->storageInitError);
        }
#endif
    } else {
        log_printf("CANopenNode - Node-id not initialized\n");
    }

    CO_CANsetNormalMode(CO->CANmodule);

    log_printf("CANopenNode - Running...\n");
    fflush(stdout);

    canopenSTM32->time_old = canopenSTM32->time_current = HAL_GetTick();
    return 0;
}

/* ===========================================================================
 * canopen_app_process
 * Call from while(1) for every active node.
 * =========================================================================*/
void
canopen_app_process(CANopenNodeSTM32* canopenSTM32) {
    CO_t* CO = canopenSTM32->canOpenStack;

    canopenSTM32->time_current = HAL_GetTick();

    if ((canopenSTM32->time_current - canopenSTM32->time_old) > 0U) {
        uint32_t timeDifference_us = (canopenSTM32->time_current - canopenSTM32->time_old) * 1000U;
        canopenSTM32->time_old = canopenSTM32->time_current;

        CO_NMT_reset_cmd_t reset_status = CO_process(CO, false, timeDifference_us, NULL);

        canopenSTM32->outStatusLEDRed   = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
        canopenSTM32->outStatusLEDGreen = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

        if (reset_status == CO_RESET_COMM) {
            HAL_TIM_Base_Stop_IT(canopenSTM32->timerHandle);
            CO_CANsetConfigurationMode((void*)canopenSTM32);
            CO_delete(CO);
            log_printf("CANopenNode Reset Communication request\n");
            canopen_app_init(canopenSTM32);   /* re-init THIS instance only */
        } else if (reset_status == CO_RESET_APP) {
            log_printf("CANopenNode Device Reset\n");
            HAL_NVIC_SystemReset();
        }
    }
}

/* Thread function executes in constant intervals, this function can be called from FreeRTOS tasks or Timers ********/
void
canopen_app_interrupt(CANopenNodeSTM32* canopenSTM32) {
    CO_t* CO = canopenSTM32->canOpenStack;

    CO_LOCK_OD(CO->CANmodule);
    if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
        bool_t syncWas         = false;
        uint32_t timeDifference_us = 1000U; /* 1 ms */

#if ((CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE)
        syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE)
        CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if ((CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE)
        CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif
    }
    CO_UNLOCK_OD(CO->CANmodule);
}
