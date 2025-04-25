//-----------------------------------------------------------------------------
// Copyright 2015 Thiago Alves
//
// Based on the LDmicro software by Jonathan Westhues
// This file is part of the OpenPLC Software Stack.
//
// OpenPLC is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// OpenPLC is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with OpenPLC.  If not, see <http://www.gnu.org/licenses/>.
//------
//
// This file is the hardware layer for the OpenPLC. If you change the platform
// where it is running, you may only need to change this file. All the I/O
// related stuff is here. Basically it provides functions to read and write
// to the OpenPLC internal buffers in order to update I/O state.
// Thiago Alves, Dec 2015
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "ladder.h"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

static int i2c_bus;
static char log_msg[1000];

#define MMC_NR_IO_MODULES 5

enum mmc_io_dir {
    OUT,
    IN,
};

struct mmc_io_component {
    mmc_io_dir dir;
    uint8_t bank_idx;
    uint8_t bit_idx;
};

struct mcc_io_mapping {
    uint8_t addr;
    uint8_t size;
    struct mmc_io_component io[8];
};

/* WARNING:
 * variable arrays bool_input, bool_output etc... are NOT initialized when initializeHardware is called
 * and during the first run of updateBuffersIn and updateBuffersOut,
 * so we can't use compile time pointers!!!!
 */

struct mcc_io_mapping mmc_io_modules[5] = {
    {   /* DO-12[0-3] */
        .addr = 0x20,
        .size = 8,
        {
            { .dir = IN, .bank_idx = 0, .bit_idx = 3 },
            { .dir = IN, .bank_idx = 0, .bit_idx = 2 },
            { .dir = IN, .bank_idx = 0, .bit_idx = 1 },
            { .dir = IN, .bank_idx = 0, .bit_idx = 0 },
            { .dir = OUT, .bank_idx = 0, .bit_idx = 3 },
            { .dir = OUT, .bank_idx = 0, .bit_idx = 2 },
            { .dir = OUT, .bank_idx = 0, .bit_idx = 1 },
            { .dir = OUT, .bank_idx = 0, .bit_idx = 0 },
        },
    },
    {   /* DO-12[4-7] */
        .addr = 0x21,
        .size = 8,
        {
            { .dir = IN, .bank_idx = 0, .bit_idx = 7 },
            { .dir = IN, .bank_idx = 0, .bit_idx = 6 },
            { .dir = IN, .bank_idx = 0, .bit_idx = 5 },
            { .dir = IN, .bank_idx = 0, .bit_idx = 4 },
            { .dir = OUT, .bank_idx = 0, .bit_idx = 7 },
            { .dir = OUT, .bank_idx = 0, .bit_idx = 6 },
            { .dir = OUT, .bank_idx = 0, .bit_idx = 5 },
            { .dir = OUT, .bank_idx = 0, .bit_idx = 4 },
        },
    },
    {   /* DO-12[8-11] */
        .addr = 0x22,
        .size = 8,
        {
            { .dir = IN, .bank_idx = 1, .bit_idx = 3 },
            { .dir = IN, .bank_idx = 1, .bit_idx = 2 },
            { .dir = IN, .bank_idx = 1, .bit_idx = 1 },
            { .dir = IN, .bank_idx = 1, .bit_idx = 0 },
            { .dir = OUT, .bank_idx = 1, .bit_idx = 3 },
            { .dir = OUT, .bank_idx = 1, .bit_idx = 2 },
            { .dir = OUT, .bank_idx = 1, .bit_idx = 1 },
            { .dir = OUT, .bank_idx = 1, .bit_idx = 0 },
        },
    },
    {   /* DI-12[0-5] */
        .addr = 0x23,
        .size = 7,
        {
            { .dir = IN, .bank_idx = 2, .bit_idx = 0 },
            { .dir = IN, .bank_idx = 2, .bit_idx = 1 },
            { .dir = IN, .bank_idx = 2, .bit_idx = 2 },
            { .dir = IN, .bank_idx = 2, .bit_idx = 3 },
            { .dir = IN, .bank_idx = 2, .bit_idx = 4 },
            { .dir = IN, .bank_idx = 2, .bit_idx = 5 },
            { .dir = IN, .bank_idx = 2, .bit_idx = 6 },
        },
    },
    {   /* DI-12[6-11] */
        .addr = 0x24,
        .size = 7,
        {
            { .dir = IN, .bank_idx = 3, .bit_idx = 0 },
            { .dir = IN, .bank_idx = 3, .bit_idx = 1 },
            { .dir = IN, .bank_idx = 3, .bit_idx = 2 },
            { .dir = IN, .bank_idx = 3, .bit_idx = 3 },
            { .dir = IN, .bank_idx = 3, .bit_idx = 4 },
            { .dir = IN, .bank_idx = 3, .bit_idx = 5 },
            { .dir = IN, .bank_idx = 3, .bit_idx = 6 },
        },
    },
};

//-----------------------------------------------------------------------------
// This function is called by the main OpenPLC routine when it is initializing.
// Hardware initialization procedures should be here.
//-----------------------------------------------------------------------------
void initializeHardware()
{
    char *bus = "/dev/i2c-0";
    uint8_t config;

    // Get I2C bus
    if((i2c_bus = open(bus, O_RDWR)) < 0)
    {
        sprintf(log_msg, "error opening i2c bus\n");
        log(log_msg);
        i2c_bus = 0;
        return;
    } else {
        sprintf(log_msg, "i2c-0 bus initalized\n");
        log(log_msg);
    }

    for(int i = 0; i < MMC_NR_IO_MODULES; i++) {
        // Get I2C device, PCF8574 I2C first  address is 0x20(32)
        if(ioctl(i2c_bus, I2C_SLAVE, mmc_io_modules[i].addr) < 0) {
            sprintf(log_msg, "error opening i2c bus device: %d, addr: %x\n", i, mmc_io_modules[i].addr);
            log(log_msg);
            return;
        } else {
            sprintf(log_msg, "i2c-0 device: %d found at addr: %x\n", i, mmc_io_modules[i].addr);
            log(log_msg);

            // Init inputs
            config = 0;
            for(int s = 0; s < mmc_io_modules[i].size; s++) {
                if(mmc_io_modules[i].io[s].dir == IN) {
                    config |= (1 << s);
                }
            }
            if(write(i2c_bus, &config, 1) < 0) {
                sprintf(log_msg, "error writing i2c device: %d\n", i);
                log(log_msg);
            } else {
                sprintf(log_msg, "configured i2 device: %d, config: %x\n", i, config);
                log(log_msg);
            }
        }
    }
}

//-----------------------------------------------------------------------------
// This function is called by the main OpenPLC routine when it is finalizing.
// Resource clearing procedures should be here.
//-----------------------------------------------------------------------------
void finalizeHardware()
{
    if(i2c_bus != 0) {
        close(i2c_bus);
    }
}

//-----------------------------------------------------------------------------
// This function is called by the OpenPLC in a loop. Here the internal buffers
// must be updated to reflect the actual Input state. The mutex bufferLock
// must be used to protect access to the buffers on a threaded environment.
//-----------------------------------------------------------------------------
void updateBuffersIn()
{
    uint8_t data; 

    pthread_mutex_lock(&bufferLock); //lock mutex
    /*********READING AND WRITING TO I/O**************/
    for(int i = 0; i < MMC_NR_IO_MODULES; i++) {
        if(ioctl(i2c_bus, I2C_SLAVE, mmc_io_modules[i].addr) < 0) {
            /* asume that higher addressed device are not available */
            goto out;
        } else {
            if(read(i2c_bus, &data, 1) < 0) {
                //sprintf(log_msg, "error reading i2c device: %x\n", mmc_io_modules[i].addr);
                //log(log_msg);
                continue;
            }
            for(int s = 0; s < mmc_io_modules[i].size; s++) {
                if((mmc_io_modules[i].io[s].dir == IN) && (bool_input[mmc_io_modules[i].io[s].bank_idx][mmc_io_modules[i].io[s].bit_idx] != NULL)) {
                    *bool_input[mmc_io_modules[i].io[s].bank_idx][mmc_io_modules[i].io[s].bit_idx] = (data >> s) & 1;
                }
            }
        }
    }
    /**************************************************/
out:
    pthread_mutex_unlock(&bufferLock); //unlock mutex
}

//-----------------------------------------------------------------------------
// This function is called by the OpenPLC in a loop. Here the internal buffers
// must be updated to reflect the actual Output state. The mutex bufferLock
// must be used to protect access to the buffers on a threaded environment.
//-----------------------------------------------------------------------------
void updateBuffersOut()
{
    uint8_t data;
    pthread_mutex_lock(&bufferLock); //lock mutex
    /*********READING AND WRITING TO I/O**************/

    for(int i = 0; i < MMC_NR_IO_MODULES; i++) {
        if(ioctl(i2c_bus, I2C_SLAVE, mmc_io_modules[i].addr) < 0) {
            /* asume that higher addressed device are not available */
            goto out;
        } else {            
            data = 0;
            for(int s = 0; s < mmc_io_modules[i].size; s++) {
                if((mmc_io_modules[i].io[s].dir == OUT) && (bool_output[mmc_io_modules[i].io[s].bank_idx][mmc_io_modules[i].io[s].bit_idx] != NULL)) {
                    data  |= ((*bool_output[mmc_io_modules[i].io[s].bank_idx][mmc_io_modules[i].io[s].bit_idx] & 1) << s);
		            //sprintf(log_msg, "dbg: bank: %d, bit: %d, data: %x, ptr_addr: %x\n", mmc_io_modules[i].io[s].bank_idx, mmc_io_modules[i].io[s].bit_idx, *bool_output[mmc_io_modules[i].io[s].bank_idx][mmc_io_modules[i].io[s].bit_idx], (uint32_t) bool_output[mmc_io_modules[i].io[s].bank_idx][mmc_io_modules[i].io[s].bit_idx]);
                    //log(log_msg);
                } else {
                    //sprintf(log_msg, "dbg: bank: %d, bit: %d, ptr_addr: %x\n", mmc_io_modules[i].io[s].bank_idx, mmc_io_modules[i].io[s].bit_idx, (uint32_t) bool_output[mmc_io_modules[i].io[s].bank_idx][mmc_io_modules[i].io[s].bit_idx]);
                    //log(log_msg);
                }
            }
            if(write(i2c_bus, &data, 1) < 0) {
                //sprintf(log_msg, "error writing i2c device: %x\n", mmc_io_modules[i].addr);
                //log(log_msg);
                continue;
            }
        }
    }

    /*

    *bool_input[0][0] = read_digital_input(0);

    *int_input[0] = read_analog_input(0);
    write_analog_output(0, *int_output[0]);

    **************************************************/

out:
    pthread_mutex_unlock(&bufferLock); //unlock mutex
}

