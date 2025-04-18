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

static int pcf8574;
static char log_msg[1000];


//-----------------------------------------------------------------------------
// This function is called by the main OpenPLC routine when it is initializing.
// Hardware initialization procedures should be here.
//-----------------------------------------------------------------------------
void initializeHardware()
{
    char *bus = "/dev/i2c-0";

    if((pcf8574 = open(bus, O_RDWR)) < 0)
    {
        sprintf(log_msg, "error opening i2c bus\n");
        log(log_msg);
        return;
    } else {
        sprintf(log_msg, "i2c-0 bus initalized\n");
        log(log_msg);
    }

    // Get I2C device, PCF8574 I2C address is 0x20(32)
    if(ioctl(pcf8574, I2C_SLAVE, 0x20) < 0) {
        sprintf(log_msg, "error opening i2c bus\n");
        log(log_msg);
        return;
    } else {
        sprintf(log_msg, "i2c-0 device found\n");
        log(log_msg);
    }
}

//-----------------------------------------------------------------------------
// This function is called by the main OpenPLC routine when it is finalizing.
// Resource clearing procedures should be here.
//-----------------------------------------------------------------------------
void finalizeHardware()
{
    if(pcf8574 != NULL) {
        close(pcf8574);
    }
}

//-----------------------------------------------------------------------------
// This function is called by the OpenPLC in a loop. Here the internal buffers
// must be updated to reflect the actual Input state. The mutex bufferLock
// must be used to protect access to the buffers on a threaded environment.
//-----------------------------------------------------------------------------
void updateBuffersIn()
{
    pthread_mutex_lock(&bufferLock); //lock mutex

    /*********READING AND WRITING TO I/O**************

    *bool_input[0][0] = read_digital_input(0);
    write_digital_output(0, *bool_output[0][0]);

    *int_input[0] = read_analog_input(0);
    write_analog_output(0, *int_output[0]);

    **************************************************/

    pthread_mutex_unlock(&bufferLock); //unlock mutex
}

//-----------------------------------------------------------------------------
// This function is called by the OpenPLC in a loop. Here the internal buffers
// must be updated to reflect the actual Output state. The mutex bufferLock
// must be used to protect access to the buffers on a threaded environment.
//-----------------------------------------------------------------------------
void updateBuffersOut()
{
    pthread_mutex_lock(&bufferLock); //lock mutex

    char data = *bool_output[0][0];

    if(pcf8574 != NULL) {
        if(write(pcf8574, data, 1) < 0) {
            sprintf(log_msg, "error writing i2c device\n");
        }
    }


    /*********READING AND WRITING TO I/O**************

    *bool_input[0][0] = read_digital_input(0);

    *int_input[0] = read_analog_input(0);
    write_analog_output(0, *int_output[0]);

    **************************************************/

    pthread_mutex_unlock(&bufferLock); //unlock mutex
}

