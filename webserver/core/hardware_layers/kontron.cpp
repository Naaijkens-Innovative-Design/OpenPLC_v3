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

#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <poll.h>

#include "ladder.h"

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))
#endif

//#define I2C_DEBUG 1 //todo: remove for production
#define GEN_DEBUG 1

#define OK 0
#define ERROR -1

#define STACK_LEVELS 8

#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */

#define GEN_DEBUG 1
//#define I2C_DEBUG 1

#define LOG_ERROR	log_error(__LINE__)
#define LOG_INFO(msg)  log_info(__LINE__, msg)

int log_error(int line)
{
    char log_msg[1000];

#ifdef GEN_DEBUG
    sprintf(log_msg, "!line %d\n", line);
    log(log_msg);
#endif
    return ERROR;
}

int log_info(int line, char *msg)
{
    char log_msg[1000];

#ifdef GEN_DEBUG
    sprintf(log_msg, "line: %d, %s\n", line, msg);
    log(log_msg);
#endif
    return OK;
}

int i2cSetup(int addr)
{
    static int file = -1;
    char filename[32];
    char log_msg[1000];

    if (file < 0)
    {
        sprintf(filename, "/dev/i2c-0");

        if ( (file = open(filename, O_RDWR)) < 0)
        {
            sprintf(log_msg, "Failed to open the bus.\n");
            log(log_msg);
            return ERROR;
        } else {
            log_info("i2c bus initialized");
        }
    }
    if (ioctl(file, I2C_SLAVE, addr) < 0)
    {
        sprintf(log_msg, "Failed to acquire bus access and/or talk to slave.\n");
        log(log_msg);
        return ERROR;
    } else {
        log_info("i2c slave initialized");
    }
    return file;
}

int i2cMemRead(int dev, int add, uint8_t *buff, int size)
{
    uint8_t intBuff[I2C_SMBUS_BLOCK_MAX];
    unsigned char log_msg[1000];

    if (NULL == buff)
    {
        return ERROR;
    }

    if (size > I2C_SMBUS_BLOCK_MAX)
    {
        return ERROR;
    }

    intBuff[0] = 0xff & add;

    if (write(dev, intBuff, 1) != 1)
    {
#ifdef I2C_DEBUG
        sprintf(log_msg, "Fail to select 0x%02hhx mem add!\n", add);
        log(log_msg);
#endif
        return ERROR;
    }
    if (read(dev, buff, size) != size)
    {
#ifdef I2C_DEBUG
        sprintf(log_msg, "Fail to read memory!\n");
        log(log_msg);
#endif
        return ERROR;
    }
    return OK; //OK
}

int i2cMemWrite(int dev, int add, uint8_t *buff, int size)
{
    uint8_t intBuff[I2C_SMBUS_BLOCK_MAX];
    char log_msg[1000];

    if (NULL == buff)
    {
        return ERROR;
    }

    if (size > I2C_SMBUS_BLOCK_MAX - 1)
    {
        return ERROR;
    }

    intBuff[0] = 0xff & add;
    memcpy(&intBuff[1], buff, size);

    if (write(dev, intBuff, size + 1) != size + 1)
    {
        sprintf(log_msg, "Fail to write memory at 0x%02hhx address!\n", add);
        log(log_msg);
        return ERROR;
    }
    return OK;
}

//------------------------------------------------------------------------------
// Eight Relays 8-Layer Stackable HAT for Raspberry Pi
//------------------------------------------------------------------------------

#define RELAY8_HW_I2C_BASE_ADD 0x20
#define RELAY8_INPORT_REG_ADD	0x00
#define RELAY8_OUTPORT_REG_ADD	0x01
#define RELAY8_POLINV_REG_ADD	0x02
#define RELAY8_CFG_REG_ADD		0x03
#define RELAY8_REG_COUNT 0x04

#define RELAY8_SL_START 0
#define RELAY8_SL_COUNT 8

const uint8_t relay8MaskRemap[8] = {0x01, 0x04, 0x40, 0x10, 0x20, 0x80, 0x08,
    0x02};

uint8_t relay8ToIO(uint8_t relay)
{
    uint8_t i;
    uint8_t val = 0;
    for (i = 0; i < 8; i++)
    {
        if ( (relay & (1 << i)) != 0)
            val += relay8MaskRemap[i];
    }
    return val;
}

uint8_t IOToRelay8(uint8_t io)
{
    uint8_t i;
    uint8_t val = 0;
    for (i = 0; i < 8; i++)
    {
        if ( (io & relay8MaskRemap[i]) != 0)
        {
            val += 1 << i;
        }
    }
    return val;
}

int relay8CardCheck(int stack)
{
    uint8_t add = 0;

    if ( (stack < 0) || (stack > 7))
    {
        //printf("Invalid stack level [0..7]!");
        return LOG_ERROR;
    }
    add = (stack + RELAY8_HW_I2C_BASE_ADD) ^ 0x07;
    return i2cSetup(add);
}

int relay8Init(int stack)
{
    int dev = -1;
    uint8_t add = 0;
    uint8_t buff[2];

    dev = relay8CardCheck(stack);
    if (dev < 0)
    {
        return LOG_ERROR;
    }

    if (ERROR == i2cMemRead(dev, RELAY8_CFG_REG_ADD, buff, 1))
    {
        return ERROR;
    }
    if (OK == i2cMemRead(dev, RELAY8_REG_COUNT, buff, 1)) //16 bits I/O expander found
    {
        return ERROR;
    }
    if (buff[0] != 0) //non initialized I/O Expander
    {
        // make all I/O pins output
        buff[0] = 0;
        if (OK > i2cMemWrite(dev, RELAY8_CFG_REG_ADD, buff, 1))
        {
            return LOG_ERROR;
        }
        // put all pins in 0-logic state
        buff[0] = 0;
        if (OK > i2cMemWrite(dev, RELAY8_OUTPORT_REG_ADD, buff, 1))
        {
            return LOG_ERROR;
        }
    }
    //relay8_presence |= 1 << stack;
    return OK;
}

int relays8Set(uint8_t stack, uint8_t val)
{
    uint8_t buff[2];
    int dev = -1;
    static uint8_t relaysOldVal[STACK_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};

    if (stack >= STACK_LEVELS)
    {
        return LOG_ERROR;
    }

    if (relaysOldVal[stack] == val)
    {
        return OK;
    }
    dev = relay8CardCheck(stack);
    if (dev < 0)
    {
        return LOG_ERROR;
    }

    buff[0] = relay8ToIO(val);

    if (OK != i2cMemWrite(dev, RELAY8_OUTPORT_REG_ADD, buff, 1))
    {
        return LOG_ERROR;
    }
    relaysOldVal[stack] = val;
    return OK;
}

//------------------------------------------------------------------------------
// Sixteen Relays 8-Layer Stackable HAT for Raspberry Pi
//-----------------------------------------------------------------------------

#define RELAY16_CHANNELS 16
#define RELAY16_HW_I2C_BASE_ADD	0x20
#define RELAY16_INPORT_REG_ADD	0x00
#define RELAY16_OUTPORT_REG_ADD	0x02
#define RELAY16_POLINV_REG_ADD	0x04
#define RELAY16_CFG_REG_ADD		0x06

#define RELAY16_X_PLC_OFFSET 64
#define RELAY16_STACK_MIN 0
#define RELAY16_STACK_LEVELS 4

const uint16_t relayMaskRemap16[RELAY16_CHANNELS] = {0x8000, 0x4000, 0x2000,
    0x1000, 0x800, 0x400, 0x200, 0x100, 0x80, 0x40, 0x20, 0x10, 0x8, 0x4, 0x2,
    0x1};

uint16_t relayToIO16(uint16_t relay)
{
    uint8_t i;
    uint16_t val = 0;
    for (i = 0; i < 16; i++)
    {
        if ( (relay & (1 << i)) != 0)
            val += relayMaskRemap16[i];
    }
    return val;
}

uint16_t IOToRelay16(uint16_t io)
{
    uint8_t i;
    uint16_t val = 0;
    for (i = 0; i < 16; i++)
    {
        if ( (io & relayMaskRemap16[i]) != 0)
        {
            val += 1 << i;
        }
    }
    return val;
}

int relay16CardCheck(uint8_t stack)
{
    uint8_t add = 0;

    if ( (stack < 0) || (stack > 7))
    {
        //printf("Invalid stack level [0..7]!");
        return LOG_ERROR;
    }
    add = (stack + RELAY16_HW_I2C_BASE_ADD) ^ 0x07;
    return i2cSetup(add);
}

int relay16Init(int stack)
{
    int dev = -1;
    uint8_t add = 0;
    uint8_t buff[2];
    uint16_t val = 0;

    dev = relay16CardCheck(stack);
    if (dev < 0)
    {
        return LOG_ERROR;
    }

    if (ERROR == i2cMemRead(dev, RELAY16_CFG_REG_ADD, buff, 2)) // 16 bits IO expander found
    {
        return ERROR;
    }
    memcpy(&val, buff, 2);
    if (val != 0) //non initialized I/O Expander
    {
        // make all I/O pins output
        val = 0;
        memcpy(buff, &val, 2);
        if (OK > i2cMemWrite(dev, RELAY16_CFG_REG_ADD, buff, 2))
        {
            return LOG_ERROR;
        }
        // put all pins in 0-logic state
        if (OK > i2cMemWrite(dev, RELAY16_OUTPORT_REG_ADD, buff, 2))
        {
            return LOG_ERROR;
        }
    }
    return OK;
}

int relay16Set(uint8_t stack, uint16_t val)
{
    static uint16_t relaysOldVal[STACK_LEVELS] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t buff[2];
    uint16_t rVal = 0;
    int dev = -1;

    if (stack >= STACK_LEVELS)
    {
        return LOG_ERROR;
    }
    if (relaysOldVal[stack] == val)
    {
        return OK;
    }
    dev = relay16CardCheck(stack);
    if (dev < 0)
    {
        return LOG_ERROR;
    }
    rVal = relayToIO16(val);
    memcpy(buff, &rVal, 2);

    if (OK == i2cMemWrite(dev, RELAY16_OUTPORT_REG_ADD, buff, 2))
    {
        relaysOldVal[stack] = val;
        return OK;
    }
    return LOG_ERROR;
}


//-----------------------------------------------------------------------------
// Eight High Voltage Digital Inputs 8-Layer Stackable HAT for Raspberry Pi
//-----------------------------------------------------------------------------
#define DIG_IN8_CHANNELS 8
#define DIG_IN8_HW_I2C_BASE_ADD	0x20
#define DIG_IN8_INPORT_REG_ADD	0x00
#define DIG_IN8_OUTPORT_REG_ADD	0x01
#define DIG_IN8_POLINV_REG_ADD	0x02
#define DIG_IN8_CFG_REG_ADD		0x03

const uint8_t inputsMaskRemap8[DIG_IN8_CHANNELS] ={0x08, 0x04, 0x02, 0x01, 0x10, 0x20, 0x40, 0x80};

int digIn8CardCheck(uint8_t stack)
{
    uint8_t add = 0;

    if ( (stack < 0) || (stack > 7))
    {
        //printf("Invalid stack level [0..7]!");
        return LOG_ERROR;
    }
    add = (stack + DIG_IN8_HW_I2C_BASE_ADD) ^ 0x07;
    return i2cSetup(add);
}

int digIn8Init(int stack)
{
    int dev = -1;
    uint8_t add = 0;
    uint8_t buff[2];
    uint8_t val = 0;

    dev = digIn8CardCheck(stack);
    if (dev < 0)
    {
        return LOG_ERROR;
    }

    if (ERROR == i2cMemRead(dev, DIG_IN8_CFG_REG_ADD, buff, 1))
    {
        return ERROR;
    }
    
    if (buff[0] != 0xff) //non initialized I/O Expander
    {
        // make all I/O pins inputs
        buff[0] = 0xff;
        if (OK != i2cMemWrite(dev, RELAY8_CFG_REG_ADD, buff, 1))
        {
            return LOG_ERROR;
        }
    }
    return OK;
}

int digIn8Get(uint8_t stack, uint8_t *val)
{
    int dev = -1;
    uint8_t buff[2];
    uint8_t raw = 0;
    uint8_t i = 0;

    if (stack >= STACK_LEVELS || val == NULL)
    {
        return LOG_ERROR;
    }
    dev = digIn8CardCheck(stack);
    if (dev < 0)
    {
        return LOG_ERROR;
    }

    if (OK != i2cMemRead(dev, DIG_IN8_INPORT_REG_ADD, buff, 1))
    {
        return LOG_ERROR;
    }
    raw = 0xff & (~buff[0]);
    *val = 0;
    for (i = 0; i < 8; i++)
    {
        if (raw & inputsMaskRemap8[i])
        {
            *val += 1 << i;
        }
    }
    return OK;
}


//-----------------------------------------------------------------------------
// This function is called by the main OpenPLC routine when it is initializing.
// Hardware initialization procedures should be here.
//-----------------------------------------------------------------------------
void initializeHardware()
{
}

//-----------------------------------------------------------------------------
// This function is called by the main OpenPLC routine when it is finalizing.
// Resource clearing procedures should be here.
//-----------------------------------------------------------------------------
void finalizeHardware()
{
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

    /*********READING AND WRITING TO I/O**************
    *bool_input[0][0] = read_digital_input(0);
    write_digital_output(0, *bool_output[0][0]);
    *int_input[0] = read_analog_input(0);
    write_analog_output(0, *int_output[0]);
    **************************************************/

    pthread_mutex_unlock(&bufferLock); //unlock mutex
}
