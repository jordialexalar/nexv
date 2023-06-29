/*
 * i2c_peripherals.c
 *
 *  Created on: 18 dic. 2018
 *      Author: josepmaria.fernandez
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : i2c_peripherals.c
  * @brief          : i2c program for memories and temp sensor
  ******************************************************************************
  ** This code will activate the readings of the temperature and humidity sensors.
  * Also the EEPROM is managed.
  *
  */

#include "main.h"
#include "i2c_peripherals.h"
#include "can_power_member.h"
#include "specific_delta.h"

/* Private variables */
unsigned char buffer[5];
unsigned int rawT, rawH;
/* Public variables */
float Temperature = -100.0;
float Humidity = 0.0;
unsigned long timerTempAcquisition = 0;
unsigned int eraseFramRequest = 0;



void initI2CtempSensor(I2C_HandleTypeDef *hi2c)
{
  HAL_Delay(15);
  buffer[0]=0x02; //Pointer buffer
  buffer[1]=0; //MSB byte
  buffer[2]=0; //LSB byte
  HAL_I2C_Master_Transmit(hi2c,0x40<<1,buffer,3,100);
}

void stateMachineTempReading(I2C_HandleTypeDef *hi2c)
{
  static I2cState i2cState = SENSOR_INIT;
  switch(i2cState)
  {
  case SENSOR_INIT:
    initI2CtempSensor(hi2c);
    timerTempAcquisition = 0;
    i2cState = SENSOR_SLEEP;
    break;
  case TEMP_READ_REQUEST:
    buffer[0]=0x00;
    HAL_I2C_Master_Transmit(hi2c,0x40<<1,buffer,1,100);
    i2cState = TEMP_READ;
    timerTempAcquisition = 0;
    break;
  case TEMP_READ:
    if (timerTempAcquisition > WAIT_20MS)
    {
      buffer[0]=0x00;
      HAL_I2C_Master_Receive(hi2c,0x40<<1,buffer,2,100);
      rawT = buffer[0]<<8 | buffer[1]; //combine 2 8-bit into 1 16bit
      Temperature = ((float)rawT/65536)*165.0 -40.0 - 7.0; //-7 is the calibration
      i2cState = HUMIDITY_READ_REQUEST;
      timerTempAcquisition = 0;
    }
    break;
  case HUMIDITY_READ_REQUEST:
    if (timerTempAcquisition > WAIT_20MS)
    {
      buffer[0]=0x00;
      HAL_I2C_Master_Transmit(hi2c,0x40<<1,buffer,1,100);
      i2cState = HUMIDITY_READ;
      timerTempAcquisition = 0;
    }
    break;
  case HUMIDITY_READ:
    if (timerTempAcquisition > WAIT_20MS)
    {
      HAL_I2C_Master_Receive(hi2c,0x40<<1,buffer,2,100);
      //buffer[0] : MSB data
      //buffer[1] : LSB data
      rawH = buffer[0]<<8 | buffer[1]; //combine 2 8-bit into 1 16bit
      Humidity = ((float)rawH/65536)*100.0;
      i2cState = SENSOR_SLEEP;
      timerTempAcquisition = 0;
    }
    break;
  case SENSOR_SLEEP:
    if (timerTempAcquisition > WAIT_30S)
    {
      //printf("The current temperature is %i �C \n", (int)Temperature);
      //printf("The current humidity is %i HR \n", (int)Humidity);
      timerTempAcquisition = 0;
      i2cState = TEMP_READ_REQUEST;

    }
    break;
  default:
    i2cState = SENSOR_SLEEP;
    timerTempAcquisition = 0;
    break;
  }
}

HAL_StatusTypeDef writeFram(I2C_HandleTypeDef *hi2c, uint16_t adress, uint8_t *pData, uint16_t Size)
{
  if (timerLedFRAM > COMS_LEDS_TIMER)
  {
    timerLedFRAM = 0;
    if (onPowerSaving) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
    else HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
  }
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
  return (HAL_I2C_Mem_Write(hi2c, 0x50<<1, adress, I2C_MEMADD_SIZE_16BIT, pData, Size, 200));
  //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
}

HAL_StatusTypeDef readFram(I2C_HandleTypeDef *hi2c, uint16_t adress, uint8_t *pData, uint16_t Size)
{
  if (timerLedFRAM > COMS_LEDS_TIMER)
  {
    timerLedFRAM = 0;
    if (onPowerSaving) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
    else HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
  }
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET); //Write protection
  return(HAL_I2C_Mem_Read(hi2c, 0x50<<1, adress, I2C_MEMADD_SIZE_16BIT, pData, Size, 200));
  //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
}

#include "ModbusDataMap.h"

/* Do not touch these declarations:*/
uint8_t memDataW[6];
uint8_t memDataR[6];

/* In case of increasing the maximum number of registers, change FRAM_MAX_REGISTERS accordingly */
memEntryStruct tableOfContents[FRAM_MAX_REGISTERS];
char framFreePositions[FRAM_MAX_REGISTERS];

void constructFramRegs (void)
{

  int i = 0;
  int j = 0;
  int h = 0;
  int modAddress[] = {DELTA25_ENERGIA_HIGH_1, DELTA25_ENERGIA_LOW_1, DELTA25_POWER_UP_CYCLES_1, DELTA25_RESERVADO0_1,DELTA25_RESERVADO1_1};

  while (i < (2 * MAXIMUM_NUMBER_OF_CONVERTERS * 5)){

	  tableOfContents[i].ModbusAdress = modAddress[j]+ (h * DELTA_ADDRESSES_PER_CONVERTER);
	  tableOfContents[i].DefaultValue = 0;
	  tableOfContents[i].SaveMask = FRAM_EVERY_MINUTE_STO | FRAM_ON_POWER_FAULT_STO;
	  tableOfContents[i].ElementFound = 0;
	  tableOfContents[i].FramAddress = 0; //Incremental one by one
	  tableOfContents[i].LastAccessedEpoch = 0;
	  tableOfContents[i].LastGetValue = 0;
	  j++;
	  if (j == 5){
		  j = 0;
		  h++;
	  }

	  i++;
  }
  
  tableOfContents[70].ModbusAdress = XCDV_FAMILY_AE_TOTAL_SOCKET_A_HIGH;
  tableOfContents[70].DefaultValue = 0;
  tableOfContents[70].SaveMask = FRAM_EVERY_MINUTE_STO | FRAM_ON_POWER_FAULT_STO;
  tableOfContents[70].ElementFound = 0;
  tableOfContents[70].FramAddress = 0; //Incremental one by one
  tableOfContents[70].LastAccessedEpoch = 0;
  tableOfContents[70].LastGetValue = 0;

  tableOfContents[71].ModbusAdress = XCDV_FAMILY_AE_TOTAL_SOCKET_A_LOW;
  tableOfContents[71].DefaultValue = 0;
  tableOfContents[71].SaveMask = FRAM_EVERY_MINUTE_STO | FRAM_ON_POWER_FAULT_STO;
  tableOfContents[71].ElementFound = 0;
  tableOfContents[71].FramAddress = 0;
  tableOfContents[71].LastAccessedEpoch = 0;
  tableOfContents[71].LastGetValue = 0;

  tableOfContents[72].ModbusAdress = XCDV_FAMILY_AE_TOTAL_SOCKET_B_HIGH;
  tableOfContents[72].DefaultValue = 0;
  tableOfContents[72].SaveMask = FRAM_EVERY_MINUTE_STO | FRAM_ON_POWER_FAULT_STO;
  tableOfContents[72].ElementFound = 0;
  tableOfContents[72].FramAddress = 0; //Incremental one by one
  tableOfContents[72].LastAccessedEpoch = 0;
  tableOfContents[72].LastGetValue = 0;

  tableOfContents[73].ModbusAdress = XCDV_FAMILY_AE_TOTAL_SOCKET_B_LOW;
  tableOfContents[73].DefaultValue = 0;
  tableOfContents[73].SaveMask = FRAM_EVERY_MINUTE_STO | FRAM_ON_POWER_FAULT_STO;
  tableOfContents[73].ElementFound = 0;
  tableOfContents[73].FramAddress = 0;
  tableOfContents[73].LastAccessedEpoch = 0;
  tableOfContents[73].LastGetValue = 0;

  tableOfContents[74].ModbusAdress = XCDV_FAMILY_AE_TOTAL_SOCKET_INIT;
  tableOfContents[74].DefaultValue = 0;
  tableOfContents[74].SaveMask = FRAM_EVERY_MINUTE_STO | FRAM_ON_POWER_FAULT_STO;
  tableOfContents[74].ElementFound = 0;
  tableOfContents[74].FramAddress = 0;
  tableOfContents[74].LastAccessedEpoch = 0;
  tableOfContents[74].LastGetValue = 0;


  for (i = 0; i < FRAM_MAX_REGISTERS; i++)
  {
    framFreePositions[i] = 0;
  }
}

unsigned long timerLedFRAM = 50;

void framDataSM(void)
{
  static unsigned int framSM = 0;
  unsigned int i, j;
  static unsigned int framAddressPointer = 0;
  uint16_t crcCalculated = 0, crcRead = 0;
  uint16_t tempModbusAddr = 0;
  uint16_t objectCurrentValueInRam = 0;
  char addrFound = 0;

  //Clear LED
  if (timerLedFRAM > (COMS_LEDS_TIMER<<1))
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
  }

  /* FRAM entry organization:
   * MODBUS ADDRESS || DATA || CRC
   * */

  switch (framSM)
  {
    case 0:
      //Read all the FRAM and put values into RAM
      if (framAddressPointer < FRAM_MAX_REGISTERS)
      {
        readFram(&hi2c1,framAddressPointer*6,(uint8_t*)memDataR,6);
        crcCalculated = generateCrcEeprom(memDataR, 4,1); //Calculate the CRC
        crcRead = (((uint16_t)memDataR[4])<<8) | memDataR[5]; //Read the FRAM CRC
        framAddressPointer++;
        tempModbusAddr = (((uint16_t)memDataR[0])<<8) | memDataR[1];
        for (i = 0; i < FRAM_MAX_REGISTERS; i++) //Look for the read Modbus address into the Modbus dictionary
        {
          if ( tableOfContents[i].ModbusAdress == tempModbusAddr )
          {
            addrFound = 1;
          }
        }
        if (crcCalculated != crcRead || addrFound == 0) //If the address is not found or the CRC is invalid
        {
          framFreePositions[framAddressPointer-1] = 1; //Clear that address
          return; //Not valid, memory is clear
        }

        //If value is valid, load it into the Modbus dictionary
        for (i = 0; i < FRAM_MAX_REGISTERS; i++)
        {
          if ( tableOfContents[i].ModbusAdress == tempModbusAddr )
          {
            tableOfContents[i].ElementFound = 1;
            tableOfContents[i].FramAddress = framAddressPointer-1;
            tableOfContents[i].LastAccessedEpoch = systemEpoch;
            tableOfContents[i].LastGetValue = ( ( (uint16_t)memDataR[2] ) <<8 ) | memDataR[3];
            objectDictionary[tempModbusAddr] = tableOfContents[i].LastGetValue;
          }
        }
      }
      else
      {
        framSM++; //Next state
        printf("Fram read!\n");
      }
      break;
    case 1:
      //Is there any value in Modbus that do not appear in RAM structure?
      for (i = 0; i < FRAM_MAX_REGISTERS; i++)
      {
        if ( tableOfContents[i].ElementFound == 0 ) //If there is an element not found...
        {
          for (j = 0; j < FRAM_MAX_REGISTERS; j++) //Look for a free position
          {
            if (framFreePositions[j] == 1) //If the position is free, save it!
            {
              framFreePositions[j] = 0;
              memDataW[0] = tableOfContents[i].ModbusAdress >> 8;
              memDataW[1] = tableOfContents[i].ModbusAdress & 0x00FF;
              memDataW[2] = tableOfContents[i].DefaultValue >> 8;
              memDataW[3] = tableOfContents[i].DefaultValue & 0x00FF;
              crcCalculated = generateCrcEeprom(memDataW, 4,1);
              memDataW[4] = crcCalculated >>8;
              memDataW[5] = crcCalculated & 0x00FF;
              if (writeFram(&hi2c1,j*6,(uint8_t*)memDataW,6) == HAL_OK)
              {
                tableOfContents[i].ElementFound = 1;
                tableOfContents[i].FramAddress = j;
                tableOfContents[i].LastAccessedEpoch = systemEpoch;
                tableOfContents[i].LastGetValue = tableOfContents[i].DefaultValue;
                j = FRAM_MAX_REGISTERS; //Stop searching for free position
              }
            }
          }
        }
      }
      framSM++; //Next state
      framAddressPointer = 0;
      break;
    case 2:
      if (framAddressPointer < FRAM_MAX_REGISTERS)
      {
        objectCurrentValueInRam = objectDictionary[tableOfContents[framAddressPointer].ModbusAdress];
        if ( systemEpoch > (tableOfContents[framAddressPointer].LastAccessedEpoch + 10) ) //Only one write every 10s is allowed per register
        {

          if ((tableOfContents[framAddressPointer].SaveMask & FRAM_EVERY_MINUTE_STO) == FRAM_EVERY_MINUTE_STO &&
              systemEpoch > (tableOfContents[framAddressPointer].LastAccessedEpoch + 60)) //60 seconds later...
          {
            if (tableOfContents[framAddressPointer].LastGetValue != objectCurrentValueInRam)
            {
              memDataW[0] = tableOfContents[framAddressPointer].ModbusAdress >> 8;
              memDataW[1] = tableOfContents[framAddressPointer].ModbusAdress & 0x00FF;
              memDataW[2] = objectCurrentValueInRam >> 8;
              memDataW[3] = objectCurrentValueInRam & 0x00FF;
              crcCalculated = generateCrcEeprom(memDataW, 4,1);
              memDataW[4] = crcCalculated >>8;
              memDataW[5] = crcCalculated & 0x00FF;
              if (writeFram(&hi2c1,tableOfContents[framAddressPointer].FramAddress*6,(uint8_t*)memDataW,6) == HAL_OK)
              {
                tableOfContents[framAddressPointer].LastAccessedEpoch = systemEpoch;
                tableOfContents[framAddressPointer].LastGetValue = objectCurrentValueInRam;
                printf("Writing Fram (every minute)\n");
              }
            }
          }

          if ((tableOfContents[framAddressPointer].SaveMask & FRAM_ON_CHANGE_STO) == FRAM_ON_CHANGE_STO &&
              tableOfContents[framAddressPointer].LastGetValue != objectCurrentValueInRam)
          {
            memDataW[0] = tableOfContents[framAddressPointer].ModbusAdress >> 8;
            memDataW[1] = tableOfContents[framAddressPointer].ModbusAdress & 0x00FF;
            memDataW[2] = objectCurrentValueInRam >> 8;
            memDataW[3] = objectCurrentValueInRam & 0x00FF;
            crcCalculated = generateCrcEeprom(memDataW, 4,1);
            memDataW[4] = crcCalculated >>8;
            memDataW[5] = crcCalculated & 0x00FF;
            if (writeFram(&hi2c1,tableOfContents[framAddressPointer].FramAddress*6,(uint8_t*)memDataW,6) == HAL_OK)
            {
              tableOfContents[framAddressPointer].LastAccessedEpoch = systemEpoch;
              tableOfContents[framAddressPointer].LastGetValue = objectCurrentValueInRam;
              printf("Writing Fram (on change)\n");
            }
          }

          if ((tableOfContents[framAddressPointer].SaveMask & FRAM_ON_POWER_FAULT_STO) == FRAM_ON_POWER_FAULT_STO &&
             onPowerSaving) //60 seconds later...
          {
            if (tableOfContents[framAddressPointer].LastGetValue != objectCurrentValueInRam)
            {
              memDataW[0] = tableOfContents[framAddressPointer].ModbusAdress >> 8;
              memDataW[1] = tableOfContents[framAddressPointer].ModbusAdress & 0x00FF;
              memDataW[2] = objectCurrentValueInRam >> 8;
              memDataW[3] = objectCurrentValueInRam & 0x00FF;
              crcCalculated = generateCrcEeprom(memDataW, 4,1);
              memDataW[4] = crcCalculated >>8;
              memDataW[5] = crcCalculated & 0x00FF;
              if (writeFram(&hi2c1,tableOfContents[framAddressPointer].FramAddress*6,(uint8_t*)memDataW,6) == HAL_OK)
              {
                 tableOfContents[framAddressPointer].LastAccessedEpoch = systemEpoch;
                 tableOfContents[framAddressPointer].LastGetValue = objectCurrentValueInRam;
                 printf("Writing Fram (on power fault)\n");
              }
            }
          }
        }
        framAddressPointer++;
      }
      else
      {
        if (objectDictionary[SERVICE_CLEAN_FRAM] == 0xFFAA)
        {
          framSM = 3;
          printf("Clearing Fram!\n");
        }
        framAddressPointer = 0;
      }
      break;
    case 3:
      //Clear all
      objectDictionary[SERVICE_CLEAN_FRAM] = 0;
      if (framAddressPointer < FRAM_MAX_REGISTERS)
      {
        memDataW[0] = 1;
        memDataW[1] = 2;
        memDataW[2] = 3;
        memDataW[3] = 4;
        memDataW[4] = 5;
        memDataW[5] = 6;
        writeFram(&hi2c1,framAddressPointer*6,(uint8_t*)memDataW,6);
        objectDictionary[ tableOfContents[framAddressPointer].ModbusAdress ] = tableOfContents[framAddressPointer].DefaultValue;
        tableOfContents[framAddressPointer].LastGetValue = tableOfContents[framAddressPointer].DefaultValue;
        tableOfContents[framAddressPointer].LastAccessedEpoch = systemEpoch;
        framAddressPointer++;
      }
      else
      {
        printf("Fram clear!\n");
        constructFramRegs();
        framSM = 0;
        framAddressPointer = 0;
      }
      break;
  }
}

uint16_t generateCrcEeprom(uint8_t * buf, int len, char swap)
{
    uint16_t crc = 0xFFFF;
    uint16_t temp = 0;
    int pos = 0;
    int i = 0;

    for (pos = 0; pos < len; pos++) {
        // XOR byte into least sig. byte of crc
        crc ^= (unsigned int) buf[pos];

        // Loop over each bit
        for (i = 8; i != 0; i--) {

            // If the LSB is set
            if ((crc & 0x0001) != 0) {
                crc >>= 1;      // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else
                crc >>= 1;      // Just shift right
        }
    }

    // Swype bytes
    if(swap == 1) {
        temp = crc;
        crc = (crc & 0xFF00) >> 8;
        crc = ( (temp & 0x00FF) << 8 ) | crc;
    }

    return crc;
}


