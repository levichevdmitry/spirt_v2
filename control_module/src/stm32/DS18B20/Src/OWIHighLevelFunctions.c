
#include "OWIHighLevelFunctions.h"
#include "OWIcrc.h"
#include "OWIdefs.h"
#include "onewireset.h"

// Буфер для приема/передачи по 1-wire
uint8_t ow_buf_rx[8];
uint8_t ow_buf_tx[8];

uint16_t OWTick;
uint16_t OWTimeout;
UART_HandleTypeDef * ow_huart;

#define OW_0		0x00
#define OW_1		0xff
#define OW_R_1	0xff

#define TIMEOUT	10   // ms

//-----------------------------------------------------------------------------
// функция преобразует один байт в восемь, для передачи через USART
// ow_byte - байт, который надо преобразовать
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}

// timeout counter

void OWI_Tick(void) {
	if (OWTick <= OWTimeout){
		OWTick ++;
	}
}

uint8_t OWI_TimeOut(void){
	return (OWTick >= OWTimeout);
}

//-----------------------------------------------------------------------------
// инициализирует USART и DMA
//-----------------------------------------------------------------------------
uint8_t OW_Init(UART_HandleTypeDef * huart) {
	
	if (huart == NULL) {
		return OW_ERROR;
	}
	
	ow_huart = 	huart;
 	ow_huart->Instance = OWI_USART;
  ow_huart->Init.BaudRate = 115200;
  ow_huart->Init.WordLength = UART_WORDLENGTH_8B;
  ow_huart->Init.StopBits = UART_STOPBITS_1;
  ow_huart->Init.Parity = UART_PARITY_NONE;
  ow_huart->Init.Mode = UART_MODE_TX_RX;
  ow_huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  ow_huart->Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(ow_huart);
	OWTimeout = TIMEOUT;
	OWTick = 0;	
	
	return OW_OK;
}

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
uint8_t OWI_DetectPresence(void) {
	uint8_t ow_presence;
	uint8_t cmd = 0xF0;
	
	ow_huart->Instance = OWI_USART;
	ow_huart->Init.BaudRate = 9600;
  ow_huart->Init.WordLength = UART_WORDLENGTH_8B;
  ow_huart->Init.StopBits = UART_STOPBITS_1;
  ow_huart->Init.Parity = UART_PARITY_NONE;
  ow_huart->Init.Mode = UART_MODE_TX_RX;
  ow_huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  ow_huart->Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(ow_huart);
	
	// отправляем 0xf0 на скорости 9600
	  
	//HAL_UART_Transmit_DMA(huart, &cmd, 1);
	//HAL_UART_Receive_DMA(huart, &ow_presence, 1);
	HAL_UART_Receive_IT(ow_huart, &ow_presence, 1);
	HAL_UART_Transmit_IT(ow_huart, &cmd, 1);
	OWTick = 0;
	while (HAL_UART_GetState(ow_huart) != HAL_UART_STATE_READY)
		{
			if (OWI_TimeOut()) break;
			#ifdef OW_GIVE_TICK_RTOS
			taskYIELD();
			#endif
		}
	//HAL_UART_DMAStop(huart);
	HAL_UART_Abort(ow_huart);
		
	ow_huart->Instance = OWI_USART;
	ow_huart->Init.BaudRate = 115200;
  ow_huart->Init.WordLength = UART_WORDLENGTH_8B;
  ow_huart->Init.StopBits = UART_STOPBITS_1;
  ow_huart->Init.Parity = UART_PARITY_NONE;
  ow_huart->Init.Mode = UART_MODE_TX_RX;
  ow_huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  ow_huart->Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(ow_huart);
	
	if (ow_presence != 0xf0) {
		return OW_OK;
	}

	return OW_NO_DEVICE;
}

unsigned char OWI_ReadBit(void)
{
	uint8_t cmd = 0xFF;
	uint8_t answer;
	
	//HAL_UART_Transmit_DMA(huart, &cmd, 1);
	//HAL_UART_Receive_DMA(huart, &answer, 1);
	HAL_UART_Receive_IT(ow_huart, &answer, 1);
	HAL_UART_Transmit_IT(ow_huart, &cmd, 1);
	OWTick = 0;
	while (HAL_UART_GetState(ow_huart) != HAL_UART_STATE_READY)
		{
			if (OWI_TimeOut()) break;
			#ifdef OW_GIVE_TICK_RTOS
			taskYIELD();
			#endif
		}
	//HAL_UART_DMAStop(huart);
	HAL_UART_Abort(ow_huart);
	if (answer != 0xFF) {
		answer = 0x00;
	}		
	return answer;
}


void OWI_WriteBit1(void)
{
	uint8_t cmd = 0xFF;
	
	//HAL_UART_Transmit_DMA(huart, &cmd, 1);
	//HAL_UART_Receive_DMA(huart, &cmd, 1);
	HAL_UART_Receive_IT(ow_huart, &cmd, 1);
	HAL_UART_Transmit_IT(ow_huart, &cmd, 1);
	OWTick = 0;
	while (HAL_UART_GetState(ow_huart) != HAL_UART_STATE_READY)
		{
			if (OWI_TimeOut()) break;
			#ifdef OW_GIVE_TICK_RTOS
			taskYIELD();
			#endif
		}
	//HAL_UART_DMAStop(huart);
	HAL_UART_Abort(ow_huart);
}

void OWI_WriteBit0(void)
{
	 uint8_t cmd = 0xF0;
	
	//HAL_UART_Transmit_DMA(huart, &cmd, 1);
	//HAL_UART_Receive_DMA(huart, &cmd, 1);
	HAL_UART_Receive_IT(ow_huart, &cmd, 1);
	HAL_UART_Transmit_IT(ow_huart, &cmd, 1);
	OWTick = 0;
	while (HAL_UART_GetState(ow_huart) != HAL_UART_STATE_READY)
		{
			if (OWI_TimeOut()) break;
			#ifdef OW_GIVE_TICK_RTOS
			taskYIELD();
			#endif
		}
	//HAL_UART_DMAStop(huart);
	HAL_UART_Abort(ow_huart);
}


void OWI_SendByte(unsigned char data)
{
	OW_toBits(data, ow_buf_tx);
	__HAL_UART_CLEAR_FLAG(ow_huart, UART_FLAG_RXNE | UART_FLAG_TC | UART_FLAG_TXE);
	
	//HAL_UART_Receive_DMA(huart, ow_buf_rx, sizeof(ow_buf_rx));
	//HAL_UART_Transmit_DMA(huart, ow_buf_tx, sizeof(ow_buf_tx));
	HAL_UART_Receive_IT(ow_huart, ow_buf_rx, sizeof(ow_buf_rx));
	HAL_UART_Transmit_IT(ow_huart, ow_buf_tx, sizeof(ow_buf_tx));
	OWTick = 0;
	while (HAL_UART_GetState(ow_huart) != HAL_UART_STATE_READY)
		{
			if (OWI_TimeOut()) break;
			#ifdef OW_GIVE_TICK_RTOS
			taskYIELD();
			#endif
		}
	//HAL_UART_DMAStop(huart);
	HAL_UART_Abort(ow_huart);
}

unsigned char OWI_ReceiveByte(void) {
	
	unsigned char data = 0xFF;
	OW_toBits(data, ow_buf_tx);
	__HAL_UART_CLEAR_FLAG(ow_huart, UART_FLAG_RXNE | UART_FLAG_TC | UART_FLAG_TXE);
	
	//HAL_UART_Receive_DMA(huart, ow_buf_rx, sizeof(ow_buf_rx));
	//HAL_UART_Transmit_DMA(huart, ow_buf_tx, sizeof(ow_buf_tx));
	HAL_UART_Receive_IT(ow_huart, ow_buf_rx, sizeof(ow_buf_rx));
	HAL_UART_Transmit_IT(ow_huart, ow_buf_tx, sizeof(ow_buf_tx));
	OWTick = 0;
	while (HAL_UART_GetState(ow_huart) != HAL_UART_STATE_READY)
		{
			if (OWI_TimeOut()) break;
			#ifdef OW_GIVE_TICK_RTOS
			taskYIELD();
			#endif
		}
	//HAL_UART_DMAStop(huart);
	HAL_UART_Abort(ow_huart);

	return OW_toByte(ow_buf_rx);
}


/*! \brief  Sends the SKIP ROM command to the 1-Wire bus(es).
 *
 *  \param  pins    A bitmask of the buses to send the SKIP ROM command to.
 */
void OWI_SkipRom(void)
{
    // Send the SKIP ROM command on the bus.
    OWI_SendByte(OWI_ROM_SKIP);
}


/*! \brief  Sends the READ ROM command and reads back the ROM id.
 *
 *  \param  romValue    A pointer where the id will be placed.
 *
 *  \param  pin     A bitmask of the bus to read from.
 */
void OWI_ReadRom(unsigned char * romValue)
{
    unsigned char bytesLeft = 8;

    // Send the READ ROM command on the bus.
    OWI_SendByte(OWI_ROM_READ);
    
    // Do 8 times.
    while (bytesLeft > 0)
    {
        // Place the received data in memory.
        *romValue++ = OWI_ReceiveByte();
        bytesLeft--;
    }
}


/*! \brief  Sends the MATCH ROM command and the ROM id to match against.
 *
 *  \param  romValue    A pointer to the ID to match against.
 *
 *  \param  pins    A bitmask of the buses to perform the MATCH ROM command on.
 */
void OWI_MatchRom(unsigned char * romValue)
{
    unsigned char bytesLeft = 8;   
    
    // Send the MATCH ROM command.
    OWI_SendByte(OWI_ROM_MATCH);

    // Do once for each byte.
    while (bytesLeft > 0)
    {
        // Transmit 1 byte of the ID to match.
        OWI_SendByte(*romValue++);
        bytesLeft--;
    }
}


/*! \brief  Sends the SEARCH ROM command and returns 1 id found on the 
 *          1-Wire(R) bus.
 *
 *  \param  bitPattern      A pointer to an 8 byte char array where the 
 *                          discovered identifier will be placed. When 
 *                          searching for several slaves, a copy of the 
 *                          last found identifier should be supplied in 
 *                          the array, or the search will fail.
 *
 *  \param  lastDeviation   The bit position where the algorithm made a 
 *                          choice the last time it was run. This argument 
 *                          should be 0 when a search is initiated. Supplying 
 *                          the return argument of this function when calling 
 *                          repeatedly will go through the complete slave 
 *                          search.
 *
 *  \param  pin             A bit-mask of the bus to perform a ROM search on.
 *
 *  \return The last bit position where there was a discrepancy between slave addresses the last time this function was run. Returns OWI_ROM_SEARCH_FAILED if an error was detected (e.g. a device was connected to the bus during the search), or OWI_ROM_SEARCH_FINISHED when there are no more devices to be discovered.
 *
 *  \note   See main.c for an example of how to utilize this function.
 */
unsigned char OWI_SearchRom(unsigned char * bitPattern, unsigned char lastDeviation)
{
    unsigned char currentBit = 1;
    unsigned char newDeviation = 0;
    unsigned char bitMask = 0x01;
    unsigned char bitA;
    unsigned char bitB;

    // Send SEARCH ROM command on the bus.
    OWI_SendByte(OWI_ROM_SEARCH);
    
    // Walk through all 64 bits.
    while (currentBit <= 64)
    {
        // Read bit from bus twice.
        bitA = OWI_ReadBit();
        bitB = OWI_ReadBit();

        if (bitA && bitB)
        {
            // Both bits 1 (Error).
            newDeviation = OWI_ROM_SEARCH_FAILED;
            return SEARCH_ERROR;
        }
        else if (bitA ^ bitB)
        {
            // Bits A and B are different. All devices have the same bit here.
            // Set the bit in bitPattern to this value.
            if (bitA)
            {
                (*bitPattern) |= bitMask;
            }
            else
            {
                (*bitPattern) &= ~bitMask;
            }
        }
        else // Both bits 0
        {
            // If this is where a choice was made the last time,
            // a '1' bit is selected this time.
            if (currentBit == lastDeviation)
            {
                (*bitPattern) |= bitMask;
            }
            // For the rest of the id, '0' bits are selected when
            // discrepancies occur.
            else if (currentBit > lastDeviation)
            {
                (*bitPattern) &= ~bitMask;
                newDeviation = currentBit;
            }
            // If current bit in bit pattern = 0, then this is
            // out new deviation.
            else if ( !(*bitPattern & bitMask)) 
            {
                newDeviation = currentBit;
            }
            // IF the bit is already 1, do nothing.
            else
            {
            
            }
        }
                
        
        // Send the selected bit to the bus.
        if ((*bitPattern) & bitMask)
        {
            OWI_WriteBit1();
        }
        else
        {
            OWI_WriteBit0();
        }

        // Increment current bit.    
        currentBit++;

        // Adjust bitMask and bitPattern pointer.    
        bitMask <<= 1;
        if (!bitMask)
        {
            bitMask = 0x01;
            bitPattern++;
        }
    }
    return newDeviation;
}

/*! \brief  Perform a 1-Wire search
 *
 *  This function shows how the OWI_SearchRom function can be used to 
 *  discover all slaves on the bus. It will also CRC check the 64 bit
 *  identifiers.
 *
 *  \param  devices Pointer to an array of type OWI_device. The discovered 
 *                  devices will be placed from the beginning of this array.
 *
 *  \param  len     The length of the device array. (Max. number of elements).
 *
 *  \param  buses   Bitmask of the buses to perform search on.
 *
 *  \retval SEARCH_SUCCESSFUL   Search completed successfully.
 *  \retval SEARCH_CRC_ERROR    A CRC error occured. Probably because of noise
 *                              during transmission.
 */
unsigned char OWI_SearchDevices(OWI_device * devices, unsigned char numDevices, unsigned char *num)
{
    unsigned char i, j;
    unsigned char * newID;
    unsigned char * currentID;
    unsigned char lastDeviation;
    unsigned char numFoundDevices;
    unsigned char flag = SEARCH_SUCCESSFUL;
    
    //?????????? ?????? 1Wire ?????????    
    for (i = 0; i < numDevices; i++)
    {
        for (j = 0; j < 8; j++)
        {
            devices[i].id[j] = 0x00;
        }
    }
    
    numFoundDevices = 0;
    newID = devices[0].id;
    lastDeviation = 0;
    currentID = newID;
    *num = 0;

    do  
    {
      memcpy(newID, currentID, 8);
      if (!OWI_DetectPresence()){
        return SEARCH_ERROR;        
      };
      lastDeviation = OWI_SearchRom(newID, lastDeviation);
      currentID = newID;
      numFoundDevices++;
      newID=devices[numFoundDevices].id;                
    } while(lastDeviation != OWI_ROM_SEARCH_FINISHED);            

    
    // Go through all the devices and do CRC check.
    for (i = 0; i < numFoundDevices; i++)
    {
        // If any id has a crc error, return error.
        if(OWI_CheckRomCRC(devices[i].id) != OWI_CRC_OK)
        {
            flag = SEARCH_CRC_ERROR;
        }
        else 
        {
           (*num)++;
        }
    }
    // Else, return Successful.
    return flag;
}

/*! \brief  Find the first device of a family based on the family id
 *
 *  This function returns a pointer to a device in the device array
 *  that matches the specified family.
 *
 *  \param  familyID    The 8 bit family ID to search for.
 *
 *  \param  devices     An array of devices to search through.
 *
 *  \param  size        The size of the array 'devices'
 *
 *  \return A pointer to a device of the family.
 *  \retval NULL    if no device of the family was found.
 */
unsigned char FindFamily(unsigned char familyID, OWI_device * devices, unsigned char numDevices, unsigned char lastNum)
{
    unsigned char i;
    
    if (lastNum == AT_FIRST){
      i = 0;
    }
    else{
      i = lastNum + 1;      
    }
      
    // Search through the array.
    while (i < numDevices)
    {
        // Return the pointer if there is a family id match.
        if ((*devices).id[0] == familyID)
        {
            return i;
        }
        devices++;
        i++;
    }
    return SEARCH_ERROR;
}


/*! \brief  Start temperature convert for all devicec on bus
 *
 *  This function 
 *
 *  \param  pin    Bitmask of the buses to perform search on.
 *
 */

void StartAllConvert_T(void)
{
    OWI_DetectPresence();
    OWI_SkipRom();
    OWI_SendByte(DS18B20_CONVERT_T);
}


float GetTemperatureSkipRom()
{
    unsigned int tmp = 0, sig = 0;
    float temperature;
    unsigned char scratchpad[9];
	
		// DEBUG-------------------------------------
		#if ONEWIRE_DEBUG == 1
		if (OWI_DetectPresence() == OW_OK){
			LogPrintI("1W","Init ok");
		} else {
			LogPrintI("1W", "Init fail");
		}
    #else
		OWI_DetectPresence();
		#endif
		
    OWI_SkipRom();
    OWI_SendByte(DS18B20_READ_SCRATCHPAD);
    scratchpad[0] = OWI_ReceiveByte();
    scratchpad[1] = OWI_ReceiveByte();
		
		// DEBUG-------------------------------------
		#if ONEWIRE_DEBUG == 1
		sprintf(debug_buf, "%2x%2x", scratchpad[0], scratchpad[1]);
		LogPrintI("1W", debug_buf);
		#endif
  
    if ((scratchpad[1]&128) != 0){
      tmp = ((unsigned int)scratchpad[1]<<8)|scratchpad[0];
      tmp = ~tmp + 1;
      scratchpad[0] = tmp;
      scratchpad[1] = tmp>>8;
      sig = 1;  
    } 
    temperature = (float)((scratchpad[0]>>4)|((scratchpad[1]&7)<<4));
    temperature += ((float)(scratchpad[0]&15) * 0.0625F);
    if (sig) {
        temperature *= -1.0;
    } 
    return temperature;     
}

float GetTemperatureMatchRom(unsigned char * romValue)
{
    unsigned int tmp = 0, sig = 0;
    float temperature;
    unsigned char scratchpad[9];
    
		// DEBUG-------------------------------------
		#if ONEWIRE_DEBUG == 1
		if (OWI_DetectPresence() == OW_OK){
			LogPrintI("1W","Init ok");
		} else {
			LogPrintI("1W", "Init fail");
		}
    #else
		OWI_DetectPresence();
		#endif
		
		OWI_MatchRom(romValue);
    OWI_SendByte(DS18B20_READ_SCRATCHPAD);
    scratchpad[0] = OWI_ReceiveByte();
    scratchpad[1] = OWI_ReceiveByte();
		
		// DEBUG-------------------------------------
		#if ONEWIRE_DEBUG == 1
		sprintf(debug_buf, "%2x%2x", scratchpad[0], scratchpad[1]);
		LogPrintI("1W", debug_buf);
		#endif
		//----------------------------------------------------
    if ((scratchpad[1]&128) != 0){
      tmp = ((unsigned int)scratchpad[1]<<8)|scratchpad[0];
      tmp = ~tmp + 1;
      scratchpad[0] = tmp;
      scratchpad[1] = tmp>>8;
      sig = 1;  
    }  
    temperature = (float)((scratchpad[0]>>4)|((scratchpad[1]&7)<<4));
    temperature += ((float)(scratchpad[0]&15) * 0.0625F);
    
    if (sig) {
        temperature *= -1.0;
    } 
    return temperature;
}


unsigned char InitSensor(unsigned char * romValue, signed char lowAlm, signed char hiAlm, unsigned char resolution)
{
     unsigned char scratchpad[9];
     unsigned char i;
     OWI_DetectPresence();
     OWI_MatchRom(romValue);
     OWI_SendByte(DS18B20_WRITE_SCRATCHPAD);
     OWI_SendByte(hiAlm);
     OWI_SendByte(lowAlm);
     OWI_SendByte(resolution);
     
     // check settings
     OWI_DetectPresence();
     OWI_MatchRom(romValue);
     OWI_SendByte(DS18B20_READ_SCRATCHPAD);
     for (i=0; i<8; i++) {
        scratchpad[i] = OWI_ReceiveByte();
     }
     
     if (scratchpad[2] == hiAlm && scratchpad[3] == lowAlm && scratchpad[4] == resolution) {
        return SET_SETTINGS_SUCCESSFUL;
     } else {   
        return SET_SETTINGS_ERROR;
     }
}
