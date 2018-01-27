#ifndef _OWI_UTILS_H_
#define _OWI_UTILS_H_


#define     OWI_CRC_OK      0x00    //!< CRC check succeded
#define     OWI_CRC_ERROR   0x01    //!< CRC check failed


unsigned char OWI_ComputeCRC8(unsigned char inData, unsigned char seed);
unsigned int OWI_ComputeCRC16(unsigned char inData, unsigned int seed);
unsigned char OWI_CheckRomCRC(unsigned char * romValue);


#endif
