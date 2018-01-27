
#include <registers.h>

 uint16_t R[REG_COUNT];  // Регистры для чтения/записи параметров
 
//--------------------------------------------------------------------------------------
// reading data from register

uint8_t read_reg(uint8_t index, uint8_t * low_byte, uint8_t * hi_byte){
	
	if (index < REG_COUNT) {
#ifdef OS_ENABLE
		portENTER_CRITICAL();
#endif
		*low_byte = WORD_BYTE1(R[index]); // low_byte
		*hi_byte = WORD_BYTE2(R[index]);	// hi byte
#ifdef OS_ENABLE
		portEXIT_CRITICAL();
#endif
		return REG_NOERR;
	} else {
		return REG_INDERR;
	}
}

//--------------------------------------------------------------------------------------
// write data to register

uint8_t write_reg(uint8_t index, uint8_t low_byte, uint8_t hi_byte){
	if (index >= REG_COUNT){
		return REG_INDERR;
	}
	if (index < REG_WR_START){
		return REG_ORERR;
	}
#ifdef OS_ENABLE
	portENTER_CRITICAL();
#endif
	R[index] = WORD(hi_byte, low_byte);
#ifdef OS_ENABLE
	portEXIT_CRITICAL();
#endif
	return REG_NOERR;
}

//--------------------------------------------------------------------------------------
// read N data to registers

uint8_t read_n_reg(uint8_t index, uint8_t *count, uint8_t * reg_bytes){
	
	uint8_t i = 0;
	if (index < REG_COUNT) {
#ifdef OS_ENABLE
		portENTER_CRITICAL();
#endif
		while ((i < *count*2) && ( index < REG_COUNT)) {
			reg_bytes[i+1] = WORD_BYTE1(R[index]);  // low_byte
			reg_bytes[i] = WORD_BYTE2(R[index]);		// hi byte
			index ++;
			i+=2;
		}
#ifdef OS_ENABLE
		portEXIT_CRITICAL();
#endif
		*count = i/2;
		
		return REG_NOERR;
	} else {
		return REG_INDERR;
	}
}

//--------------------------------------------------------------------------------------
// write N data to registers

uint8_t write_n_reg(uint8_t index, uint8_t count, uint8_t * reg_bytes){
	
	uint8_t i = 0;
	if (index >= REG_COUNT){
		return REG_INDERR;
	}
	if (index < REG_WR_START){
		return REG_ORERR;
	}
#ifdef OS_ENABLE
	portENTER_CRITICAL();
#endif
	while ((i < count*2) && ( index < REG_COUNT)) {
			R[index] = WORD(reg_bytes[i], reg_bytes[i+1]);
			index ++;
			i+=2;
		}
#ifdef OS_ENABLE
	portEXIT_CRITICAL();
#endif		
	return REG_NOERR;
}
