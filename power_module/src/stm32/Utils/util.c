
#include <util.h>
// Создает кольцевой буфер с количеством элементов capacity
CircleBuffer * CB_Create(uint16_t capacity){
	CircleBuffer * tmp;
	tmp = malloc(sizeof(CircleBuffer));
	tmp->capacity = capacity;	 
	tmp->buf = malloc(sizeof(int16_t)*capacity);
	tmp->head = 0;
	tmp->tail = 0;
	tmp->size = 0;
	return tmp;
}
// Возвращает количество элеменотов находящихся в буфере
uint16_t CB_GetSize(CircleBuffer * c){
	return c->size;
}
// Очищает буфер
void CB_Flush(CircleBuffer *c){
	c->head = 0;
	c->tail = 0;
	c->size = 0;
}
// Записывает элемент в буфер
void CB_PlaceTail(CircleBuffer * c , int16_t value){
	c->buf[c->tail] = value;
	c->tail ++;
	if (c->tail == c->capacity) { // даже, если буфер полный, то мы просто переписываем самые старые данные
		c->tail = 0;
	}
	
	if (c->size < c->capacity){ // если буфер не полный, то увеличиваем счетчик элементов
		c->size ++;
	}
	
	if (c->tail == c->head) { // если начали переписывать данные, то смещаем указатель на начало
		c->head ++;
		if (c->head == c->capacity) {
			c->head = 0;
		}
	}
}
// Извлечь элемент с начала кольцевого буфера
int16_t CB_GetHead(CircleBuffer *c){
	int16_t tmp = 0;
	if (c->size > 0) {
		tmp = c->buf[c->head];
		c->head ++;
		c->size --;
		if (c->head == c->capacity) {
			c->head = 0;
		}
	}
	return tmp;
}
// расчитать среднее квадратичное значение элементов в буфере
int16_t CB_GetRMS(CircleBuffer * c){
	uint16_t i, j;
	int32_t sqr_sum = 0;
	
	j = c->head;
	for (i = 0; i < c->size; i++){
		sqr_sum += c->buf[j]*c->buf[j];
		j++;
		if (j == c->capacity) {
			j = 0;
		}
	}
	sqr_sum /= c->size;	
	return sqrt(sqr_sum);
}

// расчитать среднее значение элементов в буфере
int16_t CB_GetAVG(CircleBuffer * c){
	uint16_t i, j;
	uint32_t avg_sum = 0;
	
	j = c->head;
	for (i = 0; i < c->size; i++){
		avg_sum += c->buf[j];
		j++;
		if (j == c->capacity) {
			j = 0;
		}
	}	
	return avg_sum /= (uint32_t)c->size;
}

//=====================================================================================================
//    Bit circle buffer
//=====================================================================================================
BitCircleBuffer *  BitCB_Create(uint8_t bit_capacity){
  BitCircleBuffer * tmp;
	uint8_t byte_num;
	tmp = malloc(sizeof(BitCircleBuffer));
	tmp->bit_capacity = bit_capacity;
	byte_num = bit_capacity / 8;
	if ((bit_capacity % 8) != 0) { // если количество бит не кратно 8
		byte_num ++;
	}
	tmp->bit_buf = malloc(sizeof(uint8_t)*byte_num); // считам кличество необходимых байт для хранения bit_capacity бит
	tmp->bit_head = 0;
	tmp->bit_tail = 0;
	tmp->bit_size = 0;
	return tmp;
}

void BitCB_Flush(BitCircleBuffer *c){
	c->bit_head = 0;
	c->bit_tail = 0;
	c->bit_size = 0;
}


void BitCB_PlaceTail(BitCircleBuffer* c, uint8_t bit_value){
	
	if (bit_value) {
		c->bit_buf[c->bit_tail / 8] |=  1 << (c->bit_tail % 8); // set bit 
	} else {
		c->bit_buf[c->bit_tail / 8] &=  ~(1 << (c->bit_tail % 8)); // reset bit
	}
	c->bit_tail ++;
	if (c->bit_tail == c->bit_capacity) { // даже, если буфер полный, то мы просто переписываем самые старые данные
		c->bit_tail = 0;
	}
	
	if (c->bit_size < c->bit_capacity){ // если буфер не полный, то увеличиваем счетчик элементов
		c->bit_size ++;
	}
	
	if (c->bit_tail == c->bit_head) { // если начали переписывать данные, то смещаем указатель на начало
		c->bit_head ++;
		if (c->bit_head == c->bit_capacity) {
			c->bit_head = 0;
		}
	}
}

// service function
uint8_t CountOnes (uint8_t n) {
  n = ((n>>1) & 0x55) + (n & 0x55);
  n = ((n>>2) & 0x33) + (n & 0x33);
  n = ((n>>4) & 0x0F) + (n & 0x0F);
  return n;
}

uint8_t BitCB_GetOnes(BitCircleBuffer* c){
	uint8_t i, j;
	uint8_t ones = 0;
	uint8_t last_byte;
	uint8_t max_byte_count;
	
	last_byte = c->bit_capacity / 8;
	if ((c->bit_capacity % 8) != 0) {
		last_byte ++;
	}
	
	max_byte_count = c->bit_size / 8;
	if ((c->bit_size % 8) != 0){
		max_byte_count++;
	}
	j = c->bit_head / 8; // byte num
	for (i = 0; i < max_byte_count; i++){
		ones += CountOnes(c->bit_buf[j]);
		j++;
		if (j == last_byte) {
			j = 0;
		}
	}	
	return ones;
}



