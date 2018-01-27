
#include "xpt2046.h"

#define RESCALE_FACTOR 		100000  
#define SAMPLE_COUNT 			20
#define MEDIAN						SAMPLE_COUNT

volatile int32_t cali_A = -21485; 
volatile int32_t cali_B = -100; 
volatile int32_t cali_C = 82926165; 
volatile int32_t cali_D = -50; 
volatile int32_t cali_E = 14225; 
volatile int32_t cali_F = -4866561;

uint8_t getTouchState(void)
{
	if (XPT_IRQ_STATE == GPIO_PIN_RESET) {
		//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); //GPIO_PIN_12
		return  1;
	} else {
		//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		return  0;
	}
}

int16_t touchGetX(SPI_HandleTypeDef *hspi)
{
	uint16_t LSB, MSB;
	int16_t ret = 4095;
	uint8_t cmd[2];

	if (getTouchState()) // read touch irq
	{
		XPT_CS_ON;
		ret = 0x2F; // 2F
		while(--ret);
		//osDelay(1);
		// x
		cmd[0] = 0x01;
	  cmd[1] = 0xD0; //D4
		HAL_SPI_TransmitReceive(hspi, cmd, (uint8_t*)&MSB, 1, 10);
		cmd[0] = 0x01;
	  cmd[1] = 0x01;
		HAL_SPI_TransmitReceive(hspi, cmd, (uint8_t*)&LSB, 1, 10);
		ret=0x0F;
		while(--ret);
		XPT_CS_OFF;
		ret = ( ((MSB<<4) & 0x0FF0) | ((LSB>>12) & 0x000F) )<<1;
	}
	return  ret;
}

int16_t touchGetY(SPI_HandleTypeDef *hspi)
{
	uint16_t LSB, MSB;
	int16_t ret = 4095;
  uint8_t cmd[2];
	
	if (getTouchState())
	{		
		XPT_CS_ON;
		ret = 0x2F; //2F
		while(--ret);
		//osDelay(1);
		// y
		cmd[0] = 0x00;
	  cmd[1] = 0x90; // 94
		HAL_SPI_TransmitReceive(hspi, cmd, (uint8_t*)&MSB, 1, 10);
		cmd[0] = 0x00;
	  cmd[1] = 0x00;
		HAL_SPI_TransmitReceive(hspi, cmd, (uint8_t*)&LSB, 1, 10);
		ret=0x0F;
		while(--ret);
		XPT_CS_OFF;

		ret = ( ((MSB<<4) & 0x0FF0) | ((LSB>>12) & 0x000F) )<<1;
	}
	return ret;
}

int GetAverageCoordinates( SPI_HandleTypeDef *hspi, uint32_t * pX, uint32_t * pY , int nSamples )
{
	int nRead = 0;
	int32_t xAcc = 0 , yAcc = 0;
	int x , y;

	while ( nRead < nSamples ) {
		//if ( !xpt2046GetCoordinates( &x , &y ) ) {
		//	break;
		//}
		x = touchGetX(hspi);
		if (x >= 4095 || x == 0) {break;}
		y = touchGetY(hspi);
		if (y >= 4095 || y == 0) {break;}
		xAcc += x;
		yAcc += y;
		nRead ++;
	}

	if ( nRead == 0 ) {
		*pX = 0;
		*pY = 0;
		return 0;
	}
	*pX = xAcc / nRead;
	*pY = yAcc / nRead;
	return 1;
}

int median(uint32_t x[]) {
    uint32_t temp;
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<MEDIAN-1; i++) {
        for(j=i+1; j<MEDIAN; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }

    if(MEDIAN%2==0) {
        // if there is an even number of elements, return mean of the two elements in the middle
        return((x[MEDIAN/2] + x[MEDIAN/2 - 1]) / 2.0);
    } else {
        // else return the element in the middle
        return x[MEDIAN/2];
    }
}


void TP_GetMedianAdXY(SPI_HandleTypeDef *hspi, uint32_t *x, uint32_t *y)
{
   uint32_t madx[MEDIAN];
   uint32_t mady[MEDIAN];
   int i;

   for (i=0; i<MEDIAN; i++)
     {
       GetAverageCoordinates(hspi, &madx[i],  &mady[i], MEDIAN);
     }
   *x= median(madx);
   *y= median(mady);
}

void touchGetSense(SPI_HandleTypeDef *hspi, uint32_t * x, uint32_t * y)
{
	uint32_t in_x, in_y;
	/*
	if (!GetAverageCoordinates(hspi, &in_x, &in_y , SAMPLE_COUNT)) {
		return;
	}
	*/
	
	TP_GetMedianAdXY(hspi, &in_x, &in_y);
	
	*x = (cali_A * in_x + cali_B * in_y + cali_C) / RESCALE_FACTOR;
  *y = (cali_D * in_x + cali_E * in_y + cali_F) / RESCALE_FACTOR;
	/*
  if (*x > DISP_WIDTH || *y > DISP_HEIGHT)
  {
		*x = 0;
		*y = 0;
  }*/
	
}

void DrawCross(uint16_t px, uint16_t py)
{
	GUI_DrawLine(px, py-25, px, py+25); // vertical line
	GUI_DrawLine(px-25, py, px+25, py); // hoorisontal line
}

// Калибровка
void TouchCalibrate3Points ( SPI_HandleTypeDef *hspi )
{
	const uint16_t xPointsCenter[] = {(90 * DISP_WIDTH) / 100, (50 * DISP_WIDTH) / 100, (10 * DISP_WIDTH) / 100};
	const uint16_t yPointsCenter[] = {(50 * DISP_HEIGHT) / 100, (90 * DISP_HEIGHT) / 100, (10 * DISP_HEIGHT) / 100};
	
	int32_t xRaw[3];
	int32_t yRaw[3];
	uint32_t tmp_x, tmp_y;
	
	double temp1, temp2;
	double cal_A = 0.0, cal_B = 0.0, cal_C = 0.0, cal_D = 0.0, cal_E = 0.0, cal_F = 0.0;

	
	GUI_SetBkColor(GUI_WHITE);
	GUI_SetColor(GUI_BLACK);
	//------- First point -----------------------
	GUI_Clear();
	GUI_DispStringAt ( "Calibration", 50, 130);
	DrawCross(xPointsCenter[0], yPointsCenter[0]);
	GUI_DispStringAt("Touch ", 50, 150);
	while (1)
	{
		// ждать нажатия
		while ( !getTouchState( ) );
		osDelay(1000);
		//GetAverageCoordinates(hspi, &tmp_x, &tmp_y , SAMPLE_COUNT);
		TP_GetMedianAdXY(hspi, &tmp_x, &tmp_y);
		if (tmp_x < 4090 && tmp_y < 4090)
		{
			xRaw[0] = tmp_x;
			yRaw[0] = tmp_y;
			break;
		} // if
	} // while
	GUI_DispStringAt("Release ", 50, 150);
	// ждать отпускания
	while ( getTouchState ( ) );
	GUI_DispStringAt("             ", 50, 150);
	
	HAL_Delay(1000);
	
	//------- Second point -----------------------
	GUI_Clear();
	GUI_DispStringAt ( "Calibration", 50, 130);
	DrawCross(xPointsCenter[1], yPointsCenter[1]);
	GUI_DispStringAt("Touch ", 50, 150);
	while (1)
	{
		// ждать нажатия
		while ( !getTouchState( ) );
		osDelay(1000);
		//GetAverageCoordinates(hspi, &tmp_x, &tmp_y , SAMPLE_COUNT);
		TP_GetMedianAdXY(hspi, &tmp_x, &tmp_y);
		if (tmp_x < 4090 && tmp_y < 4090)
		{
			xRaw[1] = tmp_x;
			yRaw[1] = tmp_y;
			break;
		} // if
	} // while
	GUI_DispStringAt("Release ", 50, 150);
	// ждать отпускания
	while ( getTouchState ( ) );
	GUI_DispStringAt("             ", 50, 150);
	
	HAL_Delay(1000);
	
	//------------ Third point -------------
	GUI_Clear();
	GUI_DispStringAt ( "Calibration", 50, 130);
	DrawCross(xPointsCenter[2], yPointsCenter[2]);
	GUI_DispStringAt("Touch ", 50, 150);
	while (1)
	{
		// ждать нажатия
		while ( !getTouchState( ) );
		osDelay(1000);
		//GetAverageCoordinates(hspi, &tmp_x, &tmp_y , SAMPLE_COUNT);
		TP_GetMedianAdXY(hspi, &tmp_x, &tmp_y);
		if (tmp_x < 4090 && tmp_y < 4090)
		{
			xRaw[2] = tmp_x;
			yRaw[2] = tmp_y;
			break;
		} // if
	} // while
	GUI_DispStringAt("Release ", 50, 150);
	// ждать отпускания
	while (getTouchState());
	
	
	GUI_Clear();
	
	//A
	temp1 = ((double) xPointsCenter[0] * ((double) yRaw[1] - (double) yRaw[2])) + ((double) xPointsCenter[1] * ((double) yRaw[2]- (double) yRaw[0])) + ((double) xPointsCenter[2] * ((double) yRaw[0] - (double) yRaw[1]));
	temp2 = ((double) xRaw[0] * ((double) yRaw[1] - (double) yRaw[2])) + ((double) xRaw[1] * ((double) yRaw[2] - (double) yRaw[0])) + ((double) xRaw[2] * ((double) yRaw[0] - (double) yRaw[1]));
	cal_A = temp1 / temp2;
	cali_A = (int32_t) ((double)cal_A * RESCALE_FACTOR);

	//B
	temp1 = (cal_A * ((double) xRaw[2] - (double) xRaw[1])) + (double) xPointsCenter[1] - (double) xPointsCenter[2];
	temp2 = (double) yRaw[1] - (double) yRaw[2];
	cal_B = temp1 / temp2;
	cali_B = (int32_t) ((double)cal_B * RESCALE_FACTOR);

	//C
	cal_C = (double) xPointsCenter[2] - (cal_A * (double) xRaw[2]) - (cal_B * (double) yRaw[2]);
	cali_C = (int32_t) (cal_C * RESCALE_FACTOR);

	//D
	temp1 = ((double) yPointsCenter[0] * ((double) yRaw[1] - (double) yRaw[2])) + ((double) yPointsCenter[1] * ((double) yRaw[2] - (double) yRaw[0])) + ((double) yPointsCenter[2] * ((double) yRaw[0] - (double) yRaw[1]));
	temp2 = ((double) xRaw[0] * ((double) yRaw[1] - (double) yRaw[2])) + ((double) xRaw[1] * ((double) yRaw[2] - (double) yRaw[0])) + ((double) xRaw[2] * ((double) yRaw[0] - (double) yRaw[1]));
	cal_D = (double)temp1 / (double)temp2;
	cali_D = (int32_t) (cal_D * RESCALE_FACTOR);

	//E
	temp1 = (cal_D * ((double) xRaw[2] - (double) xRaw[1])) + (double) yPointsCenter[1] - (double) yPointsCenter[2];
	temp2 = (double) yRaw[1] - (double) yRaw[2];
	cal_E = (double)temp1 / (double)temp2;
	cali_E = (int32_t) (cal_E * RESCALE_FACTOR);

	//F
	cal_F = (double) yPointsCenter[2] - cal_D * (double) xRaw[2] - cal_E * (double) yRaw[2];
	cali_F = (int32_t) (cal_F * RESCALE_FACTOR);
	
	GUI_DispStringAt( "Calibration done", 20, 150);
	GUI_DispStringAt( "A", 5, 20);
	GUI_DispStringAt( "B", 5, 40);
	GUI_DispStringAt( "C", 5, 60);
	GUI_DispStringAt( "D", 5, 80);
	GUI_DispStringAt( "E", 5, 100);
	GUI_DispStringAt( "F", 5, 120);
	
	GUI_DispDecAt(cali_A, 20, 20, 10);
	GUI_DispDecAt(cali_B, 20, 40, 10);
	GUI_DispDecAt(cali_C, 20, 60, 10);
	GUI_DispDecAt(cali_D, 20, 80, 10);
	GUI_DispDecAt(cali_E, 20, 100, 10);
	GUI_DispDecAt(cali_F, 20, 120, 10);
	osDelay(10000);
	
}

void TouchSetCoeff(int32_t A, int32_t B, int32_t C, int32_t D, int32_t E, int32_t F){
	cali_A = A; 
  cali_B = B; 
  cali_C = C; 
  cali_D = D; 
  cali_E = E; 
  cali_F = F;
}

void TouchGetCoeff(int32_t *A, int32_t *B, int32_t *C, int32_t *D, int32_t *E, int32_t *F){
	*A = cali_A; 
  *B = cali_B; 
  *C = cali_C; 
  *D = cali_D; 
  *E = cali_E; 
  *F = cali_F;
}
