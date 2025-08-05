#ifndef __ADC_H__
#define __ADC_H__


#include "ti/driverlib/dl_adc12.h"
#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "board.h"


unsigned int adc_getValue(void);

#endif