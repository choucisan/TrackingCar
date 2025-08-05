#include "ADC.h"


unsigned int adc_getValue(void)
{
//     unsigned int gAdcResult = 0;

//     //使能ADC转换
//     DL_ADC12_enableConversions(ADC12_0_INST);
//     //软件触发ADC开始转换
//     DL_ADC12_startConversion(ADC12_0_INST);

//     //如果当前状态 不是 空闲状态
//     while (DL_ADC12_getStatus(ADC12_0_INST) != DL_ADC12_STATUS_CONVERSION_IDLE );

//     //清除触发转换状态
//     DL_ADC12_stopConversion(ADC12_0_INST);
//     //失能ADC转换
//     DL_ADC12_disableConversions(ADC12_0_INST);

//     //获取数据
//     gAdcResult = DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_0);

//     return gAdcResult;
    

    // uint32_t timeout = 100000;
    // unsigned int result = 0;

    // DL_ADC12_enableConversions(ADC12_0_INST);
    // DL_ADC12_startConversion(ADC12_0_INST);

    // // while (DL_ADC12_getStatus(ADC12_0_INST) != DL_ADC12_STATUS_CONVERSION_IDLE)
    // // {
    // //     if (--timeout == 0)
    // //     {
    // //         DL_ADC12_stopConversion(ADC12_0_INST);
    // //         DL_ADC12_disableConversions(ADC12_0_INST);
    // //         return 0xFFFF; // 出错返回
    // //     }
    // // }

    // DL_ADC12_stopConversion(ADC12_0_INST);
    // DL_ADC12_disableConversions(ADC12_0_INST);

    // result = DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_0);
    // return result;
unsigned int gAdcResult = 0;

    //使能ADC转换
    DL_ADC12_enableConversions(ADC12_0_INST);
    //软件触发ADC开始转换
    DL_ADC12_startConversion(ADC12_0_INST);

    //如果当前状态 不是 空闲状态
    while (DL_ADC12_getStatus(ADC12_0_INST) != DL_ADC12_STATUS_CONVERSION_IDLE );

    //清除触发转换状态
    DL_ADC12_stopConversion(ADC12_0_INST);
    //失能ADC转换
    DL_ADC12_disableConversions(ADC12_0_INST);

    //获取数据
    gAdcResult = DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_0);

    return gAdcResult;
    
}
