#ifndef  __stm32_tm1637_H
#define  __stm32_tm1637_H
#ifdef __cplusplus
extern "C" {
#endif
//#pragma once

void tm1637Init(void);
void tm1637DisplayDecimal(int v, int displaySeparator);
void tm1637SetBrightness(char brightness);
#ifdef __cplusplus
}
#endif
#endif
