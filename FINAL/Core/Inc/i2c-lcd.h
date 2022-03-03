#ifndef  __i2c-lcd_H
#define  __i2c-lcd_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);

void lcd_clear_want (int row,int col);

void LCD_Print(int num);

void Data_Print(int num);
#ifdef __cplusplus
}
#endif
#endif
