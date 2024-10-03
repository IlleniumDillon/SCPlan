/*
 * vofa+.h
 *
 *  Created on: Sep 30, 2024
 *      Author: 14769
 */

#ifndef PROTOCOL_VOFA__H_
#define PROTOCOL_VOFA__H_

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include <stdlib.h>

size_t firewater_write(char* pdata, size_t max_size, const char *head, uint8_t n, float* din);
size_t firewater_read(char* pdata, size_t max_size, char *head, uint8_t n, float* dout);

size_t justfloat_write(uint8_t* pdata, size_t max_size, uint8_t n, float* din);
size_t justfloat_read(char* pdata, size_t max_size, uint8_t n, float* dout);

#endif /* PROTOCOL_VOFA__H_ */
