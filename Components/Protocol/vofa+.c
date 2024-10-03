/*
 * vofa+.c
 *
 *  Created on: Sep 30, 2024
 *      Author: 14769
 */

#include "vofa+.h"

uint8_t justfloat_tail[] = {0x00, 0x00, 0x80, 0x7f};
char tempBuffer[64];

size_t firewater_write(char* pdata, size_t max_size, const char *head, uint8_t n, float* din)
{
	size_t ret = 0;
	size_t len = 0;
	int left_size = max_size;
	if (head != NULL)
	{
		ret = snprintf(pdata + len, left_size, "%s:", head);
		if (left_size <= ret) return 0;
		left_size -= ret;
		len += ret;
	}
	for (int i = 0; i < n; i++)
	{
		ret = snprintf(pdata + len, left_size, "%f,", din[i]);
		if (left_size <= ret)
		{
			ret = left_size;
			left_size -= ret;
			len += ret;
			break;
		}
		left_size -= ret;
		len += ret;
	}
	pdata[len - 1] = '\n';
	return len;
}
size_t firewater_read(char* pdata, size_t max_size, char *head, uint8_t n, float* dout)
{
	uint8_t temp = 0;
	uint8_t num = 0;
	for (int i = 0; i < max_size; i++)
	{
		if (pdata[i] == ':')
		{
			tempBuffer[temp] = 0;
			temp = 0;
			strcpy(head, tempBuffer);
		}
		else if (pdata[i] == ',')
		{
			tempBuffer[temp] = 0;
			temp = 0;
			dout[num] = atof(tempBuffer);
			num ++;
			if (num == n)
			{
				return num;
			}
		}
		else if (pdata[i] == '\n')
		{
			tempBuffer[temp] = 0;
			temp = 0;
			dout[num] = atof(tempBuffer);
			num ++;
			return num;
		}
		else
		{
			tempBuffer[temp] = pdata[i];
			temp++;
		}
	}
	return num;
}

size_t justfloat_write(uint8_t* pdata, size_t max_size, uint8_t n, float* din)
{
    uint8_t _n = max_size / 4 - 1 > n ? n : max_size / 4 - 1;
    memcpy(pdata, din, _n * 4);
    memcpy(pdata + _n * 4, justfloat_tail, 4);
    return _n * 4 + 4;
}
size_t justfloat_read(char* pdata, size_t max_size, uint8_t n, float* dout)
{
    if (max_size < 4) return 0;
    if (
        !(pdata[max_size - 1] == justfloat_tail[3] &&
        pdata[max_size - 2] == justfloat_tail[2] &&
        pdata[max_size - 3] == justfloat_tail[1] &&
        pdata[max_size - 4] == justfloat_tail[0])
    ) return 0;
    uint8_t _n = max_size / 4 - 1 > n ? n : max_size / 4 - 1;
    memcpy(pdata, dout, _n * 4);
    return _n;
}
