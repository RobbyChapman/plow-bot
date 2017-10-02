/*
 * retarget_print_util.h
 *
 *  Created on: Sep 28, 2017
 *      Author: Robert.Chapman
 */

#ifndef RETARGET_PRINT_UTIL_H_
#define RETARGET_PRINT_UTIL_H_

#include <stdio.h>

/* Init */
void initPrintTarget(void);

/* Override functions from stdio */
int fputc(int _c, register FILE *_fp);
int fputs(const char *_ptr, register FILE *_fp);

#endif
