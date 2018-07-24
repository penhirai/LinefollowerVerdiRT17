/*
 * typedef.h
 *
 *  Created on: 2018/06/21
 *      Author: shuuichi
 */

#ifndef TYPEDEF_H_
#define TYPEDEF_H_

#include <stdio.h>


#ifndef __TYPEDEF__
typedef signed char _SBYTE;
typedef unsigned char _UBYTE;
typedef signed short _SWORD;
typedef unsigned short _UWORD;
typedef signed int _SINT;
typedef unsigned int _UINT;
typedef signed long _SDWORD;
typedef unsigned long _UDWORD;
typedef signed long long _SQWORD;
typedef unsigned long long _UQWORD;

typedef	unsigned char	uint8_t;
typedef	unsigned short	uint16_t;
typedef	unsigned int	uint32_t;

typedef	volatile unsigned char	vuint8_t;
typedef	volatile unsigned short	vuint16_t;
typedef	volatile unsigned int	vuint32_t;

typedef	char			int8_t;
typedef short			int16_t;
typedef int				int32_t;
#endif

#define __TYPEDEF__

typedef float			float32_t;

#define	UINT8_MAX					0xFF
#define	UINT8_MAX_HALF				0x7F
#define	UINT16_MAX					0xFFFF
#define	UINT16_MAX_HALF				0x7FFF
#define	INT32_MAX					0x7FFFFFFF
#define	INT32_MAX_HALF				0x3FFFFFFF

#endif /* TYPEDEF_H_ */
