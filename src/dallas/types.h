#ifndef _TYPES_H
#define _TYPES_H

#ifndef NULL
#define NULL 0
#endif

typedef unsigned char       u08;
typedef signed char         s08;
typedef unsigned short      u16;
typedef unsigned long       u32;
typedef unsigned long long  u64;

#ifdef _WINDOWS_
#define DECLEXPORT	__declspec(dllexport)
#define DALLASLIB	__stdcall
#else
#define DECLEXPORT
#define DALLASLIB  __attribute__((stdcall))
#endif


#endif
