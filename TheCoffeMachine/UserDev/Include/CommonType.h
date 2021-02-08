/// @file CommonType.h
/// @brief Common global type definition

#ifndef _COMMONTYPE_H_
#define _COMMONTYPE_H_

#define MAX_UI32            (0x0FFFFFFFFu)
#define MIN_UI32                      (0u)
#define MAX_SI32             ( 2147483647)
#define MIN_SI32             (-2147483648)


#define MAX_UI16            (0x0FFFFu)
#define MIN_UI16                  (0u)
#define MAX_SI16              ( 32767)
#define MIN_SI16              (-32768)

#define MAX_UI8             (0xFFu)
#define MIN_UI8                (0u)
#define MAX_SI8              ( 127)
#define MIN_SI8              (-128)

typedef unsigned char       tUI8;
typedef   signed char       tSI8;
typedef unsigned short      tUI16;
typedef   signed short      tSI16;
typedef unsigned int        tUI32;
typedef   signed int        tSI32;
typedef unsigned long long  tUI64;
typedef   signed long long  tSI64;

typedef float               tF32;
typedef double              tF64;

typedef enum {
  eFALSE = (0==1),
  eTRUE  = (1==1)
} tBOOL;

#endif /* _COMMONTYPE_H_ */
