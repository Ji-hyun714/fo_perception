#ifndef wafl3H
#define wafl3H

#if defined(_MSC_VER) && (_MSC_VER < 1900)
	#include <stdint_pc.h>
#else /* #ifdef _MSC_VER */
	#include <stdint.h>
#endif /* #else #ifdef _MSC_VER */

#if (defined(_WIN32) || defined(WIN32))
	#define __thread __declspec(thread)
	#if defined(__BORLANDC__)
		#pragma warn -8019    /* winnt.h 컴파일 시 경고 나오는 부분을 임시로 막음 */
		#include <windows.h>
		#pragma warn .8019
	#else /* #if defined(__BORLANDC__) */
		#include <windows.h>
	#endif /* #else defined(__BORLANDC__) */
#endif /* #if (defined(_WIN32) || defined(WIN32)) */

#ifndef __cplusplus
	#ifdef _MSC_VER
		#define bool BOOL
	#else /* #ifdef _MSC_VER */
		typedef _Bool bool;
	#endif /* #else #ifdef _MSC_VER */
#endif /* #ifndef __cplusplus */

#ifndef TRUE
#define TRUE 1
#endif //#ifndef TRUE

#ifndef FALSE
#define FALSE 0
#endif //#ifndef FALSE

#ifdef __cplusplus
extern "C" {
#endif //#else ifdef __cplusplus

#if !defined(WA_PACKAGE)
	#if defined(WIN32) || defined(_WIN32)
		#define wa_packed_
		#if defined(WA_EXPORTS)
			#define WA_PACKAGE __declspec(dllexport)
		#else /*#if defined(WA_EXPORTS)*/
			#define WA_PACKAGE __declspec(dllimport)
		#endif /*#else #if defined(WA_EXPORTS)*/
	#else /*#if defined(WIN32) || defined(_WIN32)*/
		#define wa_packed_ __attribute__((packed))
		#define WA_PACKAGE
	#endif /*#else #if defined(WIN32) || defined(_WIN32)*/
#endif /*#if !defined(WA_PACKAGE)*/

#ifndef IN
	#define IN
#endif /*#ifndef IN*/

#ifndef OUT
	#define OUT
#endif /*#ifndef OUT*/

#ifndef min
//	#define min(x,y) (((x) < (y)) ? (x) : (y)) 
#endif /*#ifndef min*/
#ifndef Min
	#define Min(x,y) (((x) < (y)) ? (x) : (y))
#endif /*#ifndef Min*/

#ifndef max
//	#define max(x,y) (((x) > (y)) ? (x) : (y))
#endif /*#ifndef max*/
#ifndef Max
	#define Max(x,y) (((x) > (y)) ? (x) : (y))
#endif /*#ifndef Max*/

/*==========================================================================
Visual C++ 에서의 호환성
==========================================================================*/
#if defined(_MSC_VER)
	#if !defined(inline)
		#define inline __inline  /* Visual C++에서만 inline을 __inline으로 변경*/
	#endif /* if !defined(inline)*/
	#define __func__ __FUNCTION__ /* Visual C++에서는 __func__를 지원하지 않음*/
	#define vsnwprintf _vsnwprintf
#endif /*(defined(_MSC_VER) && !defined(inline))*/

#if defined(__linux__)
	#define vsprintf_s(dst, dst_size, format, arg) vsnprintf(dst, dst_size, format, arg)
#endif /*#if defined(__linux__)*/



/*==========================================================================
#pragma 관련 warning 제거
==========================================================================*/
#if (defined(_WIN32) || defined(WIN32))
	#define CODE_SECTION(x,y)
	#define DATA_ALIGN(x,y)
	#define UNROLL(x)
	#define WA_FASTC __fastcall
#else /*#if (defined(_WIN32) || defined(WIN32))*/
	#if !defined(__fastcall)
		#define __fastcall
	#endif //#if !defined(__fastcall)
	#if !defined(__cdecl)
		#define __cdecl
	#endif //#if !defined(__cdecl)
	#ifndef NULL
		#define NULL 0
	#endif /*#ifndef NULL*/
	#define WA_FASTC 
#endif /*#else #if (defined(_WIN32) || defined(WIN32))*/

/*==========================================================================
Calling Convention  관련
==========================================================================*/
#if !defined(WA_API)
	#if defined(_WIN32) || defined(WIN32)
		#define WA_API __stdcall
	#else
		#define WA_API /**< 외부 노출 함수 호출 컨벤션 정의. \nWin32 환경이 아닌 경우 아무것도 지정되지 않는다.*/
	#endif
#endif

#if !defined(WA_CAPI)
	#if defined(_WIN32) || defined(WIN32)
		#define WA_CAPI __cdecl
	#else
		#define WA_CAPI /**<외부 노출 C 스타일 함수 호출 컨벤션 정의. \n
			가변 인자를 사용하려면 반드시 지정하는 것이 안전하다. Win32 환경이
			아닌 경우 아무것도 지정되지 않는다.*/
	#endif
#endif

/*==========================================================================
Type 정의
==========================================================================*/

#define WA_OK (1)
#define WA_ERR (0)

typedef uint8_t  img_t;
#define COLOR_DEPTH 8

typedef float    float32_t;
typedef double   float64_t;
/*
#if (sizeof(bool) == 4) //[[
typedef uint8_t  bool8_t;
typedef uint16_t bool16_t;
typedef bool     bool32_t;
#elif (sizeof(bool) == 1) //][
typedef bool    bool8_t;
typedef uint16_t bool16_t;
typedef uint32_t bool32_t;
#elif (sizeof(bool) == 2) //][
typedef uint8_t  bool8_t;
typedef bool     bool16_t;
typedef uint32_t bool32_t;
#endif //]]
*/
typedef uint8_t  bool8_t;
typedef uint16_t bool16_t;
typedef bool     bool32_t;

typedef char  utf8_t;
#if defined(_WIN32) || defined(WIN32) //[
	typedef wchar_t utf16_t;
#else //][
	typedef short utf16_t;
#endif //]

typedef unsigned int   utf32_t;
typedef char           ansi_t;

//!!!!! check please 2020.09.22, kjkim
#if !_WIN32
typedef void *   handle_t;
//typedef intptr_t   handle_t; //icms build error occur, 2020.09.29, kjkim
#endif

typedef uint32_t fourcc_t;
	static inline fourcc_t fourcc_from_cstr_unsafe(const ansi_t *src) {return *((fourcc_t const *)src);}
	#define make_fourcc(a,b,c,d) ((fourcc_t)((fourcc_t)(a) | (fourcc_t)((fourcc_t)(b) << 8U) | (fourcc_t)((fourcc_t)(c) << 16U) | (fourcc_t)((fourcc_t)(d) << 24U)))

typedef union tagFOURCC {
	fourcc_t value;
	ansi_t c[4];
} FOURCC_T;


typedef uint16_t twocc_t;
	static inline twocc_t twocc_from_cstr_unsafe(const ansi_t *src) {return *((twocc_t const *)src);}
	#define make_twocc(a,b) ((twocc_t)((twocc_t)(a) | (twocc_t)((twocc_t)(b) << 8U)))

#define FOURCC_VFMT "%c%c%c%c"
#define FOURCC_VARG(x) ((ansi_t)((x) & 0xffU)), ((ansi_t)(((x)>>8U) & 0xffU)), ((ansi_t)(((x)>>16U) & 0xffU)), ((ansi_t)(((x)>>24U) & 0xffU))

#define TWOCC_VFMT "%c%c"
#define TWOCC_VARG(x) ((ansi_t)((x) & 0xffU)), ((ansi_t)(((x)>>8U) & 0xffU))

#if !defined(__cplusplus)
	#if defined(_WIN32)
		#if !defined(NULL)
			#define NULL (0)
		#endif /*#if !defined(NULL)*/
	#endif /*#if defined(_WIN32)*/
#endif /*#if !defined(__cplusplus)*/



/**
@brief WA_TYPE의 하위 bit 0,1은 TYPE의 크기를 표현하는 shift 값으로 구성: 1 << (WA_TYPE & 0x03) 으로 크기를 판단할 수 있다.
@detail bit2,3이 모두 0이면 실수형, bit2만 1이면 signed 정수형 bit3만 1이면 unsigned를 의미한다.
bit2와 bit3가 모두 1인 경우는 BOOL타입이다.
@author ksg
*/
#define WA_TYPE_MASK_DIMENSION    0x30U
#define WA_TYPE_MASK_POW          0x03U
#define WA_TYPE_FLAG_MATRIX       0x40U
#define WA_TYPE_FLAG_HOMOGENEOUS  0x80U
#define WA_TYPE_FLAG_REAL         0x00U
#define WA_TYPE_FLAG_SIGNED_INT   0x04U
#define WA_TYPE_FLAG_UNSIGNED_INT 0x08U

typedef uint8_t WA_TYPE;

#pragma pack(1)
typedef union tagWA_TYPE_ADAPTOR {
	struct {
		uint8_t elem_size_in_pow_of_2 : 2;
		uint8_t elem_int_type : 2;
		uint8_t dim_minus_one : 2;
		uint8_t is_matrix : 1;
		uint8_t is_homo_geneous : 1;
	} as_bits;
	WA_TYPE as_value;
} WA_TYPE_ADAPTOR;
#pragma pack()

#define WA_TYPE_NULL    (0x00U)
#define WA_TYPE_FLOAT   (WA_TYPE_FLAG_REAL         | 0x02U) /*0x02*/
#define WA_TYPE_DOUBLE  (WA_TYPE_FLAG_REAL         | 0x03U) /*0x03*/
#define WA_TYPE_INT8    (WA_TYPE_FLAG_SIGNED_INT   | 0x00U) /*0x04*/
#define WA_TYPE_INT16   (WA_TYPE_FLAG_SIGNED_INT   | 0x01U) /*0x05*/
#define WA_TYPE_INT32   (WA_TYPE_FLAG_SIGNED_INT   | 0x02U) /*0x06*/
#define WA_TYPE_INT64   (WA_TYPE_FLAG_SIGNED_INT   | 0x03U) /*0x07*/
#define WA_TYPE_UINT8   (WA_TYPE_FLAG_UNSIGNED_INT | 0x00U) /*0x08*/
#define WA_TYPE_UINT16  (WA_TYPE_FLAG_UNSIGNED_INT | 0x01U) /*0x09*/
#define WA_TYPE_UINT32  (WA_TYPE_FLAG_UNSIGNED_INT | 0x02U) /*0x0a*/
#define WA_TYPE_UINT64  (WA_TYPE_FLAG_UNSIGNED_INT | 0x03U) /*0x0b*/

#define WA_TYPE_BOOL    (0x0eU)
#define WA_TYPE_USTR    (0x0fU)
#define WA_TYPE_STRUCT  (0x0dU)
#define WA_TYPE_OBJECT  (0x0cU)



#define WA_TYPE_VEC2X     (0x10U)

#define WA_TYPE_VEC2F     (WA_TYPE_VEC2X | WA_TYPE_FLOAT)
#define WA_TYPE_VEC2D     (WA_TYPE_VEC2X | WA_TYPE_DOUBLE)
#define WA_TYPE_VEC2I8    (WA_TYPE_VEC2X | WA_TYPE_INT8)
#define WA_TYPE_VEC2I16   (WA_TYPE_VEC2X | WA_TYPE_INT16)
#define WA_TYPE_VEC2I32   (WA_TYPE_VEC2X | WA_TYPE_INT32)
#define WA_TYPE_VEC2I64   (WA_TYPE_VEC2X | WA_TYPE_INT64)
#define WA_TYPE_VEC2U8    (WA_TYPE_VEC2X | WA_TYPE_UINT8)
#define WA_TYPE_VEC2U16   (WA_TYPE_VEC2X | WA_TYPE_UINT16)
#define WA_TYPE_VEC2U32   (WA_TYPE_VEC2X | WA_TYPE_UINT32)
#define WA_TYPE_VEC2U64   (WA_TYPE_VEC2X | WA_TYPE_UINT64)


#define WA_TYPE_HVEC2X    (WA_TYPE_FLAG_HOMOGENEOUS | 0x20U)

#define WA_TYPE_HVEC2F    (WA_TYPE_HVEC2X | WA_TYPE_FLOAT)
#define WA_TYPE_HVEC2D    (WA_TYPE_HVEC2X | WA_TYPE_DOUBLE)
#define WA_TYPE_HVEC2I8   (WA_TYPE_HVEC2X | WA_TYPE_INT8)
#define WA_TYPE_HVEC2I16  (WA_TYPE_HVEC2X | WA_TYPE_INT16)
#define WA_TYPE_HVEC2I32  (WA_TYPE_HVEC2X | WA_TYPE_INT32)
#define WA_TYPE_HVEC2I64  (WA_TYPE_HVEC2X | WA_TYPE_INT64)
#define WA_TYPE_HVEC2U8   (WA_TYPE_HVEC2X | WA_TYPE_UINT8)
#define WA_TYPE_HVEC2U16  (WA_TYPE_HVEC2X | WA_TYPE_UINT16)
#define WA_TYPE_HVEC2U32  (WA_TYPE_HVEC2X | WA_TYPE_UINT32)
#define WA_TYPE_HVEC2U64  (WA_TYPE_HVEC2X | WA_TYPE_UINT64)


#define WA_TYPE_VEC3X     (0x20U)

#define WA_TYPE_VEC3F     (WA_TYPE_VEC3X | WA_TYPE_FLOAT)
#define WA_TYPE_VEC3D     (WA_TYPE_VEC3X | WA_TYPE_DOUBLE)
#define WA_TYPE_VEC3I8    (WA_TYPE_VEC3X | WA_TYPE_INT8)
#define WA_TYPE_VEC3I16   (WA_TYPE_VEC3X | WA_TYPE_INT16)
#define WA_TYPE_VEC3I32   (WA_TYPE_VEC3X | WA_TYPE_INT32)
#define WA_TYPE_VEC3I64   (WA_TYPE_VEC3X | WA_TYPE_INT64)
#define WA_TYPE_VEC3U8    (WA_TYPE_VEC3X | WA_TYPE_UINT8)
#define WA_TYPE_VEC3U16   (WA_TYPE_VEC3X | WA_TYPE_UINT16)
#define WA_TYPE_VEC3U32   (WA_TYPE_VEC3X | WA_TYPE_UINT32)
#define WA_TYPE_VEC3U64   (WA_TYPE_VEC3X | WA_TYPE_UINT64)


#define WA_TYPE_HVEC3X    (WA_TYPE_FLAG_HOMOGENEOUS | 0x30U)

#define WA_TYPE_HVEC3F    (WA_TYPE_HVEC3X | WA_TYPE_FLOAT)
#define WA_TYPE_HVEC3D    (WA_TYPE_HVEC3X | WA_TYPE_DOUBLE)
#define WA_TYPE_HVEC3I8   (WA_TYPE_HVEC3X | WA_TYPE_INT8)
#define WA_TYPE_HVEC3I16  (WA_TYPE_HVEC3X | WA_TYPE_INT16)
#define WA_TYPE_HVEC3I32  (WA_TYPE_HVEC3X | WA_TYPE_INT32)
#define WA_TYPE_HVEC3I64  (WA_TYPE_HVEC3X | WA_TYPE_INT64)
#define WA_TYPE_HVEC3U8   (WA_TYPE_HVEC3X | WA_TYPE_UINT8)
#define WA_TYPE_HVEC3U16  (WA_TYPE_HVEC3X | WA_TYPE_UINT16)
#define WA_TYPE_HVEC3U32  (WA_TYPE_HVEC3X | WA_TYPE_UINT32)
#define WA_TYPE_HVEC3U64  (WA_TYPE_HVEC3X | WA_TYPE_UINT64)


#define WA_TYPE_VEC4X     (0x30U)

#define WA_TYPE_VEC4F     (WA_TYPE_VEC4X | WA_TYPE_FLOAT)
#define WA_TYPE_VEC4D     (WA_TYPE_VEC4X | WA_TYPE_DOUBLE)
#define WA_TYPE_VEC4I8    (WA_TYPE_VEC4X | WA_TYPE_INT8)
#define WA_TYPE_VEC4I16   (WA_TYPE_VEC4X | WA_TYPE_INT16)
#define WA_TYPE_VEC4I32   (WA_TYPE_VEC4X | WA_TYPE_INT32)
#define WA_TYPE_VEC4I64   (WA_TYPE_VEC4X | WA_TYPE_INT64)
#define WA_TYPE_VEC4U8    (WA_TYPE_VEC4X | WA_TYPE_UINT8)
#define WA_TYPE_VEC4U16   (WA_TYPE_VEC4X | WA_TYPE_UINT16)
#define WA_TYPE_VEC4U32   (WA_TYPE_VEC4X | WA_TYPE_UINT32)
#define WA_TYPE_VEC4U64   (WA_TYPE_VEC4X | WA_TYPE_UINT64)


#define WA_TYPE_MAT2X     (WA_TYPE_FLAG_MATRIX | 0x10U)

#define WA_TYPE_MAT2F     (WA_TYPE_MAT2X | WA_TYPE_FLOAT)
#define WA_TYPE_MAT2D     (WA_TYPE_MAT2X | WA_TYPE_DOUBLE)
#define WA_TYPE_MAT2I8    (WA_TYPE_MAT2X | WA_TYPE_INT8)
#define WA_TYPE_MAT2I16   (WA_TYPE_MAT2X | WA_TYPE_INT16)
#define WA_TYPE_MAT2I32   (WA_TYPE_MAT2X | WA_TYPE_INT32)
#define WA_TYPE_MAT2I64   (WA_TYPE_MAT2X | WA_TYPE_INT64)
#define WA_TYPE_MAT2U8    (WA_TYPE_MAT2X | WA_TYPE_UINT8)
#define WA_TYPE_MAT2U16   (WA_TYPE_MAT2X | WA_TYPE_UINT16)
#define WA_TYPE_MAT2U32   (WA_TYPE_MAT2X | WA_TYPE_UINT32)
#define WA_TYPE_MAT2U64   (WA_TYPE_MAT2X | WA_TYPE_UINT64)


#define WA_TYPE_HMAT2X    (WA_TYPE_FLAG_HOMOGENEOUS | WA_TYPE_FLAG_MATRIX | 0x20U)

#define WA_TYPE_HMAT2F    (WA_TYPE_HMAT2X | WA_TYPE_FLOAT)
#define WA_TYPE_HMAT2D    (WA_TYPE_HMAT2X | WA_TYPE_DOUBLE)
#define WA_TYPE_HMAT2I8   (WA_TYPE_HMAT2X | WA_TYPE_INT8)
#define WA_TYPE_HMAT2I16  (WA_TYPE_HMAT2X | WA_TYPE_INT16)
#define WA_TYPE_HMAT2I32  (WA_TYPE_HMAT2X | WA_TYPE_INT32)
#define WA_TYPE_HMAT2I64  (WA_TYPE_HMAT2X | WA_TYPE_INT64)
#define WA_TYPE_HMAT2U8   (WA_TYPE_HMAT2X | WA_TYPE_UINT8)
#define WA_TYPE_HMAT2U16  (WA_TYPE_HMAT2X | WA_TYPE_UINT16)
#define WA_TYPE_HMAT2U32  (WA_TYPE_HMAT2X | WA_TYPE_UINT32)
#define WA_TYPE_HMAT2U64  (WA_TYPE_HMAT2X | WA_TYPE_UINT64)


#define WA_TYPE_MAT3X     (WA_TYPE_FLAG_MATRIX | 0x20U)

#define WA_TYPE_MAT3F     (WA_TYPE_MAT3X | WA_TYPE_FLOAT)
#define WA_TYPE_MAT3D     (WA_TYPE_MAT3X | WA_TYPE_DOUBLE)
#define WA_TYPE_MAT3I8    (WA_TYPE_MAT3X | WA_TYPE_INT8)
#define WA_TYPE_MAT3I16   (WA_TYPE_MAT3X | WA_TYPE_INT16)
#define WA_TYPE_MAT3I32   (WA_TYPE_MAT3X | WA_TYPE_INT32)
#define WA_TYPE_MAT3I64   (WA_TYPE_MAT3X | WA_TYPE_INT64)
#define WA_TYPE_MAT3U8    (WA_TYPE_MAT3X | WA_TYPE_UINT8)
#define WA_TYPE_MAT3U16   (WA_TYPE_MAT3X | WA_TYPE_UINT16)
#define WA_TYPE_MAT3U32   (WA_TYPE_MAT3X | WA_TYPE_UINT32)
#define WA_TYPE_MAT3U64   (WA_TYPE_MAT3X | WA_TYPE_UINT64)


#define WA_TYPE_HMAT3X    (WA_TYPE_FLAG_HOMOGENEOUS | WA_TYPE_FLAG_MATRIX | 0x30U)

#define WA_TYPE_HMAT3F    (WA_TYPE_HMAT3X | WA_TYPE_FLOAT)
#define WA_TYPE_HMAT3D    (WA_TYPE_HMAT3X | WA_TYPE_DOUBLE)
#define WA_TYPE_HMAT3I8   (WA_TYPE_HMAT3X | WA_TYPE_INT8)
#define WA_TYPE_HMAT3I16  (WA_TYPE_HMAT3X | WA_TYPE_INT16)
#define WA_TYPE_HMAT3I32  (WA_TYPE_HMAT3X | WA_TYPE_INT32)
#define WA_TYPE_HMAT3I64  (WA_TYPE_HMAT3X | WA_TYPE_INT64)
#define WA_TYPE_HMAT3U8   (WA_TYPE_HMAT3X | WA_TYPE_UINT8)
#define WA_TYPE_HMAT3U16  (WA_TYPE_HMAT3X | WA_TYPE_UINT16)
#define WA_TYPE_HMAT3U32  (WA_TYPE_HMAT3X | WA_TYPE_UINT32)
#define WA_TYPE_HMAT3U64  (WA_TYPE_HMAT3X | WA_TYPE_UINT64)


#define WA_TYPE_MAT4X     (WA_TYPE_FLAG_MATRIX | 0x30U)

#define WA_TYPE_MAT4F     (WA_TYPE_MAT4X | WA_TYPE_FLOAT)
#define WA_TYPE_MAT4D     (WA_TYPE_MAT4X | WA_TYPE_DOUBLE)
#define WA_TYPE_MAT4I8    (WA_TYPE_MAT4X | WA_TYPE_INT8)
#define WA_TYPE_MAT4I16   (WA_TYPE_MAT4X | WA_TYPE_INT16)
#define WA_TYPE_MAT4I32   (WA_TYPE_MAT4X | WA_TYPE_INT32)
#define WA_TYPE_MAT4I64   (WA_TYPE_MAT4X | WA_TYPE_INT64)
#define WA_TYPE_MAT4U8    (WA_TYPE_MAT4X | WA_TYPE_UINT8)
#define WA_TYPE_MAT4U16   (WA_TYPE_MAT4X | WA_TYPE_UINT16)
#define WA_TYPE_MAT4U32   (WA_TYPE_MAT4X | WA_TYPE_UINT32)
#define WA_TYPE_MAT4U64   (WA_TYPE_MAT4X | WA_TYPE_UINT64)


/* Helper functions*/

#define WA_TYPE_IsHomogeneous(type) ((uint32_t)(((type) & WA_TYPE_FLAG_HOMOGENEOUS) >> 7))
#define WA_TYPE_IsMatrix(type)      ((uint32_t)(((type) & WA_TYPE_FLAG_MATRIX) >> 6))
#define WA_TYPE_ElementSize(type)   (1u << (uint32_t)((type) & WA_TYPE_MASK_POW))
#define WA_TYPE_DimField(type)      ((uint32_t)((type) & WA_TYPE_MASK_DIMENSION) >> 4)
#define WA_TYPE_Dimension(type)     (WA_TYPE_DimField(type) + 1)
#define WA_TYPE_CalcSize(type) \
	( WA_TYPE_ElementSize(type) * WA_TYPE_Dimension(type) * \
	(1 + WA_TYPE_IsMatrix(type) * WA_TYPE_DimField(type)) )
#define WA_TYPE_BaseType(type)      ((type) & 0x0f)




/*==========================================================================
실수 시스템
==========================================================================*/

#define REAL_SYSTEM_FLOAT   0
#define REAL_SYSTEM_DOUBLE  1
#define REAL_SYSTEM_FIXED   2

#define REAL_SYSTEM REAL_SYSTEM_FLOAT
/*#define REAL_SYSTEM REAL_SYSTEM_FIXED*/

#if REAL_SYSTEM == REAL_SYSTEM_FIXED
	typedef int32_t real_t;
	#define REAL_FRAC 10
	#define REAL_FRAC_HALF (REAL_FRAC / 2)
	#define FloatToReal(v) ((real_t)((v) * (float)(1 << REAL_FRAC)))
	#define IntToReal(v)   ((real_t)((v) << REAL_FRAC))
	#define RealToFloat(v) ((float)(v) / (float)(1 << REAL_FRAC))
	#define RealToInt(v)   ((v) >> REAL_FRAC)
	#define fp_mlt(a, b) (((a) * (b)) >> REAL_FRAC)
/*	#define fp_mlt(a, b) ((a >> REAL_FRAC_HALF) * (b >> REAL_FRAC_HALF))*/
	#define fp_div(a, b) (((a) << REAL_FRAC) / (b))
	#define fp_pow2(a) (((a) * (a)) >> REAL_FRAC)
	#define fp_pow3(a) (((((a) * (a)) >> REAL_FRAC) * (a)) >> REAL_FRAC)

	#define WA_TYPE_REAL  WA_TYPE_INT32
	#define WA_TYPE_VEC2  WA_TYPE_VEC2I32
	#define WA_TYPE_VEC3  WA_TYPE_VEC3I32
	#define WA_TYPE_VEC4  WA_TYPE_VEC4I32
	#define WA_TYPE_HVEC2 WA_TYPE_HVEC2I32
	#define WA_TYPE_HVEC3 WA_TYPE_HVEC3I32
	#define WA_TYPE_MAT2  WA_TYPE_MAT2I32
	#define WA_TYPE_MAT3  WA_TYPE_MAT3I32
	#define WA_TYPE_MAT4  WA_TYPE_MAT4I32
	#define WA_TYPE_HMAT2 WA_TYPE_HMAT2I32
	#define WA_TYPE_HMAT3 WA_TYPE_HMAT3I32

	#define real_one (1)
	#define real_zero (0)

	#define real_is_zero(a) ((a) == 0)
	#define real_is_one(a) ((a) == 1)
	#define real_is(a,b) ((a) == (b))
	#define real_is_lt(a,b) ((a) < (b))
	#define real_is_gt(a,b) ((a) > (b))
	#define real_is_le(a,b) ((a) <= (b))
	#define real_is_ge(a,b) ((a) >= (b))


#else /*#if REAL_SYSTEM == REAL_SYSTEM_FIXED*/
	#if REAL_SYSTEM == REAL_SYSTEM_DOUBLE
		typedef double real_t;
		#define REAL_EPSILON 0.0000000000001

		#define WA_TYPE_REAL  WA_TYPE_DOUBLE
		#define WA_TYPE_VEC2  WA_TYPE_VEC2D
		#define WA_TYPE_VEC3  WA_TYPE_VEC3D
		#define WA_TYPE_VEC4  WA_TYPE_VEC4D
		#define WA_TYPE_HVEC2 WA_TYPE_HVEC2D
		#define WA_TYPE_HVEC3 WA_TYPE_HVEC3D
		#define WA_TYPE_MAT2  WA_TYPE_MAT2D
		#define WA_TYPE_MAT3  WA_TYPE_MAT3D
		#define WA_TYPE_MAT4  WA_TYPE_MAT4D
		#define WA_TYPE_HMAT2 WA_TYPE_HMAT2D
		#define WA_TYPE_HMAT3 WA_TYPE_HMAT3D

		#define real_one (1.0)
		#define real_zero (0.0)


	#else /*#if REAL_SYSTEM == REAL_SYSTEM_DOUBLE*/
		typedef float real_t;
		#define REAL_EPSILON 0.0000001

		#define WA_TYPE_REAL  WA_TYPE_FLOAT
		#define WA_TYPE_VEC2  WA_TYPE_VEC2F
		#define WA_TYPE_VEC3  WA_TYPE_VEC3F
		#define WA_TYPE_VEC4  WA_TYPE_VEC4F
		#define WA_TYPE_HVEC2 WA_TYPE_HVEC2F
		#define WA_TYPE_HVEC3 WA_TYPE_HVEC3F
		#define WA_TYPE_MAT2  WA_TYPE_MAT2F
		#define WA_TYPE_MAT3  WA_TYPE_MAT3F
		#define WA_TYPE_MAT4  WA_TYPE_MAT4F
		#define WA_TYPE_HMAT2 WA_TYPE_HMAT2F
		#define WA_TYPE_HMAT3 WA_TYPE_HMAT3F

		#define real_one (1.0f)
		#define real_zero (0.0f)
	#endif /*#else REAL_SYSTEM == REAL_SYSTEM_DOUBLE*/

	#define FloatToReal(value) (value)
	#define IntToReal(value) ((real_t)(value))
	#define RealToFloat(value) (value)
	#define RealToInt(value) ((int32_t)(value))
	#define fp_mlt(a, b) ((a) * (b))
	#define fp_div(a, b) ((a) / (b))
	#define fp_pow2(a) ((a) * (a))
	#define fp_pow3(a) ((a) * (a) * (a))

	#define real_is_zero(a) ((a) < REAL_EPSILON && (a) > -REAL_EPSILON)
	#define real_is_one(a) ((((a) - real_one) < REAL_EPSILON) && (((a) - real_one) > -REAL_EPSILON))
	#define real_is(a,b) ((((a) - (b)) < REAL_EPSILON) && (((a) - (b)) > -REAL_EPSILON))
	#define real_is_lt(a,b) (((b) - (a)) >= REAL_EPSILON)
	#define real_is_gt(a,b) (((a) - (b)) >= REAL_EPSILON)
	#define real_is_le(a,b) (((a) - REAL_EPSILON) < (b))
	#define real_is_ge(a,b) (((a) + REAL_EPSILON) > (b))

#endif

#define fp_add(a, b) (a + b)
#define fp_sub(a, b) (a - b)
#define fp_abs(a) ((a > 0) ? a : -a)

#define ClampBoth(n, lower, upper)  ((n) > (upper) ? (upper) : ((n) < (lower) ? (lower) : (n)))
#define Clamp8(n)  ((n) > 255 ? 255 : ((n) < 0 ? 0 : (n)))

static inline int32_t   f32_to_i32(float32_t v) {return (int32_t)v;}
static inline uint32_t  f32_to_u32(float32_t v) {return (uint32_t)v;}
static inline int16_t   f32_to_i16(float32_t v) {return (int16_t)v;}
static inline uint16_t  f32_to_u16(float32_t v) {return (uint16_t)v;}
static inline int8_t    f32_to_i8 (float32_t v) {return (int8_t)v;}
static inline uint8_t   f32_to_u8 (float32_t v) {return (uint8_t)v;}
static inline float64_t f32_to_f64(float32_t v) {return (float64_t)v;}

static inline int32_t   f64_to_i32(float64_t v) {return (int32_t)v;}
static inline uint32_t  f64_to_u32(float64_t v) {return (uint32_t)v;}
static inline int16_t   f64_to_i16(float64_t v) {return (int16_t)v;}
static inline uint16_t  f64_to_u16(float64_t v) {return (uint16_t)v;}
static inline int8_t    f64_to_i8 (float64_t v) {return (int8_t)v;}
static inline uint8_t   f64_to_u8 (float64_t v) {return (uint8_t)v;}
static inline float32_t f64_to_f32(float64_t v) {return (float32_t)v;}


/*==========================================================================
기본 구조체 정의
==========================================================================*/

#define DEFINE_REC_STRUCT2(name, type, m1, m2) \
	typedef struct tag_##name { \
		type m1; \
		type m2; \
	} name; \
	static inline void set_##name(name *r, type m1, type m2) {r->m1 = m1; r->m2 = m2;} \
	static inline name make_##name(type m1, type m2) {name r; r.m1 = m1; r.m2 = m2; return r;}

#define DEFINE_REC_STRUCT3(name, type, m1, m2, m3) \
	typedef struct tag_##name { \
		type m1; \
		type m2; \
		type m3; \
	} name; \
	static inline void set_##name(name *r, type m1, type m2, type m3) {r->m1 = m1; r->m2 = m2; r->m3 = m3;} \
	static inline name make_##name(type m1, type m2, type m3) {name r; r.m1 = m1; r.m2 = m2; r.m3 = m3; return r;}

#define DEFINE_REC_STRUCT4(name, type, m1, m2, m3, m4) \
	typedef struct tag_##name { \
		type m1; \
		type m2; \
		type m3; \
		type m4; \
	} name; \
	static inline void set_##name(name *r, type m1, type m2, type m3, type m4) {r->m1 = m1; r->m2 = m2; r->m3 = m3; r->m4 = m4;} \
	static inline name make_##name(type m1, type m2, type m3, type m4) {name r; r.m1 = m1; r.m2 = m2; r.m3 = m3; r.m4 = m4; return r;}

#define DEFINE_REC_STRUCT6(name, type, m1, m2, m3, m4, m5, m6) \
	typedef struct tag_##name { \
		type m1, m2, m3, m4, m5, m6; \
	} name; \
	static inline void set_##name(name *r, type m1, type m2, type m3, type m4, type m5, type m6) {r->m1 = m1; r->m2 = m2; r->m3 = m3; r->m4 = m4; r->m5 = m5; r->m6 = m6;} \
	static inline name make_##name(type m1, type m2, type m3, type m4, type m5, type m6) {name r; r.m1 = m1; r.m2 = m2; r.m3 = m3; r.m4 = m4; r.m5 = m5; r.m6 = m6; return r;}

DEFINE_REC_STRUCT2(range2,    real_t,   begin, end)
DEFINE_REC_STRUCT2(range2f,   float32_t, begin, end)
DEFINE_REC_STRUCT2(range2d,   float64_t, begin, end)
DEFINE_REC_STRUCT2(range2i8,  int8_t,   begin, end)
DEFINE_REC_STRUCT2(range2u8,  uint8_t,  begin, end)
DEFINE_REC_STRUCT2(range2i16, int16_t,  begin, end)
DEFINE_REC_STRUCT2(range2u16, uint16_t, begin, end)
DEFINE_REC_STRUCT2(range2i32, int32_t,  begin, end)
DEFINE_REC_STRUCT2(range2u32, uint32_t, begin, end)
DEFINE_REC_STRUCT2(range2i64, int64_t,  begin, end)
DEFINE_REC_STRUCT2(range2u64, uint64_t, begin, end)

DEFINE_REC_STRUCT2(size2,    real_t,   width, height)
DEFINE_REC_STRUCT2(size2f,   float32_t, width, height)
DEFINE_REC_STRUCT2(size2d,   float64_t, width, height)
DEFINE_REC_STRUCT2(size2i8,  int8_t,   width, height)
DEFINE_REC_STRUCT2(size2u8,  uint8_t,  width, height)
DEFINE_REC_STRUCT2(size2i16, int16_t,  width, height)
DEFINE_REC_STRUCT2(size2u16, uint16_t, width, height)
DEFINE_REC_STRUCT2(size2i32, int32_t,  width, height)
DEFINE_REC_STRUCT2(size2u32, uint32_t, width, height)
DEFINE_REC_STRUCT2(size2i64, int64_t,  width, height)
DEFINE_REC_STRUCT2(size2u64, uint64_t, width, height)

/*DEFINE_REC_STRUCT2(vec2,    real_t,   x, y)*/
DEFINE_REC_STRUCT2(vec2f,   float32_t, x, y)
DEFINE_REC_STRUCT2(vec2d,   float64_t, x, y)
DEFINE_REC_STRUCT2(vec2i8,  int8_t,   x, y)
DEFINE_REC_STRUCT2(vec2u8,  uint8_t,  x, y)
DEFINE_REC_STRUCT2(vec2i16, int16_t,  x, y)
DEFINE_REC_STRUCT2(vec2u16, uint16_t, x, y)
DEFINE_REC_STRUCT2(vec2i32, int32_t,  x, y)
DEFINE_REC_STRUCT2(vec2u32, uint32_t, x, y)
DEFINE_REC_STRUCT2(vec2i64, int64_t,  x, y)
DEFINE_REC_STRUCT2(vec2u64, uint64_t, x, y)


typedef vec2f point2f;
#define make_point2f make_vec2f
//DEFINE_REC_STRUCT2(point2f,   float,    x, y)
DEFINE_REC_STRUCT2(point2d,   float64_t,   x, y)
DEFINE_REC_STRUCT2(point2i8,  int8_t,   x, y)
DEFINE_REC_STRUCT2(point2u8,  uint8_t,  x, y)
DEFINE_REC_STRUCT2(point2i16, int16_t,  x, y)
DEFINE_REC_STRUCT2(point2u16, uint16_t, x, y)
DEFINE_REC_STRUCT2(point2i32, int32_t,  x, y)
DEFINE_REC_STRUCT2(point2u32, uint32_t, x, y)
DEFINE_REC_STRUCT2(point2i64, int64_t,  x, y)
DEFINE_REC_STRUCT2(point2u64, uint64_t, x, y)


static inline void float_assign(void *dst, const void *src, void *context) {
	*((float*)dst) = *((const float*)src);
}
static inline int float_compare(const void *lhs, const void *rhs, void *context) {
	float diff = *((const float*)lhs) - *((const float*)rhs);
	return diff > 0.0f ? 1 : (diff < 0.0f ? -1 : 0);
}

static inline void point2f_assign(void *dst, const void *src, void *context) {
	*((point2f*)dst) = *((const point2f*)src);
}
static inline int point2f_compare_x(const void *lhs, const void *rhs, void *context) {
	float diff = ((const point2f*)lhs)->x - ((const point2f*)rhs)->x;
	return diff > 0.0f ? 1 : (diff < 0.0f ? -1 : 0);
}
static inline int point2f_compare_y(const void *lhs, const void *rhs, void *context) {
	float diff = ((const point2f*)lhs)->y - ((const point2f*)rhs)->y;
	return diff > 0.0f ? 1 : (diff < 0.0f ? -1 : 0);
}

static inline void point2i16_assign(void *dst, const void *src, void *context) {
	*((point2i16*)dst) = *((const point2i16*)src);
}
static inline int point2i16_compare_x(const void *lhs, const void *rhs, void *context) {
	int16_t diff = ((const point2i16*)lhs)->x - ((const point2i16*)rhs)->x;
	return diff > 0 ? 1 : (diff < 0 ? -1 : 0);
}
static inline int point2i16_compare_y(const void *lhs, const void *rhs, void *context) {
	int16_t diff = ((const point2i16*)lhs)->y - ((const point2i16*)rhs)->y;
	return diff > 0 ? 1 : (diff < 0 ? -1 : 0);
}


typedef struct {
	point2f *p0;
	point2f *p1;
} segment2f;

static inline void segment2f_assign(void *dst, const void *src, void *context) {
	*((segment2f*)dst) = *((const segment2f*)src);
}
static inline int segment2f_compare_y(const void *lhs, const void *rhs, void *context) {
	float diff = ((const segment2f*)lhs)->p0->y - ((const segment2f*)rhs)->p0->y;
	return diff > 0.0f ? 1 : (diff < 0.0f ? -1 : 0);
}


#if REAL_SYSTEM == REAL_SYSTEM_FIXED
#define vec2 vec2i32
#define set_vec2 set_vec2i32
#define make_vec2 make_vec2i32







#elif REAL_SYSTEM == REAL_SYSTEM_FLOAT
#define vec2 vec2f
#define set_vec2 set_vec2f
#define make_vec2 make_vec2f

#define rect2 rect2f
#define set_rect2 set_rect2f
#define make_rect2 make_rect2f

#define rect3 rect3f
#define set_rect3 set_rect3f
#define make_rect3 make_rect3f

#define xywh2 xywh2f
#define set_xywh2 set_xywh2f
#define make_xywh2 make_xywh2f

#define xyzwhd3 xyzwhd3f
#define set_xyzwhd3 set_xyzwhd3f
#define make_xyzwhd3 make_xyzwhd3f

#define vec3 vec3f
#define set_vec3 set_vec3f
#define make_vec3 make_vec3f

#define vec4 vec4f
#define set_vec4 set_vec4f
#define make_vec4 make_vec4f

#elif REAL_SYSTEM == REAL_SYSTEM_DOUBLE

#define vec2 vec2d
#define set_vec2 set_vec2d
#define make_vec2 make_vec2d

#define rect2 rect2d
#define set_rect2 set_rect2d
#define make_rect2 make_rect2d

#define rect3 rect3d
#define set_rect3 set_rect3d
#define make_rect3 make_rect3d

#define xywh2 xywh2d
#define set_xywh2 set_xywh2d
#define make_xywh2 make_xywh2d

#define xyzwhd3 xyzwhd3d
#define set_xyzwhd3 set_xyzwhd3d
#define make_xyzwhd3 make_xyzwhd3d

#define vec3 vec3d
#define set_vec3 set_vec3d
#define make_vec3 make_vec3d

#define vec4 vec4d
#define set_vec4 set_vec4d
#define make_vec4 make_vec4d

#endif


DEFINE_REC_STRUCT3(vec3f,   float32_t, x, y, z)
DEFINE_REC_STRUCT3(vec3d,   float64_t, x, y, z)
DEFINE_REC_STRUCT3(vec3i8,  int8_t,   x, y, z)
DEFINE_REC_STRUCT3(vec3u8,  uint8_t,  x, y, z)
DEFINE_REC_STRUCT3(vec3i16, int16_t,  x, y, z)
DEFINE_REC_STRUCT3(vec3u16, uint16_t, x, y, z)
DEFINE_REC_STRUCT3(vec3i32, int32_t,  x, y, z)
DEFINE_REC_STRUCT3(vec3u32, uint32_t, x, y, z)
DEFINE_REC_STRUCT3(vec3i64, int64_t,  x, y, z)
DEFINE_REC_STRUCT3(vec3u64, uint64_t, x, y, z)

typedef vec3f point3f;
#define make_pointf3f make_vec3f
//DEFINE_REC_STRUCT2(point3f,   float,    x, y)
DEFINE_REC_STRUCT3(point3d,   float64_t,   x, y, z)
DEFINE_REC_STRUCT3(point3i8,  int8_t,   x, y, z)
DEFINE_REC_STRUCT3(point3u8,  uint8_t,  x, y, z)
DEFINE_REC_STRUCT3(point3i16, int16_t,  x, y, z)
DEFINE_REC_STRUCT3(point3u16, uint16_t, x, y, z)
DEFINE_REC_STRUCT3(point3i32, int32_t,  x, y, z)
DEFINE_REC_STRUCT3(point3u32, uint32_t, x, y, z)
DEFINE_REC_STRUCT3(point3i64, int64_t,  x, y, z)
DEFINE_REC_STRUCT3(point3u64, uint64_t, x, y, z)


DEFINE_REC_STRUCT4(vec4f,   float32_t, x, y, z, w)
DEFINE_REC_STRUCT4(vec4d,   float64_t, x, y, z, w)
DEFINE_REC_STRUCT4(vec4i8,  int8_t,   x, y, z, w)
DEFINE_REC_STRUCT4(vec4u8,  uint8_t,  x, y, z, w)
DEFINE_REC_STRUCT4(vec4i16, int16_t,  x, y, z, w)
DEFINE_REC_STRUCT4(vec4u16, uint16_t, x, y, z, w)
DEFINE_REC_STRUCT4(vec4i32, int32_t,  x, y, z, w)
DEFINE_REC_STRUCT4(vec4u32, uint32_t, x, y, z, w)
DEFINE_REC_STRUCT4(vec4i64, int64_t,  x, y, z, w)
DEFINE_REC_STRUCT4(vec4u64, uint64_t, x, y, z, w)

#define vec2_add(a,b) make_vec2((float32_t)((a).x) + (float32_t)((b).x), (float32_t)((a).y) + (float32_t)((b).y))
#define vec2_sub(a,b) make_vec2((float32_t)((a).x) - (float32_t)((b).x), (float32_t)((a).y) - (float32_t)((b).y))
#define vec2_mlt(a,b) make_vec2((float32_t)((a).x) * (float32_t)((b).x), (float32_t)((a).y) * (float32_t)((b).y))
#define vec2_div(a,b) make_vec2((float32_t)((a).x) / (float32_t)((b).x), (float32_t)((a).y) / (float32_t)((b).y))
#define vec2_add_sc(a,b) make_vec2((float32_t)((a).x) + (float32_t)(b), (float32_t)((a).y) + (float32_t)(b))
#define vec2_sub_sc(a,b) make_vec2((float32_t)((a).x) - (float32_t)(b), (float32_t)((a).y) - (float32_t)(b))
#define vec2_mlt_sc(a,b) make_vec2((float32_t)((a).x) * (float32_t)(b), (float32_t)((a).y) * (float32_t)(b))
#define vec2_div_sc(a,b) make_vec2((float32_t)((a).x) / (float32_t)(b), (float32_t)((a).y) / (float32_t)(b))
#define vec2_addeq(a,b) ((a).x += (b).x, (a).y += (b).y)
#define vec2_subeq(a,b) ((a).x -= (b).x, (a).y -= (b).y)
#define vec2_mlteq(a,b) ((a).x *= (b).x, (a).y *= (b).y)
#define vec2_diveq(a,b) ((a).x /= (b).x, (a).y /= (b).y)
#define vec2_addeq_sc(a,b) ((a).x += (b), (a).y += (b))
#define vec2_subeq_sc(a,b) ((a).x -= (b), (a).y -= (b))
#define vec2_mlteq_sc(a,b) ((a).x *= (b), (a).y *= (b))
#define vec2_diveq_sc(a,b) ((a).x /= (b), (a).y /= (b))
#define vec2_sqlen(a)   ((a).x * (a).x + (a).y * (a).y)
#define vec2_sqdist(a,b)   (((a).x - (b).x) * ((a).x - (b).x) + ((a).y - (b).y) * ((a).y - (b).y))

DEFINE_REC_STRUCT4(rect2f,   float32_t, left, top, right, bottom)
DEFINE_REC_STRUCT4(rect2d,   float64_t, left, top, right, bottom)
DEFINE_REC_STRUCT4(rect2i8,  int8_t,   left, top, right, bottom)
DEFINE_REC_STRUCT4(rect2u8,  uint8_t,  left, top, right, bottom)
DEFINE_REC_STRUCT4(rect2i16, int16_t,  left, top, right, bottom)
DEFINE_REC_STRUCT4(rect2u16, uint16_t, left, top, right, bottom)
DEFINE_REC_STRUCT4(rect2i32, int32_t,  left, top, right, bottom)
DEFINE_REC_STRUCT4(rect2u32, uint32_t, left, top, right, bottom)
DEFINE_REC_STRUCT4(rect2i64, int64_t,  left, top, right, bottom)
DEFINE_REC_STRUCT4(rect2u64, uint64_t, left, top, right, bottom)

DEFINE_REC_STRUCT6(rect3f,   float32_t, left, top, right, bottom, front, rear)
DEFINE_REC_STRUCT6(rect3d,   float64_t, left, top, right, bottom, front, rear)
DEFINE_REC_STRUCT6(rect3i8,  int8_t,   left, top, right, bottom, front, rear)
DEFINE_REC_STRUCT6(rect3u8,  uint8_t,  left, top, right, bottom, front, rear)
DEFINE_REC_STRUCT6(rect3i16, int16_t,  left, top, right, bottom, front, rear)
DEFINE_REC_STRUCT6(rect3u16, uint16_t, left, top, right, bottom, front, rear)
DEFINE_REC_STRUCT6(rect3i32, int32_t,  left, top, right, bottom, front, rear)
DEFINE_REC_STRUCT6(rect3u32, uint32_t, left, top, right, bottom, front, rear)
DEFINE_REC_STRUCT6(rect3i64, int64_t,  left, top, right, bottom, front, rear)
DEFINE_REC_STRUCT6(rect3u64, uint64_t, left, top, right, bottom, front, rear)

DEFINE_REC_STRUCT4(xywh2f,   float32_t, x, y, width, height)
DEFINE_REC_STRUCT4(xywh2d,   float64_t, x, y, width, height)
DEFINE_REC_STRUCT4(xywh2i8,  int8_t,   x, y, width, height)
DEFINE_REC_STRUCT4(xywh2u8,  uint8_t,  x, y, width, height)
DEFINE_REC_STRUCT4(xywh2i16, int16_t,  x, y, width, height)
DEFINE_REC_STRUCT4(xywh2u16, uint16_t, x, y, width, height)
DEFINE_REC_STRUCT4(xywh2i32, int32_t,  x, y, width, height)
DEFINE_REC_STRUCT4(xywh2u32, uint32_t, x, y, width, height)
DEFINE_REC_STRUCT4(xywh2i64, int64_t,  x, y, width, height)
DEFINE_REC_STRUCT4(xywh2u64, uint64_t, x, y, width, height)

DEFINE_REC_STRUCT6(xyzwhd2f,   float32_t, x, y, z, width, height, depth)
DEFINE_REC_STRUCT6(xyzwhd2d,   float64_t, x, y, z, width, height, depth)
DEFINE_REC_STRUCT6(xyzwhd2i8,  int8_t,   x, y, z, width, height, depth)
DEFINE_REC_STRUCT6(xyzwhd2u8,  uint8_t,  x, y, z, width, height, depth)
DEFINE_REC_STRUCT6(xyzwhd2i16, int16_t,  x, y, z, width, height, depth)
DEFINE_REC_STRUCT6(xyzwhd2u16, uint16_t, x, y, z, width, height, depth)
DEFINE_REC_STRUCT6(xyzwhd2i32, int32_t,  x, y, z, width, height, depth)
DEFINE_REC_STRUCT6(xyzwhd2u32, uint32_t, x, y, z, width, height, depth)
DEFINE_REC_STRUCT6(xyzwhd2i64, int64_t,  x, y, z, width, height, depth)
DEFINE_REC_STRUCT6(xyzwhd2u64, uint64_t, x, y, z, width, height, depth)

#define vec3_add(a,b) make_vec3((float32_t)((a).x) + (float32_t)((b).x), (float32_t)((a).y) + (float32_t)((b).y), (float32_t)((a).z) + (float32_t)((b).z))
#define vec3_sub(a,b) make_vec3((float32_t)((a).x) - (float32_t)((b).x), (float32_t)((a).y) - (float32_t)((b).y), (float32_t)((a).z) - (float32_t)((b).z))
#define vec3_mlt(a,b) make_vec3((float32_t)((a).x) * (float32_t)((b).x), (float32_t)((a).y) * (float32_t)((b).y), (float32_t)((a).z) * (float32_t)((b).z))
#define vec3_div(a,b) make_vec3((float32_t)((a).x) / (float32_t)((b).x), (float32_t)((a).y) / (float32_t)((b).y), (float32_t)((a).z) / (float32_t)((b).z))
#define vec3_add_sc(a,b) make_vec3((float32_t)((a).x) + (float32_t)(b), (float32_t)((a).y) + (float32_t)(b), (float32_t)((a).z) + (float32_t)(b))
#define vec3_sub_sc(a,b) make_vec3((float32_t)((a).x) - (float32_t)(b), (float32_t)((a).y) - (float32_t)(b), (float32_t)((a).z) - (float32_t)(b))
#define vec3_mlt_sc(a,b) make_vec3((float32_t)((a).x) * (float32_t)(b), (float32_t)((a).y) * (float32_t)(b), (float32_t)((a).z) * (float32_t)(b))
#define vec3_div_sc(a,b) make_vec3((float32_t)((a).x) / (float32_t)(b), (float32_t)((a).y) / (float32_t)(b), (float32_t)((a).z) / (float32_t)(b))
#define vec3_addeq(a,b) ((a).x += (b).x, (a).y += (b).y, (a).z += (b).z)
#define vec3_subeq(a,b) ((a).x -= (b).x, (a).y -= (b).y, (a).z -= (b).z)
#define vec3_mlteq(a,b) ((a).x *= (b).x, (a).y *= (b).y, (a).z *= (b).z)
#define vec3_diveq(a,b) ((a).x /= (b).x, (a).y /= (b).y, (a).z /= (b).z)
#define vec3_addeq_sc(a,b) ((a).x += (b), (a).y += (b), (a).z += (b))
#define vec3_subeq_sc(a,b) ((a).x -= (b), (a).y -= (b), (a).z -= (b))
#define vec3_mlteq_sc(a,b) ((a).x *= (b), (a).y *= (b), (a).z *= (b))
#define vec3_diveq_sc(a,b) ((a).x /= (b), (a).y /= (b), (a).z /= (b))
#define vec3_sqlen(a)      ((a).x * (a).x + (a).y * (a).y + (a).z * (a).z)
#define vec3_sqdist(a,b)   (((a).x - (b).x) * ((a).x - (b).x) + ((a).y - (b).y) * ((a).y - (b).y) + ((a).z - (b).z) * ((a).z - (b).z))


#define UNIT_in      make_twocc('i','n')   /* = 1 inch*/
#define UNIT_point   make_twocc('p','t')   /* = 0.01 inch*/
#define UNIT_mil     make_twocc('m','l')   /* = 0.001 inch*/
#define UNIT_ft      make_twocc('f','t')   /* = 12 inch*/
#define UNIT_yd      make_twocc('y','d')   /* = 36 inch*/
#define UNIT_mile    make_twocc('M','L')   /* = 63360 inch*/

#define UNIT_km      make_twocc('k','m')   /* = 1000 meter*/
#define UNIT_m       make_twocc('m',' ')   /* = 1 meter*/
#define UNIT_cm      make_twocc('c','m')   /* = 0.01 meter*/
#define UNIT_mm      make_twocc('m','m')   /* = 0.001 meter*/
#define UNIT_um      make_twocc('u','m')   /* = 0.000001 meter*/
#define UNIT_nm      make_twocc('n','m')   /* = 0.000000001 meter*/
#define UNIT_pm      make_twocc('p','m')   /* = 0.000000000001 meter*/

#define UNIT_t       make_twocc('t',' ')   /* ton*/
#define UNIT_kg      make_twocc('k','g')   /* kg*/
#define UNIT_g       make_twocc('g',' ')   /* g*/
#define UNIT_mg      make_twocc('m','g')   /* mg*/

#define UNIT_px      make_twocc('p','x')



#define STD_IMG_RES      96
#define STD_IMG_RES_UNIT UNIT_in

#define IMGFMT_PROP_INTERLACED_

#define USE_IMGFMT2 1

#if USE_IMGFMT2

	#pragma pack(1)
	typedef struct tagIMGPLN {
		int16_t line_size;
		int16_t line_count;
		uint32_t  offset;
	} IMGPLN;
	#pragma pack()


	#pragma pack(1)
	typedef struct tagIMGFMT {
		fourcc_t  pixel_format;
		size2i16  size;
		IMGPLN   *planes; /**< DVEC type*/
		float32_t     fps; /**< frame per sec*/
		vec2u16   resolution;
		twocc_t   resolution_unit;
		uint16_t  props;
	} IMGFMT;

	typedef struct tagIMGBUF {
		IMGFMT  *fmt;
		uint8_t *data;
	} WA_IMG_BUF;

	typedef struct tagCONST_IMGBUF {
		IMGFMT  *fmt;
		const uint8_t *data;
	} WA_CONST_IMG_BUF;

	#pragma pack()


	#define IMGFMT_CLONE(pTrg, pSrc) DVEC_FREE(&((pTrg)->planes)), *(pTrg) = *(pSrc), (pTrg)->planes = NULL, DVEC_Assign((DVEC*)&((pTrg)->planes), pSrc->planes)
	#define IMGFMT_INIT_PACKED(target, w, h, pf) (target)->pixel_format = pf, (target)->size = make_size2i16(w,h), (target)->planes = (IMGPLN*)DVEC_InitWithLength(1, sizeof(IMGPLN)), (target)->planes[0].line_count = h, (target)->planes[0].offset = 0, (target)->fps = 30.0, (target)->resolution = make_vec2u16(96,96), (target)->resolution_unit = UNIT_in, (target)->props = 0
	#define IMGFMT_INIT_RGBA(target, w, h) IMGFMT_INIT_PACKED(target, w, h, make_fourcc('R','G','B','A')), (target)->planes[0].line_size = w * 4
	#define IMGFMT_INIT_RGB3(target, w, h) IMGFMT_INIT_PACKED(target, w, h, make_fourcc('R','G','B','3')), (target)->planes[0].line_size = ((w * 3 + 3) / 4) * 4
	#define IMGFMT_INIT_RGB IMGFMT_INIT_RGB3
	#define IMGFMT_INIT_YUYV(target, w, h) IMGFMT_INIT_PACKED(target, w, h, make_fourcc('Y','U','Y','V')), (target)->planes[0].line_size = ((w * 2 + 3) / 4) * 4
	#define IMGFMT_INIT_YUY2 IMGFMT_INIT_YUYV
	#define IMGFMT_INIT_UYVY(target, w, h) IMGFMT_INIT_PACKED(target, w, h, make_fourcc('U','Y','V','Y')), (target)->planes[0].line_size = ((w * 2 + 3) / 4) * 4
	#define IMGFMT_INIT_Y8(target, w, h) IMGFMT_INIT_PACKED(target, w, h, make_fourcc('Y','8',' ',' ')), (target)->planes[0].line_size = ((w + 3) / 4) * 4

	#define IMGFMT_GET_PLANE_COUNT(pImgFmt) DVEC_Length((pImgFmt)->planes)
	#define IMGFMT_SET_PLANE_COUNT(pImgFmt, plane_count) DVEC_SETLENGTH(IMGPLN, &((pImgFmt)->planes), plane_count)
	#define IMGFMT_APPEND_PLANE(pImgFmt, plane_count) DVEC_APPEND(IMGPLN, &((pImgFmt)->planes), plane_count)
	#define IMGFMT_FREE(pImgFmt) DVEC_FREE(&((pImgFmt)->planes))

	#define IMGFMT_PRIMARY_BPL(pImgFmt) (pImgFmt)->planes[0].line_size

#else /*#if USE_IMGFMT2*/

	#pragma pack(1)
	typedef struct tagIMGFMT {
		size2u16  size;
		vec2u16  resolution;
		float32_t     fps; /**< frame per sec*/
		int32_t       bytes_per_line;
		fourcc_t  pixel_format;
		twocc_t   resolution_unit;
		uint16_t  props;
	} IMGFMT;
	#pragma pack()

	#define IMGFMT_GET_PLANE_COUNT(pImgFmt) 1
	#define IMGFMT_PRIMARY_BPL(pImgFmt) pImgFmt->bytes_per_line
	#define IMGFMT_FREE(pImgFmt)

#endif /*#if USE_IMGFMT2*/

/* 새로운 이미지 포맷 구조체 테스트용. 아직 적용하지 말 것*/

typedef struct tagIMGPLN3 {
	size2i16  extent;
	int32_t   bytes_per_line;
	int8_t    bits_per_pixel;
	int8_t    bits_of_packing_unit;
	int8_t    pixel_per_packing_unit;
	int8_t    plane_offset;
	void *    data;
} IMGPLN3;


typedef struct tagIMGFMT3 {
	fourcc_t pixel_format;
	IMGPLN3 planes[4];
	int8_t plane_count;
} IMGFMT3;

/*==========================================================================
유틸리티 매크로 함수 정의
==========================================================================*/


#define SetBit_(dst, bit, value)  ((value) ? (dst) |= (1u << (bit)) : (dst) &= ~(1u << (bit)))
#define GetBit_(src, bit)         ((src >> bit) & 1u)

#define SafeDiff(bigger, smaller) ((~(smaller)) + bigger + 1)


#ifndef MIX_RATIO_R
	#define MIX_RATIO_R (30)
#endif
#ifndef MIX_RATIO_G
	#define MIX_RATIO_G (59)
#endif
#ifndef MIX_RATIO_B
	#define MIX_RATIO_B (11)
#endif

#define MIX_RGB(r,g,b) ((((int32_t)(r))*MIX_RATIO_R + ((int32_t)(g))*MIX_RATIO_G + ((int32_t)(b))*MIX_RATIO_B) / (MIX_RATIO_R + MIX_RATIO_G + MIX_RATIO_B))
#define MIX_BGR(b,g,r) ((((int32_t)(b))*MIX_RATIO_B + ((int32_t)(g))*MIX_RATIO_G + ((int32_t)(r))*MIX_RATIO_R) / (MIX_RATIO_R + MIX_RATIO_G + MIX_RATIO_B))


#ifdef __cplusplus
}
#else /*#ifdef __cplusplus*/
	#define WA_DEFAULT(x)
#endif /*#if __cplusplus*/
#endif /*wafl3H*/

/*C++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#ifdef __cplusplus
#ifndef wafl3HPP
#define wafl3HPP

#define WA_DEFAULT(x) = x

#endif /*wafl3HPP*/
#endif //#ifdef __cplusplus

