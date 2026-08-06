/**--------------------------------------------------------------------------
@file cstr_util.h
@brief cstr_util.c 파일의 헤더
@author ksg
*/
//---------------------------------------------------------------------------

#ifndef cstr_utilH
#define cstr_utilH
//---------------------------------------------------------------------------

#include "wafl3.h"

#include <string.h>

#if defined(__cplusplus)
extern "C" {
#endif //#if defined(__cplusplus)

extern fourcc_t str_to_fourcc(const char *str);
extern twocc_t str_to_twocc(const char *str);

extern int str_to_numeric(const char *str, void *dst, WA_TYPE type);
extern int str_to_int(const char *value);
extern uint64_t str_to_uint64(const char *value);
extern int str_to_array(const char **pSrc, const char delimiter, void *array, int length, WA_TYPE type);

/**--------------------------------------------------------------------------
@brief char 단위로 대문자를 소문자로 변경

@param [in ] c 문자
@return 소문자로 변경된 문자

@author ksg
*/
static inline char lower_char(const char c) {return (c <= 'Z' && c >= 'A') ? c + ('a'-'A') : c;}

/**--------------------------------------------------------------------------
@brief 대소문자를 가리지 않고 문자가 서로 일치하는지 검사

플랫폼 별로 해당 표준 함수의 이름이 다르기 때문에 이를 통일하기 위해 만들어짐
@param [in ] a 문자열
@param [in ] b 비교 대상 문자열
@retval 1 일치
@retval 0 불일치

@author ksg
*/
#if defined(_WIN32)
#  if _MSC_VER
     static inline int strisame(const char *a, const char *b) {return (_stricmp(a,b) == 0);}
#  else //#if _MSC_VER
     static inline int strisame(const char *a, const char *b) {return (stricmp(a,b) == 0);}
#  endif //#else #if _MSC_VER
#else
static inline int strisame(const char *a, const char *b) {return (strcasecmp(a,b) == 0);}
#endif

extern int __cdecl str_to_array_if_match(void *array, int length, const WA_TYPE type, const char delimiter, const char *key, const char *value, const char *base_name, ...);

#if defined(__cplusplus)
}
#else
#endif //#if defined(__cplusplus)
#endif
