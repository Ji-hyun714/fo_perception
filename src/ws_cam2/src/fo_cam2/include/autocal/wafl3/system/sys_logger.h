/**--------------------------------------------------------------------------
@file sys_logger.h
@brief sys_logger.c 파일의 헤더

모든 로그 메시지는 logmsg 함수를 호출하여 이루어지며 이 함수가 호출되면
지정된 로그 핸들러 함수로 해당 로그 메시지를 전달하는 방식으로 이루어진다.

C 표준에 따라 C89까지만 지원하는 컴파일러의 경우 전역 변수를 이용하도록
설계되어 있으므로 쓰레드 환경에서 사용하는 경우 문제가 발생할 수 있으므로
주의를 요한다.
C99를 지원하는 컴파일러에서는 가변인자 메크로를 사용하기 때문에 로그 핸들러
함수만 쓰레드 안전성이 보장되도록 설계하면 멀티 쓰레드 환경에서도 안전하게
사용할 수 있다.

매크로 함수들은 logmsg를 보다 편리하게 사용할 수 있도록 포장한 것으로 다음
의 4가지를 지원한다.

Log_(...) : 로그 메시지. 타입: ' ', ENABLE_LOG가 선언되어 있을 때에만 동작
Ln_(...)  : 로그 메시지. 메시지만 출력. 타입: '-', ENABLE_LOG가 선언되어 있을 때에만 동작
Err_(...) : 오류 메시지. 타입: '!', 항상 동작하며 DISABLE_ERR가 선언되어 있으면 동작하지 않음
Dbg_(...) : 디버그용 메시지. 타입: '*', ENABLE_DBG가 선언되어 있을 때에만 동작

Ln은 파일명, 함수명, 라인 번호 등을 표시하지 않고 메시지만 그대로 출력하기
위한 용도로 사용한다. 물론 로그 핸들러를 개발할 때 이를 지원하도록 설계해야 한다.

@author ksg
*/
/*---------------------------------------------------------------------------*/

#ifndef sys_logger3H
#define sys_logger3H
/*---------------------------------------------------------------------------*/
#include <stdarg.h>
#include "wafl3.h"

#ifdef __cplusplus
extern "C" {
#endif /*#ifdef __cplusplus*/

/**--------------------------------------------------------------------------
@brief 로그 핸들러 함수 타입 정의
@author ksg
*/
typedef void (*TLogHandler) (const ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line, const ansi_t *fmt, va_list args);

extern TLogHandler SetLogHandler(TLogHandler func);

#if defined(__STDC__)
# define PREDEF_STANDARD_C_1989            1
# if defined(__STDC_VERSION__)
//__stdc__
#  define PREDEF_STANDARD_C_1990           1
#  if (__STDC_VERSION__ >= 199409L)
#   define PREDEF_STANDARD_C_1994          1
#  endif
#  if (__STDC_VERSION__ >= 199901L)
#   define PREDEF_STANDARD_C_1999          1
//stdc_version >= 199x
#  endif
# endif
#elif defined(__BORLANDC__)
# define PREDEF_STANDARD_C_1999            1
#endif

//2022.04.20, kjkim add
# define PREDEF_STANDARD_C_1999            1

#ifdef PREDEF_STANDARD_C_1999
  extern void __cdecl logmsg(ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line, const ansi_t *fmt, ...);

  #if defined(ENABLE_LOG)
    #define Log_(...) logmsg(' ', __FILE__, __func__, __LINE__, __VA_ARGS__) /**< 로그 출력 (C99용)*/
    #define Ln_(...) logmsg('-', __FILE__, __func__, __LINE__, __VA_ARGS__)  /**< 로그(메시지만) 출력 (C99용)*/
  #else /*#if defined(ENABLE_LOG)*/
    #define Log_(...)
    #define Ln_(...)
  #endif /*#else defined(ENABLE_LOG)*/

  #if defined(ENABLE_DBG)
    #define Dbg_(...) logmsg('*', __FILE__, __func__, __LINE__, __VA_ARGS__)  /**< 디버그 출력 (C99용)*/
  #else /*#if defined(ENABLE_DBG)*/
    #define Dbg_(...)
  #endif /*#else defined(ENABLE_DBG)*/

  #if defined(DISABLE_ERR)
    #define Err_(...)
  #else /*#if defined(DISABLE_ERR)*/
    #define Err_(...) logmsg('!', __FILE__, __func__, __LINE__, __VA_ARGS__)  /**< 오류 출력 (C99용)*/
  #endif /*#else defined(DISABLE_ERR)*/

#else /*#ifdef PREDEF_STANDARD_C_1999*/
  //#error Old Style!!

  static inline void __cdecl NullFunc(const ansi_t *fmt, ...) {}

  typedef void (__cdecl *TLogMsgFunc)(const ansi_t *fmt, ...);
  extern TLogMsgFunc logmsg(ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line);

  #if defined(ENABLE_LOG)
    #define Log_ logmsg(' ', __FILE__, __func__, __LINE__)  /**< 로그 출력 (C89용)*/
    #define Ln_ logmsg('-', __FILE__, __func__, __LINE__)   /**< 로그(메시지만) 출력 (C89용)*/
  #else /*#if defined(ENABLE_LOG)*/
    #define Log_ NullFunc
    #define Ln_ NullFunc
  #endif /*#else defined(ENABLE_LOG)*/

  #if defined(ENABLE_DBG)
    #define Dbg_ logmsg('*', __FILE__, __func__, __LINE__)  /**< 디버그 출력 (C89용)*/
  #else /*#if defined(ENABLE_DBG)*/
    #define Dbg_ NullFunc
  #endif /*#else defined(ENABLE_DBG)*/

  #if defined(DISABLE_ERR)
    #define Err_ NullFunc
  #else /*#if defined(DISABLE_ERR)*/
    #define Err_ logmsg('!', __FILE__, __func__, __LINE__)   /**< 오류 출력 (C89용)*/
  #endif /*#if defined(DISABLE_ERR)*/

#define Log_lk NullFunc
#define Log_lkf NullFunc

#endif /*#else PREDEF_STANDARD_C_1999*/

#ifdef __cplusplus
}
#if 0
#if _WIN32
#include <windows.h>
extern void __cdecl logmsg(ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line, const wchar_t *fmt, ...);
#endif /*_WIN32*/
#endif

#endif /*#ifdef __cplusplus*/

#endif
