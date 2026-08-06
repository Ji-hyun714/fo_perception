/**--------------------------------------------------------------------------
@file sys_logger.c
@brief Log 관련 함수 모음
@author ksg
*/
//---------------------------------------------------------------------------
#include <stdio.h>
#if (defined(CCS) || defined(__BORLANDC__))
#pragma hdrstop
#endif //(defined(CCS) || defined(__BORLANDC__))


#include "sys_logger.h"
#include <cstring>
#if defined(_WIN32)
#include <windows.h>
#endif //#if defined(_WIN32)
//---------------------------------------------------------------------------
#if defined(__BORLANDC__)
#pragma package(smart_init)
#endif //#if defined(__BORLANDC__)

#define MAX_LOGMSG_BUF 512

#ifdef __cplusplus
extern "C" {       // the mangling of 'va_list' has changed in GCC 4.4 경고가 나지 않게 하려면 extern "C"를 해 줘야 함
#endif //#ifdef __cplusplus

#if defined(_WIN32)
/**--------------------------------------------------------------------------
WIN32 환경에서의 기본 로그 핸들러

@param type 메시지 종류
@param unit 소스 파일 이름
@param func 함수 이름
@param line 라인 번호
@param fmt 메시지 포맷
@param args 가변인자

*/

void defaultLogHandler(const ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line, const ansi_t *fmt, va_list args)
{
  ansi_t msg[MAX_LOGMSG_BUF];
  ansi_t out[MAX_LOGMSG_BUF * 2];
  vsprintf_s(msg, MAX_LOGMSG_BUF, fmt, args);
  if (type == '-') {
    OutputDebugStringA(msg);
  }
  sprintf_s(out, MAX_LOGMSG_BUF, "%c %s [%s][%s:%d]", type, msg, func, unit, line);
  OutputDebugStringA(out);
}
#elif defined(__linux__)
/**--------------------------------------------------------------------------
PISIX 환경에서의 기본 로그 핸들러

@param type 메시지 종류
@param unit 소스 파일 이름
@param func 함수 이름
@param line 라인 번호
@param fmt 메시지 포맷
@param args 가변인자

*/
//2016.02.16 kjkim add __cplusplus define
void defaultLogHandler (const ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line, const ansi_t *fmt, va_list args)
{
  ansi_t msg[MAX_LOGMSG_BUF];
  if (type == '-') {
    vfprintf(stdout, fmt, args);
    return;
  }
//  vsprintf_s(msg, MAX_LOGMSG_BUF, fmt, args);
  vsprintf(msg, fmt, args);
  fprintf((type == '!') ? stderr : stdout, "%c %s [%s][%s:%d]\n", type, msg, func, unit, line);
}
#else //#elif defined(__linux__)
void defaultLogHandler(const ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line, const ansi_t *fmt, va_list args)
{
	ansi_t msg[MAX_LOGMSG_BUF];
	if (type == '-') {
		vfprintf(stdout, fmt, args);
		return;
	}

	//  vsprintf_s(msg, MAX_LOGMSG_BUF, fmt, args);
	vsprintf(msg, fmt, args);
//	fprintf((type == '!') ? stderr : stdout, "%c %s [%s][%s:%d]\n", type, msg, func, unit, line);
}

#endif //#else #elif defined(__linux__)

//2016.02.16 kjkim add __cplusplus define
#ifdef __cplusplus
}
#endif

static TLogHandler gpLogHandler = defaultLogHandler;

/**--------------------------------------------------------------------------
@brief 로그 핸들러 함수를 등록한다.

func 인자를 NULL로 설정할 경우 이전 로그 핸들러를 그대로 두게 되므로
Get함수 대신 사용할 수 있다.

@param [in ] func 로그 핸들러 함수
@return 이전 로그 핸들러

@author ksg
*/
TLogHandler SetLogHandler(TLogHandler func)
{
  TLogHandler old = gpLogHandler;
  if (func) {
    gpLogHandler = func;
  }
  return old;
}

//#if _WIN32
//void __cdecl logmsgw(ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line, const wchar_t *fmt, ...)
//{
//  wchar_t msg[512]={0,};
//  ansi_t utf8[512];
//  int32_t l;
//  va_list va;
//  va_start(va, fmt);
//    l = vswprintf_s(msg, 512, fmt, va);
//    WideCharToMultiByte(CP_UTF8, 0, msg, -1, utf8, 512-1, NULL, NULL);
//    gpLogHandler(type, unit, func, line, utf8, NULL);
//  va_end(va);
//}
//#endif //#if _WIN32


#if PREDEF_STANDARD_C_1999
  /**--------------------------------------------------------------------------
  @brief C99 표준이 지원될 경우 사용되는 가변인자 처리기

  @param [in ] type 메시지 종류
  @param [in ] unit 소스 파일 이름
  @param [in ] func 함수 이름
  @param [in ] line 라인 번호
  @param [in ] fmt  메시지 포맷
  @param [in ] ...  가변인자

  @author ksg
  */
  void __cdecl logmsg(ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line, const ansi_t *fmt, ...)
  {
    va_list va;
    va_start(va, fmt);
      gpLogHandler(type, unit, func, line, fmt, va);
    va_end(va);
  }
#else //#if PREDEF_STANDARD_C_1999

  static ansi_t log_type_;
  static const ansi_t *log_unit_;
  static const ansi_t *log_func_;
  static int32_t log_line_;

  /**--------------------------------------------------------------------------
  @brief C99 표준이 지원되지 않는 경우 사용되는 가변인자 처리기

  @param [in ] fmt  메시지 포맷
  @param [in ] ...  가변인자

  @author ksg
  */
  static void __cdecl internal_logmsg(const ansi_t *fmt, ...)
  {
    va_list va;
    va_start(va, fmt);
      gpLogHandler(log_type_, log_unit_, log_func_, log_line_, fmt, va);
    va_end(va);
  }
  /**--------------------------------------------------------------------------
  @brief C99 표준이 지원되지 않는 경우 사용되는 로그 정보 등록 함수
  
  C99 표준이 지원되지 않는 경우 type, unit, func, line 정보를 전역 변수에
  저장하는 방식을 사용한다.

  @param [in ] type 메시지 종류
  @param [in ] unit 소스 파일 이름
  @param [in ] func 함수 이름
  @param [in ] line 라인 번호
  @return internal_logmsg 함수의 포인터
  
  @author ksg
  */
  TLogMsgFunc logmsg(ansi_t type, const ansi_t *unit, const ansi_t *func, int32_t line)
  {
    log_type_ = type;
    log_unit_ = unit;
    log_func_ = func;
    log_line_ = line;
    return internal_logmsg;
  }


#endif //else PREDEF_STANDARD_C_1999
