/**--------------------------------------------------------------------------
@file string_base.h
@brief string_base.c 파일의 헤더
@author ksg
*/
//---------------------------------------------------------------------------

#ifndef string_baseH
#define string_baseH
//---------------------------------------------------------------------------
#include <stdarg.h>
#include <string.h>
#include "system/mem_dynamic.h"


#if defined(__cplusplus)
extern "C" {
#endif // defined(__cplusplus)
/** 8bit 문자열 */
typedef void* STR8;
/** 16bit 문자열 */
typedef void* STR16;
/** Universal 문자열 */
typedef void* USTR;
/** @brief Codepage Descripter

상위 16bit는 문자 요소의 크기, 하위 16bit는 윈도의 코드 페이지 값으로 구성

@remark 코드페이지가 0인 경우 ACP(Active Code Page)임을 의미한다.
*/
typedef uint32_t CPD;
#define CPD_UTF8 0x0001FDE9 ///< UTF-8
#define CPD_UTF16 0x000204B0 ///< UTF-16 LE
#define CPD_UTF16LE 0x000204B0 ///< UTF-16 LE
#define CPD_UTF16BE 0x000204B1 ///< UTF-16 BE
#define CPD_ACP 0x00010000  ///< Active Code Page

extern ansi_t str8_null[1];

#if !defined(SUPPORT_STR16)
#ifdef _WIN32
	#define SUPPORT_STR16 1
#else
	#define SUPPORT_STR16 0
#endif //#ifdef _WIN32
#endif //!defined(SUPPORT_STR16)


#if (SUPPORT_STR16==1)
extern utf16_t str16_null[1];
#endif // if (SUPPORT_STR16==1)

#pragma pack(1)
/**
@brief 스트링 관리 헤더 구조체
*/
typedef struct {
#ifdef _WIN64
	uint32_t _padding64;          ///< 64bit OS의 경우 64bit 정렬을 위해 추가되는 데이터
#endif
	uint16_t codePage;            ///< 코드페이지
	uint16_t elemSize;            ///< 해더 사이즈
	int32_t refCnt;               ///< 참조 카운트
	int32_t length;               ///< 실제 데이터 사이즈
} STR_REC;
#pragma pack()

// STR8/STR16 Common Functions
extern void WA_API STR_MakeUnique(void **dst);


// STR8 Functions
//inline STR8 WA_API STR8_Init(void) {return NULL;}
//inline STR8 WA_API STR8_InitWith(STR8 src) {return (STR8)RCREC_clone(src); }
extern STR8 WA_API STR8_InitWithLength(int length);
//inline int  WA_API STR8_Free(STR8 *dst); {return DVEC_FREE(dst); }
//
//inline void WA_API STR8_Assign(STR8 *dst, STR8 src) {DVEC_ASSIGN(dst, src); }
//inline void WA_API STR8_MakeUnique(STR8 *dst) {STR_MakeUnique((void**)dst); }
//void WA_API STR8_SetLength(STR8 *dst, int length);
//inline bool32_t STR8_IsUnique(STR8 src) { return (src == NULL || ((STR_REC*)src - 1)->refCnt == 1); }
//
//int WA_API STR8_Length(STR8 src);
//ansi_t * WA_API STR8_CStr(STR8 src);
//STR_REC * WA_API STR8_GetRec(STR8 src) {return src == NULL ? NULL : (STR_REC*)src - 1; }

extern CPD check_code_page(const ansi_t *str, int length); // length를 -1로 지정하면 0를 만날 때 까지 검사, UTF-16은 검사 불가
extern USTR WA_API USTR_InitWithLength(int length, CPD cpd);

extern int WA_API USTR_Compare(USTR a, USTR b);

/**--------------------------------------------------------------------------
@brief USTR을 지정한 길이로 초기화(ANSI버전)

@param [in ] length 길이
@return 초기화된 USTR

@author ksg
*/
static inline USTR WA_API CSTR_InitWithLength(int length) {return USTR_InitWithLength(length, CPD_ACP);}
/**--------------------------------------------------------------------------
@brief USTR을 지정한 길이로 초기화(UTF-8버전)

@param [in ] length 길이
@return 초기화된 USTR

@author ksg
*/
static inline USTR WA_API UTF8_InitWithLength(int length) {return USTR_InitWithLength(length, CPD_UTF8);}
/**--------------------------------------------------------------------------
@brief USTR을 지정한 길이로 초기화(UTF-16버전)

@param [in ] length 길이
@return 초기화된 USTR

@author ksg
*/
static inline USTR WA_API UTF16_InitWithLength(int length) {return USTR_InitWithLength(length, CPD_UTF16);}


extern USTR WA_API USTR_InitWithStrLen(const void* s, int len, CPD cpd);
/**--------------------------------------------------------------------------
@brief ANSI문자열로부터 USTR 복사 초기화(길이 지정)

@param [in ] s   원본 문자열
@param [in ] len 원본 문자열의 길이
@return 원본 문자열을 복사한 새로운 USTR

@author ksg
*/
static inline USTR WA_API USTR_InitWithCStrLen(const ansi_t* s, int len) {return USTR_InitWithStrLen(s, len, CPD_ACP);}
extern int  __cdecl USTR_AppendFmtV(USTR *s, CPD cpd, const void *fmt, va_list a);
extern USTR __cdecl USTR_InitWithCStrFormat(const ansi_t *fmt, ...);
extern int  __cdecl USTR_AppendCStrFormat(USTR *s, const ansi_t *fmt, ...);
/**--------------------------------------------------------------------------
@brief ANSI문자열로부터 USTR 복사 초기화(길이 미지정)

@param [in ] s   원본 문자열
@return 원본 문자열을 복사한 새로운 USTR

@author ksg
*/
static inline USTR WA_API USTR_InitWithCStr(const ansi_t* s) {return USTR_InitWithCStrLen(s, strlen(s));}
#if _WIN32
/**--------------------------------------------------------------------------
@brief UTF-16문자열로부터 USTR 복사 초기화(길이 지정)

@param [in ] s   원본 문자열
@param [in ] len 원본 문자열의 길이
@return 원본 문자열을 복사한 새로운 USTR

@author ksg
*/
static inline USTR WA_API USTR_InitWithWStrLen(const utf16_t* s, int len) {return USTR_InitWithStrLen(s, len, CPD_UTF16);}
extern USTR __cdecl USTR_InitWithWStrFormat(const utf16_t *fmt, ...);
extern int  __cdecl USTR_AppendWStrFormat(USTR *s, const utf16_t *fmt, ...);
/**--------------------------------------------------------------------------
@brief UTF-16문자열로부터 USTR 복사 초기화(길이 미지정)

@param [in ] s   원본 문자열
@param [in ] len 원본 문자열의 길이
@return 원본 문자열을 복사한 새로운 USTR

@author ksg
*/
static inline USTR WA_API USTR_InitWithWStr(const utf16_t* s) {return USTR_InitWithWStrLen(s, wcslen(s));}
extern void WA_API USTR_ChangeCodePage(USTR *s, CPD cpd);
#endif //#if _WIN32

extern CPD WA_API USTR_GetCPD(const USTR s);
extern void WA_API USTR_SetCPD(USTR *s, CPD cpd);
/**--------------------------------------------------------------------------
@brief USTR 소멸

@param [out] s 소멸시킬 USTR의 주소

@author ksg
*/
static inline void WA_API USTR_Free(USTR *s) {rcm_free(s);}
/**--------------------------------------------------------------------------
@brief USTR 대입

USTR 대입은 문자열에 대한 메모리 복사를 수행하지 않고 참조 카운트만 증가
시켜 동일한 메모리를 공유하도록 한다.

@param [out] dst 대상 문자열
@param [in ] src 원본(공유할) 문자열

@author ksg
*/
static inline void WA_API USTR_Assign(USTR *dst, USTR src) {rcm_assign(dst, src);}

static inline void WA_API USTR_AssignCStrLen(USTR *dst, const void* s, int len, CPD cpd) {USTR_Free(dst); *dst = USTR_InitWithStrLen(s, len, cpd);}

/**--------------------------------------------------------------------------
@brief USTR 병합

대상 문자열에 지정한 문자열을 더한다.
@param [in,out] dst 병합할 대상
@param [in ]    src 원본 문자열

@author ksg
*/
static inline void WA_API USTR_Append(USTR *dst, USTR src) {rcm_append(dst, src);}
/**--------------------------------------------------------------------------
@brief USTR의 길이 조회

Terminal Zero를 제외한 문자열의 논리적인 길이를 반환한다.

@remark 문자열의 길이(length)는 UTF-16 문자열의 경우 문자열이 차지하는
메모리의 크기(size, in bytes)와 일치하지 않을 수 있다.
@param [in ] s 대상 문자열
@return 문자열의 길이

@author ksg
*/
static inline int  WA_API USTR_Length(USTR s) {return rcm_get_len(s);}
//static inline void WA_API USTR_MakeUnique(USTR *s) {rcm_unique(s);}
extern USTR WA_API USTR_SetLength(USTR *dst, int length, CPD cpd);

extern int __cdecl USTR_AppendCStrFormat(USTR *s, const ansi_t *fmt, ...);

#ifdef _WIN32
extern void USTR_GetErrorString(USTR *s, uint32_t err_code);
#endif

extern uint16_t USTR_LastCharacter(const USTR s);
extern USTR USTR_LoadFromFILE(USTR *dst, FILE *fp);
extern int USTR_SaveToFILE(FILE *fp, const USTR s, bool32_t includeBOM);

extern USTR WA_API USTR_Dequoted(USTR *dst, const USTR src, CPD cpd);

#if defined(__cplusplus)
}

#if defined(__BORLANDC__)
#include <system.hpp>
#endif //#if defined(__BORLANDC__)

/**--------------------------------------------------------------------------
@brief USTR을 C++에서 편리하게 사용하기 위한 Wrapper 클래스

USTR은 C에서 사용할 목적으로 개발되었기 때문에 동작 원리를 잘 이해하지
못 한 상태에서 사용하면 메모리 누수나 충돌의 가능성이 존재한다.

UString은 USTR에 대한 C++용 Wrapper클래스로 생성자와 소멸자를 이용하여
USTR을 안전하고 편리하게 사용할 수 있도록 도와준다.

모든 멤버 함수는 내부적으로 C 함수들을 단순 호출하는 inline으로 구현되어
있으므로 코드 크기 및 성능상의 손해를 보지 않고 안전하게 사용할 수 있다.
*/
class UString
{
private:
	union {
		utf8_t  *u8;
		ansi_t  *c; ///< ANSI 형식으로 접근할 경우
		utf16_t *w; ///< UTF-16 문자열 형식으로 접근할 경우
		void    *v; ///< USTR 형식으로 접근할 경우
	} data; ///< USTR 데이터
protected:
public:
	/// 소멸자
	inline ~UString() {USTR_Free(&data.v);}
	/// 기본 생성자
	inline UString() {data.v=0;}
	/// USTR로부터 복사 생성자
	inline UString(const USTR &s) {data.v=rcm_clone(s);}
	/// 복사 생성자
	inline UString(const UString &s) {data.v=rcm_clone(s.data.v);}
	/// ANSI또는 UTF-8문자열로부터 복사 생성자 (길이 미지정)
	inline UString(const ansi_t *s) {data.v=USTR_InitWithStrLen(s, strlen(s), CPD_ACP);}
	/// ANSI또는 UTF-8문자열로부터 복사 생성자 (길이 지정)
	inline UString(const ansi_t *s, int length) {data.v=USTR_InitWithCStrLen(s, length);}
	/// ANSI또는 UTF-8문자열로부터 복사 생성자 (길이 미지정)
	inline UString(CPD cpd, const ansi_t *s) {data.v=USTR_InitWithStrLen(s, strlen(s), cpd);}
	/// ANSI또는 UTF-8문자열로부터 복사 생성자 (길이 지정)
	inline UString(CPD cpd, const ansi_t *s, int length) {data.v=USTR_InitWithStrLen(s, length, cpd);}
#if (SUPPORT_STR16==1)
	/// UTF-16문자열로부터 복사 생성자 (길이 미지정)
	inline UString(const utf16_t *s) {data.v=USTR_InitWithStrLen(s, wcslen(s), CPD_UTF16);}
	/// UTF-16문자열로부터 복사 생성자 (길이 지정)
	inline UString(const utf16_t *s, int length) {data.v=USTR_InitWithWStrLen(s, length);}
#endif // if (SUPPORT_STR16==1)
#if defined(__BORLANDC__) && defined(SystemHPP)
	/// AnsiString으로부터 복사 생성자 (C++ Builder 전용)
	inline UString(const AnsiString &s) {data.v = *((void**)(&s)); if (data.v) rcm_inc_ref(data.v);}
	/// UTF8String으로부터 복사 생성자 (C++ Builder 전용)
	inline UString(const UTF8String &s) {data.v = *((void**)(&s)); if (data.v) rcm_inc_ref(data.v);}
	/// UnicodeString으로부터 복사 생성자 (C++ Builder 전용)
	inline UString(const UnicodeString &s) {data.v = *((void**)(&s)); if (data.v) rcm_inc_ref(data.v);}
#endif //#if defined(__BORLANDC__) && defined(SystemHPP)

	/// 문자열 데이터를 단독 점유하고 있는지 검사
	inline bool32_t IsUnique(void) const {return rcm_is_unique(data.v);}
	/// 비어있는 문자열인지 검사
	inline bool32_t IsEmpty(void) {return data.v == NULL;}
	/// 문자열 데이터를 단독으로 점유하도록 변경
	inline void Unique(void) {rcm_unique(&data.v);}

	/// 대입 연산자 (메모리 공유)
	inline UString & WA_API operator=(const UString &s) {rcm_assign(&(data.v), s.data.v); return *this;}
	/// 문자열 병합 연산자
	inline UString & WA_API operator+=(const UString &s) {rcm_append(&(data.v), s.data.v); return *this;}
	/// 문자열 결합 연산자
	inline UString WA_API operator+(const UString &s) {UString r; r.data.v = rcm_add(data.v, s.data.v); return r;}

	inline bool32_t WA_API operator==(const UString &s) const {return USTR_Compare(data.v, s.data.v) == 0;}

	ansi_t const & WA_API operator[](const int idx) const {return data.c[idx];}
	ansi_t & WA_API operator[](const int idx) {return data.c[idx];}

	/// 문자열 길이 변경(CPD 미지정, CPD_ACP를 기본 사용)
	inline void WA_API SetLength(int length) {rcm_set_len_ctx(&(data.v), length, CPD_ACP);}
	/// 문자열 길이 변경(CPD 지정)
	inline void WA_API SetLength(int length, CPD cpd) {rcm_set_len_ctx(&(data.v), length, cpd);}
	/// 문자열 길이 조회
	inline int WA_API Length(void) const {return rcm_get_len(data.v);}

	/// 문자열에 대한 char* 주소 획득 (조회용) @remark 코드페이지 전환은 수행하지 않는다.
	inline const ansi_t * WA_API c_str(void) const {return data.c;}
	/// 문자열에 대한 char* 주소 획득 (기록용) @remark 코드페이지 전환은 수행하지 않는다.
	inline ansi_t * WA_API c_str(void) {return data.c;}
#if (SUPPORT_STR16==1)
	/// 문자열에 대한 wchar_t* 주소 획득 (조회용) @remark 코드페이지 전환은 수행하지 않는다.
	inline const utf16_t * WA_API w_str(void) const {return data.w;}
	/// 문자열에 대한 wchar_t* 주소 획득 (기록용) @remark 코드페이지 전환은 수행하지 않는다.
	inline utf16_t * WA_API w_str(void) {return data.w;}
#endif // if (SUPPORT_STR16==1)

	/// 파일로부터 문자를 읽어들임. BOM을 체크하여 적당한 CodePage를 자동 설정
	inline int LoadFromFILE(FILE *fp) {return USTR_Length(USTR_LoadFromFILE(&data.v, fp));}
	/// 코드페이지(CPD)를 값을 단순 대입
	inline void SetCPD(CPD cpd) {USTR_SetCPD(&data.v, cpd);}

#if (SUPPORT_STR16==1 && defined(_WIN32))
	/// 코드페이지(CPD) 변경 (코드페이지 전환 수행)
	inline UString& ChangeCodePage(CPD cpd) {USTR_ChangeCodePage(&data.v, cpd); return *this;}
	#ifdef SystemHPP
		/// UnicodeString으로 캐스팅(C++ Builder전용)
		inline operator UnicodeString() {USTR_ChangeCodePage(&data.v, CPD_UTF16); rcm_inc_ref(data.v); return *(UnicodeString*)this;}
	#endif //#ifdef SystemHPP
#endif
	#ifdef SystemHPP
	/// UTF8String으로 캐스팅(C++ Builder전용)
	inline operator UTF8String() {USTR_ChangeCodePage(&data.v, CPD_UTF8); rcm_inc_ref(data.v); return *(UTF8String*)this;}
	/// AnsiString으로 캐스팅(C++ Builder전용)
	inline operator AnsiString() {USTR_ChangeCodePage(&data.v, CPD_ACP); rcm_inc_ref(data.v); return *(AnsiString*)this;}
	#endif //#ifdef SystemHPP

	/// 마지막 문자 반환
	inline uint16_t LastCharacter(void) const {return USTR_LastCharacter(data.v);}
	/// printf 형식으로 문자열을 채움
	/// @return 출력된 문자의 수
	int __cdecl printf(const ansi_t *fmt, ...) {
		int r;
		rcm_free(&data.v);
		va_list ap;
		va_start(ap, fmt);
			r = USTR_AppendFmtV(&data.v, CPD_ACP, fmt, ap);
		va_end(ap);
		return r;
	}
	/// printf 형식으로 문자열을 채움
	/// @return 출력된 문자
	UString __cdecl sprintf(const ansi_t *fmt, ...) {
		rcm_free(&data.v);
		va_list ap;
		va_start(ap, fmt);
			USTR_AppendFmtV(&data.v, CPD_ACP, fmt, ap);
		va_end(ap);
		return *this;
	}

	inline USTR u_str(void) {return data.v;}
};

inline UString UStringDequoted(const UString src) {
	UString r;
	USTR_Dequoted((USTR*)&r, *((USTR*)&src), CPD_UTF8);
	return r;
}

#ifdef _WIN32
#define u8_(x) UString(L##x).ChangeCodePage(CPD_UTF8).c_str()
#else //#ifdef _WIN32
#define u8_(x) x
#endif //#else ifdef _WIN32


#endif // defined(__cplusplus)

#endif
