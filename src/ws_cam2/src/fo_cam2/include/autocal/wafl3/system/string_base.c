/**--------------------------------------------------------------------------
@file  string_base.c
@brief 동적 문자열 함수 및 클래스 구현
*/
//---------------------------------------------------------------------------

#if (defined(CCS) || defined(__BORLANDC__))
#pragma hdrstop
#endif //(defined(CCS) || defined(__BORLANDC__))

#include "string_base.h"
#include "system/sys_logger.h"
//---------------------------------------------------------------------------
#if defined(__BORLANDC__)
#pragma package(smart_init)
#endif //defined(__BORLANDC__)

char str8_null[] = ""; ///< 널 문자열(ANSI, UTF-8용) @details c_str() 함수 호출 시 데이터가 NULL이면 이 배열을 전달 @todo 아직 연동 안됨
#if _WIN32
wchar_t str16_null[] = L""; ///< 널 문자열(UTF-16용) @details w_str() 함수 호출 시 데이터가 NULL이면 이 배열을 전달 @todo 아직 연동 안됨
#endif //#if _WIN32

int  WA_API STR8_Free(STR8 *dst); ///< STR8 문자열 소멸 @return: Free후 refCnt값, 0이면 메모리 해제됨을 의미 @remark 사용되지 않음

/**--------------------------------------------------------------------------
@brief RCM 레코드의 context를 CPD로 접근하기 위한 구조체. 내부에서만 사용
*/
#pragma pack(1)
typedef struct {
	CPD      cpd;               ///< CPD
	int32_t refCnt;               ///< 참조 카운트
	int32_t length;               ///< 실제 데이터 사이즈
} STR_CPD;
#pragma pack()

/**--------------------------------------------------------------------------
@brief C문자열의 내용으로부터 코드페이지를 인식한다.

@param [in ] str    입력 문자열
@param [in ] length 입력 문자열의 길이
@return 인식된 코드페이지 (CPD 타입)

@author ksg
*/
CPD check_code_page(const char *str, int length)
{
  int i, l;
  const uint8_t *p = (const uint8_t*)str;
  CPD cpd = CPD_UTF8;
  l = (length == -1) ? strlen(str) : length;

  for (i = 0; i < l; i++) {
    if (p[i] == 0) {
      return (length == -1 || i % 2 == 0) ? cpd : CPD_UTF16;
    }
    if (p[i] < 128) {
      continue;
    }
    if ((p[i] & 0xE0) == 0xC0 && (p[i+1] & 0xC0) == 0x80) {
      i++;
      continue;
    }
    if ((p[i] & 0xF0) == 0xE0 && (p[i+1] & 0xC0) == 0x80 && (p[i+2] & 0xC0) == 0x80) {
      i+=2;
      continue;
    }
    if ((p[i] & 0xF8) == 0xF0 && (p[i+1] & 0xC0) == 0x80 && (p[i+2] & 0xC0) == 0x80 && (p[i+3] & 0xC0) == 0x80) {
      i+=3;
      continue;
    }
    cpd = CPD_ACP;
    break;
  }
  return cpd;
}

/**--------------------------------------------------------------------------
@brief 텍스트 파일을 USTR 타입으로 읽어들인다.

텍스트파일의 BOM을 체크하여 정확한 코드페이지로 읽어들인다.

@param [out] s 결과를 저장할 USTR타입의 문자열의 포인터
@param [in ] fp 파일 포인터
@return s의 인스턴스 값을 그대로 반환

@author ksg
*/
USTR USTR_LoadFromFILE(USTR *s, FILE *fp)
{
  uint8_t bom[3] = {0,0,0};
  int read;
  int size;
  STR_CPD *rec;

  fseek(fp, 0, SEEK_END);
  size = ftell(fp);
  fseek(fp, 0, SEEK_SET);

  read = fread(bom, 1, 3, fp);
  // BOM 체크
  if (bom[0] == 0xff && bom[1] == 0xfe) {
    // UTF-16 Little Endian
    size -= 2;
    USTR_SetLength(s, (size) >> 1, CPD_UTF16);
    fseek(fp, 2, SEEK_SET);
  } else if (bom[0] == 0xef && bom[1] == 0xbb && bom[2] == 0xbf) {
    // UTF-8
    size -= 3;
    USTR_SetLength(s, size, CPD_UTF8);
  } else {
    USTR_SetLength(s, size, CPD_ACP);
    fseek(fp, 0, SEEK_SET);
  }

  if (size == 0) {
    return *s;
  }

  read = fread(*s, 1, size, fp);
  if (read != size) {
    Err_("Cannot read data");
    return *s;
  }

  rec = ((STR_CPD*)(*s) - 1);
  if (rec->cpd == CPD_ACP) {
    rec->cpd = check_code_page((char*)(*s), size);
    rec->length /= (rec->cpd >> 16); // 길이 조정
  }
  return *s;
}
/**--------------------------------------------------------------------------
@brief USTR 문자열을 파일로 저장

사용되지 않기 때문에 아직 구현되지 않음

@param [in ] fp         파일 포인터
@param [in ] s          저장할 문자열
@param [in ] includeBOM BOM을 포함할 것인지 여부
@return 저장된 데이터의 양 (byte)

@author ksg
*/
int USTR_SaveToFILE(FILE *fp, const USTR s, bool includeBOM)
{
  return 1;
}


/**--------------------------------------------------------------------------
@brief USTR을 주어진 CPD와 길이로 초기화

@param [in ] length 길이
@param [in ] cpd    CPD
@return 초기화된 USTR

@author ksg
*/
USTR WA_API USTR_InitWithLength(int length, CPD cpd)
{
	return (length > 0 && (cpd >> 16)) ? (USTR)rcm_init_len_ctx(length, cpd) : NULL;
}
/**--------------------------------------------------------------------------
@brief USTR을 주어진 CPD의 문자열로 초기화(길이 지정)

@param [in ] s   문자열
@param [in ] len 문자열의 길이
@param [in ] cpd 문자열의 CPD
@return 복사 생성된 USTR

@author ksg
*/
USTR WA_API USTR_InitWithStrLen(const void* s, int len, CPD cpd)
{
	USTR r;
	if ((r=USTR_InitWithLength(len, cpd)) != NULL)
		copyMemory(r, (void*)s, len * (cpd >> 16));
	return r;
}
/**--------------------------------------------------------------------------
@brief USTR의 길이를 변경

USTR의 기존 CPD와 새로 지정한 CPD가 일치하지 않으면 실패한다.
@param [in,out] dst    대상 USTR
@param [in ]    length 새로운 길이
@param [in ]    cpd    새로운 CPD
@return 성공시 새로운 문자열 길이, 실패시 -1

@author ksg
*/
USTR WA_API USTR_SetLength(USTR *dst, int length, CPD cpd)
{
  STR_CPD *rec;
  if (*dst == NULL) {
    rcm_set_len_ctx(dst, length, cpd);
    return *dst;
  }
  rec = ((STR_CPD *)(*dst))-1;
  if (rec->cpd == cpd) {
    rcm_set_len_ctx(dst, length, cpd);
  } else {
    Err_("The CPDs are not matching: dst=%08x, cpd=%08x", rec->cpd, cpd);
  }
  return *dst;
}

/**--------------------------------------------------------------------------
@brief 두 USTR 타입의 문자열을 비교함. strcmp에 대응하는 함수

@param [in ] a 문자열 A
@param [in ] b 문자열 B
@return strcmp의 결과 값

@author ksg
*/
int WA_API USTR_Compare(USTR a, USTR b)
{
  STR_CPD *rec_a = NULL, *rec_b = NULL;
  if (a == NULL && b == NULL) {
    return 0;
  }
  if (a == NULL) {
    return -1;
  }
  if (b == NULL) {
    return 1;
  }

  rec_a = ((STR_CPD*)a)-1;
  rec_b = ((STR_CPD*)b)-1;
  if (rec_a->cpd != rec_b->cpd) {
    // 오류상황. 비교 불가
  }
  if ((rec_a->cpd >> 16) == 1) {
    return strcmp((char*)a, (char*)b);
  }
  // 오류상황
  return 0;
}

/**--------------------------------------------------------------------------
@brief 기존 USTR에 새롭게 printf 방식으로 생성한 문자열을 더한다.

@param [in,out] s   대상 USTR
@param [in ]    cpd 추가될 문자열의 CPD
@param [in ]    fmt 포멧 문자열
@param [in ]    a   가변 인수 리스트
@return 추가된 문자열의 길이. CPD가 일치하지 않을 경우 -1을 반환

@author ksg
*/
int __cdecl USTR_AppendFmtV(USTR *s, CPD cpd, const void *fmt, va_list a)
{
  int l0 = 0, l1;
  STR_CPD *rec = NULL;
  l1 =
#if _WIN32
    (cpd == CPD_UTF16) ? vsnwprintf(NULL, 0, (wchar_t*)fmt, a) :
#endif //#if _WIN32
    vsnprintf(NULL, 0, (char*)fmt, a);
  if (l1 > 0) {
    if (*s == NULL) {
      rcm_set_len_ctx(s, l1, cpd);
    } else {
      rec = ((STR_CPD*)(*s))-1;
      if (rec->cpd == cpd) {
        l0 = rec->length;
        rcm_set_len_ctx(s, l0 + l1, cpd);
      } else {
        return -1;
      }
    }
    return
#if _WIN32
      (cpd == CPD_UTF16) ? vsnwprintf((wchar_t*)*s + l0, l1, (wchar_t*)fmt, a) :
#endif //#if _WIN32
      vsnprintf((char*)*s + l0, l1+1, (char*)fmt, a);   //vsnprintf 버전에 따라 l1만 보내도 되는경우도 있고 l1+1을 해야 하는 경우도 있다.
  }
  return 0;
}
/**--------------------------------------------------------------------------
@brief printf 방식으로 USTR을 초기화(ANSI, UTF-8버전)

@param [in ] fmt 포멧 문자열
@param [in ] ... 가변 인자
@return 초기화 된 USTR

@author ksg
*/
USTR __cdecl USTR_InitWithCStrFormat(const char *fmt, ...)
{
  USTR s = 0;
  va_list ap;
  va_start(ap, fmt);
    USTR_AppendFmtV(&s, CPD_ACP, fmt, ap);
  va_end(ap);
  return s;
}


/**--------------------------------------------------------------------------
@brief 기존 USTR에 새롭게 printf방식으로 문자열을 추가(ANSI, UTF-8버전)

@param [in,out] s   대상 USTR
@param [in ] fmt 포멧 문자열
@param [in ] ... 가변 인자
@return 추가된 문자열의 길이

@author ksg
*/
int __cdecl USTR_AppendCStrFormat(USTR *s, const char *fmt, ...)
{
  int r;
  va_list ap;
  va_start(ap, fmt);
    r = USTR_AppendFmtV(s, CPD_ACP, fmt, ap);
  va_end(ap);
  return r;
}


#ifdef _WIN32
/**--------------------------------------------------------------------------
@brief 윈도 오류 코드를 오류 메시지로 변환

@param [out] s        대상 USTR
@param [in ] err_code 윈도 오류 코드

@author ksg
*/
void USTR_GetErrorString(USTR *s, uint32_t err_code)
{
  wchar_t *lpMsgBuf = NULL;
  int l = 0;

  l = FormatMessageW(
      FORMAT_MESSAGE_ALLOCATE_BUFFER |
      FORMAT_MESSAGE_FROM_SYSTEM |
      FORMAT_MESSAGE_IGNORE_INSERTS,
      NULL,
      err_code,
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
      (wchar_t*)&lpMsgBuf,
      0,
      NULL
  );

  USTR_SetLength(s, l, CPD_UTF16);
  if (l > 0) {
    memcpy(*s, lpMsgBuf, l * sizeof(wchar_t));
    LocalFree(lpMsgBuf);
  }
}
#endif

/**--------------------------------------------------------------------------
@brief USTR 타입의 문자열의 코드페이지 정보를 반환

@param [in ] s 입력 문자열
@return 코드페이지 정보

@author ksg
*/
CPD WA_API USTR_GetCPD(const USTR s)
{
  return (s == NULL) ? CPD_ACP : ((STR_CPD*)(s) - 1)->cpd;
}

/**--------------------------------------------------------------------------
@brief USTR 타입의 문자열의 코드페이지 정보를 강제로 바꿈

데이터에 대한 코드페이지 변환은 수행하지 않는다.

@param [in ] s   바꿀 문자열
@param [in ] cpd 새로운 코드페이지 정보

@author ksg
*/
void WA_API USTR_SetCPD(USTR *s, CPD cpd)
{
  if (*s == NULL) {
    return;
  }
  ((STR_CPD*)(*s) - 1)->cpd = cpd;
}
/**--------------------------------------------------------------------------
@brief 마지막 문자를 반환

@param [in ] s   문자열
@return 마지막 문자
@author ksg
*/
uint16_t USTR_LastCharacter(const USTR s)
{
  int l = DVEC_Length(s);
  if (l) {
    int elemSize = DVEC_ElemSize(s);
    switch (elemSize) {
      case 1: {
        return (uint16_t)(((char*)s)[l-1]);
      }
      case 2: {
        return (uint16_t)(((uint16_t*)s)[l-1]);
      }
      default:
        break;
    }
  }
  return 0;
}

char const * skip_blanks(char const *p) {
  for (;*p <= ' '; ++p) {
  }
  return p;
}

USTR WA_API USTR_Dequoted(USTR *dst, USTR const src, CPD cpd)
{
  char *pD;
  char const *pS, *pS_end;

  pS = (char const *)src;
  pS_end = pS + USTR_Length(src);

  pS = skip_blanks(pS);

  if (*pS != '"') {
    USTR_Assign(dst, src);
    return *dst;
  }

  USTR_SetLength(dst, USTR_Length(src), cpd);
  pD = (char*)(*dst);
  for (++pS; pS != pS_end; ++pS) {
    if (*pS == '"') {
      if (pS[1] == '"') {
        ++pS;
      } else {
        USTR_SetLength(dst, pD - (char*)(*dst) - 1, cpd);
        return *dst;
      }
    }
    *pD = *pS;
    ++pD;
  }
  return *dst;
}

#ifdef _WIN32
/**--------------------------------------------------------------------------
@brief CPD 변경 (코드페이지 변환)

문자열의 코드페이지를 변환한다. ANSI 문자열과 UTF-16만을 지원하는 윈도에서만
필요한 함수로 MultiByteToWideChar 및 WideCharToMultiByte API함수를 사용하여
문자열을 서로 다른 코드페이지로 변환한다. 기존 CPD와 일치하는 경우 아무 것도
바꾸지 않고 함수를 종료한다.
@param [in,out] s   대상 USTR
@param [in ] cpd 변환할 CPD

@author ksg
*/
void WA_API USTR_ChangeCodePage(USTR *s, CPD cpd)
{
  STR_CPD *rec;
  USTR r = 0;
  int l = 0;

  if (*s == NULL)
    return;
  rec = (STR_CPD*)(*s) - 1;
  if (rec->cpd == cpd)
    return;
  if (rec->refCnt > 1) {
    rcm_unique(s);
    rec = (STR_CPD*)(*s) - 1;
  }

  if (rec->cpd != CPD_UTF16) {
    l = MultiByteToWideChar(rec->cpd & 0x0000ffff, 0, (char*)(*s), rec->length, NULL, 0);
    rcm_set_len_ctx(&r, l, CPD_UTF16);
    MultiByteToWideChar(rec->cpd & 0x0000ffff, 0, (char*)(*s), rec->length, (wchar_t*)r, l);
    USTR_Free(s);
    *s = r;
    rec = (STR_CPD*)(r) - 1;
    r = 0;
  }
  if (cpd == CPD_UTF16)
    return;

  l = WideCharToMultiByte(cpd & 0x0000ffff, 0, (wchar_t*)(*s), rec->length, NULL, 0, NULL, NULL);
  rcm_set_len_ctx(&r, l, cpd);
  WideCharToMultiByte(cpd & 0x0000ffff, 0, (wchar_t*)(*s), rec->length, (char*)r, l, NULL, NULL);
  USTR_Free(s);
  *s = r;
  return;
}

/**--------------------------------------------------------------------------
@brief printf 방식으로 USTR을 초기화(UTF-16버전)

@param [in ] fmt 포멧 문자열
@param [in ] ... 가변 인자
@return 초기화 된 USTR

@author ksg
*/
USTR __cdecl USTR_InitWithWStrFormat(const wchar_t *fmt, ...)
{
  USTR r = 0;
  va_list ap;
  va_start(ap, fmt);
    USTR_AppendFmtV(&r, CPD_UTF16, fmt, ap);
  va_end(ap);
  return r;
}
/**--------------------------------------------------------------------------
@brief 기존 USTR에 새롭게 printf방식으로 문자열을 추가(UTF-16버전)

@param [in,out] s   대상 USTR
@param [in ] fmt 포멧 문자열
@param [in ] ... 가변 인자
@return 추가된 문자열의 길이

@author ksg
*/
int __cdecl USTR_AppendWStrFormat(USTR *s, const wchar_t *fmt, ...)
{
  int r;
  va_list ap;
  va_start(ap, fmt);
    r = USTR_AppendFmtV(s, CPD_UTF16, fmt, ap);
  va_end(ap);
  return r;
}

#endif //#ifdef _WIN32
///**
//@brief      참조 카운트 1인 문자열 메모리 생성
//@param dst      : 타겟 문자열 메모리 주소
//@remarks    타겟 메모리의 사용을 유일한 참조로 재생성한다.
//*/
//void WA_API STR_MakeUnique(void **dst)
//{
//	STR_CPD *p, *p1;
//	int sz;
//
//	/// 문자열 메모리 참조가 유일한 경우 점검
//	/// @code
//	if (*dst == NULL || (p = ((STR_CPD*)(*dst) - 1))->refCnt == 1)
//		return;
//	/// @endcode
//
//	/// 벡테 메모리를 생성하고 타켓 메모리를 다시 생성한 후
//	/// 신규 생성된 메모리 주소를 타겟 메모리 주소로 변경함.
//	/// @code
//	InterlockedDecrement((long*)(&(p->refCnt)));
//	sz = p->elemSize * (p->length + 1) + sizeof(STR_CPD);
//	p1 = (STR_CPD*)allocMemory(sz);
//	copyMemory(p1, p, sz);
//	p1->refCnt = 1;
//	*dst = (STR8)(p1+1);
//	/// @endcode
//}
//
///**
//@brief      문자열 메모리의 길이를 재설정한다.
//@param dst      : 타켓 문자열 메모리 주소
//@param length    : 재할당 메모리 사이즈
//@param elemSize : 메모리 증가 단위
//@remarks    기존의 할당된 문자열 메모리의 데이터 메모리 영역의 사이즈를 재설정한다.
//*/
//void WA_API STR8_SetLength(STR8 *dst, int length, UINT16 elemSize)
//{
//	STR_CPD *p, *p1;
//
//	/// 새로 할당해야 할 길이가 0인 경우 메모리 해제하고 반환
//	/// @code
//	if (length <= 0) {
//		DVEC_Free(dst);
//		return;
//	}
//	/// @endcode
//
//	/// 타겟 메모리의 처음 할당인경우
//	/// @code
//	if (*dst == NULL) {
//		*dst = STR8_InitWithLength(length);
//		return;
//	}
//	/// @endcode
//
//	/// 타겟 메모리의 증가 단위가 다른 경우 오류로 처리함.
//	/// @code
//	p = (STR_CPD*)(*dst) - 1;
//	if (p->elemSize != elemSize)
//		exit(1);
//	/// @endcode
//
//	/// 문자열 메모리를 참조 하는 곳이 있는 경우 재할당 수행
//	/// 참조 메모리가 없는 경우 참조 카운트를 증가 시키고 새로운 메모리 할당 수행
//	/// @code
//	if (p->refCnt == 1) {
//		if (p->length == length)
//			return;
//		p = (STR_CPD*)reallocMemory(p, (length + 1) * elemSize + sizeof(STR_CPD));
//		p->length = length;
//		*dst = (STR8)(p+1);
//		return;
//	} else {
//		InterlockedDecrement((long*)(&(p->refCnt)));
//		p1 = (STR_CPD*)allocMemory((length + 1) * elemSize + sizeof(STR_CPD));
//		p1->elemSize = elemSize;
//		p1->reserved = 1;
//		p1->refCnt = 1;
//		p1->length = length;
//		copyMemory(p1+1, p+1, min(length, p->length) * elemSize);
//		*dst = (STR8)(p1+1);
//	}
//	/// @endcode
//}
//
///**
//@brief      타겟 문자열 데이터 메모리에 메모리 영역의 확장
//@param dst      : 타겟 문자열 메모리
//@param length   : 재 할당 하고자 하는 메모리 사이즈
//@param elemSize : 메모리 증가 단위
//@return     1
//@remark     기존의 존재하는 문자열 데이터 메모리 영역에 추가 영역을 확보 한다.
//*/
//int WA_API STR8_Add(STR8 *dst, int length, UINT16 elemSize)
//{
//	int l = STR8_Length(*dst);
//	STR8_SetLength(dst, length + l, elemSize);
//	return l;
//}
//
///**
//@brief      문자열 메모리의 할당된 데이터 메모리 사이즈를 알려줌
//@param src      : 소스 문자열 메모리
//@return     할당된 데이터 메모리 사이즈
//*/
//int WA_API STR8_Size(STR8 src)
//{
//	STR_CPD *p;
//	if (src) {
//		p = (STR_CPD *)src - 1;
//		return p->elemSize * p->length;
//	}
//	return 0;
//}
//
///**
//@brief      문자열 메모리의 할당된 데이터 메모리 사이즈 알려줌
//*/
//int WA_API STR8_Length(STR8 src)
//{
//	return (src) ? ((STR_CPD *)src - 1)->length : 0;
//}
//
///**
//@brief      문자열 메모리의 할당된 메모리 증가 사이즈 알려줌
//*/
//int WA_API STR8_ElemSize(STR8 src)
//{
//	return (src) ? ((STR_CPD *)src - 1)->elemSize : 0;
//}
//
///**
//@brief      문자열 메모리의 헤더 메모리 주소값을 알려줌
//*/
//STR_CPD * WA_API STR8_GetRec(STR8 src)
//{
//	return (src) ? (STR_CPD *)src - 1 : NULL;
//}
//
///**
//@brief      문자열 메모리의 데이터를 확인 하는 함수
//@param src      : 소스 문자열 메모리
//@param Index    : 소스 문자열 메모리의 위치
//@return     문자열 메모리의 데이터 특정 위치의 주소값
//*/
//void * WA_API STR8_Element(STR8 src, int Index)
//{
//	STR_CPD *p;
//#if (defined(_DEBUG) || defined(DEBUG))
//	if (src == NULL || Index < 0 || Index >= (p = (STR_CPD *)src - 1)->length) {
//		exit(1);
//	}
//#else
//	p = (STR_CPD *)src - 1;
//#endif
//	return (char*)(p+1) + p->elemSize * Index;
//}

