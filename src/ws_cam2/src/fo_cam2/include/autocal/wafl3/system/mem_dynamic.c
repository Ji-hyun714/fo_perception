/*!***************************************************************************
@file mem_dynamic.c
@brief 동적 컨테이너 구현

### 개요

mem_dynamic은 크게 3가지 그룹으로 구성되어 있으며 RCM(Reference Couting Memory, 
참조 카운트 관리 방식의 메모리) 관리용 일반 함수, DVEC 타입으로 선언된 가변 벡터형 메모리, 
그리고 DVEC을 C++에서 편리하게 사용할 수 있도록 wrapping 클래스인 TDynVec 구현이다.

### 길이(length)와 크기(size)
RCM에서 길이와 크기는 명확히 구분되어야 하는 개념이다. \n
**길이(length)**는 논리적인 개념으로 정해진 단위가 없다. 배열의 경우 원소 
수에 해당한다. \n
**크기(size)**는 이와는 달리 물리적 개념으로 byte단위의 양을 의미한다.

예를 들어 다음과 같이 배열을 선언한 경우

~~~~~~{.c}
int array[10];
~~~~~~

array 배열의 길이(length)는 10, 요소 크기는 4bytes (sizeof(int)), 배열의 
크기(size)는 40bytes가 된다.

### RCM

RCM은 Reference Couting Memory의 약자로 마이너스 인덱스 기법을 이용하여 감춰진 
레코드에 배열 요소의 크기(size, in bytes)와 논리적 길이(length), 참조 카운트를
저장하여 다양한 종류의 동적 배열 구현의 근간이 되는 요소이다.

RCM 관련 함수들은 사용자가 RCM 구조를 잘 알고 있다는 전제하에 설계되어 있다.
RCM 구조를 제대로 이해하지 못 한 상태에서 관련 함수를 사용할 경우 심각한
문제를 야기할 수 있다.

감춰진 헤더 구조는 다음과 같다.

~~~~~~{.c}
struct {
  uint32_t context;
  int32_t refCnt;
  int32_t length;
}
~~~~~~

- 빈 데이터: 추가 메모리 할당이 없음
|data|
|---|
|0|

- 데이터가 할당된 경우: \n
헤더 크기 + 요소크기 x (길이) 만큼 메모리를 할당한다.\n
RCM은 기본적으로 터미널 제로 처리를 하지 않지만 요소 크기가 2 이하인 경우에는
예외적으로 터미널 제로 처리를 위해 추가 메모리(요소크기만큼)를 할당하고 
추가된 메모리 내용을 0으로 채워준다.
short 타입으로 1,2,3을 저장할 경우 short타입의 요소 크기는 2이므로 터미널 제로
처리를 하게 되어 12 + 2 x (3+1) = 20bytes가 할당되고 포인터 값은 할당된 주소 + 
12bytes(헤더 크기)위치를 가리킨다. 만일 할당된 주소가 0x40000000이라면
|data|
|---|
|0x4000000C|
0x40000000에 저장된 내용은 
|context|refCnt|length|*element1|element2|element3|zero|
|-------|------|------|--------|--------|--------|----|
|0x00020000|0x00000001|0x00000003|0x0001|0x0002|0x0003|0x0000|
|4bytes|4bytes|4bytes|2bytes|2bytes|2bytes|2bytes|


터미널 제로 데이터 추가를 요소 크기가 2 이하인 경우에만 적용하도록 설계한 이유는 
요소 크기가 작은 경우 터미널 zero로 인해 낭비되는 양이 적을 뿐만 아니라 RCM을 
사용하여 문자열을 관리할 때 표준 C함수와의 호환성을 쉽게 유지할 수 있는 장점이 
있기 때문이다.

context는 32bit 부호없는 정수형 데이터이고 이 중 하위 16bit는 RCM 기반 배열에서 
자유롭게 사용할 수 있는 공간이다. 상위 16bit는 배열의 요소 크기(size, in bytes)
를 반드시 담고 있어야 한다.
예를 들어 동적 char배열의 경우 context값은 기본적으로 0x00010000으로 설정되어야
한다.

context를 분명하게 16bit씩 구분하지 않은 이유는 RCM 관련 함수들이 일반적인
용도가 아닌 저레벨 개발을 위한 함수들이기 때문에 함수 인자 수가 성능에 미치는
영향이 크기 때문이다.


@warning RCM 함수들은 RCM을 완전히 이해하지 못 한 상태에서 직접 호출하는 것은 
피해야 한다.
@author ksg
*****************************************************************************/
#include <string.h>

#if defined(CONFIG_SOC_A9AQ)
#else //#if defined(CONFIG_SOC_A9AQ)
	#include <stdlib.h>
#endif //#else #if defined(CONFIG_SOC_A9AQ)

#if (defined(CCS) || defined(__BORLANDC__))
#pragma hdrstop
#endif //(defined(CCS) || defined(__BORLANDC__))

#if defined(CONFIG_SOC_A9AQ)
	// AMBA Platform인 경우
	#include "AmbaDataType.h"
	#include "AmbaKAL.h"
#endif //#if defined(CONFIG_SOC_A9AQ)

#include "system/mem_dynamic.h"
#include "system/sys_logger.h"

//---------------------------------------------------------------------------

#if defined(__BORLANDC__)
#pragma package(smart_init)
#endif //defined(__BORLANDC__)

/**---------------------------------------------------------------------------
@brief RCM 디버그용 함수. 감춰진 레코드의 각 필드 값(context, refCnt, length)을 로깅 함수로 출력해준다.

@param [in] src RCM 포인터
@author ksg
*/
void   WA_API rcm_debug(void *src)
{
  if (src == NULL) {
    Dbg_("rcm_debug: Empty source");
  } else {
    #if defined(ENABLE_DBG)
    RCREC *rec = (RCREC*)src - 1;
    Dbg_("rcm_debug: context=0x%08x, refCnt=%d, length=%d", rec->context, rec->refCnt, rec->length);
    #endif //#if ENABLE_DBG
  }
}
//#ifdef _WIN64
//#define rcm_inc_ref(dst) InterlockedIncrement((long*)(&((((RCREC*)dst) - 1)->refCnt)))
//#else
//#endif

/**---------------------------------------------------------------------------
@brief RCM 공유 함수

이름은 clone이지만 실제 메모리 복사는 없이 참조 카운트만 하나 증가시킨다.
@param [in] src RCM 포인터
@return 입력받은 RCM포인터를 그대로 반환함
@author ksg
*/
void * WA_API rcm_clone(void *src)
{
  if (src != NULL) {
    InterlockedIncrement((long*)(&((((RCREC*)src) - 1)->refCnt)));
  }
  return src;
}
/**---------------------------------------------------------------------------
@brief      벡터 메모리 증가 사이즈에 따른 메모리 할당 및 초기화 함수
@param [in] length  길이 (논리적 길이, byte단위의 사이즈가 아님)
@param [in] context 컨텍스트 값 (상위 16bit는 요소 사이즈를 가지고 있어야 한다)
@return     할당된 메모리에서 실제 데이터 영역의 메모리 시작 주소, 실패시 NULL
*/
void * WA_API rcm_init_len_ctx(int length, uint32_t context)
{
  RCREC *p;
  int elemSize;

  elemSize = (context >> 16);
  switch (elemSize) {
    case 0:
      return NULL;
    case 1:
      p = (RCREC *)allocMemory(length + 1 + sizeof(RCREC));
      ((char*)(p + 1))[length] = 0;
      break;
    case 2:
      p = (RCREC *)allocMemory((length + 1) * 2 + sizeof(RCREC));
      ((uint16_t*)(p + 1))[length] = 0;
      break;
    default:
      p = (RCREC *)allocMemory(length * elemSize + sizeof(RCREC));
      break;
  }
  p->context = context;
  p->refCnt = 1;
  p->length = length;

  return (p + 1);
}

/**---------------------------------------------------------------------------
@brief RCM 해제

우선 참조 카운트를 하나 감소시킨 후 참조 카운트가 0이 아니라면 해당 주소를
참조하고 있는 다른 인스턴스가 있는 경우이므로 메모리를 해제하지 않고 RCM 
포인터만 0으로 설정한다. 0인 경우 메모리를 해제한다.
@param [in,out] dst 대상
@return 해당 메모리의 최종 참조 카운트. 
@retval 0 메모리가 해제되었음 
@retval n n개의 인스턴스에 의해 아직 참조되고 있음
@author ksg
*/
int WA_API rcm_free(void **dst)
{
  int ret = 0;
  RCREC* p;
  if (*dst) {
    p = ((RCREC*)(*dst)) - 1;
    if ((ret = InterlockedDecrement((long*)(&(p->refCnt)))) == 0) {
      freeMemory(p);
    }
    *dst = NULL;
  }
  return ret;
}
/**---------------------------------------------------------------------------
@brief RCM을 참조하고 있는 인스턴스가 유일한지 검사

@param [in] src 대상
@return 유일한지 여부
@retval true 유일한 경우 
@retval false 유일하지 않은 경우

@author ksg
*/
bool WA_API rcm_is_unique(void *src)
{
  return (src == NULL || ((RCREC*)src - 1)->refCnt == 1);
}
/**---------------------------------------------------------------------------
@brief RCM을 유일하게 만듦

만약 RCM을 여러 인스턴스가 공유하고 있다면 메모리를 복사하여 단독으로만
참조할 수 있도록 한다. 
@param [in,out] dst 대상

@author ksg
*/
void WA_API rcm_unique(void **dst)
{
  RCREC *rec;
  if (*dst != NULL && (rec = ((RCREC*)*dst - 1))->refCnt != 1) {
    rcm_set_len_ctx(dst, rec->length, rec->context);
  }
}
/**---------------------------------------------------------------------------
@brief RCM 대입 연산

dst에 src를 대입한다. dst에 RCM이 할당된 경우 dst를 먼저 해제한다.
주소를 대입한 후에는 src의 참조 카운트를 하나 증가시킨다.

@param [out] dst 대상
@param [in] src  대입할 메모리
@author ksg
*/
void WA_API rcm_assign(void **dst, void *src)
{
  if (*dst == src)
    return;
  rcm_free(dst);
  if (src) {
    InterlockedIncrement((long*)(&((((RCREC*)src) - 1)->refCnt)));
    *dst = src;
  }
}

/**---------------------------------------------------------------------------
@brief RCM 길이 변경

@param [out] dst     대상
@param [in] length  길이 (논리적 길이, byte단위의 사이즈가 아님)
@param [in] context 컨텍스트 값 (상위 16bit는 요소 사이즈를 가지고 있어야 한다)

@author ksg
*/
void WA_API rcm_set_len_ctx(void **dst, int length, uint32_t context)
{
  RCREC *p;
  uint32_t elemSize;

  //Dbg_("rcm_set_len_ctx entered");
  // 새로 지정된 길이가 0이거나 elemSize가 0인 경우 타겟을 free하고 종료
  if (length == 0 || (elemSize = (context >> 16)) == 0) {
    rcm_free(dst);
    return;
  }

  //Dbg_("length=%d and elemSize=%d check complete", length, elemSize);
  // 첫 할당인경우
  if (*dst == NULL) {
    *dst = rcm_init_len_ctx(length, context);
    return;
  }

  //Dbg_("Not first allocation");

  p = (RCREC*)(*dst) - 1;
#if defined(_DEBUG) || defined(DEBUG)
  // 타겟 메모리의 elemSize가 다른 경우 오류로 처리함.
  if ((p->context >> 16) != elemSize) {
    exit(1);
  }
#endif //if defined(_DEBUG) || defined(DEBUG)

  // 벡터 메모리를 참조 하는 곳이 있는 경우 재할당 수행
  // 참조 메모리가 없는 경우 참조 카운트를 증가 시키고 새로운 메모리 할당 수행
  if (p->refCnt == 1) {
    //Dbg_("Unique rcm");
    if (p->length == length)
      return;
    //Dbg_("Not same length");
    switch (elemSize) {
      case 0:
      case 1:
        //Dbg_("elemSize is 1");
        p = (RCREC*)reallocMemory(p, length + 1 + sizeof(RCREC));
        ((char*)(p+1))[length] = 0;
        break;
      case 2:
        //Dbg_("elemSize is 2");
        p = (RCREC*)reallocMemory(p, (length + 1) * 2 + sizeof(RCREC));
        ((uint16_t*)(p+1))[length] = 0;
        break;
      default:
        //Dbg_f("elemSize is %d", elemSize);
        p = (RCREC*)reallocMemory(p, length * elemSize + sizeof(RCREC));
        break;
    }
    p->length = length;
    p->context = context;
    *dst = (void*)(p+1);
    //Dbg_("realloc complete");
  } else {
    //Dbg_("Not unique rcm, begins copyMemory");

    InterlockedDecrement((long*)(&(p->refCnt)));
    *dst = rcm_init_len_ctx(length, context);

    	//2022.04.20,kjkim
    //copyMemory(*dst, p+1, min(length, p->length) * elemSize);
    copyMemory(*dst, p+1, Min(length, p->length) * elemSize);
    //Dbg_("copyMemory complete");
  }
  //Dbg_("rcm_set_len_ctx leaving");
}
/**---------------------------------------------------------------------------
@brief  RCM의 길이 늘림
@return RCM 길이(논리적 길이, byte단위의 크기가 아님)
*/
// wafl3에서 새로 추가됨
void * WA_API rcm_inc_len_ctx(void **dst, int length, uint32_t context)
{
  RCREC *p = (RCREC*)(*dst);
  int l;
  if (p == NULL) {
    rcm_set_len_ctx(dst, length, context);
    return *dst;
  }
  p -= 1;
  l = p->length;
  if (p->context != context) {
    Err_("Context mismatching");
    return NULL;
  }
  rcm_set_len_ctx(dst, l + length, context);
  return (void*)((char*)(*dst) + l * (context >> 16));
}

/**---------------------------------------------------------------------------
@brief  RCM의 길이 늘림
@return RCM 길이(논리적 길이, byte단위의 크기가 아님)
*/
// wafl3에서 새로 추가됨
void * WA_API rcm_insert_len_ctx(void **dst, int index, int length, uint32_t context)
{
  char *p;
  int l, elemSize;
  l = rcm_get_len(*dst);
  if (index >= l) {
    return rcm_inc_len_ctx(dst, length, context);
  }
  if (index < 0) {
    index = 0;
  }
  rcm_inc_len_ctx(dst, length, context);
  elemSize = (context >> 16);
  p = (char*)(*dst) + index * elemSize;
  moveMemory(p + length * elemSize, p, (l - index) * elemSize);
  return (void*)p;
}

/**---------------------------------------------------------------------------
@brief  RCM의 길이 반환
@return RCM 길이(논리적 길이, byte단위의 크기가 아님)
*/
int  WA_API rcm_get_len(const void *src)
{
  return src ? (((RCREC*)src) - 1)->length : 0;
}

/**---------------------------------------------------------------------------
@brief      RCM의 할당된 메모리 크기

메모리 크기는 byte단위로 반환되며 감춰진 헤더 크기와 터미널 제로의 크기는 고려하지 않는다.
@return RCM 크기 (in bytes)
*/
int  WA_API rcm_get_size(const void *src)
{
  RCREC *p;
  if (src) {
    p = ((RCREC*)src) - 1;
    return (p->context >> 16) * p->length;
  }
  return 0;
}
// wafl3에서 새로 추가됨
uint32_t WA_API rcm_get_context(const void *src)
{
  RCREC *p;
  if (src) {
    p = ((RCREC*)src) - 1;
    return p->context;
  }
  return 0;
}

/**---------------------------------------------------------------------------
@brief RCM에 다른 RCM을 추가해 넣음

@param [in,out] dst 대상
@param [in] src 추가할 RCM

@author ksg
*/
void WA_API rcm_append(void **dst, void *src)
{
  RCREC *p;
  int l0;
  int elemSize;


  if (src == NULL)
    return;
  if (*dst == NULL) {
    rcm_inc_ref(src);
    *dst = src;
    return;
  }

  p = ((RCREC*)(*dst)) - 1;
  l0 = p->length;
  elemSize = (p->context >> 16);

  if (*dst == src) {
    rcm_set_len_ctx(dst, l0 * 2, p->context);
    l0 *= elemSize;
    copyMemory((char*)(*dst) + l0, *dst, l0);
  } else {
    rcm_set_len_ctx(dst, l0 + rcm_get_len(src), p->context);
    copyMemory((char*)(*dst) + l0 * elemSize, src, rcm_get_size(src));
  }
}
/**---------------------------------------------------------------------------
@brief 두 RCM을 연결한 새로운 RCM을 반환

@param [in] lhs 첫번째 RCM
@param [in] rhs 두번째 RCM
@return 두 RCM이 연결된 하나의 RCM

@author ksg
*/
void * WA_API rcm_add(void *lhs, void *rhs)
{
  void *dst;
  RCREC *p1, *p2;
  int l;
  if (rhs == NULL) {
    if (lhs)
      rcm_inc_ref(lhs);
    return lhs;
  }
  if (lhs == NULL) {
    if (rhs)
      rcm_inc_ref(rhs);
    return rhs;
  }
  p1 = (RCREC*)lhs - 1;
  l = p1->context >> 16;
  if (lhs == rhs) {
    dst = rcm_init_len_ctx(p1->length * 2, p1->context);
    l *= p1->length;
    copyMemory(dst, lhs, l);
    copyMemory((char*)dst + l, lhs, l);
  } else {
    p2 = (RCREC*)rhs - 1;
#if defined(DEBUG) || defined(_DEBUG)
    if (p1->context != p2->context)
      return NULL;
#endif // if defined(DEBUG) || defined(_DEBUG)
    dst = rcm_init_len_ctx(p1->length + p2->length, p1->context);
    copyMemory(dst, lhs, p1->length * l);
    copyMemory((char*)dst + p1->length * l, rhs, p2->length * l);
  }
  return dst;
}

/**---------------------------------------------------------------------------
@brief  RCM의 특정 인덱스의 요소들을 삭제
@param [in,out] dst   대상 RCM 메모리
@param [in ]    index 삭제할 요소의 인덱스
@param [in ]    length 삭제할 요소의 수
@return 삭제된 요소의 직후에 있는 요소의 주소
*/
void * WA_API rcm_delete_len(void **dst, int index, int length)
{
  RCREC *rec;
  int l, elemSize;
  char *p;

  if (dst == NULL || *dst == NULL || length == 0)
    return NULL;
  rec = ((RCREC*)(*dst)) - 1;
  l = rec->length;

  if (index >= l) {
    // 맨 뒤를 기준으로 삭제
    index = l - length - 1;
    if (index <= 0) {
      rcm_free(dst);
      return NULL;
    }
    rcm_set_len_ctx(dst, index, rec->context);
    return NULL;
  }

  if (index < 0) {
    index = 0;
  }
  if (index + length >= l) {
    rcm_set_len_ctx(dst, index, rec->context);
    return NULL;
  }

  elemSize = (rec->context >> 16);
  p = (char*)(*dst) + index * elemSize;
  moveMemory(p, p + length * elemSize, (l - index - length) * elemSize);
  rcm_set_len_ctx(dst, l-length, rec->context);
  return (void*)p;
}

/**---------------------------------------------------------------------------
@brief RCM을 파일로 저장

@param [in] src  저장할 RCM
@param [in] fp   FILE 포인터
@param [in] opts 옵션
- RCM_DONT_CLOSE: FILE을 Close하지 않음(지정하지 않으면 기록 후 자동으로 Close)
@return 기록된 양(bytes)

@author ksg
*/
int    WA_API rcm_save_to_file(void *src, FILE *fp, uint32_t opts)
{
  int ret = -3;
  RCREC *p;

  if (fp == NULL)
    return -1;

  if (src == NULL) {
    ret = 0;
    if (fwrite(&ret, 1, sizeof(int), fp) != sizeof(int))
      ret = -2;
    goto lbl_finalize;
  }
  p = (RCREC*)src - 1;
#if defined(DEBUG) || defined(_DEBUG)
  if (p->context == 0)
    goto lbl_finalize;
#endif // if defined(DEBUG) || defined(_DEBUG)
  if (fwrite(p, 1, sizeof(uint32_t), fp) != sizeof(uint32_t)) {
    ret = -4;
    goto lbl_finalize;
  }
  ret = p->length * (p->context >> 16);
  if (fwrite(&(p->length), 1, sizeof(int) + ret, fp) != sizeof(int) + ret)
    ret = -5;

lbl_finalize:
  if (!(opts & RCM_DONT_CLOSE))
    fclose(fp);
  return ret;
}
/**---------------------------------------------------------------------------
@brief FILE로부터 RCM을 로딩

@param [in] dst  대상 RCM
@param [in] fp   FILE 포인터
@param [in] opts 옵션
- RCM_DONT_CLOSE: FILE을 Close하지 않음(지정하지 않으면 로딩 후 자동으로 Close)
@return Return_Description

@author ksg
*/
int    WA_API rcm_load_from_file(void **dst, FILE *fp, uint32_t opts)
{
  int ret = 0;
  RCREC r;

  if (fp == NULL || dst == NULL)
    return -1;

  rcm_free(dst);
  if (fread(&r, 1, sizeof(uint32_t), fp) != sizeof(uint32_t)) {
    ret = -2;
    goto lbl_finalize;
  }
  if (r.context == 0) {
    ret = 0;
    goto lbl_finalize;
  }
  if (r.context >= 0x00010000) { // 신버전
    if (fread(&r.length, 1, sizeof(int), fp) != sizeof(int)) {
      ret = -3;
      goto lbl_finalize;
    }
  } else { // 구버전
    r.context <<= 16;
    if (fread(&r.refCnt, 1, sizeof(int) * 2, fp) != sizeof(int) * 2) {
      ret = -3;
      goto lbl_finalize;
    }
  }

#if defined(DEBUG) || defined(_DEBUG)
  if (r.length <= 0) {
    ret = (r.length == 0) ? 0 : -4;
    goto lbl_finalize;
  }
#endif // defined(DEBUG) || defined(_DEBUG)

  *dst = rcm_init_len_ctx(r.length, r.context);
  r.context >>= 16;
  ret = r.length * r.context;
  if (fread(*dst, 1, ret, fp) != (size_t)ret) {
    rcm_free(dst);
    ret = -5;
  }

lbl_finalize:
  if (!(opts & RCM_DONT_CLOSE))
	fclose(fp);
  return ret;
}




/**---------------------------------------------------------------------------
@brief Heap 메모리 할당 
@param [in] sz 할당할 크기(in bytes)
@return 할당된 메모리의 주소
@remarks 윈도우 환경에서는 Heapalloc을 사용하고 표준 C경우 malloc을 사용하여 할당
*/
#if defined(CONFIG_SOC_A9AQ)
extern AMBA_KAL_BYTE_POOL_t  AmbaBytePool_Cached;
#endif //#if defined(CONFIG_SOC_A9AQ)
void * WA_API allocMemory(int sz)
{
#if defined(USE_MM) && USE_MM == MM_WIN32_HEAPALLOC
	static HANDLE ghHeap = GetProcessHeap();
	return HeapAlloc(ghHeap, 0, sz);
#else
	#if defined(CONFIG_SOC_A9AQ)
		//AMBA 사용시에만
		int32_t lSuccess = 0;
		AMBA_MEM_CTRL_s hMem;
		int8_t *ret;
		lSuccess = AmbaKAL_MemAllocate(&AmbaBytePool_Cached, &hMem, sz + sizeof(AMBA_MEM_CTRL_s) + sizeof(int), 32);
		if (lSuccess != OK) {
			Err_("Allocation Failed");
			return NULL;
		}
		ret = (int8_t*)hMem.pMemAlignedBase;
		*(AMBA_MEM_CTRL_s*)ret = hMem;
		ret += sizeof(AMBA_MEM_CTRL_s);
		*(int*)ret = sz;
		return (void*)(ret + sizeof(int));
	#else //#if defined(CONFIG_SOC_A9AQ)
		return malloc(sz);
	#endif //#else #if defined(CONFIG_SOC_A9AQ)
#endif

}
void * WA_API allocZeroMemory(int sz, int len)
{
#if defined(USE_MM) && USE_MM == MM_WIN32_HEAPALLOC
	static HANDLE ghHeap = GetProcessHeap();
	return HeapAlloc(ghHeap, 0, sz);
#else
	#if defined(CONFIG_SOC_A9AQ)
		void * ret = allocMemory(sz * len);
		memset(ret, 0, sz * len);
		return ret;
	#else //#if defined(CONFIG_SOC_A9AQ)
		return calloc(sz, len);
	#endif //#else #if defined(CONFIG_SOC_A9AQ)
#endif
}

/**---------------------------------------------------------------------------
@brief Heap 메모리 재할당
@param [in] mem  할당된 메모리 주소
@param [in] sz   재할당할 크기(in bytes)
@return     재할당된 메모리 주소
*/
void * WA_API reallocMemory(void *mem, int sz)
{
#if defined(USE_MM) && USE_MM == MM_WIN32_HEAPALLOC
	static HANDLE ghHeap = GetProcessHeap();
	return HeapReAlloc(ghHeap, 0, mem, sz);
#else
	#if defined(CONFIG_SOC_A9AQ)
		int l;
		void *ret = NULL;
		if (sz == 0) {
			if (mem) {
				freeMemory(mem);
			}
			return ret;
		}
		l = *((int*)mem - 1);
		if (l == sz) {
			return mem;
		}
		ret = allocMemory(sz);
		if (ret) {
			memcpy(ret, mem, Min(sz, l));
			freeMemory(mem);
		} else {
			Err_("reallocMemory failed");
		}
		return ret;
	#else //#if defined(CONFIG_SOC_A9AQ)
		return realloc(mem, sz);
	#endif //#else #if defined(CONFIG_SOC_A9AQ)
#endif
}

/**---------------------------------------------------------------------------
@brief Heap 메모리 해제
@param [in] mem  할당된 메모리 주소
*/
void WA_API freeMemory(void *mem)
{
#if defined(USE_MM) && USE_MM == MM_WIN32_HEAPALLOC
  static HANDLE ghHeap = GetProcessHeap();
  HeapFree(ghHeap, 0, mem);
#else
	#if defined(CONFIG_SOC_A9AQ)
		AmbaKAL_MemFree((AMBA_MEM_CTRL_s*)((int8_t*)mem - sizeof(AMBA_MEM_CTRL_s) - sizeof(int)));
	#else //#if defined(CONFIG_SOC_A9AQ)
		free(mem);
	#endif //#else #if defined(CONFIG_SOC_A9AQ)
#endif
}

/**---------------------------------------------------------------------------
@brief 메모리 복사
@param [out] dst 복사본 메모리 주소
@param [in] src 원본 메모리 주소
@param [in] sz  복사할 메모리 크기(in bytes)
*/
void WA_API copyMemory(void *dst, const void *src, int sz)
{
#if (defined(_WIN32) || defined(WIN32))
  CopyMemory(dst, src, sz);
#else
  memcpy(dst, src, sz);
#endif
}
/**---------------------------------------------------------------------------
@brief 메모리 이동
@param [out] dst 이동할 메모리 주소
@param [in] src 원본 메모리 주소
@param [in] sz  이동할 메모리 크기(in bytes)
*/
void WA_API moveMemory(void *dst, void *src, int sz)
{
#if (defined(_WIN32) || defined(WIN32))
  MoveMemory(dst, src, sz);
#else
  memmove(dst, src, sz);
#endif
}
/**---------------------------------------------------------------------------
@brief DVEC 초기화(기본)
@return NULL로 초기화 된 DVEC
*/
DVEC WA_API DVEC_Init(void)
{
  return (DVEC)NULL;
}

/**---------------------------------------------------------------------------
@brief  DVEC 초기화(기존 DVEC 객체로부터)
@param [in] src  원본 DVEC 객체
@return     기존 DVEC 객체와 메모리를 공유하는 새로운 DVEC객체
*/
DVEC WA_API DVEC_InitWith(DVEC src)
{
  // 소스메모리의 해더 접근을 위해서 소스 메모리에서 해더메모리 접근후
  // 메모리 참조 카운트 증가 시킴, 참조 카운트의 유일한 접근을 위해 락 접근을 시도함
  if (src != NULL) {
    InterlockedIncrement((long*)(&((((DVEC_REC*)src) - 1)->refCnt)));
  }
  return src;
}

/**---------------------------------------------------------------------------
@brief      DVEC의 참조 카운트가 1이 되도록 변경. rcm_unique 함수 설명 참조
@param [out] dst   대상 DVEC
*/
void WA_API DVEC_MakeUnique(DVEC *dst)
{
  DVEC_REC *p, *p1;
  int sz;

  // 벡터 메모리 참조가 유일한 경우 점검
  if (*dst == NULL || (p = ((DVEC_REC*)(*dst) - 1))->refCnt == 1)
    return;

  // 벡터 메모리를 생성하고 타켓 메모리를 다시 생성한 후
  // 신규 생성된 메모리 주소를 타겟 메모리 주소로 변경함.
  InterlockedDecrement((long*)(&(p->refCnt)));
  sz = p->elemSize * p->length + sizeof(DVEC_REC);
  p1 = (DVEC_REC*)allocMemory(sz);
  copyMemory(p1, p, sz);
  p1->refCnt = 1;
  *dst = (DVEC)(p1+1);
}

/**---------------------------------------------------------------------------
@brief      DVEC 길이 증가 (추가된 요소의 시작 인덱스 반환)
@param [in,out] dst  대상 DVEC
@param [in] length   추가할 요소 갯수
@param [in] elemSize 요소 크기
@return     추가된 요소의 시작 인덱스
@author ksg
*/
int WA_API DVEC_Add(DVEC *dst, int length, uint16_t elemSize)
{
  int l = DVEC_Length(*dst);
  DVEC_SetLength(dst, length + l, elemSize);
  return l;
}

/**---------------------------------------------------------------------------
@brief      DVEC 길이 증가 (추가된 요소의 시작 포인터 반환)
@param [in,out] dst  대상 DVEC
@param [in] length   추가할 요소 갯수
@param [in] elemSize 요소 크기
@return     추가된 요소의 시작 포인터
@author ksg
*/
void * WA_API DVEC_Append(DVEC *dst, int length, uint16_t elemSize)
{
  int l = DVEC_Length(*dst);
  DVEC_SetLength(dst, length + l, elemSize);
  return (char*)(*dst) + l * elemSize;
}


/**---------------------------------------------------------------------------
@brief DVEC의 요소 크기를 반환

@param [in] src 대상 DVEC
@return 요소 크기

@author ksg
*/
int WA_API DVEC_ElemSize(DVEC src)
{
  return (src) ? ((DVEC_REC *)src - 1)->elemSize : 0;
}

/**---------------------------------------------------------------------------
@brief  DVEC의 헤더 포인터를 반환
@param [in] src 대상 DVEC
@remark 비어있는 DVEC은 헤더가 없기 때문에 NULL을 반환할 수 있으므로
DVEC_GetRec로 얻은 결과값을 사용 하기 전에 NULL 체크 루틴을 꼭 포함해야 한다.
@return 헤더 주소
*/
DVEC_REC * WA_API DVEC_GetRec(DVEC src)
{
  return (src) ? (DVEC_REC *)src - 1 : NULL;
}

/**---------------------------------------------------------------------------
@brief      벡터 메모리의 데이터를 확인 하는 함수
@param src      : 소스 벡터 메모리
@param Index    : 소스 벡터 메모리의 위치
@return     벡터 메모리의 데이터 특정 위치의 주소값
*/
void * WA_API DVEC_Element(DVEC src, int Index)
{
  DVEC_REC *p;
#if (defined(_DEBUG) || defined(DEBUG))
  if (src == NULL || Index < 0 || Index >= (p = (DVEC_REC *)src - 1)->length) {
    exit(1);
  }
#else
  p = (DVEC_REC *)src - 1;
#endif
  return (char*)(p+1) + p->elemSize * Index;
}


/**---------------------------------------------------------------------------
@brief      벡터 메모리의 데이터를 파일로 저장
@param fileName : 저장할 파일 이름
@param src      : 소스 벡터 메모리
@return     파일 쓰기 사이즈
*/
int DVEC_SaveToFileA(const char *fileName, DVEC src)
{
  int ret;
  if ((ret = rcm_save_to_file(src, fopen(fileName, "wb"), 0)) < 0) {
    Err_("Cannot save to file: %s", fileName);
  }
  return ret;
//	return DVEC_SaveToFILE(fopen(fileName, "wb"), src, 1);
}


/**---------------------------------------------------------------------------
@brief      벡터 메모리의 데이터를 파일로 부터 읽어 드리는 함수
@param dst      : 읽어 오는 데이터저장 벡터 메모리
@param fileName : 읽어들이는 파일 이름
@return     읽어들인 데이터 사이즈
*/
int DVEC_LoadFromFileA(DVEC *dst, const char *fileName)
{
  FILE *fp;
  fp = fopen(fileName, "rb");
  if (fp == NULL) {
    Err_("Cannot open file for read: %s", fileName);
  }
  return rcm_load_from_file(dst, fp, 0);
//	return DVEC_LoadFromFILE(dst, fopen(fileName, "rb"), 1);
}



/*
//////////////////////////////////////////////////////////////////////////////
int WA_API UMAT_REC_DataSize(UMAT_REC *R)
{
  int r_stride = R->r_stride;
  return (r_stride == 0) ? R->c_stride * R->dim.as1D : r_stride * R->dim.as2D.row;
}
//---------------------------------------------------------------------------


//////////////////////////////////////////////////////////////////////////////
UMAT WA_API UMAT_InitWith(UMAT src)
{
  if (src != NULL)
    InterlockedIncrement((long*)(&((((UMAT_REC*)src) - 1)->refCnt)));
  return src;
}
//---------------------------------------------------------------------------
UMAT WA_API UMAT_InitWithLength(uint32_t length, uint16_t elemSize)
{
  UMAT_REC *p;
  uint32_t sz = length * elemSize;
  if (sz > 0) {
    p = (UMAT_REC *)allocMemory(sz + sizeof(UMAT_REC));
    p->r_stride = 0;
    p->c_stride = elemSize;
    p->dim.as1D = length;
    p->refCnt = 1;
    return (UMAT)(p + 1);
  }
  return (UMAT)NULL;
}
//---------------------------------------------------------------------------
UMAT WA_API UMAT_InitWithDimension(uint16_t row, uint16_t col, uint16_t elemSize)
{
  UMAT_REC *p;
  uint32_t sz = row * col * elemSize;
  if (sz > 0) {
    p = (UMAT_REC *)allocMemory(sz + sizeof(UMAT_REC));
    p->r_stride = elemSize * col;
    p->c_stride = elemSize;
    p->dim.as2D.row = row;
    p->dim.as2D.col = col;
    p->refCnt = 1;
    return (UMAT)(p + 1);
  }
  return (UMAT)NULL;
}
//---------------------------------------------------------------------------
int WA_API UMAT_Free(DVEC *dst)
{
  int ret = 0;
  UMAT_REC* p;
  if (*dst) {
    p = (UMAT_REC*)(*dst) - 1;
    if ((ret = InterlockedDecrement((long*)(&(p->refCnt)))) == 0)
      freeMemory(p);
    *dst = NULL;
  }
  return ret;
}
//---------------------------------------------------------------------------
void WA_API UMAT_Assign(UMAT *dst, UMAT src)
{
  if (*dst != src) {
    UMAT_Free(dst);
    *dst = UMAT_InitWith(src);
    }
}
//---------------------------------------------------------------------------
void WA_API UMAT_MakeUnique(UMAT *dst)
{
  UMAT_REC *p, *p1;
  int sz;
  if (*dst == NULL || (p = ((UMAT_REC*)(*dst) - 1))->refCnt == 1)
    return;
  InterlockedDecrement((long*)(&(p->refCnt)));
  sz = UMAT_REC_DataSize(p);
  p1 = (UMAT_REC*)allocMemory(sz) + sizeof(UMAT_REC);
  copyMemory(p1, p, sz);
  p1->refCnt = 1;
  *dst = (UMAT)(p1+1);
}
//---------------------------------------------------------------------------
void WA_API UMAT_SetDimension(UMAT *dst, int length, uint16_t elemSize)
{

}
//---------------------------------------------------------------------------
void WA_API UMAT_ChangeDimension(UMAT *dst, int length, uint16_t elemSize)
{

}
//---------------------------------------------------------------------------
*/

