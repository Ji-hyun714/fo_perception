/**--------------------------------------------------------------------------
@file mem_dynamic.h
@brief mem_dynamic.c 파일의 헤더
@author ksg
*/
//---------------------------------------------------------------------------

#ifndef mem_dynamicH
#define mem_dynamicH
//---------------------------------------------------------------------------
#include "wafl3.h"
#include <stdio.h>

#define MM_MALLOC          0
#define MM_WIN32_HEAPALLOC 1

#define USE_MM  MM_MALLOC

#if (defined(_WIN32) || defined(WIN32))
	#if !defined(USE_MM)
		#define USE_MM        MM_WIN32_HEAPALLOC
	#endif //if !defined(USE_MM)
#else //#if (defined(_WIN32) || defined(WIN32))

	#ifdef SUPPORT_GCC_ATOMIC_BUILTIN_FUNC
		#define InterlockedIncrement(x) (__sync_add_and_fetch(x, 1))
		#define InterlockedDecrement(x) (__sync_sub_and_fetch(x, 1))
	#else // SUPPORT_GCC_ATOMIC_BUILTIN_FUNC
		static inline long InterlockedIncrement(long *x) {return ++(*x);} ///< SUPPORT_GCC_ATOMIC_BUILTIN_FUNC이 지원되지 않는 경우 InterlockedIncrement를 모사
		static inline long InterlockedDecrement(long *x) {return --(*x);} ///< SUPPORT_GCC_ATOMIC_BUILTIN_FUNC이 지원되지 않는 경우 InterlockedDecrement를 모사
	#endif // SUPPORT_GCC_ATOMIC_BUILTIN_FUNC

#endif //#else #if (defined(_WIN32) || defined(WIN32))

#if defined(__cplusplus)
extern "C" {
#endif // defined(__cplusplus)

// Memory Management
void * WA_API allocMemory(int sz);
void * WA_API allocZeroMemory(int sz, int len);
void * WA_API reallocMemory(void *mem, int sz);
void   WA_API freeMemory(void *mem);
void   WA_API copyMemory(void *dst, const void *src, int sz);
void   WA_API moveMemory(void *dst, void *src, int sz);


/**--------------------------------------------------------------------------
@brief RCM 레코드 구조체
*/
#pragma pack(1)
typedef struct {
#ifdef _WIN64
  uint32_t _padding64;
#endif //#ifdef _WIN64
  uint32_t context;            ///< context정보. context >> 16의 결과는 elemSize가 되어야 한다.
  int32_t refCnt;               ///< 참조 카운트
  int32_t length;               ///< 실제 데이터 사이즈
} RCREC;
#pragma pack()

typedef struct {
  int offset_header;
  int offset_length;
  int offset_ref_cnt;
  int offset_context;
  uint32_t default_context;

} RCM_DESCRIPTOR;

#define RCM_LAST_INDEX (0x7ffffff)

extern void * WA_API rcm_clone(void *src);
extern void * WA_API rcm_init_len_ctx(int length, uint32_t context);
extern int    WA_API rcm_free(void **dst);
extern bool   WA_API rcm_is_unique(void *src);
extern void   WA_API rcm_unique(void **dst);
extern void   WA_API rcm_assign(void **dst, void *src);
extern void   WA_API rcm_set_len_ctx(void **dst, int length, uint32_t context);
extern void * WA_API rcm_inc_len_ctx(void **dst, int length, uint32_t context);
extern void * WA_API rcm_insert_len_ctx(void **dst, int index, int length, uint32_t context);
extern void * WA_API rcm_delete_len(void **dst, int index, int length);
extern int    WA_API rcm_get_len(const void *src);
extern int    WA_API rcm_get_size(const void *src);
extern void   WA_API rcm_append(void **dst, void *src);
extern void * WA_API rcm_add(void *lhs, void *rhs);
extern uint32_t WA_API rcm_get_context(const void *src);
/**--------------------------------------------------------------------------
@brief RCM에서 특정 인덱스의 요소 하나를 삭제

@param [in,out] dst   대상 RCM의 주소
@param [in ]    index 삭제할 요소의 인덱스
@return 삭제된 요소의 직후에 있는 요소의 주소

@author ksg
*/
static inline void * WA_API rcm_delete(void **dst, int index) {return rcm_delete_len(dst, index, 1);}
/**--------------------------------------------------------------------------
@brief RCM의 참조 카운트를 증가시킴
@param [in,out] dst 대상 RCM
@warning RCM의 메커니즘을 확실히 이해하고 있지 않으면 함부로 쓰지 말 것
@author ksg
*/
static inline void rcm_inc_ref(void *dst) {InterlockedIncrement((long*)(&((((RCREC*)dst) - 1)->refCnt)));}
/**--------------------------------------------------------------------------
@brief RCM의 참조 카운트를 감소시킴
@param [in,out] dst 대상 RCM
@warning RCM의 메커니즘을 확실히 이해하고 있지 않으면 함부로 쓰지 말 것
@author ksg
*/
static inline void rcm_dec_ref(void *dst) {InterlockedDecrement((long*)(&((((RCREC*)dst) - 1)->refCnt)));}


#define RCM_DONT_CLOSE 0x00000001
extern int    WA_API rcm_save_to_file(void *src, FILE *fp, uint32_t opts);
extern int    WA_API rcm_load_from_file(void **dst, FILE *fp, uint32_t opts);

extern void   WA_API rcm_debug(void *src);

/**--------------------------------------------------------------------------
@brief 동적 벡터(배열) 타입 선언
*/
typedef void* DVEC;
/**--------------------------------------------------------------------------
@brief 벡터 메모리 관리 헤더 구조체
*/
#pragma pack(1)
typedef struct {
#ifdef _WIN64
  uint32_t _padding64;
#endif //#ifdef _WIN64
  uint16_t reserved;            ///< 예약공간
  uint16_t elemSize;            ///< 요소 사이즈
  int32_t refCnt;               ///< 참조 카운트
  int32_t length;               ///< 실제 데이터 사이즈
} DVEC_REC;
#pragma pack()

DVEC WA_API DVEC_Init(void);
DVEC WA_API DVEC_InitWith(DVEC src);
/**--------------------------------------------------------------------------
@brief DVEC 초기화 함수

배열의 길이와 요소 크기로 초기화

@param [in ] length   배열의 길이
@param [in ] elemSize 요소의 크기(bytes)
@return 초기화된 DVEC

@author ksg
*/
static inline DVEC WA_API DVEC_InitWithLength(int length, uint16_t elemSize) {return (length * elemSize > 0) ? rcm_init_len_ctx(length, (((uint32_t)elemSize) << 16)) : NULL;}
/**--------------------------------------------------------------------------
@brief DVEC 초기화 함수

배열의 크기(bytes)로 초기화. 요소의 크기는 1byte로 지정됨

@param [in ] Size 배열의 크기(bytes)
@return 초기화된 DVEC

@author ksg
*/
static inline DVEC WA_API DVEC_InitWithSize(int Size) {return (Size > 0) ? rcm_init_len_ctx(Size, 0x00010000) : NULL;}
/**--------------------------------------------------------------------------
@brief DVEC을 소멸시킴

@param [in ] dst 대상 DVEC의 주소
@return: Free후 refCnt값, 0이면 메모리 해제됨을 의미

@author ksg
*/
static inline int WA_API DVEC_Free(DVEC *dst) {return rcm_free(dst);}
/// DVEC을 대입 (rcm_assign 참조)
static inline void WA_API DVEC_Assign(DVEC *dst, DVEC src) {rcm_assign(dst, src);}
void WA_API DVEC_MakeUnique(DVEC *dst);
/// DVEC의 길이를 변경 (rcm_set_len 참조)
static inline void WA_API DVEC_SetLength(DVEC *dst, int length, uint16_t elemSize) {rcm_set_len_ctx(dst, length, (((uint32_t)elemSize) << 16));}
int WA_API DVEC_Add(DVEC *dst, int length, uint16_t elemSize);
void * WA_API DVEC_Append(DVEC *dst, int length, uint16_t elemSize);
/// DVEC의 특정 인덱스의 요소를 삭제 (rcm_delete 참조)
static inline void WA_API DVEC_Delete(DVEC *dst, int idx) {rcm_delete(dst, idx);}
/// DVEC의 크기를 변경 (rcm_set_len 참조)
static inline void DVEC_SetSize(DVEC *dst, int Size) {rcm_set_len_ctx(dst, Size, 0x00010000);}
/// DVEC의 참조 카운트가 1인지 점검 (rcm_is_unique 참조)
static inline bool DVEC_IsUnique(DVEC src) { return rcm_is_unique(src);}
/// DVEC의 크기(bytes) 조회 (rcm_get_size 참조)
static inline int WA_API DVEC_Size(const DVEC src) {return rcm_get_size(src);}
/// DVEC의 길이 조회 (rcm_get_len 참조)
static inline int WA_API DVEC_Length(const DVEC src) {return rcm_get_len(src);}
int WA_API DVEC_ElemSize(DVEC src);
DVEC_REC * WA_API DVEC_GetRec(DVEC src);

void * WA_API DVEC_Element(DVEC src, int Index);

// File IO
//int DVEC_SaveToFILE(FILE *file, DVEC src, bool closeFile);
int DVEC_SaveToFileA(const char *fileName, DVEC src);
//int DVEC_LoadFromFILE(DVEC *dst, FILE *file, bool closeFile);
int DVEC_LoadFromFileA(DVEC *dst, const char *fileName);

/// rcm_insert 함수를 템플릿처럼 사용하기 위한 매크로
#define DVEC_INSERT(type, dst, index, count) ((type*)rcm_insert_len_ctx((void**)(dst), (index), (count), (sizeof(type)<<16)))
/// DVEC_Append 함수를 템플릿처럼 사용하기 위한 매크로
#define DVEC_APPEND(type, dst, count) ((type*)DVEC_Append((DVEC*)(dst), (count), sizeof(type)))
/// DVEC_Free 함수를 템플릿처럼 사용하기 위한 매크로
#define DVEC_FREE(dst) DVEC_Free((DVEC*)(dst))
/// DVEC_Assign 함수를 템플릿처럼 사용하기 위한 매크로
#define DVEC_ASSIGN(dst, src) DVEC_Assign((DVEC*)(dst), (src))
/// DVEC_MakeUnique 함수를 템플릿처럼 사용하기 위한 매크로
#define DVEC_UNIQUE(dst) DVEC_MakeUnique((DVEC*)(dst))
/// DVEC_Add 함수를 템플릿처럼 사용하기 위한 매크로
#define DVEC_ADD(type, dst, count) DVEC_Add((DVEC*)(dst), (count), sizeof(type))
/// DVEC_Delete 함수를 템플릿처럼 사용하기 위한 매크로
#define DVEC_DELETE(dst, idx) DVEC_Delete((DVEC*)(dst), (idx))
/// DVEC_SetLength 함수를 템플릿처럼 사용하기 위한 매크로
#define DVEC_SETLENGTH(type, dst, count) DVEC_SetLength((DVEC*)(dst), (count), sizeof(type))
/// DVEC_MakeUnique 함수를 템플릿처럼 사용하기 위한 매크로
#define DVEC_MAKEUNIQUE(dst) DVEC_MakeUnique((DVEC*)(dst))
/// 특정 타입의 포인터를 DVEC으로 선언하고 이를 초기화 하는 매크로
#define DVEC_WITH_LENGTH(type, name, count) \
type * name = (type *)DVEC_InitWithLength((count), sizeof(type))


#define DSTACK_EXPERIMENTAL 1

#if DSTACK_EXPERIMENTAL == 1

#define DSTACK_OF_TYPE(type) \
  struct { \
    type *data; \
    int16_t top; \
  }

#define DSTACK_INIT(storage) storage.data = NULL, storage.top = 0
#define DSTACK_POP(storage) storage.data[--storage.top]
#define DSTACK_PEEP(storage) storage.data[storage.top-1]
#define DSTACK_PUSH(storage, val) { \
  if ((storage).top >= DVEC_Length((storage).data)) { \
    DVEC_Append((void*)&((storage).data), 4, sizeof(*((storage).data))); \
  } \
  (storage).data[(storage).top++] = (val);  \
}
#define DSTACK_FREE(storage) DVEC_Free((void*)&((storage).data))


#endif //DSTACK_EXPERIMENTAL == 1

#define RINGBUF_EXPERIMENTAL 1

#if RINGBUF_EXPERIMENTAL == 1

typedef struct tagStructInfo {
  const char *name;
  void (*constructor) (void *target, int count);
  void (*destructor) (void *target, int count);
} TStructInfo;

#define RINGBUF_OF_TYPE(type, max_len) \
  struct { \
    type data[max_len]; \
  }
#define RINGBUF_Init(target, type) \
  get_type_info(#type)->constructor(target, sizeof(target) / sizeof(type));

#define RINGBUF_Free(target, type) \
  get_type_info(#type)->constructor(target, sizeof(target) / sizeof(type));


#endif //#if RINGBUF_EXPERIMENTAL == 1


//---------------------------------------------------------------------------
/*
#pragma pack(1)
typedef struct {
  uint16_t c_stride; ///< 다음 열(column)로 이동하기 위해 더해야 할 값 (byte단위)
  uint16_t r_stride; ///< 다음 행(row)으로 이동하기 위해 더해야 할 값 (byte단위) 0인 경우 1차원으로 해석해야 함.
  int32_t refCnt; ///< 참조 카운트
  union {
    uint32_t as1D; /// 1차원 배열의 경우 길이
    struct {
      uint16_t col; /// 2차원 배열 열 수
      uint16_t row; /// 2차원 배열 행 수
    } as2D;
  } dim;
} UMAT_REC;
#pragma pack()

int WA_API UMAT_REC_DataSize(UMAT_REC *R);

typedef void * UMAT;

inline UMAT WA_API UMAT_Init(void) {return 0;}
UMAT WA_API UMAT_InitWith(UMAT src);
UMAT WA_API UMAT_InitWithLength(uint32_t length, uint16_t elemSize);
UMAT WA_API UMAT_InitWithDimension(uint16_t row, uint16_t col, uint16_t elemSize);
inline UMAT WA_API UMAT_InitWithSize(uint32_t size) {return UMAT_InitWithLength(size,1);}
int WA_API UMAT_Free(DVEC *dst); ///< @return: Free후 참조 카운트 수. 0이면 메모리 해제됨을 의미

void WA_API UMAT_Assign(UMAT *dst, UMAT src);
void WA_API UMAT_MakeUnique(UMAT *dst);
void WA_API UMAT_SetDimension(UMAT *dst, int length, uint16_t elemSize);
void WA_API UMAT_ChangeDimension(UMAT *dst, int length, uint16_t elemSize);
*/


#if defined(__cplusplus)
}
#endif // defined(__cplusplus)


#if defined(__cplusplus)
/**
@brief C++환경에서 사용할 수 있는 DVEC에 대한 템플릿 클래스 정의
@details 모든 맴버 함수는 인라인으로 구현되어 있으므로 별도의 구현부가 필요하지 않게 설계되었다.
*/
template <class T>
class TDynVec
{
protected:
  /// @brief 데이터
  DVEC Data;
public:
  /// @brief 기본 생성자
  inline TDynVec() : Data(NULL) {}
  /// @brief DVEC으로부터 복사 생성자 @param [in] rhs 원본
  inline TDynVec(const DVEC &rhs) : Data(DVEC_InitWith(rhs)) {}
  /// @brief 복사 생성자 @param [in] rhs 원본
  inline TDynVec(const TDynVec<T> &rhs) : Data(DVEC_InitWith(rhs.Data)) {}
  /// @brief 길이 지정 생성자 @param [in] len 길이
  inline TDynVec(int len) : Data(DVEC_InitWithLength(len, sizeof(T))) {}
  /// @brief 일반 배열로부터 복사 생성자 @param [in] src 원본 배열 @param [in] 원본 배열의 길이
  inline TDynVec(T *src, int len) : Data(NULL) {SetLength(len); memcpy(Data, src, len * sizeof(T));}
//	inline TDynVec(const T &src) : Data(NULL) {SetLength(1); ((T*)Data)[0] = src;}
  /// @brief 소멸자
  inline ~TDynVec() {DVEC_Free(&Data);}
  /// @brief 데이터 삭제
  inline void Clear(void) {DVEC_Free(&Data);}

  /// @brief 대입 연산자(메모리 공유) @param [in] rhs 원본
  /// @return 자신에 대한 참조
  inline TDynVec & operator=(const TDynVec &rhs) {DVEC_Assign(&Data, rhs.Data); return *this;}
  /// @brief DVEC에 대한 대입 연산자(메모리 공유) @param [in] rhs 원본
  /// @return 자신에 대한 참조
  inline TDynVec & operator=(const DVEC &rhs) {if (DVEC_ElemSize(rhs) == sizeof(T)) DVEC_Assign(&Data, rhs); return *this;}
  /// @brief 메모리를 단독으로 소유하도록 변경 (메모리를 공유하고 있을 때는 복사하여 새로 생성)
  inline void Unique(void) {DVEC_MakeUnique(&Data);}
  /// @brief 메모리 길이 변경 @param [in] len 새로운 길이
  inline void SetLength(int len) {DVEC_SetLength(&Data, len, sizeof(T));}
  /// @brief 메모리를 단독 소유하고 있는지 조회
  /// @return 단독 소유한 경우 true, 공유시 false
  inline bool IsUnique(void) {return DVEC_IsUnique(Data);}
  /// @brief 데이터를 가지고 있지 않은지 조회
  /// @return 데이터가 없는 경우 true, 있는 경우 false
  inline bool IsEmpty(void) {return (Data == NULL);}
  /// @brief 길이 조회
  /// @return 길이. 비어있는 경우 0
  inline int Length(void) {return DVEC_Length(Data);}
  /// @brief 요소 크기 조회
  /// @return 요소 크기 (in bytes). sizeof(T)와 일치함
  inline int ElemSize(void) {return sizeof(T);}
  /// @brief 관리용 레코드 구조체 조회
  /// @return 관리용 레코드 구조체의 주소
  inline DVEC_REC * GetRec(void) {return DVEC_GetRec(Data);}
  /// @brief 메모리 길이 연장 @param length 추가할 길이 
  /// @return 추가된 메모리의 첫 시작 위치(인덱스 값)
  inline int Add(int length=1) {return DVEC_Add(&Data, length, sizeof(T));}
  /// @brief 메모리 길이 연장 @param length 추가할 길이
  /// @return 추가된 메모리의 첫 시작 위치(주소) @details Add함수와 동일하나 반환값이 주소값이다.
  inline T* Append(int length=1) {return (T*)DVEC_Append(&Data, length, sizeof(T));}
  /// @brief 요소 삭제 @param index 삭제할 요소 위치(인덱스 값)
  inline void Delete(int index) {DVEC_Delete(&Data, index);}
  /// @brief 데이터 영역의 주소 반환 @details 반환된 포인터는 일반 배열처럼 접근할 수 있다.
  /// @return 데이터 배열 주소
  inline T* data(void) {return (T*)Data;}

  /// @brief 동적 배열의 내용을 파일로 저장 (ANSI버전) 
  /// @param [in] fileName 저장할 파일 이름(ANSI/UTF8문자열) 
  /// @return 저장한 크기(in bytes)
  inline int SaveToFileA(char *fileName) {return DVEC_SaveToFileA(fileName, Data);}
  /// @brief 파일로부터 동적 배열을 읽어들임 (ANSI버전) 
  /// @param [in] fileName 파일 이름(ANSI/UTF8문자열) 
  /// @return 읽어온 크기(in bytes)
  inline int LoadFromFileA(char *fileName) {return DVEC_LoadFromFileA(&Data, fileName);}

  /// @brief 요소 접근용 [] 연산자 
  /// @param [in] idx 인덱스 값 
  /// @warning 성능을 위해 인덱스에 대한 범위 검사를 하지 않으므로 주의해서 사용해야 한다.
  /// @return 해당 인덱스의 값
  inline T& operator[] (int idx) {return ((T*)Data)[idx];}
  /// @brief DVEC타입으로 캐스팅 연산자 @return DVEC타입의 동적 배열
  inline operator DVEC() {return Data;}

  // 수학 연산
  /// @brief += 연산자 @details 두 동적 배열의 요소간 += 연산 수행
  inline TDynVec<T> & operator+=(const T &v) {for (int i = 0, len = DVEC_Length(Data); i < len; i++) ((T*)Data)[i] += v; return *this;}
  /// @brief -= 연산자 @details 두 동적 배열의 요소간 += 연산 수행
  inline TDynVec<T> & operator-=(const T &v) {for (int i = 0, len = DVEC_Length(Data); i < len; i++) ((T*)Data)[i] -= v; return *this;}
  /// @brief *= 연산자 @details 두 동적 배열의 요소간 += 연산 수행
  inline TDynVec<T> & operator*=(const T &v) {for (int i = 0, len = DVEC_Length(Data); i < len; i++) ((T*)Data)[i] *= v; return *this;}
  /// @brief /= 연산자 @details 두 동적 배열의 요소간 += 연산 수행
  inline TDynVec<T> & operator/=(const T &v) {for (int i = 0, len = DVEC_Length(Data); i < len; i++) ((T*)Data)[i] /= v; return *this;}
};
#endif



#endif
