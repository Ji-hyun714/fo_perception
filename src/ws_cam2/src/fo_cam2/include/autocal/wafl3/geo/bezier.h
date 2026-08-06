/**--------------------------------------------------------------------------
@file bezier.h
@brief bezier.c 파일의 헤더
@author ksg
*/
//---------------------------------------------------------------------------

#ifndef bezierH
#define bezierH
//---------------------------------------------------------------------------
#include "system/mem_dynamic.h"
#if MCNEX
#include "api/api_mcnex.h"
#endif
#define MAX_UVORDER 12

#if defined(__cplusplus)
extern "C" {
#endif // defined(__cplusplus)

/**--------------------------------------------------------------------------
@brief 다중 Bezier 곡면 구현을 위해 테스트했던 구조체 (폐기 예정)
@author ksg
*/
#pragma pack(1)
typedef struct tagBEZIER_PATCH {
  size2u8 cnt; ///< w,h 방향으로 패치 수
  uint8_t w_order; ///< 가로 방향 차수 // 향후 배열형을 쓸 수 있게 개선해 보자
  uint8_t v_order; ///< 세로 방향 차수 // 향후 배열형을 쓸 수 있게 개선해 보자
  DVEC cp; // 전체 cp
} BEZIER_PATCH;
#pragma pack()


#pragma pack(1)
/**--------------------------------------------------------------------------
@brief DVEC기반인 BEZIER2 타입으로부터 Bezier 곡면 정보를 추출, 저장하기
위한 구조체
@author ksg
*/
typedef struct tagBEZIER_INFO {
  uint8_t u_count;   ///< u방향 패치 수
  uint8_t v_count;   ///< v방향 패치 수
  uint8_t u_order;   ///< u방향 차수
  uint8_t v_order;   ///< v방향 차수
  uint16_t cp_dim;   ///< 조종 점(cp: Control Point)의 차원. 2차원인 경우 2, 3차원인 경우 3
  int cp_cnt;      ///< 조종 점의 수. (u_order * u_patch_cnt + 1) * (v_order * v_patch_cnt + 1)과 일치
} BEZIER_INFO;
#pragma pack()

/* 사용되지 않는 구조체
typedef struct {
  DVEC u;
  DVEC v;
  DVEC points;
} POINT_SAMPLE;
*/

extern const int * BEZIER_GetBinomial(int order); 

/**--------------------------------------------------------------------------
@brief 베지어 곡면 파라미터 저장용 구조체 (폐기)

현재는 사용되지 않는다. 향후 현재 사용되고 있는 BEZIER2의 약점을 보완하여
재개발 할 예정이다.
@author ksg
*/
#pragma pack(1)
typedef struct {
  DVEC u_binomial; ///< u방향 binomial (int 타입)
  DVEC v_binomial; ///< v방향 binomial (int 타입)
  DVEC cp; ///< 컨트롤 포인트 배열. 2차원 포인트의 경우 sizeof(real_t) * 2를 ElemSize로 지정
} BEZIER;
#pragma pack()

// Init/Free Functions
/**--------------------------------------------------------------------------
@brief BEZIER 구조체 초기화 (폐기)

모든 파라미터를 0으로 설정
@return 초기화된 BEZIER 구조체

@author ksg
*/
static inline BEZIER BEZIER_Init(void) {BEZIER r={0,}; return r;}
extern BEZIER BEZIER_InitCurve(int u_order, int dim);
extern BEZIER BEZIER_InitSurface(int u_order, int v_order, int dim);
extern BEZIER BEZIER_InitCurveWithBinomialAndControlPoints(DVEC binomial, DVEC controlPoints);
extern BEZIER BEZIER_InitSurfaceWithBinomialAndControlPoints(DVEC u_binomial, DVEC v_binomial, DVEC controlPoints);
extern BEZIER BEZIER_InitCurveWithBinomialAndDimension(DVEC binomial, int dim);
extern BEZIER BEZIER_InitSurfaceWithBinomialAndDimension(DVEC u_binomial, DVEC v_binomial, int dim);
extern BEZIER BEZIER_InitWith(const BEZIER *src);
extern void BEZIER_Free(BEZIER *dst);

// Query Functions
/**--------------------------------------------------------------------------
@brief BEZIER 구조체로부터 u방향 차수 조회 (폐기)

@param [in ] src BEZIER 구조체
@return 차수

@author ksg
*/
static inline int BEZIER_UOrder(const BEZIER *src) {int order=DVEC_Length(src->u_binomial); return (order == 0) ? 0 : order - 1;}
/**--------------------------------------------------------------------------
@brief BEZIER 구조체로부터 v방향 차수 조회 (폐기)

@param [in ] src BEZIER 구조체
@return 차수

@author ksg
*/
static inline int BEZIER_VOrder(const BEZIER *src) {int order=DVEC_Length(src->v_binomial); return (order == 0) ? 0 : order - 1;}
/**--------------------------------------------------------------------------
@brief BEZIER 구조체로부터 조종점 차원 조회

@param [in ] src BEZIER 구조체
@return 조종점 벡터의 차원

@author ksg
*/
static inline int BEZIER_Dimension(const BEZIER *src) {return DVEC_ElemSize(src->cp) / sizeof(real_t);}

// Set Functions
extern int BEZIER_Set(BEZIER *dst, DVEC u_binomial, DVEC v_binomial, DVEC controlPoints);
extern int BEZIER_SetControlPoints(BEZIER *dst, DVEC cp);

// Sampling
extern int BEZIER_Sample(DVEC *dst, BEZIER *src, DVEC u, DVEC v);

// Transform
extern void BEZIER_Move(BEZIER *dst, real_t *offset);
extern void BEZIER_Planar(BEZIER *dst);


//extern DVEC InitBinomial(int order);
/**--------------------------------------------------------------------------
@brief int형 동적 배열(DVEC) 생성

@param [in ] length 필요한 배열의 길이
@return 동적 할당된 DVEC

@author ksg
*/
static inline DVEC InitArrayOfInt(int length) {return DVEC_InitWithLength(length, sizeof(int));}
/**--------------------------------------------------------------------------
@brief real_t형 동적 배열(DVEC) 생성

@param [in ] length 필요한 배열의 길이
@return 동적 할당된 DVEC

@author ksg
*/
static inline DVEC InitArrayOfreal_t(int length) {return DVEC_InitWithLength(length, sizeof(real_t));}
/**--------------------------------------------------------------------------
@brief VEC2형 동적 배열(DVEC) 생성

@param [in ] length 필요한 배열의 길이
@return 동적 할당된 DVEC

@author ksg
*/
static inline DVEC InitArrayOfVEC2(int length) {return DVEC_InitWithLength(length, sizeof(vec2));}
/**--------------------------------------------------------------------------
@brief VEC3형 동적 배열(DVEC) 생성

@param [in ] length 필요한 배열의 길이
@return 동적 할당된 DVEC

@author ksg
*/
static inline DVEC InitArrayOfVEC3(int length) {return DVEC_InitWithLength(length, sizeof(vec3));}

DVEC InitSamplePositions(real_t begin, real_t end, int segments);
void SetSamplePositions(DVEC *dst, real_t begin, real_t end, int segments);


int binomial_coeff(int n, int i);
void get_binomial_coeff_vector(int *dst, int n); // the length of dst should be n+1

void calc_bezier_point(real_t *dst, const int *binomial, real_t *controlPoints, real_t t, int n, int dim);
void calc_bezier_point2(real_t *dst, int dst_dim, int order, const int *binomial, real_t *controlPoints, int cp_dim, real_t t);

void calc_bezier_curve(real_t *dst, int *binomial, real_t *controlPoints, real_t t_min, real_t t_max, int t_cnt, int n, int dim);
void calc_bezier_curve2(real_t *dst, int dst_dim, int order, real_t *controlPoints, int cp_dim, real_t t_min, real_t t_max, int segments);

void calc_bezier_points(real_t *dst, int *binomial, real_t *controlPoints, real_t *t, int t_cnt, int n, int dim);
int calc_bezier_surface(real_t *dst, int *u_b, int u_order, int *v_b, int v_order,
          real_t *cp, int cp_dim, real_t *pu, int pu_cnt, real_t *pv, int pv_cnt );

/**--------------------------------------------------------------------------
@brief 베지어 곡면 샘플링에 필요한 파라미터를 보관하기 위한 구조체
*/
typedef struct tagBEZIER_SAMPLE_INFO {
  real_t *dst;        ///< 결과를 저장할 좌표 배열
  vec2i32 dst_stride;  ///< 결과 좌표 저장시 x 및 y 방향으로 건너뛸 stride값

  struct { 
    int *u;  ///< u방향 차수의 바이노미얼 벡터 주소
    int *v;  ///< v방향 차수의 바이노미얼 벡터 주소
  } binomial; ///< 바이노미얼 벡터 정보

  real_t *cp; ///< 조종점 배열
  vec2i32 cp_stride; ///< 조종점 배열 선택시 x 및 y방향으로 건너뛸 stride값

  real_t *sample_u; ///< u방향 샘플링 포인트 배열
  int     sample_u_cnt; ///< u방향 샘플링 포인트 수
  real_t *sample_v; ///< v방향 샘플링 포인트 배열
  int     sample_v_cnt; ///< v방향 샘플링 포인트 수

} BEZIER_SAMPLE_INFO; 

int calc_bezier_surface2(BEZIER_INFO bzr, BEZIER_SAMPLE_INFO bsi);


//void BEZIER_InitSystem(void);
//void BEZIER_FreeSystem(void);



/**--------------------------------------------------------------------------
@brief BEZIER2 타입 선언

BEZIER2 타입은 DVEC 타입으로 선언되며 곡면 파라미터는 RCM레코드에 저장된다.
@author ksg
*/
typedef DVEC BEZIER2; 
/**--------------------------------------------------------------------------
@brief BEZIER2 기본 초기화

@return NULL로 초기화된 BEZIER2

@author ksg
*/
static inline BEZIER2 BEZIER2_Init(void) {return 0;}
BEZIER2        BEZIER2_InitCurve(uint8_t u_order, uint8_t dim);
BEZIER2        BEZIER2_InitSurface(uint8_t u_order, uint8_t v_order, uint8_t dim);
BEZIER2        BEZIER2_InitSurfacePatch(uint8_t u_count, uint8_t v_count, uint8_t u_order, uint8_t v_order, uint8_t dim);
/**--------------------------------------------------------------------------
@brief BEZIER2 복사 초기화

@param [in ] src 곡면
@return 원본과 메모리를 공유하는 BEZIER2

@author ksg
*/
static inline BEZIER2 BEZIER2_InitWith(const BEZIER2 src) {return (BEZIER2)DVEC_InitWith((DVEC)src);}
/**--------------------------------------------------------------------------
@brief BEZIER2 소멸

@param [out] dst 소멸시킬 대상

@author ksg
*/
static inline void    BEZIER2_Free(BEZIER2 *dst) {DVEC_FREE(dst);}

int BEZIER2_Info(BEZIER_INFO *dst, BEZIER2 src);
/**--------------------------------------------------------------------------
@brief u방향 차수 조회

BEZIER2의 RCM 레코드로부터 u방향 차수를 추출한다. 
u방향 차수는 reserved 영역의 0~3bit 위치에 저장되어 있다. (최대 15차)

@param [in ] src 대상
@return u방향 차수

@author ksg
*/
static inline int BEZIER2_UOrder(const BEZIER2 src) {DVEC_REC *pRec; pRec=DVEC_GetRec(src); return (pRec) ? (pRec->reserved & 0x0fU) : 0;}
/**--------------------------------------------------------------------------
@brief v방향 차수 조회

BEZIER2의 RCM 레코드로부터 v방향 차수를 추출한다. 
u방향 차수는 reserved 영역의 8~11bit 위치에 저장되어 있다. (최대 15차)

@param [in ] src 대상
@return u방향 차수

@author ksg
*/
static inline int BEZIER2_VOrder(const BEZIER2 src) {DVEC_REC *pRec; pRec=DVEC_GetRec(src); return (pRec) ? ((pRec->reserved >> 8) & 0x0fU): 0;}
int * BEZIER2_UBinomial(const BEZIER2 src);
int * BEZIER2_VBinomial(const BEZIER2 src);
/**--------------------------------------------------------------------------
@brief 곡면의 조종점 차원을 조회

@param [in ] src 곡면 
@return 조종점 벡터의 차원

@author ksg
*/
static inline int BEZIER2_Dimension(const BEZIER2 src) {return DVEC_ElemSize(src) / sizeof(real_t);}

// Sampling
int BEZIER2_Sample(real_t *dst, int dst_dim, BEZIER2 src, vec2 const *uv, int count);
int BEZIER2_SampleCrossPoints(DVEC *dst, int dst_dim, BEZIER2 src, DVEC u, DVEC v);
int BEZIER2_SampleWithParams(DVEC *dst, int dst_dim, BEZIER2 src, real_t u_begin, real_t u_end, int u_segments, real_t v_begin, real_t v_end, int v_segments);
int BEZIER2_SampleOutline(DVEC *dst, int dst_dim, BEZIER2 src, int u_segments, int v_segments);

int BEZIER2_SampleOutlineToArray(real_t *dst, int dst_dim, BEZIER2 src, int u_segments, int v_segments);

real_t BEZIER2_GetUV(real_t *dst, BEZIER2 src, vec3 p);
void   BEZIER2_GetXYZ_Implicit(real_t *dst, int dst_dim, BEZIER2 src, real_t u, real_t v);
void   BEZIER2_GetXYZ_DeCastel(real_t *dst, int dst_dim, BEZIER2 src, real_t u, real_t v);
//#define BEZIER2_GetXYZ     BEZIER2_GetXYZ_Implicit
#define BEZIER2_GetXYZ     BEZIER2_GetXYZ_DeCastel
//static inline void BEZIER2_GetXYZ(real_t *dst, int dst_dim, BEZIER2 src, real_t u, real_t v){BEZIER2_GetXYZ_DeCastel(dst, dst_dim, src, u, v);}

void BEZIER2_Planar(BEZIER2 *dst);
void BEZIER2_PlanarWithBoundary(BEZIER2 *dst, xywh2 *bounds);

void BEZIER2_FlipX(BEZIER2 *dst, float x_base);
void BEZIER2_MirrorX(BEZIER2 *dst, float x_base);
void BEZIER2_ReformX(BEZIER2 *dst, float x_base);
void BEZIER2_FlipY(BEZIER2 *dst, float y_base);
void BEZIER2_ReformY(BEZIER2 *dst, float x_base);
void BEZIER2_ReformXY(BEZIER2 *dst, float ax, float bx, float ay, float by);
void BEZIER2_ReformXYCorner(BEZIER2 *dst, float sx_left, float sy_left, float sx_right, float sy_right);
void BEZIER2_ROTATE(BEZIER2 *dst, float angle);
#if MCNEX
void BEZIER2_DISTORT(BEZIER2 *dst, BEZIER2 *bzr1, BEZIER2 *bzr2, MCNEX_PARAM p, int img_width, int img_height, float ref_rate);
void BEZIER2_TRANSFORM(BEZIER2 *dst, MCNEX_PARAM p, int img_width, int img_height);
void BEZIER2_CLONE(BEZIER2 *src, BEZIER2 *tgt);
void BEZIER2_GET_POINT(BEZIER2 *src, vec3f *p, int index);
void BEZIER2_COPY_CP(BEZIER2 *dst, float *cp);
#endif //#if MCNEX

void BEZIER2_MakeSymmetryQuadrant1(BEZIER2 obj);
void BEZIER2_MakeSymmetryX(BEZIER2 obj, int option);



#if defined(__cplusplus)
}
#endif // defined(__cplusplus)



#if defined(__cplusplus)
/**--------------------------------------------------------------------------
@brief BEZIER 구조체로 직접 캐스팅 가능한 2차원 베지어 곡면에 대한 C++ 버전 클래스 (폐기)
*/
class T2DBezier {
public:
  TDynVec<int> u_binomial; ///< u방향 binomial (int 타입)
  TDynVec<int> v_binomial; ///< v방향 binomial (int 타입)
  TDynVec<vec2> cp; ///< 컨트롤 포인트 배열. 2차원 포인트의 경우 sizeof(real_t) * 2를 ElemSize로 지정
};
/**--------------------------------------------------------------------------
@brief BEZIER 구조체로 직접 캐스팅 가능한 3차원 베지어 곡면에 대한 C++ 버전 클래스 (폐기)
*/
class T3DBezier {
public:
  TDynVec<int> u_binomial; ///< u방향 binomial (int 타입)
  TDynVec<int> v_binomial; ///< v방향 binomial (int 타입)
  TDynVec<vec3> cp; ///< 컨트롤 포인트 배열. 2차원 포인트의 경우 sizeof(real_t) * 2를 ElemSize로 지정
};
#endif



#endif
