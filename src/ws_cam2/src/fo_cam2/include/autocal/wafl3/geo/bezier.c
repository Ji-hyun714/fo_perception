/**--------------------------------------------------------------------------
@file bezier.c
@brief Bezier 곡선 및 곡면 관련 함수 구현

### 배경 이론

베지어 곡선은 다음과 같이 정의된다.

\f$\mathbf{B}(t) = \sum_{i=0}^n \mathbf{b}_{i,n}(t)\mathbf{P}_i,\quad t\in[0,1]\f$

이 때 다항식

\f$\mathbf{b}_{i,n}(t) = {n\choose i} t^i (1-t)^{n-i},\quad i=0,\ldots n\f$

은 \f$n\f$차 번스타인 기저 다항식(Bernstein basis polynomials)이라고 한다.

자연수 \f$n\f$과 정수 \f$i\f$에 대한 이항계수(binomial coefficient)
\f$\scriptstyle {n \choose i}\f$ 는 \f$n\f$개의 서로 다른 물건 중에서 순서 없이
\f$i\f$개를 뽑는 조합의 가짓수로, \f$^n{\mathbf{C}}_i\f$ 또는
\f${{\mathbf{C}}_i}^n\f$ 로도 표기하며 다음과 같이 정의된다.

\f${n \choose i} = \frac{n!}{i!(n-i)!}.\f$

\f$\mathbf{P}_i\f$는 조종점(control points)을 의미하며 베지어 곡선은 첫번째 조종점과 마지막 조종점 만을 통과한다.

### 참고 사항

현재 베지어 곡선/곡면은 BEZIER 와 BEZIER2, 두가지 버전이 구현이 존재한다.
BEZIER는 전통적인 구조체 방식의 구현으로 단일 곡면만을 지원하고
BEZIER2는 DVEC 기반의 구현으로 다중 곡면까지 지원된다.

BEZIER2는 BEZIER에 비해 곡선/곡면의 조종점을 서로 공유할 수 있는 장점이
있는 반면 구현 방식이 직관적이지 않고 곡선/곡면의 파라미터 정보를 RCM
(Reference Counting Memory, mem_dynamic.c 참조)의 context 필드에 저장하기
때문에 향후 필요할 수도 있는 보다 복잡한 형식의 곡면 구현에 제약이 따르는
단점이 있다.

BEZIER 구조체와 BEZIER_로 시작되는 관련 함수들은 모두 폐기된 버전으로 봐야
하며 향후 BEZIER2를 대체하는 새로운 버전으로 재 작성될 예정에 있다.

### BEZIER2 파라미터 저장 방식

BEZIER2는 DVEC 타입이며 베지어 곡면 파라미터는 RCM 레코드의 context 영역
(16bit)에 저장된다.

RCM 컨텍스트 사용 방식

|      |elemSize|v_count|v_order|u_count|u_order|
|------|--------|-------|-------|-------|-------|
|length| 16bit  | 4bit  | 4bit  | 4bit  | 4bit  |

BEZIER2 타입에서 곡면 파라미터 추출은 BEZIER2_Info 함수를 통해 안전하게
처리할 수 있다.

@author ksg
*/
//---------------------------------------------------------------------------

#include <math.h>
#if (defined(CCS) || defined(__BORLANDC__))
#pragma hdrstop
#endif //(defined(CCS) || defined(__BORLANDC__))

#include "bezier.h"
#include "system/sys_logger.h"
#include <cstring>
#if MCNEX
#include "Projection.h"
#endif
//---------------------------------------------------------------------------

#if defined(__BORLANDC__)
#pragma package(smart_init)
#endif //defined(__BORLANDC__)



const int gBinomial0 [] =                      {1}; ///< binomial (0차)
const int gBinomial1 [] =                     {1,1}; ///< binomial (1차)
const int gBinomial2 [] =                    {1,2,1}; ///< binomial (2차)
const int gBinomial3 [] =                   {1,3,3,1}; ///< binomial (3차)
#if (MAX_UVORDER >= 4)
const int gBinomial4 [] =                  {1,4,6,4,1}; ///< binomial (4차)
#endif
#if (MAX_UVORDER >= 5)
const int gBinomial5 [] =                {1,5,10,10,5,1}; ///< binomial (5차)
#endif
#if (MAX_UVORDER >= 6)
const int gBinomial6 [] =               {1,6,15,20,15,6,1}; ///< binomial (6차)
#endif
#if (MAX_UVORDER >= 7)
const int gBinomial7 [] =             {1,7,21,35,35,21,7,1}; ///< binomial (7차)
#endif
#if (MAX_UVORDER >= 8)
const int gBinomial8 [] =           {1,8,28,56,70,56,28,8,1}; ///< binomial (8차)
#endif
#if (MAX_UVORDER >= 9)
const int gBinomial9 [] =        {1,9,36,84,126,126,84,36,9,1}; ///< binomial (9차)
#endif
#if (MAX_UVORDER >= 10)
const int gBinomial10[] =     {1,10,45,120,210,252,210,120,45,10,1}; ///< binomial (10차)
#endif
#if (MAX_UVORDER >= 11)
const int gBinomial11[] =   {1,11,55,165,330,462,462,330,165,55,11,1}; ///< binomial (11차)
#endif
#if (MAX_UVORDER >= 12)
const int gBinomial12[] = {1,12,66,220,495,792,924,792,495,220,66,12,1}; ///< binomial (12차)
#endif

/**--------------------------------------------------------------------------
@brief 차수별 binomial 배열의 주소를 저장하는 배열
@author ksg
*/
const int *gBinomials[] = {
	gBinomial0,
	gBinomial1,
	gBinomial2,
	gBinomial3,
#if (MAX_UVORDER >= 4)
	gBinomial4,
#endif
#if (MAX_UVORDER >= 5)
	gBinomial5,
#endif
#if (MAX_UVORDER >= 6)
	gBinomial6,
#endif
#if (MAX_UVORDER >= 7)
	gBinomial7,
#endif
#if (MAX_UVORDER >= 8)
	gBinomial8,
#endif
#if (MAX_UVORDER >= 9)
	gBinomial9,
#endif
#if (MAX_UVORDER >= 10)
	gBinomial10,
#endif
#if (MAX_UVORDER >= 11)
	gBinomial11,
#endif
#if (MAX_UVORDER >= 12)
	gBinomial12
#endif
};


/**--------------------------------------------------------------------------
@brief n! 계산

@param [in ] n n
@return 펙토리얼 값

@author ksg
*/
int factorial(int n)
{
	int i, f=1;
	for(i = 1; i <= n; i++) {
		f *= i;
	}
	return f;
}
/**--------------------------------------------------------------------------
@brief 이항계수 계산

@param [in ] n 차수
@param [in ] i 인덱스 값
@return 이항계수

@author ksg
*/
int binomial_coeff(int n, int i)
{
	// n! / (i!(n-i)!)
	return factorial(n) / (factorial(i) * factorial(n-i));
}
/**--------------------------------------------------------------------------
@brief 이항계수 벡터 계산

@param [out] dst 결과 벡터(배열형)
@param [in ] n   차수
@return Return_Description

@author ksg
*/
void get_binomial_coeff_vector(int *dst, int n)
{
	int i;
	for (i = 0; i <= n; i++)
		dst[i] = binomial_coeff(n, i);
}

/**--------------------------------------------------------------------------
@brief 베지어 곡선에서 특정 매개변수 값의 샘플링 포인트 계산 (구버전)

조종점의 차원과 샘플링 포인트의 차원이 일치할 때에만 사용할 수 있는
구 버전. 내부적으로 신 버전을 호출한다.

@param [out] dst           결과를 저장할 벡터
@param [in ] binomial      이항계수 벡터
@param [in ] controlPoints 조종점 배열
@param [in ] t             베지어 곡선 매개 변수
@param [in ] n             베지어 곡선의 차수
@param [in ] dim           차원

@author ksg
*/
void calc_bezier_point(real_t *dst, const int *binomial, real_t *controlPoints, real_t t, int n, int dim)
{
	calc_bezier_point2(dst, dim, n, binomial, controlPoints, dim, t);
}
/**--------------------------------------------------------------------------
@brief 베지어 곡선에서 특정 매개변수 값의 샘플링 포인트 계산 (신버전)

조종점의 차원과 샘플링 포인트의 차원이 일치하지 않아도 사용 가능한 신
버전
@param [out] dst           결과를 저장할 벡터
@param [in ] dst_dim       결과를 저장할 벡터의 차원
@param [in ] order         베지어 곡선의 차수
@param [in ] binomial      이항계수 벡터
@param [in ] controlPoints 조종점 배열
@param [in ] cp_dim        조종점 차원
@param [in ] t             베지어 곡선 매개 변수

@author ksg
*/
void calc_bezier_point2(real_t *dst, int dst_dim, int order, const int *binomial, real_t *controlPoints, int cp_dim, real_t t)
{
	int i, d;
	real_t b;

	if (real_is_zero(t)) {
		memcpy(dst, controlPoints, sizeof(real_t) * dst_dim);
		return;
	}
	if (real_is(t,1.0)) {
		memcpy(dst, controlPoints + cp_dim * order, sizeof(real_t) * dst_dim);
		return;
	}
	memset(dst, 0, sizeof(real_t) * dst_dim);
	for (i = 0; i <= order; i++) {
		b = (real_t)binomial[i] * (real_t)pow(t, (double)i) * (real_t)pow(real_one - t, (double)(order-i));
		for (d = 0; d < dst_dim; d++)
			dst[d] += b * controlPoints[i * cp_dim + d];
	}
}

/**--------------------------------------------------------------------------
@brief 베지어 곡선 샘플링 (구버전)

베지어 곡선을 매개변수를 t_min에서 t_max까지 t_cnt로 지정된 수 만큼 균등
분할하여 각 포인트를 샘플링 한다. 최종 샘플링된 포인트 수는 t_cnt와
일치하며 t_cnt는 최소 2이상이 되어야 한다.

조종점의 차원과 샘플링 포인트의 차원이 일치할 때에만 사용할 수 있는
구 버전. 내부적으로 신 버전을 호출한다.

@param [out] dst           샘플링 포인트를 저장할 벡터 배열의 주소
@param [in ] binomial      이항계수 벡터
@param [in ] controlPoints 조종점 배열
@param [in ] t_min         매개변수 최소 값
@param [in ] t_max         매개변수 최대 값
@param [in ] t_cnt         분할할 최종 매개변수 수
@param [in ] n             베지어 곡선의 차수
@param [in ] dim           조종점 및 샘플링 포인트의 차원

@author ksg
*/
void calc_bezier_curve(real_t *dst, int *binomial, real_t *controlPoints, real_t t_min, real_t t_max, int t_cnt, int n, int dim)
{
	calc_bezier_curve2(dst, dim, n, controlPoints, dim, t_min, t_max, t_cnt-1);
}
/**--------------------------------------------------------------------------
@brief 베지어 곡선 샘플링 (신버전)

베지어 곡선을 매개변수를 t_min에서 t_max사이를 segments 수 만큼 균등 분할하여
각 포인트를 샘플링 한다. 최종 샘플링된 포인트의 수는 segments+1이 된다.

조종점의 차원과 샘플링 포인트의 차원이 일치하지 않아도 사용 가능한 신
버전

@todo 구현에서 i값이 0이 아닌 1에서 시작하는 점이 이상함. 버그인지 확인
필요함

@param [out] dst           샘플링 포인트를 저장할 벡터 배열의 주소
@param [in ] dst_dim       샘플링 포인트의 차원
@param [in ] order         베지어 곡선의 차수
@param [in ] controlPoints 조종점 배열
@param [in ] cp_dim        조종점의 차원
@param [in ] t_min         매개변수 최소 값
@param [in ] t_max         매개변수 최대 값
@param [in ] segments      매개변수 분할 구간 수 (최소 1)

@author ksg
*/
void calc_bezier_curve2(real_t *dst, int dst_dim, int order, real_t *controlPoints, int cp_dim, real_t t_min, real_t t_max, int segments)
{
	int i;
	real_t t, t_step;
	const int *binomial;

	t_step = (t_max - t_min) / (real_t)segments;
	binomial = gBinomials[order];

	for (t = t_min, i = 1; i < segments; i++, t += t_step) {
		calc_bezier_point2(dst + i * dst_dim, dst_dim, order, binomial, controlPoints, cp_dim, t);
	}
	calc_bezier_point2(dst + segments * dst_dim, dst_dim, order, binomial, controlPoints, cp_dim, t_max);
}
/**--------------------------------------------------------------------------
@brief 베지어 곡선 샘플링 (매개변수 t 배열 기반)

베지어 곡선을 매개변수 t배열 에 지정된 위치마다 샘플링 한다.
비 균등 분할 샘플링을 위한 함수이다.

@param [out] dst           샘플링 포인트를 저장할 벡터 배열의 주소
@param [in ] binomial      이항계수 벡터
@param [in ] controlPoints 조종점 배열
@param [in ] t             매개변수 배열
@param [in ] t_cnt         매개변수 배열 길이
@param [in ] n             베지어 곡선의 차수
@param [in ] dim           조종점 및 샘플링 포인트의 차원

@author ksg
*/
void calc_bezier_points(real_t *dst, int *binomial, real_t *controlPoints, real_t *t, int t_cnt, int n, int dim)
{
	int i;
	for (i = 0; i < t_cnt; i++) {
		calc_bezier_point(dst + i*2, binomial, controlPoints, t[i], n, dim);
	}
}
/**--------------------------------------------------------------------------
@brief 베지어 곡면 샘플링 (미사용)

기존 calc_bezier_surface 함수에 사용되는 복잡한 입력 인자를 구조체를 적용하여
함수 호출 부하를 줄이고 입력 인자를 재활용할 수 있도록 수정.

@remark 현재는 BEZIER2_GetXYZ 함수로 대체되어 사용되고 있지 않음
@param [in ] bzr 베지어 곡면 파라미터
@param [in,out] bsi 베지어 샘플링 정보
@return 샘플링 된 포인트 수

@author ksg
*/
int calc_bezier_surface2(BEZIER_INFO bzr, BEZIER_SAMPLE_INFO bsi)
{
	int u_idx, v_idx;
	int i, j, d; // iterator. d:dimension

	real_t B, B_v;
	real_t u, v, zero = 0.0;
	real_t *pCP = NULL;
	int one = 1;
	DVEC pow_cache;
	real_t *dst;

	dst = bsi.dst;

	if (bsi.sample_u_cnt == 0) {
		bsi.sample_u_cnt = 1;
		bsi.sample_u = &zero;
		bsi.binomial.u = &one;
	}

	if (bsi.sample_v_cnt == 0) {
		bsi.sample_v_cnt = 1;
		bsi.sample_v = &zero;
		bsi.binomial.v = &one;
	}

	pow_cache = InitArrayOfreal_t(bzr.u_order * bzr.v_order +2);
	for (v_idx = 0; v_idx < bsi.sample_v_cnt; v_idx++, bsi.dst += bzr.cp_dim * bsi.dst_stride.y, dst = bsi.dst) {
		v = bsi.sample_v[v_idx];
		// 제곱근 캐시(v) 채우기
		for (j = 0; j <= bzr.v_order; j++) {
			if (j == 0 && real_is_zero(v)) {
				((real_t*)pow_cache)[j + bzr.u_order+1] = (real_t)bsi.binomial.v[j];
			} else if (j == bzr.v_order && real_is_one(v)) {
				((real_t*)pow_cache)[j + bzr.u_order+1] = (real_t)bsi.binomial.v[j];
			} else {
				((real_t*)pow_cache)[j + bzr.u_order+1] = (real_t)bsi.binomial.v[j] * (real_t)pow(v, (double)j) * (real_t)pow(real_one-v, (double)(bzr.v_order-j));
			}
		}

		for (u_idx = 0; u_idx < bsi.sample_u_cnt; u_idx++, dst += bzr.cp_dim * bsi.dst_stride.x) {
			u = bsi.sample_u[u_idx];

			// 제곱근 캐시(u) 채우기
			for (i = 0; i <= bzr.u_order; i++) {
				if (i == 0 && real_is_zero(u)) {
					((real_t*)pow_cache)[i] = (real_t)bsi.binomial.u[i];
				} else if (i == bzr.u_order && real_is_one(u)) {
					((real_t*)pow_cache)[i] = (real_t)bsi.binomial.u[i];
				} else {
					((real_t*)pow_cache)[i] = (real_t)bsi.binomial.u[i] * (real_t)pow(u, (double)i) * (real_t)pow(real_one-u, (double)(bzr.u_order-i));
				}
			}

			// 좌표 초기화
			for (d = 0; d < bzr.cp_dim; d++)
				dst[d] = 0;

			for (j = 0; j <= bzr.v_order; j++, pCP = bsi.cp + j * bsi.cp_stride.y) {
				B_v = ((real_t*)pow_cache)[j+bzr.u_order+1];
				for (i = 0; i <= bzr.u_order; i++, pCP += bzr.cp_dim * bsi.cp_stride.x) {
					B = B_v * ((real_t*)pow_cache)[i];
					for (d = 0; d < bzr.cp_dim; d++)
						dst[d] += B * pCP[d];
				}
			}
		}
	}

	DVEC_Free(&pow_cache);
	return bsi.sample_u_cnt * bsi.sample_v_cnt;
}

/**--------------------------------------------------------------------------
@brief 베지어 곡면 샘플링 (미사용)

@remark BEZIER 구조체 방식을 위해 개발되었으며 현재는 사용하고 있지 않음

@param [out] dst     결과를 저장할 좌표 배열
@param [in ] u_b     u방향 차수의 이항계수 벡터
@param [in ] u_order u방향 차수
@param [in ] v_b     v방향 차수의 바이너미얼 벡터
@param [in ] v_order v방향 차수
@param [in ] cp      조종점 배열
@param [in ] cp_dim  조종점 차원
@param [in ] pu      u방향 샘플링 포인트 배열
@param [in ] pu_cnt  u방향 샘플링 포인트 수
@param [in ] pv      v방향 샘플링 포인트 배열
@param [in ] pv_cnt  v방향 샘플링 포인트 수
@return Return_Description

@author ksg
*/
int calc_bezier_surface(real_t *dst, int *u_b, int u_order, int *v_b, int v_order, real_t *cp, int cp_dim, real_t *pu, int pu_cnt, real_t *pv, int pv_cnt)
{
	int u_idx, v_idx;
	int i, j, d; // iterator. d:dimension

	real_t B, B_v;
	real_t u, v, zero = 0.0;
	real_t *pCP;
	int one = 1;
	DVEC pow_cache;

	if (pu_cnt == 0) {
		pu_cnt = 1;
		pu = &zero;
		u_b = &one;
	}

	if (pv_cnt == 0) {
		pv_cnt = 1;
		pv = &zero;
		v_b = &one;
	}

	memset(dst, 0, (pu_cnt * pv_cnt * cp_dim) * sizeof(real_t));

	pow_cache = InitArrayOfreal_t(u_order+v_order+2);
	for (v_idx = 0; v_idx < pv_cnt; v_idx++) {
		v = pv[v_idx];
		for (j = 0; j <= v_order; j++) {
			if (j == 0 && real_is_zero(v)) {
				((real_t*)pow_cache)[j + u_order+1] = (real_t)v_b[j];
			} else if (j == v_order && real_is_one(v)) {
				((real_t*)pow_cache)[j + u_order+1] = (real_t)v_b[j];
			} else {
				((real_t*)pow_cache)[j + u_order+1] = (real_t)v_b[j] * (real_t)pow(v, (double)j) * (real_t)pow(real_one-v, (double)(v_order-j));
			}
		}

		for (u_idx = 0; u_idx < pu_cnt; u_idx++, dst += cp_dim) {
			u = pu[u_idx];

			for (i = 0; i <= u_order; i++) {
				if (i == 0 && real_is_zero(u)) {
					((real_t*)pow_cache)[i] = (real_t)u_b[i];
				} else if (i == u_order && real_is_one(u)) {
					((real_t*)pow_cache)[i] = (real_t)u_b[i];
				} else {
					((real_t*)pow_cache)[i] = (real_t)u_b[i] * (real_t)pow(u, (double)i) * (real_t)pow(real_one-u, (double)(u_order-i));
				}
			}

			pCP = cp;
			for (j = 0; j <= v_order; j++) {
				B_v = ((real_t*)pow_cache)[j+u_order+1];
				for (i = 0; i <= u_order; i++, pCP += cp_dim) {
					B = B_v * ((real_t*)pow_cache)[i];
					for (d = 0; d < cp_dim; d++)
						dst[d] += B * pCP[d];
				}
			}
		}
	}

	DVEC_Free(&pow_cache);
	return pu_cnt * pv_cnt * cp_dim;
}




/*
DVEC InitBinomial(int order)
{
	DVEC r = 0;
	if (order > 0) {
		r = DVEC_InitWithLength(order + 1, sizeof(int));
		get_binomial_coeff_vector((int*)r, order);
		return r;
	}
	return r;
}
*/
//int ginitCnt = 0;
//int* gBinomials[MAX_UVORDER+1];

/*
void BEZIER_InitSystem(void)
{
	int i;
	if (ginitCnt == 0) {
		gBinomials[0] = NULL;
		for (i = 1; i <= MAX_UVORDER; i++) {
			gBinomials[i] = (int*)InitBinomial(i);
		}
		ginitCnt++;
	}
}
*/
/*
void BEZIER_FreeSystem(void)
{
	int i;
	--ginitCnt;
	if (ginitCnt == 0) {
		for (i = 1; i <= MAX_UVORDER; i++) {
			DVEC_Free((DVEC*)(gBinomials+i));
		}
	}
}
*/
/**--------------------------------------------------------------------------
@brief 특정 차수에 대한 Binomial 배열 반환

@param [in] order 차수 (1~12)
@return Binomial 배열 (길이는 차수+1이 된다)

@author ksg
*/
const int * BEZIER_GetBinomial(int order)
{
		return gBinomials[order];
}

/**--------------------------------------------------------------------------
@brief 시작 값과 끝 값 사이를 균등 배분하는 배열을 초기화

@param [in ] begin    시작 값
@param [in ] end      끝 값
@param [in ] segments 분할 갯수
@return 초기화 된 DVEC

@author ksg
*/
DVEC InitSamplePositions(real_t begin, real_t end, int segments)
{
	real_t *p = NULL;
	SetSamplePositions((DVEC*)&p, begin, end, segments);
	return (DVEC)p;
}
/**--------------------------------------------------------------------------
@brief 시작 값과 끝 값 사이를 균등 배분하는 배열을 생성

최종 배열의 길이는 분할 갯수 + 1이 된다.
@param [out] dst      결과를 저장할 배열
@param [in ] begin    시작 값
@param [in ] end      끝 값
@param [in ] segments 분할 갯수

@author ksg
*/
void SetSamplePositions(DVEC *dst, real_t begin, real_t end, int segments)
{
	real_t *p, inc;
	int i;
	DVEC_SetLength(dst, segments+1, sizeof(real_t));
	p = (real_t*)(*dst);
	p[0] = begin;
	if (segments > 0) {
		inc = (end - begin) / (real_t)segments;
		p[segments] = end;
		for (i = 1; i < segments; i++)
			p[i] = begin + inc * i;
	}
}

/**--------------------------------------------------------------------------
@brief BEZIER형 곡선 초기화 (폐기)

@param [in ] u_order 차수
@param [in ] dim     조종점 차원
@return 초기화된 BEZIER 구조체

@author ksg
*/
BEZIER BEZIER_InitCurve(int u_order, int dim)
{
	return BEZIER_InitSurface(u_order, 0, dim);
}
/**--------------------------------------------------------------------------
@brief BEZIER형 곡면 초기화 (폐기)

@param [in ] u_order u방향 차수
@param [in ] v_order v방향 차수
@param [in ] dim     조종점 차원
@return 초기화된 BEZIER 구조체

@author ksg
*/
BEZIER BEZIER_InitSurface(int u_order, int v_order, int dim)
{
	BEZIER r = {0,};
	if (u_order > 0) {
		r.u_binomial = InitArrayOfInt(u_order+1);
		get_binomial_coeff_vector((int*)r.u_binomial, u_order);
	}
	if (v_order > 0) {
		r.v_binomial = InitArrayOfInt(v_order+1);
		get_binomial_coeff_vector((int*)r.v_binomial, v_order);
	}
	r.cp = DVEC_InitWithLength((u_order+1) * (v_order+1), sizeof(real_t) * dim);
	return r;
}
/**--------------------------------------------------------------------------
@brief BEZIER형 곡선 초기화 (폐기)

이항계수 벡터와 조종점 배열을 지정하여 초기화

@param [in ] binomial    이항계수 벡터
@param [in ] controlPoints 조종점 배열
@return 초기화된 BEZIER 구조체

@author ksg
*/

BEZIER BEZIER_InitCurveWithBinomialAndControlPoints(DVEC binomial, DVEC controlPoints)
{
	return BEZIER_InitSurfaceWithBinomialAndControlPoints(binomial, NULL, controlPoints);
}
/**--------------------------------------------------------------------------
@brief BEZIER형 곡면 초기화 (폐기)

이항계수 벡터와 조종점 배열을 지정하여 초기화

@param [in ] u_binomial    u방향 이항계수 벡터
@param [in ] v_binomial    v방향 이항계수 벡터
@param [in ] controlPoints 조종점 배열
@return 초기화된 BEZIER 구조체

@author ksg
*/
BEZIER BEZIER_InitSurfaceWithBinomialAndControlPoints(DVEC u_binomial, DVEC v_binomial, DVEC controlPoints)
{
	BEZIER r = {0,};
	int u_order, v_order;

	u_order = DVEC_Length(u_binomial);
	v_order = DVEC_Length(v_binomial);
	if (DVEC_Length(controlPoints) == (u_order + 1) * (v_order + 1)) {
		DVEC_Assign(&r.u_binomial, u_binomial);
		DVEC_Assign(&r.v_binomial, v_binomial);
		DVEC_Assign(&r.cp, controlPoints);
	}
	return r;
}
/**--------------------------------------------------------------------------
@brief BEZIER형 곡선 초기화 (폐기)

이항계수 벡터와 조종점의 차원을 지정하여 초기화
@param [in ] binomial    이항계수 벡터
@param [in ] dim           조종점 차원
@return 초기화된 BEZIER 구조체

@author ksg
*/
BEZIER BEZIER_InitCurveWithBinomialAndDimension(DVEC binomial, int dim)
{
	return BEZIER_InitSurfaceWithBinomialAndDimension(binomial, NULL, dim);
}
/**--------------------------------------------------------------------------
@brief BEZIER형 곡면 초기화 (폐기)

이항계수 벡터와 조종점의 차원을 지정하여 초기화
@param [in ] u_binomial    u방향 이항계수 벡터
@param [in ] v_binomial    v방향 이항계수 벡터
@param [in ] dim           조종점 차원
@return 초기화된 BEZIER 구조체

@author ksg
*/
BEZIER BEZIER_InitSurfaceWithBinomialAndDimension(DVEC u_binomial, DVEC v_binomial, int dim)
{
	BEZIER r = {0,};
	int u_order, v_order;

	u_order = DVEC_Length(u_binomial);
	v_order = DVEC_Length(v_binomial);
	DVEC_Assign(&r.u_binomial, u_binomial);
	DVEC_Assign(&r.v_binomial, v_binomial);
	r.cp = DVEC_InitWithLength((u_order+1) * (v_order+1), sizeof(real_t) * dim);
	return r;
}
/**--------------------------------------------------------------------------
@brief BEZIER형 복사 초기화 (폐기)

@param [in ] src 원본
@return 복사된 BEZEIR 구조체

@author ksg
*/
BEZIER BEZIER_InitWith(const BEZIER *src)
{
	BEZIER r = {0,};
	DVEC_Assign(&r.u_binomial, src->u_binomial);
	DVEC_Assign(&r.v_binomial, src->v_binomial);
	DVEC_Assign(&r.cp, src->cp);
	return r;
}
/**--------------------------------------------------------------------------
@brief BEZIER형 소멸 (폐기)

@param [out] dst 소멸시킬 대상

@author ksg
*/
void BEZIER_Free(BEZIER *dst)
{
	DVEC_Free(&dst->u_binomial);
	DVEC_Free(&dst->v_binomial);
	DVEC_Free(&dst->cp);
}


/**--------------------------------------------------------------------------
@brief BEZIER형 변수에 이항계수 벡터와 조종점 배열을 대입 (폐기)

@param [out] dst           대상
@param [in ] u_binomial    u방향 이항계수 벡터
@param [in ] v_binomial    v방향 이항계수 벡터
@param [in ] controlPoints 조종점 벡터
@retval WA_RESULT_OK 성공
@retval WA_RESULA_ERR 실패

@author ksg
*/
int BEZIER_Set(BEZIER *dst, DVEC u_binomial, DVEC v_binomial, DVEC controlPoints)
{
	BEZIER r = {0,};
	int u_order, v_order;
	u_order = DVEC_Length(u_binomial);
	v_order = DVEC_Length(v_binomial);
	if (DVEC_Length(controlPoints) == (u_order + 1) * (v_order + 1)) {
		DVEC_Assign(&r.u_binomial, u_binomial);
		DVEC_Assign(&r.v_binomial, v_binomial);
		DVEC_Assign(&r.cp, controlPoints);
		return WA_OK;
	}
	return WA_ERR;
}
/**--------------------------------------------------------------------------
@brief BEZIER형 변수에 조종점 배열을 대입 (폐기)

@param [out] dst 대상
@param [in ] cp  조종점 벡터
@retval WA_RESULT_OK 성공
@retval WA_RESULA_ERR 실패

@author ksg
*/
int BEZIER_SetControlPoints(BEZIER *dst, DVEC cp)
{
	if (DVEC_Length(cp) == (BEZIER_UOrder(dst) + 1) * (BEZIER_VOrder(dst) + 1)) {
		DVEC_Assign(&dst->cp, cp);
		return WA_OK;
	}
	return WA_ERR;
}
/**--------------------------------------------------------------------------
@brief 베지어 곡면 샘플링 (폐기)

@param [out] dst     결과를 저장할 동적 배열
@param [in ] src     BEZIER 구조체 주소
@param [in ] u_array u방향 샘플링 포인트 배열
@param [in ] v_array v항 샘플링 포인트 배열
@return 샘플링 된 포인트의 수

@author ksg
*/
int BEZIER_Sample(DVEC *dst, BEZIER *src, DVEC u_array, DVEC v_array)
{
	const int dim = BEZIER_Dimension(src);
	int dst_cnt;

	int m,n;
	int *u_b, *v_b; // binomial array
	real_t *pu, *pv;
	int pu_cnt, pv_cnt;

	m = BEZIER_UOrder(src);          n = BEZIER_VOrder(src);
	pu = (real_t*)u_array;           pv = (real_t*)v_array;
	u_b = (int*)gBinomials[m];       v_b = (int*)gBinomials[n];
	pu_cnt = DVEC_Length(u_array);   pv_cnt = DVEC_Length(v_array);

	if (pu_cnt == 0) {
		pu_cnt = 1;
	}

	if (pv_cnt == 0) {
		pv_cnt = 1;
	}

	dst_cnt = pu_cnt * pv_cnt;
	if (DVEC_Length(*dst) != dst_cnt)
		DVEC_SetLength(dst, dst_cnt, dim * sizeof(real_t));

	return calc_bezier_surface((real_t*)*dst, u_b, m, v_b, n, (real_t*)src->cp, dim, pu, pu_cnt, pv, pv_cnt);
}


/**--------------------------------------------------------------------------
@brief 베지어 곡면 이동 (폐기)

@param [in,out] dst    이동시킬 곡면
@param [in ]    offset 이동시킬 양

offset은 벡터로, BEZIER 구조체의 조종점 차원과 동일한 차원으로 지정해야
한다.

@author ksg
*/
void BEZIER_Move(BEZIER *dst, real_t *offset)
{
	int dim, cp_cnt, i, d;
	dim = BEZIER_Dimension(dst);
	cp_cnt = DVEC_Length(dst->cp);
	for (i = 0; i < cp_cnt; i++) {
		for (d = 0; d < dim; d++) {
			((real_t*)(dst->cp))[i*dim + d] += offset[d];
		}
	}
}

/**--------------------------------------------------------------------------
@brief 배열의 첫번째 요소와 마지막 요소를 기준으로 중간의 요소의 값을
선형으로 균등 분포하도록 변경한다.

곡면의 평탄화 작업시 조종점의 시작과 끝을 고정시키고 나머지 중간 조종
점들을 직선화 시키는 용도로 사용될 수 있다.

@param [in,out] p        대상 배열

배열의 길이는 (segments+1) * stride와 일치한다고 가정한다.
@param [in ] stride   배열 요소 접근시 건너 뛸 길이.
@param [in ] segments 균등 분포시킬 구간 수

@author ksg
*/
void Distribute(real_t *p, int stride, int segments)
{
	int i;
	real_t d;
	if (segments > 1) {
		d = p[stride*(segments)];
		d = (d - p[0]) / (real_t)segments;
		for (i = 1; i < segments; i++)
			p[i*stride] = p[0] + d * (real_t)i;
	}
}
/**--------------------------------------------------------------------------
@brief BEZIER형 곡면을 네 모서리의 조종점을 고정시킨 채로 평탄화 시킨다. (폐기)

이름은 평탄화이나 엄밀하게는 u,v 방향으로 각각 평탄화를 시키기 때문에
조종점이 3차원적으로 꼬인 형상의 곡면의 경우 평탄화 결과로 얻어진 곡면
은 평면이 아닐 수도 있다.
@param [in,out] dst 평탄화 시킬 대상 곡면

@author ksg
*/
void BEZIER_Planar(BEZIER *dst)
{
	int u_cnt, v_cnt, dim, v, d;
	real_t *cp;

	cp = (real_t *)(dst->cp);
	u_cnt = BEZIER_UOrder(dst) + 1;
	v_cnt = BEZIER_VOrder(dst) + 1;
	dim = BEZIER_Dimension(dst);

	for (d = 0; d < dim; d++) {
		cp = (real_t *)(dst->cp) + d;
		Distribute(cp, u_cnt * dim, v_cnt-1);
		Distribute(cp + (u_cnt-1)*dim, u_cnt * dim, v_cnt-1);
		for (v = 0; v < v_cnt; v++, cp += u_cnt * dim)
			Distribute(cp, dim, u_cnt-1);
	}
}
/**--------------------------------------------------------------------------
@brief BEZIER2형 곡선 초기화

@param [in ] u_order 곡선 차수
@param [in ] dim     조종점 차원
@return 초기화된 BEZIER2형 곡선

@author ksg
*/
BEZIER2 BEZIER2_InitCurve(uint8_t u_order, uint8_t dim)
{
	return BEZIER2_InitSurface(u_order, 0, dim);
}
/**--------------------------------------------------------------------------
@brief BEZIER2형 단일 곡면 초기화

내부적으로 u_count=1, v_count=1인 다중 곡면을 생성한다.
@param [in ] u_order 곡면의 u방향 차수
@param [in ] v_order 곡면의 v방향 차수
@param [in ] dim     조종점 차원
@return 초기화된 BEZIER2형 곡면

@author ksg
*/
BEZIER2 BEZIER2_InitSurface(uint8_t u_order, uint8_t v_order, uint8_t dim)
{
	return BEZIER2_InitSurfacePatch(1, 1, u_order, v_order, dim);
}
/**--------------------------------------------------------------------------
@brief BEZIER2형 다중 곡면 초기화

다수의 베지어 곡면 패치로 구성된 다중 곡면을 생성한다.
@param [in ] u_count 곡면의 u방향 패치 수
@param [in ] v_count 곡면의 v방향 패치 수
@param [in ] u_order 곡면의 u방향 차수
@param [in ] v_order 곡면의 v방향 차수
@param [in ] dim     조종점 차원
@return 초기화된 BEZIER2형 곡면

@author ksg
*/
BEZIER2 BEZIER2_InitSurfacePatch(uint8_t u_count, uint8_t v_count, uint8_t u_order, uint8_t v_order, uint8_t dim)
{
	BEZIER2 r;
	r = DVEC_InitWithLength((u_count * u_order + 1) * (v_count * v_order + 1), sizeof(real_t) * dim);
	if (r) {
		DVEC_REC *pRec = DVEC_GetRec(r);
		pRec->reserved = (u_order | (u_count << 4) | (v_order << 8) | (v_count << 12));
	}
	return r;
}
//---------------------------------------------------------------------------
//#define INTERSECT_EP 0.01 ///< 교점 오차
#define INTERSECT_EP 0.00001 ///< 교점 오차
#define INTERSECT_SQEP (INTERSECT_EP*INTERSECT_EP) ///< 교점 오차를 제곱한 값
#define INTERSECT_MAX_LOOP 200 ///< 최대 해 탐색 반복 횟수
#define D_SCALE 0.4  ///< Delta Scale, GetUV함수에서 해 탐색 시 스텝 축소 비율
#define MIN_D 0.00000001 ///< 최소 Delta 값
/**--------------------------------------------------------------------------
@brief 두 VEC3타입의 벡터간 거리의 제곱값을 반환

@param [in ] a 첫번째 벡터
@param [in ] b 두번째 벡터
@return 벡터간 거리의 제곱

@author ksg
*/
static inline real_t vec3_sqr_distance(vec3 a, vec3 b) {a.x -= b.x; a.y -= b.y; a.z -= b.z; return a.x * a.x + a.y * a.y + a.z * a.z;}
/**--------------------------------------------------------------------------
@brief BEZIER2형 곡면에서 주어진 p 벡터와 가장 가까운 u,v
파라미터를 계산

수치적 반복법을 통해 u,v 좌표를 산출하기 때문에 계산량이 매우 많다.

@param [out] dst 결과를 저장할 배열
@param [in ] src 베지어 곡면
@param [in ] p   xyz 좌표계에서의 위치
@return 찾아낸 u,v 파라미터에서 p 벡터까지의 거리의 제곱 값

@author ksg
*/
real_t BEZIER2_GetUV(real_t *dst, BEZIER2 src, vec3 p)
{
	vec3 r0={0,};
	int it;
	real_t u,v, d;
	BEZIER_INFO bzr;

#if 1
	real_t u_base, v_base, u_min, v_min, d_min, step;
	int iu, iv, unchanged = 0;

	BEZIER2_Info(&bzr, src);
	u_base = 0.5;
	v_base = 0.5;
	step = 0.5;

	BEZIER2_GetXYZ((real_t*)(&r0), bzr.cp_dim, src, u_base, v_base);
	d_min = vec3_sqr_distance(r0, p);
	u_min = 0.5;
	v_min = 0.5;


	#define SEG 5
	#define SUB_STAGE 10
	unchanged = 0;
	for (it = 0; it < SUB_STAGE; it++) {
		for (iv = -SEG; iv <= SEG; iv++) {
			v = v_base + iv * step / (float)SEG;
			if (v < 0.0 || v > 1.0) {
				continue;
			}

			for (iu = -SEG; iu <= SEG; iu++) {
				u = u_base + iu * step / (float)SEG;
				if (u < 0.0 || u > 1.0) {
					continue;
				}
				BEZIER2_GetXYZ((real_t*)(&r0), bzr.cp_dim, src, u, v);
				d = vec3_sqr_distance(r0, p);
				if (d < d_min) {
					if (d < INTERSECT_SQEP) {
						dst[0] = u;
						dst[1] = v;
//            Dbg_("GetUV: SUCCESS!  x,y=[%f,%f] dist=%f uv=[%f,%f]", p.x, p.y, d, u, v);
						return d;
					}
					u_min = u;
					v_min = v;
					d_min = d;
					unchanged = 0;
				}
			}
		}
		unchanged ++;
		if (unchanged > 3) {
			break;
		}
		u_base = u_min;
		v_base = v_min;
		step /= (float)SEG;
	}

	Dbg_("GetUV: FAILED!   x,y=[%.3f,%.3f] dist=%.5f uv=[%.3f,%.3f]", p.x, p.y, d_min, u_min, v_min);

	dst[0] = u_min;
	dst[1] = v_min;

	return d_min;


#else //#if 1
	int i;
	real_t d0, d1, du, dv;
	vec3 r1={0,};


	BEZIER2_Info(&bzr, src);
	u = 0.5; v = 0.5;
	du = 0.5; dv = 0.5;
	i = 0;

	BEZIER2_GetXYZ((real_t*)(&r0), bzr.cp_dim, src, u, v);
	d = vec3_sqr_distance(r0, p);

	for (it=0; d > INTERSECT_SQEP && du > MIN_D && dv > MIN_D && it < INTERSECT_MAX_LOOP; it++, i = 1 - i) {
		if (i == 0) {
			BEZIER2_GetXYZ((real_t*)(&r0), bzr.cp_dim, src, u+du, v);
			d0 = vec3_sqr_distance(r0,p);
			BEZIER2_GetXYZ((real_t*)(&r1), bzr.cp_dim, src, u-du, v);
			d1 = vec3_sqr_distance(r1,p);

			if (d0 < d1) {
				if (/*u + du <= 1.0 && */d0 < d) {
					u += du;
					d = d0;
//          du *= 1.1;
				} else {
					du *= D_SCALE;
				}
			} else {
				if (/*u - du >= 0.0 && */d1 < d) {
					u -= du;
					d = d1;
//          du *= 1.1;
				} else {
					du *= D_SCALE;
				}
			}
		} else {
			BEZIER2_GetXYZ((real_t*)(&r0), bzr.cp_dim, src, u, v+dv);
			d0 = vec3_sqr_distance(r0,p);
			BEZIER2_GetXYZ((real_t*)(&r1), bzr.cp_dim, src, u, v-dv);
			d1 = vec3_sqr_distance(r1,p);

			if (d0 < d1) {
				if (/*v + dv <= 1.0 && */d0 < d) {
					v += dv;
					d = d0;
//          dv *= 1.1;
				} else {
					dv *= D_SCALE;
				}
			} else {
				if (/*v - dv >= 0.0 &&*/ d1 < d) {
					v -= dv;
					d = d1;
//          dv *= 1.1;
				} else {
					dv *= D_SCALE;
				}
			}
		}
	}
	if (d > INTERSECT_SQEP) {
		Dbg_("Failed: d=%f at [%f,%f] it=%d", sqrt(d), u, v, it);
	} else {
		//Dbg_("Success: d=%f at [%f,%f] it=%d", sqrt(d), u, v, it);
	}
	dst[0] = u;
	dst[1] = v;
	return d;
#endif //#else 1
}
/**--------------------------------------------------------------------------
@brief BEZIER2형 곡면에서 특정 u,v 값에 대한 샘플링

@param [out] dst     샘플링된 xyz좌표를 저장할 벡터의 주소
@param [in ] dst_dim 결과 벡터의 차원
@param [in ] src     베지어 곡면
@param [in ] u       셈플링 할 u 파라미터
@param [in ] v       셈플링 할 v 파라미터

@author ksg
*/
void BEZIER2_GetXYZ_Implicit(real_t *dst, int dst_dim, BEZIER2 src, real_t u, real_t v)
{
	int it_u, it_v; // iterator. d:dimension
	int u_seg, v_seg;
	real_t u_local, v_local;
	BEZIER_INFO bzr;
	int cp_stride;

	real_t B, B_v;
	real_t *pCP, *pBase;
	DVEC pow_cache_u;
	DVEC pow_cache_v;

	const int *binomial_u;
	const int *binomial_v;

	BEZIER2_Info(&bzr, src);

	binomial_u = gBinomials[bzr.u_order];
	binomial_v = gBinomials[bzr.v_order];

	u_seg = (int)(u * (real_t)bzr.u_count);  // 몇번째 patch에 속하는지
	if (u_seg >= bzr.u_count) u_seg = bzr.u_count - 1;
	if (u_seg < 0) u_seg = 0;
	v_seg = (int)(v * (real_t)bzr.v_count);  // 몇번째 patch에 속하는지
	if (v_seg >= bzr.v_count) v_seg = bzr.v_count - 1;
	if (v_seg < 0) v_seg = 0;

	u_local = u * (real_t)bzr.u_count - (real_t)u_seg; // 해당 patch에서의 u 좌표 환산
	v_local = v * (real_t)bzr.v_count - (real_t)v_seg; // 해당 patch에서의 v 좌표 환산

	pow_cache_u = InitArrayOfreal_t(bzr.u_order + 1);
	pow_cache_v = InitArrayOfreal_t(bzr.v_order + 1);

	// 제곱근 캐시(v) 채우기
	for (it_v = 0; it_v <= bzr.v_order; it_v++) {
		if (it_v == 0 && real_is_zero(v_local)) {
			((real_t*)pow_cache_v)[it_v] = real_one;
		} else if (it_v == bzr.v_order && real_is_one(v_local)) {
			((real_t*)pow_cache_v)[it_v] = real_one;
		} else {
			((real_t*)pow_cache_v)[it_v] = (real_t)pow(v_local, (double)it_v) * (real_t)pow(real_one-v_local, (double)(bzr.v_order-it_v));
		}
	}

	// 제곱근 캐시(u) 채우기
	for (it_u = 0; it_u <= bzr.u_order; it_u++) {
		if (it_u == 0 && real_is_zero(u_local)) {
			((real_t*)pow_cache_u)[it_u] = real_one;
		} else if (it_u == bzr.u_order && real_is_one(u_local)) {
			((real_t*)pow_cache_u)[it_u] = real_one;
		} else {
			((real_t*)pow_cache_u)[it_u] = (real_t)pow(u_local, (double)it_u) * (real_t)pow(real_one-u_local, (double)(bzr.u_order-it_u));
		}
	}

	cp_stride = bzr.cp_dim * (bzr.u_order * bzr.u_count + 1);
	pBase = (real_t*)src;
	pBase += bzr.cp_dim * bzr.u_order * u_seg;
	pBase += cp_stride * bzr.v_order * v_seg;
	pCP = pBase;

	// Target Point 초기화
	memset(dst, 0, dst_dim * sizeof(real_t));

	for (it_v = 0; it_v <= bzr.v_order; ++it_v, pCP = pBase + it_v * cp_stride) {
		B_v = ((real_t)binomial_v[it_v]) * ((real_t*)pow_cache_v)[it_v];
		for (it_u = 0; it_u <= bzr.u_order; ++it_u, pCP += bzr.cp_dim) {
			B = B_v * ((real_t)binomial_u[it_u]) * ((real_t*)pow_cache_u)[it_u];

			switch (dst_dim) {
				case 1:
					dst[0] += B * pCP[0];
					break;
				case 2:
					dst[0] += B * pCP[0];
					dst[1] += B * pCP[1];
					break;
				case 3:
					dst[0] += B * pCP[0];
					dst[1] += B * pCP[1];
					dst[2] += B * pCP[2];
					break;
				case 4:
					dst[0] += B * pCP[0];
					dst[1] += B * pCP[1];
					dst[2] += B * pCP[2];
					dst[3] += B * pCP[3];
					break;
				default:
					Err_("Cannot support %d dimension", dst_dim);
					break;
			}
		}
	}

	DVEC_Free(&pow_cache_u);
	DVEC_Free(&pow_cache_v);
	return;
}

void CopyCPs(real_t *dst, real_t *cp, int cp_dim, int cp_stride, int cp_cnt)
{
	int i;
	for (i = 0; i < cp_cnt; i++, cp += cp_stride, dst += cp_dim) {
		switch (cp_dim) {
			case 0:
				break;
			case 1:
				*dst = *cp;
				break;
			case 2:
				dst[0] = cp[0];
				dst[1] = cp[1];
				break;
			case 3:
				dst[0] = cp[0];
				dst[1] = cp[1];
				dst[2] = cp[2];
				break;
			case 4:
				dst[0] = cp[0];
				dst[1] = cp[1];
				dst[2] = cp[2];
				dst[3] = cp[3];
				break;
		}
	}
}

void ReduceCPs(real_t *cp, int cp_dim, int cp_cnt, real_t t)
{
	int i, i_end;
	for (  ;cp_cnt > 1; --cp_cnt) {
		for (i = 0, i_end = (cp_cnt - 1) * cp_dim ; i < i_end; i++) {
			cp[i] += (cp[i+cp_dim] - cp[i]) * t;
		}
	}
}

/// De Casteljau 방식으로 계산
void BEZIER2_GetXYZ_DeCastel(real_t *dst, int dst_dim, const BEZIER2 src, real_t u, real_t v)
{
	int it_u, it_v; // iterator. d:dimension
	int u_seg, v_seg;
	real_t u_local, v_local;
	BEZIER_INFO bzr;
	int cp_stride;
	real_t *pCP;

	real_t u_buf[12*4];
	real_t v_buf[12*4];
	int i, j, u_buf_len, v_buf_len;

	BEZIER2_Info(&bzr, src);

	u_seg = (int)(u * (real_t)bzr.u_count);  // 몇번째 patch에 속하는지
	if (u_seg >= bzr.u_count) u_seg = bzr.u_count - 1;
	if (u_seg < 0) u_seg = 0;
	v_seg = (int)(v * (real_t)bzr.v_count);  // 몇번째 patch에 속하는지
	if (v_seg >= bzr.v_count) v_seg = bzr.v_count - 1;
	if (v_seg < 0) v_seg = 0;

	u_local = u * (real_t)bzr.u_count - (real_t)u_seg; // 해당 patch에서의 u 좌표 환산
	v_local = v * (real_t)bzr.v_count - (real_t)v_seg; // 해당 patch에서의 v 좌표 환산


	cp_stride = bzr.cp_dim * (bzr.u_order * bzr.u_count + 1);
	pCP = (real_t*)src;
	pCP += bzr.cp_dim * bzr.u_order * u_seg;
	pCP += cp_stride * bzr.v_order * v_seg;

	/*
	if (u_local < 0) {
		u_local = 0;
	} else if (u_local > 1) {
		u_local = 1;
	}
	if (v_local < 0) {
		v_local = 0;
	} else if (v_local > 1) {
		v_local = 1;
	}
	*/
	for (it_v = 0; it_v <= bzr.v_order; ++it_v) {
		CopyCPs(u_buf, pCP + it_v * cp_stride, bzr.cp_dim, bzr.cp_dim, bzr.u_order + 1);
		ReduceCPs(u_buf, bzr.cp_dim, bzr.u_order + 1, u_local);
		memcpy(v_buf + it_v * bzr.cp_dim, u_buf, sizeof(real_t) * bzr.cp_dim);
	}
	ReduceCPs(v_buf, bzr.cp_dim, bzr.v_order + 1, v_local);
	switch (dst_dim) {
		case 1: dst[0] = v_buf[0]; break;
		case 2: ((vec2*)dst)[0] = ((vec2*)v_buf)[0]; break;
		case 3: ((vec3*)dst)[0] = ((vec3*)v_buf)[0]; break;
		case 4: ((vec4*)dst)[0] = ((vec4*)v_buf)[0]; break;
		default :
			memcpy(dst, v_buf, sizeof(real_t) * dst_dim); break;
	}

	return;
}

/**--------------------------------------------------------------------------
@brief BEZIER2형 곡면의 외곽선을 샘플링한다.

dst를 NULL로 지정하여 호출한 경우 샘플링 결과로 얻어질 폴리건 벡터의 수만 반환한다.

@param [out] dst        샘플링된 폴리건 벡터를 저장할 배열
@param [in ] dst_dim    폴리건 벡터의 차원
@param [in ] src        대상 곡면
@param [in ] u_segments u방향 분할 구간 수
@param [in ] v_segments v방향 분할 구간 수
@return 샘플링 된 포인트 벡터의 수

성공한 경우 u_segments * 2 + v_segments * 2 와 일치하며 실패시 0을 반환한다.

@author ksg
*/
int BEZIER2_SampleOutlineToArray(real_t *dst, int dst_dim, BEZIER2 src, int u_segments, int v_segments)
{
	int dst_cnt;
	int i;
	BEZIER_INFO bi;
	real_t u=0.0, v=0.0, u_inc, v_inc;

	if (BEZIER2_Info(&bi, src) == 0)
		return 0;
	dst_cnt = (u_segments * 2) + (v_segments * 2);

	if (dst == NULL)
		return dst_cnt;
	u_inc = (real_t)1.0 / (real_t)u_segments;
	v_inc = (real_t)1.0 / (real_t)v_segments;

	for (i = 0; i < u_segments; i++, u += u_inc, dst+=dst_dim)
		BEZIER2_GetXYZ(dst, dst_dim, src, u, v);
	u = 1.0;
	for (i = 0; i < v_segments; i++, v += v_inc, dst+=dst_dim)
		BEZIER2_GetXYZ(dst, dst_dim, src, u, v);
	v = 1.0;
	for (i = 0; i < u_segments; i++, u -= u_inc, dst+=dst_dim)
		BEZIER2_GetXYZ(dst, dst_dim, src, u, v);
	u = 0.0;
	for (i = 0; i < v_segments; i++, v -= v_inc, dst+=dst_dim)
		BEZIER2_GetXYZ(dst, dst_dim, src, u, v);

	return dst_cnt;
}
/**--------------------------------------------------------------------------
@brief BEZIER2형 곡면의 외곽선을 샘플링한다.

BEZIER2_SampleOutlineToArray 함수를 보다 편리하게 사용하기 위해 dst를
일반 배열에서 동적 배열로 바꾼 버전. 내부적으로는
BEZIER2_SampleOutlineToArray 함수를 두 번 호출하는 방식으로 구현되어 있다.

@param [out] dst        샘플링된 폴리건 벡터를 저장할 동적 배열
@param [in ] dst_dim    폴리건 벡터의 차원
@param [in ] src        대상 곡면
@param [in ] u_segments u방향 분할 구간 수
@param [in ] v_segments v방향 분할 구간 수
@return 샘플링된 포인트 벡터의 수

@author ksg
*/
int BEZIER2_SampleOutline(DVEC *dst, int dst_dim, BEZIER2 src, int u_segments, int v_segments)
{
	int dst_cnt;
	dst_cnt = BEZIER2_SampleOutlineToArray(NULL, dst_dim, src, u_segments, v_segments);
	if (dst_cnt > 0) {
		if (DVEC_Length(*dst) != dst_cnt)
			DVEC_SetLength(dst, dst_cnt, dst_dim * sizeof(real_t));
		return BEZIER2_SampleOutlineToArray((real_t*)(*dst), dst_dim, src, u_segments, v_segments);
	}
	return 0;
}

int BEZIER2_Sample(real_t *dst, int dst_dim, BEZIER2 src, vec2 const *uv, int count)
{
	int i;
//  vec4f vImp = {0,}, vDeC = {0,};
	for (i = 0; i < count; i++, dst += dst_dim) {
//    BEZIER2_GetXYZ_DeCastel(&vDeC, dst_dim, src, uv[i].x, uv[i].y);
		BEZIER2_GetXYZ(dst, dst_dim, src, uv[i].x, uv[i].y);
//    BEZIER2_GetXYZ_DeCastel(dst, dst_dim, src, uv[i].x, uv[i].y);
//    BEZIER2_GetXYZ_Implicit(dst, dst_dim, src, uv[i].x, uv[i].y);
	}
	return count;
}


/**--------------------------------------------------------------------------
@brief BEZIER2형 곡면을 샘플링한다.

베지어 곡면을 포인트 클라우드로 샘플링한다. u 및 v 방향으로 샘플링할 위치
를 동적 배열 형태로 입력받기 때문에 등간격이 아닌 임의의 간격으로 샘플링
하는 것도 가능하다.

@param [in ] dst     샘플링된 포인트 클라우드를 저장할 동적 배열
@param [in ] dst_dim 폴리건 벡터의 차원
@param [in ] src     대상 곡면
@param [in ] u_array u방향 샘플링 위치 동적 배열
@param [in ] v_array v방향 샘플링 위치 동적 배열
@return 샘플링된 포인트 벡터의 수

@author ksg
*/
int BEZIER2_SampleCrossPoints(DVEC *dst, int dst_dim, BEZIER2 src, DVEC u_array, DVEC v_array)
{
	int u_cnt, v_cnt, dst_cnt;
	int i,j;
	vec3 r;
	BEZIER_INFO bi;
	real_t *pD;

	if (BEZIER2_Info(&bi, src) == 0)
		return 0;

	u_cnt = DVEC_Length(u_array);
	v_cnt = DVEC_Length(v_array);

	if (u_cnt == 0) u_cnt = 1;
	if (v_cnt == 0) v_cnt = 1;

	dst_cnt = u_cnt * v_cnt;
	if (DVEC_Length(*dst) != dst_cnt)
		DVEC_SetLength(dst, dst_cnt, dst_dim * sizeof(real_t));

	pD = (real_t*)(*dst);

	for (j = 0; j < v_cnt; j++) {
		for (i = 0; i < u_cnt; i++) {
			BEZIER2_GetXYZ(pD, dst_dim, src, ((real_t*)u_array)[i], ((real_t*)v_array)[j]);
			memcpy(&r, pD, sizeof(real_t) * dst_dim);
			pD += dst_dim;
		}
	}

	return u_cnt * v_cnt;
}
/**--------------------------------------------------------------------------
@brief BEZIER2형 곡면을 샘플링한다.

베지어 곡면을 포인트 클라우드로 샘플링한다.
BEZIER2_SampleCrossPoints 함수와는 달리 샘플링할 포인트 배열을 입력으로 사용하지 않고
각 방향으로 샘플링 시작과 종료 위치, 분할 수를 지정한다.
내부적으로는 BEZIER2_SampleCrossPoints 함수를 호출한다.

@param [in ] dst        샘플링된 포인트 클라우드를 저장할 동적 배열
@param [in ] dst_dim    폴리건 벡터의 차원
@param [in ] src        대상 곡면
@param [in ] u_begin    u방향 샘플링 시작 위치
@param [in ] u_end      u방향 샘플링 종료 위치
@param [in ] u_segments u방향 샘플링 구간 수
@param [in ] v_begin    v방향 샘플링 시작 위치
@param [in ] v_end      v방향 샘플링 종료 위치
@param [in ] v_segments v방향 샘플링 구간 수
@return 샘플링된 포인트 벡터의 수

@author ksg
*/
int BEZIER2_SampleWithParams(DVEC *dst, int dst_dim, BEZIER2 src, real_t u_begin, real_t u_end, int u_segments, real_t v_begin, real_t v_end, int v_segments)
{
	DVEC u_array, v_array;
	int ret;

	u_array = InitSamplePositions(u_begin, u_end, u_segments);
	v_array = InitSamplePositions(v_begin, v_end, v_segments);

	ret = BEZIER2_SampleCrossPoints(dst, dst_dim, src, u_array, v_array);

	DVEC_Free(&u_array);
	DVEC_Free(&v_array);

	return ret;
}
/**--------------------------------------------------------------------------
@brief BEZIER2형 곡면에서 곡면 파라미터를 추출한다.

@param [in ] dst 곡면 파라미터를 저장할 BEZIER_INFO 구조체
@param [in ] src BEZIER2형 곡면
@return 성공하면 1, 실패하면 0

@author ksg
*/
int BEZIER2_Info(BEZIER_INFO *dst, BEZIER2 src)
{
	DVEC_REC *R;
	if (src == NULL)
		return 0;
	R = DVEC_GetRec(src);
	dst->u_order = ((R->reserved) & 0x0fU);
	dst->u_count = ((R->reserved >> 4) & 0x0fU);
	dst->v_order = ((R->reserved >> 8) & 0x0fU);
	dst->v_count = ((R->reserved >> 12) & 0x0fU);
	dst->cp_dim = R->elemSize/sizeof(real_t);
	dst->cp_cnt = R->length;
	return 1;
}
/**--------------------------------------------------------------------------
@brief BEZIER2형 곡면을 네 모서리의 조종점을 고정시킨 채로 평탄화 시킨다.

이름은 평탄화이나 엄밀하게는 u,v 방향으로 각각 평탄화를 시키기 때문에
조종점이 3차원적으로 꼬인 형상의 곡면의 경우 평탄화 결과로 얻어진 곡면
은 평면이 아닐 수도 있다.
@param [in,out] dst 평탄화 시킬 대상 곡면

@author ksg
*/
void BEZIER2_Planar(BEZIER2 *dst)
{
	int v, d;
	real_t *cp;
	BEZIER_INFO bi;

	if (BEZIER2_Info(&bi, *dst) == 0)
		return;

	for (d = 0; d < bi.cp_dim; d++) {
		cp = (real_t *)(*dst) + d;
		Distribute(cp, (bi.u_order * bi.u_count + 1) * bi.cp_dim, bi.v_order * bi.v_count);
		Distribute(cp + (bi.u_order * bi.u_count) * bi.cp_dim, (bi.u_order * bi.u_count + 1) * bi.cp_dim, bi.v_order * bi.v_count);
		for (v = 0; v < (bi.v_order * bi.v_count + 1); v++, cp += (bi.u_order * bi.u_count + 1) * bi.cp_dim)
			Distribute(cp, bi.cp_dim, bi.u_order * bi.u_count);
	}

}
/**--------------------------------------------------------------------------
@brief BEZIER2형 곡면을 지정한 범위의 직사각형 형태로 평탄화 시킨다.

평탄화된 곡면은 언제나 직사각형의 평면이다.
@param [in,out] dst 평탄화 시킬 대상 곡면
@param [in ]    bounds 직사각형 범위

@author ksg
*/
void BEZIER2_PlanarWithBoundary(BEZIER2 *dst, xywh2 *bounds)
{
	BEZIER_INFO bi;
	vec3 *cp;
	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
	cp = (vec3 *)(*dst);
	cp[0] = make_vec3(bounds->x, bounds->y, 0);
	cp[bi.u_order * bi.u_count] = make_vec3(bounds->x + bounds->width, bounds->y, 0);
	cp[(bi.u_order * bi.u_count+1) * (bi.v_order * bi.v_count)] = make_vec3(bounds->x, bounds->y - bounds->height, 0);
	cp[(bi.u_order * bi.u_count+1) * (bi.v_order * bi.v_count) + (bi.u_order * bi.u_count)] = make_vec3(bounds->x + bounds->width, bounds->y - bounds->height, 0);
	BEZIER2_Planar(dst);
}


void BEZIER2_FlipX(BEZIER2 *dst, float x_base)
{
	BEZIER_INFO bi;
	float *p_x;
	int i;
	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
	p_x = (float*)(*dst);
	for (i = 0; i < bi.cp_cnt; ++i, p_x += bi.cp_dim) {
		*p_x = -(*p_x) + 2 * x_base;
	}
}

void BEZIER2_MirrorX(BEZIER2 *dst, float x_base)
{
	BEZIER_INFO bi;
	float *p_x;
	int i, j;
	int idx1, idx2;
	float tempx, tempy;
    int w, h, ex_cnt;
	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
	p_x = (float*)(*dst);

	w = (bi.u_count * bi.u_order) + 1;
	h = (bi.v_count * bi.v_order) + 1;
	ex_cnt = w / 2;

	for(j = 0; j < h; j++)
	{
		for(i = 0; i < ex_cnt; i++)
		{
			idx1 = (j * w + i) * bi.cp_dim;
			idx2 = (j * w + (w - 1 - i)) * bi.cp_dim;
			tempx = p_x[idx2];
			tempy = p_x[idx2 + 1];
			p_x[idx2] = p_x[idx1];
			p_x[idx2 + 1] = p_x[idx1 + 1];
			p_x[idx1] = tempx;
			p_x[idx1 + 1] = tempy;
        }
    }
}
void BEZIER2_FlipY(BEZIER2 *dst, float y_base)
{
	BEZIER_INFO bi;
	float *p_y;
	int i;
	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
	p_y = (float*)(*dst) + 1;
	for (i = 0; i < bi.cp_cnt; ++i, p_y += bi.cp_dim) {
		*p_y = -(*p_y) + 2 * y_base;
	}
}

void BEZIER2_ReformX(BEZIER2 *dst, float x_base)
{
	BEZIER_INFO bi;
	float *p_x;
	int i;
	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
	p_x = (float*)(*dst);
	for (i = 0; i < bi.cp_cnt; ++i, p_x += bi.cp_dim) {
		//*p_x = -(*p_x) + 2 * x_base;
		//*p_x = ((*p_x) + 512) * 988 / 1024 - 512;
        *p_x = ((*p_x) - 512) * 984 / 1024 + 512;
	}
}

void BEZIER2_ReformY(BEZIER2 *dst, float x_base)
{
	BEZIER_INFO bi;
	float *p_y;
	int i;
	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
	p_y = (float*)(*dst) + 1;
	for (i = 0; i < bi.cp_cnt; ++i, p_y += bi.cp_dim) {
		//*p_x = -(*p_x) + 2 * x_base;
		//*p_x = ((*p_x) + 512) * 988 / 1024 - 512;
        *p_y = ((*p_y) - 384) * 749 / 768 + 384;
	}
}

void BEZIER2_ReformXYCorner(BEZIER2 *dst, float sx_left, float sy_left, float sx_right, float sy_right)
{
    BEZIER_INFO bi;
	float *p_x;
	float *p_y;
	int w, h;



	if (BEZIER2_Info(&bi, *dst) == 0)
		return;

	w = bi.u_count * bi.u_order + 1;
	h = bi.v_count * bi.v_order + 1;
	p_x = (float*)(*dst);
	p_y = (float*)(*dst) + 1;

	*p_x = (*p_x) * sx_left;
    *p_y = (*p_y) * sy_left;

    p_x = (float*)(*dst) + (w - 1) * bi.cp_dim;
	p_y = (float*)(*dst) + (w - 1) * bi.cp_dim + 1;

	*p_x = (*p_x) * sx_right;
	*p_y = (*p_y) * sy_right;

    p_x = (float*)(*dst) + (h - 1) * w * bi.cp_dim;
	p_y = (float*)(*dst) + (h - 1) * w * bi.cp_dim + 1;

	*p_x = (*p_x) * sx_left;
	*p_y = (*p_y) * sy_left;

	p_x = (float*)(*dst) + ((h - 1) * w + w - 1) * bi.cp_dim;
	p_y = (float*)(*dst) + ((h - 1) * w + w - 1) * bi.cp_dim + 1;

	*p_x = (*p_x) * sx_right;
	*p_y = (*p_y) * sy_right;


}

void BEZIER2_ReformXY(BEZIER2 *dst, float ax, float bx, float ay, float by)
{
	BEZIER_INFO bi;
	float *p_x;
	float *p_y;
	int i;
	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
    p_x = (float*)(*dst);
	p_y = (float*)(*dst) + 1;
	for (i = 0; i < bi.cp_cnt; ++i, p_x += bi.cp_dim, p_y += bi.cp_dim) {
	    *p_x = (*p_x) * ax + bx;
        *p_y = (*p_y) * ay + by;
	}

}

/// 1사분면의 좌표를 나머지 사분면에 대칭적으로 복사
void BEZIER2_MakeSymmetryQuadrant1(BEZIER2 obj)
{
	BEZIER_INFO bi;
	float *p_y, cx, cy;
	int i, w, h, w2, h2, x, y, ref;
	if (BEZIER2_Info(&bi, obj) == 0)
		return;
	w = bi.u_count * bi.u_order + 1;
	h = bi.v_count * bi.v_order + 1;
	if (w % 2 == 0 || h % 2 == 0) {
		return;
	}
	w2 = w / 2;
	h2 = h / 2;
	switch (bi.cp_dim) {
		case 2: {
			vec2 *p = (vec2*)obj;
			cx = p[w * h2 + w2].x;
			cy = p[w * h2 + w2].y;
			for (y = 0; y <= h2; y++) {
				for (x = 0; x < w2; x++) {
					vec2 *pD = p + y * w + x;
					vec2 vS = p[y * w + w - 1 - x];
					pD->x = 2.0 * cx - vS.x;
					pD->y = vS.y;
				}
			}
			ref = 0;
			for (y = h - 1; y > h2; y--, ref++) {
				for (x = 0; x < w; x++) {
					vec2 *pD = p + y * w + x;
					vec2 vS = p[ref * w + x];
					pD->y = 2.0 * cy - vS.y;
					pD->x = vS.x;
				}
			}
		}
		case 3: {
			vec3 *p = (vec3*)obj;
			cx = p[w * h2 + w2].x;
			cy = p[w * h2 + w2].y;
			for (y = 0; y <= h2; y++) {
				ref = w - 1;
				for (x = 0; x < w2; x++, ref--) {
					vec3 *pD = p + y * w + x;
					vec3 vS = p[y * w + ref];
					pD->x = 2.0 * cx - vS.x;
					pD->y = vS.y;
				}
			}
			ref = 0;
			for (y = h - 1; y > h2; y--, ref++) {
				for (x = 0; x < w; x++) {
					vec3 *pD = p + y * w + x;
					vec3 vS = p[ref * w + x];
					pD->y = 2.0 * cy - vS.y;
					pD->x = vS.x;
				}
			}
		}
		default:
			break;
	}


}

/**
    @brief  곡선 컨트롤 포인트를 X 축 방향 대칭으로 만들기
*/
void BEZIER2_MakeSymmetryX(BEZIER2 obj, int option)
{
	BEZIER_INFO bi;
	float *p_y, cx, cy;
	int i, w, h, w2, h2, x, y, ref;
	if (BEZIER2_Info(&bi, obj) == 0)
		return;
	w = bi.u_count * bi.u_order + 1;
	h = bi.v_count * bi.v_order + 1;
	if (w % 2 == 0) {
		return;
	}
	w2 = w / 2;

	switch (bi.cp_dim) {
		case 2: {
			vec2 *p = (vec2*)obj;
			if(option == 0)   //왼쪽을 오른쪽에 복사
			{
				x = w2 - 1;
				for(y = 0; y < h; y++) //가운데 선 바로 옆 좌표 센터와 맞추기
				{
					vec2 *pD = p + y * w + x;
					vec2 vS = p[y * w + w2];
					pD->y = vS.y;
				}

				for(y = 0; y < h; y++) //가운데 선 바로 옆 좌표 센터와 맞추기
				{
					cx = p[w * y + w2].x;
					for(x = 0; x < w2; x++)
					{
						vec2 *pD = p + y * w + x;
						vec2 vS = p[y * w + w - 1 - x];
						pD->y = vS.y;
						pD->x = 2.0 * cx - vS.x;
					}

				}
			}
			else if(option == 1)  //오른쪽을 왼쪽에 복사
			{
                x = w2 + 1;
				for(y = 0; y < h; y++) //가운데 선 바로 옆 좌표 센터와 맞추기
				{
					vec2 *pD = p + y * w + x;
					vec2 vS = p[y * w + w2];
					pD->y = vS.y;
				}

				for(y = 0; y < h; y++) //가운데 선 바로 옆 좌표 센터와 맞추기
				{
					cx = p[w * y + w2].x;
					for(x = w2 + 1; x < w; x++)
					{
						vec2 *pD = p + y * w + x;
						vec2 vS = p[y * w + w - 1 - x];
						pD->y = vS.y;
						pD->x = 2.0 * cx - vS.x;
					}

				}

            }




	
		}
		case 3: {
			vec3 *p = (vec3*)obj;
			if(option == 0)   //왼쪽을 오른쪽에 복사
			{
				x = w2 - 1;
				for(y = 0; y < h; y++) //가운데 선 바로 옆 좌표 센터와 맞추기
				{
					vec3 *pD = p + y * w + x;
					vec3 vS = p[y * w + w2];
					pD->y = vS.y;
				}
				for(y = 0; y < h; y++) //가운데 선 바로 옆 좌표 센터와 맞추기
				{
					cx = p[w * y + w2].x;
					for(x = w2 + 1; x < w; x++)
					{
						vec3 *pD = p + y * w + x;
						vec3 vS = p[y * w + w - 1 - x];
						pD->y = vS.y;
						pD->x = 2.0 * cx - vS.x;
					}

				}



			}
			else if(option == 1)  //오른쪽을 왼쪽에 복사
			{
                x = w2 + 1;
				for(y = 0; y < h; y++) //가운데 선 바로 옆 좌표 센터와 맞추기
				{
					vec3 *pD = p + y * w + x;
					vec3 vS = p[y * w + w2];
					pD->y = vS.y;
				}

                for(y = 0; y < h; y++) //가운데 선 바로 옆 좌표 센터와 맞추기
				{
					cx = p[w * y + w2].x;
					for(x = 0; x < w2; x++)
					{
						vec3 *pD = p + y * w + x;
						vec3 vS = p[y * w + w - 1 - x];
						pD->y = vS.y;
						pD->x = 2.0 * cx - vS.x;
					}

				}


            }
		}
		default:
			break;
	}


}

void BEZIER2_ROTATE(BEZIER2 *dst, float angle)
{
	BEZIER_INFO bi;
	float *p_x, *p_y;
    float x, y;
	int i;
	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
	p_x = (float*)(*dst);
	p_y = (float*)(*dst) + 1;
	for (i = 0; i < bi.cp_cnt; ++i, p_x += bi.cp_dim, p_y += bi.cp_dim) {
		//angle = -rolla_p * 3.14159 / 180;
		x = cos(angle) * (*p_x) - sin(angle) * (*p_y);
		y = sin(angle) * (*p_x) + cos(angle) * (*p_y);
		(*p_x) = x;
		(*p_y) = y;

	}
}

#if MCNEX
void BEZIER2_CLONE(BEZIER2 *src, BEZIER2 *tgt)
{
	int i;
    float *p_x;
	float *p_y;
	float *t_x;
    float *t_y;
    if(tgt != NULL)
	{
		BEZIER_INFO bi;
		if (BEZIER2_Info(&bi, *src) == 0) {
			return;
		}

		p_x = (float*)(*src);
		p_y = (float*)(*src) + 1;
		t_x = (float*)(*tgt);
		t_y = (float*)(*tgt) + 1;
		for (i = 0; i < bi.cp_cnt; ++i, p_x += bi.cp_dim, p_y += bi.cp_dim, t_x += bi.cp_dim, t_y += bi.cp_dim) {
			(*t_x) = (*p_x);
            (*t_y) = (*p_y);

        }

    }
}

void BEZIER2_GET_POINT(BEZIER2 *src, vec3f *p, int index)
{
	BEZIER_INFO bi;
    float *pp;
	if (BEZIER2_Info(&bi, *src) == 0)
		return;
    pp = (float*)(*src) + index * bi.cp_dim;
	p->x = *pp;
	pp += 1;
	p->y = *pp;
	pp += 1;
    p->z = *pp;
}

/**
    @brief 베지어 커브 기준 1, 2를 받아 그 사이 왜곡률에 따라 interpolation하여 새로운 베지어 커브 컨트롤 포인트 계산
*/
void BEZIER2_DISTORT(BEZIER2 *dst, BEZIER2 *bzr1, BEZIER2 *bzr2, MCNEX_PARAM p, int img_width, int img_height, float ref_rate)
{
	float r, counter_r;
	float dx, dy, sx, sy, rolla, yawa, pitcha;
	int x_anchor;
	int y_anchor;
	float half_width, half_height;
	BEZIER_INFO bi;
    uint16_t dim;
	float *d_x, *d_y, *bz1_x, *bz1_y, *bz2_x, *bz2_y;
	float ori_x, ori_y;
    float ref_x, ref_y;
	float x, y;
	matrix *projMat;
	double maty[9], matp[9];
	float ptf_x, ptf_y, ptf_z; //perspective transform 계산용
	double angle;
	int i;
	double sinv, cosv;
	vec3d p1, p2, p3, p4;
	vec3d p1d, p2d, p3d, p4d;
	vec2d src[4];
	vec2d tgt[4];

	r = p.distortion_rate / ref_rate;
    counter_r = 1.0f - r;
    dx = p.dx;
	dy = p.dy;
	sx = p.scalex;
	sy = p.scaley;
	rolla = p.roll_angle;
	yawa = p.yaw_angle;
	pitcha = p.pitch_angle;
	x_anchor = p.x_anchor;
	y_anchor = p.y_anchor;
	half_width = (float)img_width / 2.0f;
	half_height = (float)img_height / 2.0f;

    if(yawa != 0.0f) {
		angle = yawa * 3.141592653589 / 180;
		sinv = sin(angle);
		cosv = cos(angle);
		//perspective transform 기준 좌표 구하기

		p1.x = -2.0; p1.y = 2.0; p1.z = 2.0;
		p2.x = 2.0; p2.y = 2.0; p2.z = 2.0;
		p3.x = -2.0; p3.y = -2.0; p3.z = 2.0;
		p4.x = 2.0; p4.y = -2.0; p4.z = 2.0;
		p1d.x = p1.x * cosv;
		p1d.y = p1.y;
		p1d.z = p1.z + p1.x * sinv;
		p2d.x = p2.x * cosv;
		p2d.y = p2.y;
		p2d.z = p2.z + p2.x * sinv;
		p3d.x = p3.x * cosv;
		p3d.y = p3.y;
		p3d.z = p3.z + p3.x * sinv;
		p4d.x = p4.x * cosv;
		p4d.y = p4.y;
		p4d.z = p4.z + p4.x * sinv;

		src[0].x = (p1.x / p1.z) * half_width;
		src[0].y = (p1.y / p1.z) * half_height;
		src[1].x = (p2.x / p2.z) * half_width;
		src[1].y = (p2.y / p2.z) * half_height;
		src[2].x = (p3.x / p3.z) * half_width;
		src[2].y = (p3.y / p3.z) * half_height;
		src[3].x = (p4.x / p4.z) * half_width;
		src[3].y = (p4.y / p4.z) * half_height;

		tgt[0].x = (p1d.x / p1d.z) * half_width;
		tgt[0].y = (p1d.y / p1d.z) * half_height;
		tgt[1].x = (p2d.x / p2d.z) * half_width;
		tgt[1].y = (p2d.y / p2d.z) * half_height;
		tgt[2].x = (p3d.x / p3d.z) * half_width;
		tgt[2].y = (p3d.y / p3d.z) * half_height;
		tgt[3].x = (p4d.x / p4d.z) * half_width;
		tgt[3].y = (p4d.y / p4d.z) * half_height;

        projMat = projection_matrix2(tgt, src);

		maty[0] = projMat->var[0][0];
		maty[1] = projMat->var[0][1];
		maty[2] = projMat->var[0][2];
		maty[3] = projMat->var[1][0];
		maty[4] = projMat->var[1][1];
		maty[5] = projMat->var[1][2];
		maty[6] = projMat->var[2][0];
		maty[7] = projMat->var[2][1];
		maty[8] = projMat->var[2][2];

		matrix_free(projMat);
	}

	if(pitcha != 0.0f) {
        angle = pitcha * 3.141592653589 / 180;
		sinv = sin(angle);
		cosv = cos(angle);
		//perspective transform 기준 좌표 구하기

		p1.x = -2.0; p1.y = 2.0; p1.z = 2.0;
		p2.x = 2.0; p2.y = 2.0; p2.z = 2.0;
		p3.x = -2.0; p3.y = -2.0; p3.z = 2.0;
		p4.x = 2.0; p4.y = -2.0; p4.z = 2.0;
		p1d.x = p1.x;
		p1d.y = p1.y * cosv;
		p1d.z = p1.z + p1.y * sinv;
		p2d.x = p2.x;
		p2d.y = p2.y * cosv;
		p2d.z = p2.z + p2.y * sinv;
		p3d.x = p3.x;
		p3d.y = p3.y * cosv;
		p3d.z = p3.z + p3.y * sinv;
		p4d.x = p4.x;
		p4d.y = p4.y * cosv;
		p4d.z = p4.z + p4.y * sinv;

		src[0].x = (p1.x / p1.z) * half_width;
		src[0].y = (p1.y / p1.z) * half_height;
		src[1].x = (p2.x / p2.z) * half_width;
		src[1].y = (p2.y / p2.z) * half_height;
		src[2].x = (p3.x / p3.z) * half_width;
		src[2].y = (p3.y / p3.z) * half_height;
		src[3].x = (p4.x / p4.z) * half_width;
		src[3].y = (p4.y / p4.z) * half_height;

		tgt[0].x = (p1d.x / p1d.z) * half_width;
		tgt[0].y = (p1d.y / p1d.z) * half_height;
		tgt[1].x = (p2d.x / p2d.z) * half_width;
		tgt[1].y = (p2d.y / p2d.z) * half_height;
		tgt[2].x = (p3d.x / p3d.z) * half_width;
		tgt[2].y = (p3d.y / p3d.z) * half_height;
		tgt[3].x = (p4d.x / p4d.z) * half_width;
		tgt[3].y = (p4d.y / p4d.z) * half_height;

        projMat = projection_matrix2(tgt, src);

		matp[0] = projMat->var[0][0];
		matp[1] = projMat->var[0][1];
		matp[2] = projMat->var[0][2];
		matp[3] = projMat->var[1][0];
		matp[4] = projMat->var[1][1];
		matp[5] = projMat->var[1][2];
		matp[6] = projMat->var[2][0];
		matp[7] = projMat->var[2][1];
		matp[8] = projMat->var[2][2];

		matrix_free(projMat);
	}

    if (BEZIER2_Info(&bi, *dst) == 0)
		return;
	d_x = (float*)(*dst);
	d_y = (float*)(*dst) + 1;
	bz1_x = (float*)(*bzr1);
	bz1_y = (float*)(*bzr1) + 1;
	bz2_x = (float*)(*bzr2);
	bz2_y = (float*)(*bzr2) + 1;
	dim = bi.cp_dim;
    //컨트롤 포인트 좌표 계산
	for (i = 0; i < bi.cp_cnt; ++i, d_x += dim, d_y += dim, bz1_x += dim, bz1_y += dim, bz2_x += dim, bz2_y +=dim) {
		ori_x = *bz1_x;
		ori_y = *bz1_y;
		ref_x = *bz2_x;
		ref_y = *bz2_y;
        ///////////// 왜곡률 0인 컨트롤 포인트 변환 /////////////////////
        //scale
		if(x_anchor == 0) {
			ori_x = (ori_x + half_width) * sx - half_width;  //left anchor
		} else if(x_anchor == 1) {
			ori_x = ori_x * sx;  //center anchor
		} else if(x_anchor == 2) {
			ori_x = (ori_x - half_width) * sx + half_width;  //right anchor
		}
		if(y_anchor == 0) {
			ori_y = (ori_y - half_height) * sy + half_height; //top anchor
		} else if(y_anchor == 1) {
			ori_y = ori_y * sy; //center anchor
		} else if(y_anchor == 2) {
			ori_y = (ori_y + half_height) * sy - half_height; //bottom anchor
		}

		/////// pitch ////////////
		if(pitcha != 0.0f) {
			ptf_x = ori_x * matp[0] + ori_y * matp[1] + matp[2];
			ptf_y = ori_x * matp[3] + ori_y * matp[4] + matp[5];
			ptf_z = ori_x * matp[6] + ori_y * matp[7] + matp[8];

			ori_x = ptf_x / ptf_z;
			ori_y = ptf_y / ptf_z;

		}


		/////// yaw ////////////
		if(yawa != 0.0f) {
			ptf_x = ori_x * maty[0] + ori_y * maty[1] + maty[2];
			ptf_y = ori_x * maty[3] + ori_y * maty[4] + maty[5];
			ptf_z = ori_x * maty[6] + ori_y * maty[7] + maty[8];

			ori_x = ptf_x / ptf_z;
			ori_y = ptf_y / ptf_z;

        }


		//roll
		angle = rolla * 3.14159 / 180;
		x = cos(angle) * ori_x - sin(angle) * ori_y;
		y = sin(angle) * ori_x + cos(angle) * ori_y;
		ori_x = x;
		ori_y = y;

		//transpose
		ori_x = ori_x + dx;
		ori_y = ori_y + dy;

        ///////////// 왜곡률 reference인 컨트롤 포인트 변환 /////////////////////
        //scale
		if(x_anchor == 0) {
			ref_x = (ref_x + half_width) * sx - half_width;  //left anchor
		} else if(x_anchor == 1) {
			ref_x = ref_x * sx;  //center anchor
		} else if(x_anchor == 2) {
			ref_x = (ref_x - half_width) * sx + half_width;  //right anchor
		}
		if(y_anchor == 0) {
			ref_y = (ref_y - half_height) * sy + half_height; //top anchor
		} else if(y_anchor == 1) {
			ref_y = ref_y * sy; //center anchor
		} else if(y_anchor == 2) {
			ref_y = (ref_y + half_height) * sy - half_height; //bottom anchor
		}

		/////// pitch ////////////
		if(pitcha != 0.0f) {
			ptf_x = ref_x * matp[0] + ref_y * matp[1] + matp[2];
			ptf_y = ref_x * matp[3] + ref_y * matp[4] + matp[5];
			ptf_z = ref_x * matp[6] + ref_y * matp[7] + matp[8];

			ref_x = ptf_x / ptf_z;
			ref_y = ptf_y / ptf_z;
		}

		/////// yaw ////////////
		if(yawa != 0.0f) {
			ptf_x = ref_x * maty[0] + ref_y * maty[1] + maty[2];
			ptf_y = ref_x * maty[3] + ref_y * maty[4] + maty[5];
			ptf_z = ref_x * maty[6] + ref_y * maty[7] + maty[8];

			ref_x = ptf_x / ptf_z;
			ref_y = ptf_y / ptf_z;

		}
		//roll
		angle = rolla * 3.14159 / 180;
		x = cos(angle) * ref_x - sin(angle) * ref_y;
		y = sin(angle) * ref_x + cos(angle) * ref_y;
		ref_x = x;
		ref_y = y;

		//transpose
		ref_x = ref_x + dx;
		ref_y = ref_y + dy;


		///// 왜곡률 적용 계산 ///////
		(*d_x) = ori_x * counter_r + ref_x * r;
		(*d_y) = ori_y * counter_r + ref_y * r;


	}
}

void BEZIER2_TRANSFORM(BEZIER2 *dst, MCNEX_PARAM p, int img_width, int img_height)
{
	float dx, dy, sx, sy, rolla, yawa, pitcha, dx_p, dy_p, sx_p, sy_p, rolla_p, yawa_p, pitcha_p;
	int x_anchor;
	int y_anchor;
	float half_width, half_height;
	BEZIER_INFO bi;
	float *p_x;
	float *p_y;
	float x, y;
    double angle;
	int i;
	double sinv, cosv;
	vec3d p1, p2, p3, p4;
	vec3d p1d, p2d, p3d, p4d;
	vec2d src[4];
    vec2d tgt[4];
	matrix *projMat, *projMat_p, *projMat_p_temp;
	double matv[9], matv_p[9];
    float ptf_x, ptf_y, ptf_z; //perspective transform 계산용

    dx = p.dx;
	dy = p.dy;
	sx = p.scalex;
	sy = p.scaley;
	rolla = p.roll_angle;
	yawa = p.yaw_angle;
    pitcha = p.pitch_angle;
	dx_p = p.dx_prev;
	dy_p = p.dy_prev;
	sx_p = p.scalex_prev;
	sy_p = p.scaley_prev;
	rolla_p = p.roll_angle_prev;
	yawa_p = p.yaw_angle_prev;
    pitcha_p = p.pitch_angle_prev;
	x_anchor = p.x_anchor;
	y_anchor = p.y_anchor;
	half_width = (float)img_width / 2.0f;
	half_height = (float)img_height / 2.0f;

	if(yawa != yawa_p) {
        angle = yawa_p * 3.141592653589 / 180;  //주의, 각도를 -로 넣지 않음
		sinv = sin(angle);
		cosv = cos(angle);
		//perspective transform 기준 좌표 구하기
		p1.x = -2.0; p1.y = 2.0; p1.z = 2.0;
		p2.x = 2.0; p2.y = 2.0; p2.z = 2.0;
		p3.x = -2.0; p3.y = -2.0; p3.z = 2.0;
		p4.x = 2.0; p4.y = -2.0; p4.z = 2.0;
		p1d.x = p1.x * cosv;
		p1d.y = p1.y;
		p1d.z = p1.z + p1.x * sinv;
		p2d.x = p2.x * cosv;
		p2d.y = p2.y;
		p2d.z = p2.z + p2.x * sinv;
		p3d.x = p3.x * cosv;
		p3d.y = p3.y;
		p3d.z = p3.z + p3.x * sinv;
		p4d.x = p4.x * cosv;
		p4d.y = p4.y;
		p4d.z = p4.z + p4.x * sinv;

		src[0].x = (p1.x / p1.z) * half_width;
		src[0].y = (p1.y / p1.z) * half_height;
		src[1].x = (p2.x / p2.z) * half_width;
		src[1].y = (p2.y / p2.z) * half_height;
		src[2].x = (p3.x / p3.z) * half_width;
		src[2].y = (p3.y / p3.z) * half_height;
		src[3].x = (p4.x / p4.z) * half_width;
		src[3].y = (p4.y / p4.z) * half_height;

		tgt[0].x = (p1d.x / p1d.z) * half_width;
		tgt[0].y = (p1d.y / p1d.z) * half_height;
		tgt[1].x = (p2d.x / p2d.z) * half_width;
		tgt[1].y = (p2d.y / p2d.z) * half_height;
		tgt[2].x = (p3d.x / p3d.z) * half_width;
		tgt[2].y = (p3d.y / p3d.z) * half_height;
		tgt[3].x = (p4d.x / p4d.z) * half_width;
		tgt[3].y = (p4d.y / p4d.z) * half_height;

		projMat_p_temp = projection_matrix2(tgt, src); //src, tgt 순서로 호출하면 발산하는 문제 있음
		projMat_p = matrix_inv(projMat_p_temp);  //따라서 호출을 tgt, src 순으로 하고 inverse matrix 구함

		matv_p[0] = projMat_p->var[0][0];
		matv_p[1] = projMat_p->var[0][1];
		matv_p[2] = projMat_p->var[0][2];
		matv_p[3] = projMat_p->var[1][0];
		matv_p[4] = projMat_p->var[1][1];
		matv_p[5] = projMat_p->var[1][2];
		matv_p[6] = projMat_p->var[2][0];
		matv_p[7] = projMat_p->var[2][1];
		matv_p[8] = projMat_p->var[2][2];

		matrix_free(projMat_p);
		matrix_free(projMat_p_temp);

		angle = yawa * 3.141592653589 / 180;
		sinv = sin(angle);
		cosv = cos(angle);
		//perspective transform 기준 좌표 구하기

		p1.x = -2.0; p1.y = 2.0; p1.z = 2.0;
		p2.x = 2.0; p2.y = 2.0; p2.z = 2.0;
		p3.x = -2.0; p3.y = -2.0; p3.z = 2.0;
		p4.x = 2.0; p4.y = -2.0; p4.z = 2.0;
		p1d.x = p1.x * cosv;
		p1d.y = p1.y;
		p1d.z = p1.z + p1.x * sinv;
		p2d.x = p2.x * cosv;
		p2d.y = p2.y;
		p2d.z = p2.z + p2.x * sinv;
		p3d.x = p3.x * cosv;
		p3d.y = p3.y;
		p3d.z = p3.z + p3.x * sinv;
		p4d.x = p4.x * cosv;
		p4d.y = p4.y;
		p4d.z = p4.z + p4.x * sinv;

		src[0].x = (p1.x / p1.z) * half_width;
		src[0].y = (p1.y / p1.z) * half_height;
		src[1].x = (p2.x / p2.z) * half_width;
		src[1].y = (p2.y / p2.z) * half_height;
		src[2].x = (p3.x / p3.z) * half_width;
		src[2].y = (p3.y / p3.z) * half_height;
		src[3].x = (p4.x / p4.z) * half_width;
		src[3].y = (p4.y / p4.z) * half_height;

		tgt[0].x = (p1d.x / p1d.z) * half_width;
		tgt[0].y = (p1d.y / p1d.z) * half_height;
		tgt[1].x = (p2d.x / p2d.z) * half_width;
		tgt[1].y = (p2d.y / p2d.z) * half_height;
		tgt[2].x = (p3d.x / p3d.z) * half_width;
		tgt[2].y = (p3d.y / p3d.z) * half_height;
		tgt[3].x = (p4d.x / p4d.z) * half_width;
		tgt[3].y = (p4d.y / p4d.z) * half_height;

        projMat = projection_matrix2(tgt, src);

		matv[0] = projMat->var[0][0];
		matv[1] = projMat->var[0][1];
		matv[2] = projMat->var[0][2];
		matv[3] = projMat->var[1][0];
		matv[4] = projMat->var[1][1];
		matv[5] = projMat->var[1][2];
		matv[6] = projMat->var[2][0];
		matv[7] = projMat->var[2][1];
		matv[8] = projMat->var[2][2];

		matrix_free(projMat);
	}


	if(pitcha != pitcha_p) {
        angle = pitcha_p * 3.141592653589 / 180;  //주의, 각도를 -로 넣지 않음
		sinv = sin(angle);
		cosv = cos(angle);
		//perspective transform 기준 좌표 구하기
		p1.x = -2.0; p1.y = 2.0; p1.z = 2.0;
		p2.x = 2.0; p2.y = 2.0; p2.z = 2.0;
		p3.x = -2.0; p3.y = -2.0; p3.z = 2.0;
		p4.x = 2.0; p4.y = -2.0; p4.z = 2.0;
		p1d.x = p1.x;
		p1d.y = p1.y * cosv;
		p1d.z = p1.z + p1.y * sinv;
		p2d.x = p2.x;
		p2d.y = p2.y * cosv;
		p2d.z = p2.z + p2.y * sinv;
		p3d.x = p3.x;
		p3d.y = p3.y * cosv;
		p3d.z = p3.z + p3.y * sinv;
		p4d.x = p4.x;
		p4d.y = p4.y * cosv;
		p4d.z = p4.z + p4.y * sinv;

		src[0].x = (p1.x / p1.z) * half_width;
		src[0].y = (p1.y / p1.z) * half_height;
		src[1].x = (p2.x / p2.z) * half_width;
		src[1].y = (p2.y / p2.z) * half_height;
		src[2].x = (p3.x / p3.z) * half_width;
		src[2].y = (p3.y / p3.z) * half_height;
		src[3].x = (p4.x / p4.z) * half_width;
		src[3].y = (p4.y / p4.z) * half_height;

		tgt[0].x = (p1d.x / p1d.z) * half_width;
		tgt[0].y = (p1d.y / p1d.z) * half_height;
		tgt[1].x = (p2d.x / p2d.z) * half_width;
		tgt[1].y = (p2d.y / p2d.z) * half_height;
		tgt[2].x = (p3d.x / p3d.z) * half_width;
		tgt[2].y = (p3d.y / p3d.z) * half_height;
		tgt[3].x = (p4d.x / p4d.z) * half_width;
		tgt[3].y = (p4d.y / p4d.z) * half_height;

		projMat_p_temp = projection_matrix2(tgt, src); //src, tgt 순서로 호출하면 발산하는 문제 있음
		projMat_p = matrix_inv(projMat_p_temp);  //따라서 호출을 tgt, src 순으로 하고 inverse matrix 구함

		matv_p[0] = projMat_p->var[0][0];
		matv_p[1] = projMat_p->var[0][1];
		matv_p[2] = projMat_p->var[0][2];
		matv_p[3] = projMat_p->var[1][0];
		matv_p[4] = projMat_p->var[1][1];
		matv_p[5] = projMat_p->var[1][2];
		matv_p[6] = projMat_p->var[2][0];
		matv_p[7] = projMat_p->var[2][1];
		matv_p[8] = projMat_p->var[2][2];

		matrix_free(projMat_p);
		matrix_free(projMat_p_temp);

		angle = pitcha * 3.141592653589 / 180;
		sinv = sin(angle);
		cosv = cos(angle);
		//perspective transform 기준 좌표 구하기

		p1.x = -2.0; p1.y = 2.0; p1.z = 2.0;
		p2.x = 2.0; p2.y = 2.0; p2.z = 2.0;
		p3.x = -2.0; p3.y = -2.0; p3.z = 2.0;
		p4.x = 2.0; p4.y = -2.0; p4.z = 2.0;
		p1d.x = p1.x;
		p1d.y = p1.y * cosv;
		p1d.z = p1.z + p1.y * sinv;
		p2d.x = p2.x;
		p2d.y = p2.y * cosv;
		p2d.z = p2.z + p2.y * sinv;
		p3d.x = p3.x;
		p3d.y = p3.y * cosv;
		p3d.z = p3.z + p3.y * sinv;
		p4d.x = p4.x;
		p4d.y = p4.y * cosv;
		p4d.z = p4.z + p4.y * sinv;

		src[0].x = (p1.x / p1.z) * half_width;
		src[0].y = (p1.y / p1.z) * half_height;
		src[1].x = (p2.x / p2.z) * half_width;
		src[1].y = (p2.y / p2.z) * half_height;
		src[2].x = (p3.x / p3.z) * half_width;
		src[2].y = (p3.y / p3.z) * half_height;
		src[3].x = (p4.x / p4.z) * half_width;
		src[3].y = (p4.y / p4.z) * half_height;

		tgt[0].x = (p1d.x / p1d.z) * half_width;
		tgt[0].y = (p1d.y / p1d.z) * half_height;
		tgt[1].x = (p2d.x / p2d.z) * half_width;
		tgt[1].y = (p2d.y / p2d.z) * half_height;
		tgt[2].x = (p3d.x / p3d.z) * half_width;
		tgt[2].y = (p3d.y / p3d.z) * half_height;
		tgt[3].x = (p4d.x / p4d.z) * half_width;
		tgt[3].y = (p4d.y / p4d.z) * half_height;

        projMat = projection_matrix2(tgt, src);

		matv[0] = projMat->var[0][0];
		matv[1] = projMat->var[0][1];
		matv[2] = projMat->var[0][2];
		matv[3] = projMat->var[1][0];
		matv[4] = projMat->var[1][1];
		matv[5] = projMat->var[1][2];
		matv[6] = projMat->var[2][0];
		matv[7] = projMat->var[2][1];
		matv[8] = projMat->var[2][2];

		matrix_free(projMat);
	}

	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
    p_x = (float*)(*dst);
	p_y = (float*)(*dst) + 1;
	for (i = 0; i < bi.cp_cnt; ++i, p_x += bi.cp_dim, p_y += bi.cp_dim) {
        //untranspose
		(*p_x) = (*p_x) - dx_p;
		(*p_y) = (*p_y) - dy_p;


		//unroll
		angle = -rolla_p * 3.14159 / 180;
		x = cos(angle) * (*p_x) - sin(angle) * (*p_y);
		y = sin(angle) * (*p_x) + cos(angle) * (*p_y);
		(*p_x) = x;
		(*p_y) = y;

        /////// unyaw ////////////
		if(yawa != yawa_p) {
			ptf_x = (*p_x) * matv_p[0] + (*p_y) * matv_p[1] + matv_p[2];
			ptf_y = (*p_x) * matv_p[3] + (*p_y) * matv_p[4] + matv_p[5];
			ptf_z = (*p_x) * matv_p[6] + (*p_y) * matv_p[7] + matv_p[8];

			(*p_x) = ptf_x / ptf_z;
            (*p_y) = ptf_y / ptf_z;

        }

		////// un pitch //////////
        if(pitcha != pitcha_p) {
			ptf_x = (*p_x) * matv_p[0] + (*p_y) * matv_p[1] + matv_p[2];
			ptf_y = (*p_x) * matv_p[3] + (*p_y) * matv_p[4] + matv_p[5];
			ptf_z = (*p_x) * matv_p[6] + (*p_y) * matv_p[7] + matv_p[8];

			(*p_x) = ptf_x / ptf_z;
            (*p_y) = ptf_y / ptf_z;

        }

		//unscale
		if(x_anchor == 0) {
			(*p_x) = (*p_x + half_width) / sx_p - half_width;  //left anchor
		} else if(x_anchor == 1) {
            (*p_x) = (*p_x) / sx_p;   //center anchor
		} else if(x_anchor == 2) {
			(*p_x) = (*p_x - half_width) / sx_p + half_width;  //right anchor
		}
		if(y_anchor == 0) {
			(*p_y) = (*p_y - half_height) / sy_p + half_height; //top anchor
		} else if(y_anchor == 1) {
			(*p_y) = (*p_y) / sy_p; //center anchor
		} else if(y_anchor == 2) {
			(*p_y) = (*p_y + half_height) / sy_p - half_height; //bottom anchor
		}

//        //scale
		if(x_anchor == 0) {
			(*p_x) = (*p_x + half_width) * sx - half_width;  //left anchor
		} else if(x_anchor == 1) {
			(*p_x) = (*p_x) * sx;  //center anchor
		} else if(x_anchor == 2) {
			(*p_x) = (*p_x - half_width) * sx + half_width;  //right anchor
		}
		if(y_anchor == 0) {
			(*p_y) = (*p_y - half_height) * sy + half_height; //top anchor
		} else if(y_anchor == 1) {
			(*p_y) = (*p_y) * sy; //center anchor
		} else if(y_anchor == 2) {
			(*p_y) = (*p_y + half_height) * sy - half_height; //bottom anchor
		}

        /////// pitch ////////////
		if(pitcha != pitcha_p) {
			ptf_x = (*p_x) * matv[0] + (*p_y) * matv[1] + matv[2];
			ptf_y = (*p_x) * matv[3] + (*p_y) * matv[4] + matv[5];
			ptf_z = (*p_x) * matv[6] + (*p_y) * matv[7] + matv[8];

			(*p_x) = ptf_x / ptf_z;
            (*p_y) = ptf_y / ptf_z;

        }


		/////// yaw ////////////
		if(yawa != yawa_p) {
			ptf_x = (*p_x) * matv[0] + (*p_y) * matv[1] + matv[2];
			ptf_y = (*p_x) * matv[3] + (*p_y) * matv[4] + matv[5];
			ptf_z = (*p_x) * matv[6] + (*p_y) * matv[7] + matv[8];

			(*p_x) = ptf_x / ptf_z;
            (*p_y) = ptf_y / ptf_z;

        }


		//roll
		angle = rolla * 3.14159 / 180;
		x = cos(angle) * (*p_x) - sin(angle) * (*p_y);
		y = sin(angle) * (*p_x) + cos(angle) * (*p_y);
		(*p_x) = x;
        (*p_y) = y;

        //transpose
		(*p_x) = (*p_x) + dx;
        (*p_y) = (*p_y) + dy;
	}
}

void BEZIER2_COPY_CP(BEZIER2 *dst, float *cp)
{
	float *p;
	BEZIER_INFO bi;
	int i;
    int cnt;

	if (BEZIER2_Info(&bi, *dst) == 0)
		return;
	cnt = bi.cp_cnt * bi.cp_dim;
    p = (float*)(*dst);
	for(i = 0; i < cnt; i++, p++)
	{
        *p = cp[i];
	}

}
#endif //#if MCNEX

