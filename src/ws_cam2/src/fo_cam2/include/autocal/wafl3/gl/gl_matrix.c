/**--------------------------------------------------------------------------
@file gl_matrix.c
@brief OpenGL에 최적화 된 행렬 및 벡터 구현
@author ksg
*/
//---------------------------------------------------------------------------
#include <string.h>
#if (defined(CCS) || defined(__BORLANDC__))
#pragma hdrstop
#endif //(defined(CCS) || defined(__BORLANDC__))

#include "gl_matrix.h"
#include "system/mem_dynamic.h"
#include "system/sys_logger.h"
//---------------------------------------------------------------------------
#if defined(__BORLANDC__)
#pragma package(smart_init)
#endif //#if defined(__BORLANDC__)

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif //#ifndef M_PI

/////////////////////////////////////////////////////////////////////////////
/**--------------------------------------------------------------------------
@brief 2차원 배열 할당
@return 할당된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_init (void)
{
  vec2f_t out = (vec2f_t)allocZeroMemory(sizeof(float), 2);
  out[0] = out[1] = 0;
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 배열을 할당하면서 입력된 값으로 초기화함, 벡터의 두 값이 동일하게 초기화됨
@param [in ] scalar vec2f_t의 각 값을 초기화할 값
@return 할당된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_initWithScalar (float scalar)
{
  vec2f_t out = (vec2f_t)allocZeroMemory(sizeof(float), 2);
  out[0] = out[1] = scalar;
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 배열을 할당하면서 입력된 값으로 초기화함, 벡터의 두 값이 각각 초기화됨
@param [in ] s1 vec2f_t 타입의 첫번째 값
@param [in ] s2 vec2f_t 타입의 두번째 값
@return 할당된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_initWithComponents (float s1, float s2)
{
  vec2f_t out = (vec2f_t)allocZeroMemory(sizeof(float), 2);
  out[0] = s1;
  out[1] = s2;
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 배열을 할당하고 vec2f_t 타입을 입력 받아 그 값으로 초기화함
@param [in ] src vec2f_t를 초기화할 입력
@return 할당된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_initWithVec2f (const vec2f_t src)
{
  vec2f_t out = (vec2f_t)allocZeroMemory(sizeof(float), 2);
  out[0] = src[0];
  out[1] = src[1];
  return out;
}

/**--------------------------------------------------------------------------
@brief 할당된 2차원 배열을 받아서 그 값을 기존 2차원 배열 변수의 값으로 세팅함
@param [out] out 값을 세팅할 대상
@param [in ] v 세팅할 값
@return 할당된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_set (vec2f_t out, const vec2f_t v)
{
  out[0] = v[0];
  out[1] = v[1];
  return out;
}
/**--------------------------------------------------------------------------
@brief 할당된 2차원 배열을 받아서 그 값을 스칼라 값으로 세팅함, 두 값을 동일하게
@param [out] out 값을 세팅할 대상
@param [in ] scalar 세팅할 스칼라 값
@return 할당된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_setScalar (vec2f_t out, float scalar)
{
  out[0] = out[1] = scalar;
  return out;
}
/**--------------------------------------------------------------------------
@brief 할당된 2차원 배열을 받아서 그 값을 스칼라 값으로 세팅함, 두 값을 각자
@param [out] out 값을 세팅할 대상
@param [in ] s1 세팅할 스칼라 값1
@param [in ] s2 세팅할 스칼라 값2
@return 할당된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_setComponents(vec2f_t out, float s1, float s2)
{
  out[0] = s1;
  out[1] = s2;
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 벡터 합 연산
@param [in ] v1 합 연산의 인자1
@param [in ] v2 합 연산의 인자2
@param [in,out] out 합 연산의 결과
@return 계산된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_add (vec2f_t v1, const vec2f_t v2, vec2f_t out)
{
  if (out == NULL || v1 == out) {
	v1[0] += v2[0];
	v1[1] += v2[1];
	return v1;
  }
  out[0] = v1[0] + v2[0];
  out[1] = v1[1] + v2[1];
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 벡터에 스칼라 값을 더하기
@param [in ] v 합 연산에서 벡터 인자
@param [in ] scalar 합 연산에서 스칼라 인자
@param [in,out] out 합 연산의 결과
@return 계산된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_addScalar (vec2f_t v, float scalar, vec2f_t out)
{
  if (out == NULL || v == out) {
    v[0] += scalar;
    v[1] += scalar;
    return v;
  }
  out[0] = v[0] + scalar;
  out[1] = v[1] + scalar;
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 벡터 감산 연산
@param [in ] v1 감산 연산의 인자1
@param [in ] v2 감산 연산의 인자2
@param [in,out] out 감산 연산의 결과
@return 계산된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_subtract (vec2f_t v1, const vec2f_t v2, vec2f_t out)
{
  if (out == NULL || v1 == out) {
    v1[0] -= v2[0];
    v1[1] -= v2[1];
    return v1;
  }
  out[0] = v1[0] - v2[0];
  out[1] = v1[1] - v2[1];
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 벡터 곱연산
@param [in ] v1 곱 연산 벡터 인자1
@param [in ] v2 곱 연산 벡터 인자2
@param [in,out] out 곱 연산의 결과
@return 계산된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_multiply (vec2f_t v1, const vec2f_t v2, vec2f_t out)
{
  if (out == NULL || v1 == out) {
    v1[0] *= v2[0];
    v1[1] *= v2[1];
    return v1;
  }
  out[0] = v1[0] * v2[0];
  out[1] = v1[1] * v2[1];
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 벡터와 스칼라 곱연산
@param [in ] v 곱 연산 벡터 인자
@param [in ] scalar 곱 연산 스칼라 인자
@param [in,out] out 곱 연산의 결과
@return 계산된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_multiplyScalar (vec2f_t v, float scalar, vec2f_t out)
{
  if (out == NULL || v == out) {
    v[0] *= scalar;
    v[1] *= scalar;
    return v;
  }
  out[0] = v[0] * scalar;
  out[1] = v[1] * scalar;
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 벡터 나눗셈 연산
@param [in ] v1 나눗셈 연산 벡터 인자1
@param [in ] v2 나눗셈 연산 벡터 인자2
@param [in,out] out 연산의 결과
@return 계산된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_divide (vec2f_t v1, const vec2f_t v2, vec2f_t out)
{
  if (out == NULL || v1 == out) {
    v1[0] /= v2[0];
    v1[1] /= v2[1];
    return v1;
  }
  out[0] = v1[0] / v2[0];
  out[1] = v1[1] / v2[1];
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 벡터의 크기가 1이 되도록 normal 벡터 계산
@param [in ] v normalize 대상 벡터
@param [in,out] out 연산의 결과
@return 계산된 vec2f_t 타입
@author ksg
*/
vec2f_t WA_API vec2f_normalize (vec2f_t v, vec2f_t out)
{
  float x = v[0], y = v[1],
      len = (float)sqrt(x * x + y * y);

  if (out == NULL) {
     out = v;
  }
  if (float_is_zero(len)) {
    out[0] = 0;
    out[1] = 0;
    return out;
  } else if (float_is_one(len)) {
    out[0] = x;
    out[1] = y;
    return out;
  }

  len = 1.0f / len;
  out[0] = x * len;
  out[1] = y * len;
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 벡터의 크기의 제곱 구하기
@param [in ] v 연산 대상 벡터
@return 벡터의 크기의 제곱 값
@author ksg
*/
float   WA_API vec2f_squaredLength (const vec2f_t v)
{
  float x = v[0], y = v[1];
  return x * x + y * y;
}
/**--------------------------------------------------------------------------
@brief 2차원 벡터의 내적 구하기(. 연산)
@param [in ] v1 연산 대상 벡터1
@param [in ] v2 연산 대상 벡터2
@return 내적
@author ksg
*/
float   WA_API vec2f_dot (const vec2f_t v1, const vec2f_t v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1];
}
/**--------------------------------------------------------------------------
@brief 2개의 2차원 백터 사이의 방향을 normal 벡터로 구하기(벡터의 방향)
@param [in ] v1 연산 대상 벡터1
@param [in ] v2 연산 대상 벡터2
@param [in,out] out 연산 결과 벡터
@return v2->v1 벡터의 normal 벡터
@author ksg
*/
vec2f_t WA_API vec2f_direction (vec2f_t v1, const vec2f_t v2, vec2f_t out)
{
  float x = v1[0] - v2[0],
        y = v1[1] - v2[1],
      len = (float)sqrt(x * x + y * y);

  if (out == NULL) {
    out = v1;
  }
  if (float_is_zero(len)) {
    out[0] = 0;
    out[1] = 0;
    return out;
  }

  len = 1.0f / len;
  out[0] = x * len;
  out[1] = y * len;

  return out;
}
/**--------------------------------------------------------------------------
@brief 2개의 2차원 백터 사이에 있는 벡터 구하기
@param [in ] v1 연산 대상 벡터1
@param [in ] v2 연산 대상 벡터2
@param [in ] lerp 2개 백터 사이의 위치 지정, 0.0이면 v1, 1.0이면 v2, 0.0 ~ 1.0 사이면
  두 백터의 사이가 됨
@param [in,out] out 연산 결과 벡터
@return 연산의 결과 벡터
@author ksg
*/
vec2f_t WA_API vec2f_lerp (vec2f_t v1, const vec2f_t v2, float lerp, vec2f_t out)
{
  if (out == NULL) {
    out = v1;
  }

  out[0] = v1[0] + lerp * (v2[0] - v1[0]);
  out[1] = v1[1] + lerp * (v2[1] - v1[1]);

  return out;
}
/**--------------------------------------------------------------------------
@brief 2개의 2차원 백터 사이의 거리의 제곱 구하기
@param [in ] v1 연산 대상 벡터1
@param [in ] v2 연산 대상 벡터2
@return 연산의 결과
@author ksg
*/
float   WA_API vec2f_squaredDistance (const vec2f_t v1, const vec2f_t v2)
{
  float x = v2[0] - v1[0],
        y = v2[1] - v1[1];

  return x*x + y*y;
}
/**--------------------------------------------------------------------------
@brief 2차원 백터의 마이너스 벡터(반대 방향) 구하기
@param [in ] v 연산 대상 벡터
@param [in,out] out 연산 출력 벡터
@return 연산의 결과
@author ksg
*/
vec2f_t WA_API vec2f_negate (vec2f_t v, vec2f_t out)
{
  if (out == NULL) {
   out = v;
  }

  out[0] = -v[0];
  out[1] = -v[1];
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 백터의 인버스 연산
@param [in ] v 연산 대상 벡터
@param [in,out] out 연산 출력 벡터
@return 연산의 결과
@author ksg
*/
vec2f_t WA_API vec2f_inverse (vec2f_t v, vec2f_t out)
{
  if (out == NULL) {
   out = v;
  }
  out[0] = 1.0f / v[0];
  out[1] = 1.0f / v[1];
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 백터를 정의된 4x4행렬로 perspective 변환함
@param [in ] v 연산 대상 벡터
@param [in ] mat perspective 변환을 위한 4x4 행렬
@param [in,out] out 연산 출력 벡터
@return 연산의 결과
@author ksg
*/
vec2f_t WA_API vec2f_transformMat4f(vec2f_t v, const mat4f_t mat, vec2f_t out)
{
  float x = v[0], y = v[1],
        w = mat[3] * x + mat[7] * y + mat[15];
  if (!out) {
    out = v;
  }
  out[0] = mat[0] * x + mat[4] * y + mat[12];
  out[1] = mat[1] * x + mat[5] * y + mat[13];

  if (!float_is_one(w)) {
    w = 1.0f / w;

    out[0] *= w;

    out[1] *= w;

  }


  return out;

}

/////////////////////////////////////////////////////////////////////////////

/**--------------------------------------------------------------------------
@brief 3차원 배열 할당
@return 할당된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_init (void)
{
  vec3f_t out = (vec3f_t)allocZeroMemory(sizeof(float), 3);
  out[0] = out[1] = out[2] = 0;
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 배열을 할당하면서 입력된 값으로 초기화함, 벡터의 세 값이 동일하게 초기화됨
@param [in ] scalar vec3f_t의 각 값을 초기화할 값
@return 할당된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_initWithScalar (float scalar)
{
  vec3f_t out = (vec3f_t)allocZeroMemory(sizeof(float), 3);
  out[0] = out[1] = out[2] = scalar;
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 배열을 할당하면서 입력된 값으로 초기화함, 벡터의 세 값이 각각 초기화됨
@param [in ] s1 vec3f_t 타입의 첫번째 값
@param [in ] s2 vec3f_t 타입의 두번째 값
@param [in ] s3 vec3f_t 타입의 세번째 값
@return 할당된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_initWithComponents (float s1, float s2, float s3)
{
  vec3f_t out = (vec3f_t)allocZeroMemory(sizeof(float), 3);
  out[0] = s1;
  out[1] = s2;
  out[2] = s3;
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 배열을 할당하고 vec3f_t 타입을 입력 받아 그 값으로 초기화함
@param [in ] src vec3f_t를 초기화할 입력
@return 할당된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_initWithVec3f (const vec3f_t src)
{
  vec3f_t out = (vec3f_t)allocZeroMemory(sizeof(float), 3);
  out[0] = src[0];
  out[1] = src[1];
  out[2] = src[2];
  return out;
}

/**--------------------------------------------------------------------------
@brief 할당된 3차원 배열을 받아서 그 값을 기존 3차원 배열 변수의 값으로 세팅함
@param [out] out 값을 세팅할 대상
@param [in ] v 세팅할 값
@return 할당된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_set (vec3f_t out, const vec3f_t v)
{
  out[0] = v[0];
  out[1] = v[1];
  out[2] = v[2];
  return out;
}
/**--------------------------------------------------------------------------
@brief 할당된 3차원 배열을 받아서 그 값을 스칼라 값으로 세팅함, 세 값을 동일하게
@param [out] out 값을 세팅할 대상
@param [in ] scalar 세팅할 스칼라 값
@return 할당된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_setScalar (vec3f_t out, float scalar)
{
  out[0] = out[1] = out[2] = scalar;
  return out;
}
/**--------------------------------------------------------------------------
@brief 할당된 3차원 배열을 받아서 그 값을 스칼라 값으로 세팅함, 세 값을 각자
@param [out] out 값을 세팅할 대상
@param [in ] s1 세팅할 스칼라 값1
@param [in ] s2 세팅할 스칼라 값2
@param [in ] s3 세팅할 스칼라 값3
@return 할당된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_setComponents(vec3f_t out, float s1, float s2, float s3)
{
  out[0] = s1;
  out[1] = s2;
  out[2] = s3;
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터 합 연산
@param [in ] v1 합 연산의 인자1
@param [in ] v2 합 연산의 인자2
@param [in,out] out 합 연산의 결과
@return 계산된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_add (vec3f_t v1, const vec3f_t v2, vec3f_t out)
{
  if (out == NULL || v1 == out) {
    v1[0] += v2[0];
    v1[1] += v2[1];
    v1[2] += v2[2];
    return v1;
  }
  out[0] = v1[0] + v2[0];
  out[1] = v1[1] + v2[1];
  out[2] = v1[2] + v2[2];
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터에 스칼라 값을 더하기
@param [in ] v 합 연산에서 벡터 인자
@param [in ] scalar 합 연산에서 스칼라 인자
@param [in,out] out 합 연산의 결과
@return 계산된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_addScalar (vec3f_t v, float scalar, vec3f_t out)
{
  if (out == NULL || v == out) {
    v[0] += scalar;
    v[1] += scalar;
    v[2] += scalar;
    return v;
  }
  out[0] = v[0] + scalar;
  out[1] = v[1] + scalar;
  out[2] = v[2] + scalar;
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터 감산 연산
@param [in ] v1 감산 연산의 인자1
@param [in ] v2 감산 연산의 인자2
@param [in,out] out 감산 연산의 결과
@return 계산된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_subtract (vec3f_t v1, const vec3f_t v2, vec3f_t out)
{
  if (out == NULL || v1 == out) {
    v1[0] -= v2[0];
    v1[1] -= v2[1];
    v1[2] -= v2[2];
    return v1;
  }
  out[0] = v1[0] - v2[0];
  out[1] = v1[1] - v2[1];
  out[2] = v1[2] - v2[2];
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터 곱연산
@param [in ] v1 곱 연산 벡터 인자1
@param [in ] v2 곱 연산 벡터 인자2
@param [in,out] out 곱 연산의 결과
@return 계산된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_multiply (vec3f_t v1, const vec3f_t v2, vec3f_t out)
{
  if (out == NULL || v1 == out) {
    v1[0] *= v2[0];
    v1[1] *= v2[1];
    v1[2] *= v2[2];
    return v1;
  }
  out[0] = v1[0] * v2[0];
  out[1] = v1[1] * v2[1];
  out[2] = v1[2] * v2[2];
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터와 스칼라 곱연산
@param [in ] v 곱 연산 벡터 인자
@param [in ] scalar 곱 연산 스칼라 인자
@param [in,out] out 곱 연산의 결과
@return 계산된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_multiplyScalar (vec3f_t v, float scalar, vec3f_t out)
{
  if (out == NULL || v == out) {
    v[0] *= scalar;
    v[1] *= scalar;
    v[2] *= scalar;
    return v;
  }
  out[0] = v[0] * scalar;
  out[1] = v[1] * scalar;
  out[2] = v[2] * scalar;
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터 나눗셈 연산
@param [in ] v1 나눗셈 연산 벡터 인자1
@param [in ] v2 나눗셈 연산 벡터 인자2
@param [in,out] out 연산의 결과
@return 계산된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_divide (vec3f_t v1, const vec3f_t v2, vec3f_t out)
{
  if (out == NULL || v1 == out) {
    v1[0] /= v2[0];
    v1[1] /= v2[1];
    v1[2] /= v2[2];
    return v1;
  }
  out[0] = v1[0] / v2[0];
  out[1] = v1[1] / v2[1];
  out[2] = v1[2] / v2[2];
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터의 크기가 1이 되도록 normal 벡터 계산
@param [in ] v normalize 대상 벡터
@param [in,out] out 연산의 결과
@return 계산된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_normalize (vec3f_t v, vec3f_t out)
{
  float x = v[0], y = v[1], z = v[2],
      len = (float)sqrt(x * x + y * y + z * z);

  if (out == NULL) {
     out = v;
  }
  if (float_is_zero(len)) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    return out;
  } else if (float_is_one(len)) {
    out[0] = x;
    out[1] = y;
    out[2] = z;
    return out;
  }

  len = 1.0f / len;
  out[0] = x * len;
  out[1] = y * len;
  out[2] = z * len;
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터 외적 계산, x 연산
@param [in ] v1 연산 대상 벡터1
@param [in ] v2 연산 대상 벡터2
@param [in,out] out 연산의 결과
@return 계산된 vec3f_t 타입
@author ksg
*/
vec3f_t WA_API vec3f_cross (vec3f_t v1, const vec3f_t v2, vec3f_t out)
{
  float x1 = v1[0], y1 = v1[1], z1 = v1[2],
        x2 = v2[0], y2 = v2[1], z2 = v2[2];
  if (out == NULL) {
    out = v1;
  }
  out[0] = y1 * z2 - z1 * y2;
  out[1] = z1 * x2 - x1 * z2;
  out[2] = x1 * y2 - y1 * x2;
  return out;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터의 크기의 제곱 구하기
@param [in ] v 연산 대상 벡터
@return 벡터의 크기의 제곱 값
@author ksg
*/
float   WA_API vec3f_squaredLength (const vec3f_t v)
{
  float x = v[0], y = v[1], z = v[2];
  return x * x + y * y + z * z;
}
/**--------------------------------------------------------------------------
@brief 3차원 벡터의 내적 구하기(. 연산)
@param [in ] v1 연산 대상 벡터1
@param [in ] v2 연산 대상 벡터2
@return 내적
@author ksg
*/
real_t   WA_API vec3f_dot (const vec3f_t v1, const vec3f_t v2)
{
  return (v1[0] * v2[0]) + (v1[1] * v2[1]) + (v1[2] * v2[2]);
}
/**--------------------------------------------------------------------------
@brief 2개의 3차원 백터 사이의 방향을 normal 벡터로 구하기(벡터의 방향)
@param [in ] v1 연산 대상 벡터1
@param [in ] v2 연산 대상 벡터2
@param [in,out] out 연산의 결과
@return v2->v1 벡터의 normal 벡터
@author ksg
*/
vec3f_t WA_API vec3f_direction (vec3f_t v1, const vec3f_t v2, vec3f_t out)
{
  float x = v1[0] - v2[0],
        y = v1[1] - v2[1],
        z = v1[2] - v2[2],
      len = (float)sqrt(x * x + y * y + z * z);

  if (out == NULL) {
    out = v1;
  }
  if (float_is_zero(len)) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    return out;
  }

  len = 1 / len;
  out[0] = x * len;
  out[1] = y * len;
  out[2] = z * len;

  return out;
}
/**--------------------------------------------------------------------------
@brief 2개의 3차원 백터 사이에 있는 벡터 구하기
@param [in ] v1 연산 대상 벡터1
@param [in ] v2 연산 대상 벡터2
@param [in ] lerp 2개 백터 사이의 위치 지정, 0.0이면 v1, 1.0이면 v2, 0.0 ~ 1.0 사이면
  두 백터의 사이가 됨
@param [in,out] out 연산의 결과
@return 연산의 결과 벡터
@author ksg
*/
vec3f_t WA_API vec3f_lerp (vec3f_t v1, const vec3f_t v2, float lerp, vec3f_t out)
{
  if (out == NULL) {
    out = v1;
  }

  out[0] = v1[0] + lerp * (v2[0] - v1[0]);
  out[1] = v1[1] + lerp * (v2[1] - v1[1]);
  out[2] = v1[2] + lerp * (v2[2] - v1[2]);

  return out;
}
/**--------------------------------------------------------------------------
@brief 2개의 3차원 백터 사이의 거리의 제곱 구하기
@param [in ] v1 연산 대상 벡터1
@param [in ] v2 연산 대상 벡터2
@return 연산의 결과
@author ksg
*/
float   WA_API vec3f_squaredDistance (const vec3f_t v1, const vec3f_t v2)
{
  float x = v2[0] - v1[0],
        y = v2[1] - v1[1],
        z = v2[2] - v1[2];

  return x*x + y*y + z*z;
}
/**--------------------------------------------------------------------------
@brief 3차원 백터의 마이너스 벡터(반대 방향) 구하기
@param [in ] v 연산 대상 벡터
@param [in,out] out 연산 출력 벡터
@return 연산의 결과
@author ksg
*/
vec3f_t WA_API vec3f_negate (vec3f_t v, vec3f_t out)
{
  if (out == NULL) {
   out = v;
  }

  out[0] = -v[0];
  out[1] = -v[1];
  out[2] = -v[2];
  return out;
}
/**--------------------------------------------------------------------------
@brief 2차원 백터의 인버스 연산
@param [in ] v 연산 대상 벡터
@param [in,out] out 연산 출력 벡터
@return 연산의 결과
@author ksg
*/
vec3f_t WA_API vec3f_inverse (vec3f_t v, vec3f_t out)
{
  if (out == NULL) {
   out = v;
  }
  out[0] = 1.0f / v[0];
  out[1] = 1.0f / v[1];
  out[2] = 1.0f / v[2];
  return out;
}

/**--------------------------------------------------------------------------
@brief 3차원 백터를 정의된 4x4행렬로 perspective 변환함
@param [in ] v 연산 대상 벡터
@param [in ] mat perspective 변환을 위한 4x4 행렬
@param [in,out] out 연산 출력 벡터
@return 연산의 결과
@author ksg
*/
vec3f_t WA_API vec3f_transformMat4f(vec3f_t v, const mat4f_t mat, vec3f_t out)
{
/*
  float x = v[0], y = v[1], z = v[2],
		w = mat[3] * x + mat[7] * y + mat[11] * z + mat[15];
*/
	double  x = v[0], y = v[1], z = v[2],
		w = mat[3] * x + mat[7] * y + mat[11] * z + mat[15];
  if (!out) {
    out = v;
  }
  out[0] = mat[0] * x + mat[4] * y + mat[8] * z + mat[12];
  out[1] = mat[1] * x + mat[5] * y + mat[9] * z + mat[13];
  out[2] = mat[2] * x + mat[6] * y + mat[10] * z + mat[14];

  if (w != 1.0f) {
    w = 1.0f / w;

    out[0] *= w;

    out[1] *= w;

    out[2] *= w;

  }


  return out;

}

/////////////////////////////////////////////////////////////////////////////
/**--------------------------------------------------------------------------
@brief 4차원 백터를 정의된 4x4행렬로 perspective 변환함
@param [in ] v 연산 대상 벡터
@param [in ] mat perspective 변환을 위한 4x4 행렬
@param [in,out] out 연산 출력 벡터
@return 연산의 결과
@author ksg
*/
vec4f_t WA_API vec4f_transformMat4f(vec4f_t v, const mat4f_t mat, vec4f_t out)
{
  float x = v[0], y = v[1], z = v[2], w = v[3];
  if (!out) {
    out = v;
  }

  out[0] = mat[0] * x + mat[4] * y + mat[8] * z + mat[12] * w;
  out[1] = mat[1] * x + mat[5] * y + mat[9] * z + mat[13] * w;
  out[2] = mat[2] * x + mat[6] * y + mat[10] * z + mat[14] * w;
  out[3] = mat[3] * x + mat[7] * y + mat[11] * z + mat[15] * w;

  return out;
}






/////////////////////////////////////////////////////////////////////////////
/**--------------------------------------------------------------------------
@brief 4x4 행렬을 할당, 초기값은 단위 행렬
@return 할당된 mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_init(void)
{
  mat4f_t out = (mat4f_t)allocZeroMemory(sizeof(float), 16);

  out[0] = 1;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = 1;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 1;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;

  return out;
}

/**--------------------------------------------------------------------------
@brief 3x3 행렬을 할당, 초기값은 단위 행렬
@return 할당된 mat3d_t 타입 행렬
@author ksg
*/
mat3d_t WA_API mat3d_init(void)
{
	mat3d_t out = (mat3d_t)allocZeroMemory(sizeof(double), 9);

	out[0] = 1;
	out[1] = 0;
	out[2] = 0;
	out[3] = 0;
	out[4] = 1;
	out[5] = 0;
	out[6] = 0;
	out[7] = 0;
	out[8] = 1;
	
	return out;
}

vec4d_t WA_API vec4d_init(void)
{
	vec4d_t out = (vec4d_t)allocZeroMemory(sizeof(double), 4);
	out[0] = 0;
	out[1] = 0;
	out[2] = 0;
	out[3] = 0;

	return out;
}

vec8d_t WA_API vec8d_init(void)
{
	vec8d_t out = (vec8d_t)calloc(sizeof(double), 8);
	out[0] = 0;
	out[1] = 0;
	out[2] = 0;
	out[3] = 0;
	out[4] = 0;
	out[5] = 0;
	out[6] = 0;
	out[7] = 0;

	return out;
}

/**--------------------------------------------------------------------------
@brief 4x4 행렬을 할당하고 입력 받은 행렬로 초기화
@param [in ] mat 초기화 값으로 참조하는 4x4 행렬
@return 할당된 mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_initWithMat4f(const mat4f_t mat)
{
  mat4f_t out = (mat4f_t)allocZeroMemory(sizeof(float), 16);
  out[0]  = mat[0];
  out[1]  = mat[1];
  out[2]  = mat[2];
  out[3]  = mat[3];
  out[4]  = mat[4];
  out[5]  = mat[5];
  out[6]  = mat[6];
  out[7]  = mat[7];
  out[8]  = mat[8];
  out[9]  = mat[9];
  out[10] = mat[10];
  out[11] = mat[11];
  out[12] = mat[12];
  out[13] = mat[13];
  out[14] = mat[14];
  out[15] = mat[15];
  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 행렬을 입력 받은 행렬로 값을 넣음
@param [out] out 값을 넣을 대상 행렬
@param [in ] mat 값을 넣을 때 참조하는 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_set(mat4f_t out, const mat4f_t mat)
{
  out[0]  = mat[0];
  out[1]  = mat[1];
  out[2]  = mat[2];
  out[3]  = mat[3];
  out[4]  = mat[4];
  out[5]  = mat[5];
  out[6]  = mat[6];
  out[7]  = mat[7];
  out[8]  = mat[8];
  out[9]  = mat[9];
  out[10] = mat[10];
  out[11] = mat[11];
  out[12] = mat[12];
  out[13] = mat[13];
  out[14] = mat[14];
  out[15] = mat[15];
  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 단위 행렬로 세팅
@param [out] out 값을 넣을 대상 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_identity(mat4f_t out)
{
  out[0] = 1;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = 1;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = 1;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 행렬의 transpose 연산
@param [in ] mat transpose할 행렬
@param [in,out] out transpose한 값을 넣을 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_transpose(mat4f_t mat, mat4f_t out)
{
  if (!out || mat == out) {
    float a01 = mat[1], a02 = mat[2], a03 = mat[3],
          a12 = mat[6], a13 = mat[7],
          a23 = mat[11];
    mat[1]  = mat[4];
    mat[2]  = mat[8];
    mat[3]  = mat[12];
    mat[4]  = a01;
    mat[6]  = mat[9];
    mat[7]  = mat[13];
    mat[8]  = a02;
    mat[9]  = a12;
    mat[11] = mat[14];
    mat[12] = a03;
    mat[13] = a13;
    mat[14] = a23;
    return mat;
  }

  out[0]  = mat[0];
  out[1]  = mat[4];
  out[2]  = mat[8];
  out[3]  = mat[12];
  out[4]  = mat[1];
  out[5]  = mat[5];
  out[6]  = mat[9];
  out[7]  = mat[13];
  out[8]  = mat[2];
  out[9]  = mat[6];
  out[10] = mat[10];
  out[11] = mat[14];
  out[12] = mat[3];
  out[13] = mat[7];
  out[14] = mat[11];
  out[15] = mat[15];

  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 행렬의 determinant 계산
@param [in ] mat 계산 대상 행렬
@return determinant 계산한 결과
@author ksg
*/
float   WA_API mat4f_determinant(const mat4f_t mat)
{
  float a00 = mat[0],  a01 = mat[1],  a02 = mat[2],  a03 = mat[3],
        a10 = mat[4],  a11 = mat[5],  a12 = mat[6],  a13 = mat[7],
        a20 = mat[8],  a21 = mat[9],  a22 = mat[10], a23 = mat[11],
        a30 = mat[12], a31 = mat[13], a32 = mat[14], a33 = mat[15],

        b00 = a00 * a11 - a01 * a10,
        b01 = a00 * a12 - a02 * a10,
        b02 = a00 * a13 - a03 * a10,
        b03 = a01 * a12 - a02 * a11,
        b04 = a01 * a13 - a03 * a11,
        b05 = a02 * a13 - a03 * a12,
        b06 = a20 * a31 - a21 * a30,
        b07 = a20 * a32 - a22 * a30,
        b08 = a20 * a33 - a23 * a30,
        b09 = a21 * a32 - a22 * a31,
        b10 = a21 * a33 - a23 * a31,
        b11 = a22 * a33 - a23 * a32;

  return b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
}
/**--------------------------------------------------------------------------
@brief 4x4 행렬의 역행렬 계산
@param [in ] mat 계산 대상 행렬
@param [in,out] out 계산 결과 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_inverse(mat4f_t mat, mat4f_t out)
{
  double a00 = mat[0], a01 = mat[1], a02 = mat[2], a03 = mat[3],
        a10 = mat[4], a11 = mat[5], a12 = mat[6], a13 = mat[7],
        a20 = mat[8], a21 = mat[9], a22 = mat[10], a23 = mat[11],
        a30 = mat[12], a31 = mat[13], a32 = mat[14], a33 = mat[15],

        b00 = a00 * a11 - a01 * a10,
        b01 = a00 * a12 - a02 * a10,
        b02 = a00 * a13 - a03 * a10,
        b03 = a01 * a12 - a02 * a11,
        b04 = a01 * a13 - a03 * a11,
        b05 = a02 * a13 - a03 * a12,
        b06 = a20 * a31 - a21 * a30,
        b07 = a20 * a32 - a22 * a30,
        b08 = a20 * a33 - a23 * a30,
        b09 = a21 * a32 - a22 * a31,
        b10 = a21 * a33 - a23 * a31,
        b11 = a22 * a33 - a23 * a32,

        det = (b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06);

  if (!out) {
    out = mat;
  }
  // Calculate the determinant
  if (det < DBL_EPSILON && det > -DBL_EPSILON) {
    return NULL;
  }
/*
  if (float_is_zero(det)) {
    return NULL;
  }
*/
  det = 1.0f / det;

  out[0]  = (real_t)(( a11 * b11 - a12 * b10 + a13 * b09) * det);
  out[1]  = (real_t)((-a01 * b11 + a02 * b10 - a03 * b09) * det);
  out[2]  = (real_t)(( a31 * b05 - a32 * b04 + a33 * b03) * det);
  out[3]  = (real_t)((-a21 * b05 + a22 * b04 - a23 * b03) * det);
  out[4]  = (real_t)((-a10 * b11 + a12 * b08 - a13 * b07) * det);
  out[5]  = (real_t)(( a00 * b11 - a02 * b08 + a03 * b07) * det);
  out[6]  = (real_t)((-a30 * b05 + a32 * b02 - a33 * b01) * det);
  out[7]  = (real_t)(( a20 * b05 - a22 * b02 + a23 * b01) * det);
  out[8]  = (real_t)(( a10 * b10 - a11 * b08 + a13 * b06) * det);
  out[9]  = (real_t)((-a00 * b10 + a01 * b08 - a03 * b06) * det);
  out[10] = (real_t)(( a30 * b04 - a31 * b02 + a33 * b00) * det);
  out[11] = (real_t)((-a20 * b04 + a21 * b02 - a23 * b00) * det);
  out[12] = (real_t)((-a10 * b09 + a11 * b07 - a12 * b06) * det);
  out[13] = (real_t)(( a00 * b09 - a01 * b07 + a02 * b06) * det);
  out[14] = (real_t)((-a30 * b03 + a31 * b01 - a32 * b00) * det);
  out[15] = (real_t)(( a20 * b03 - a21 * b01 + a22 * b00) * det);

  return out;
}
/*
mat4f_t WA_API mat4f_toRotationMat(mat4f_t mat, mat4f_t out)
{
    if (!out) { out = mat4f_init(); }

    out[0] = mat[0];
    out[1] = mat[1];
    out[2] = mat[2];
    out[3] = mat[3];
    out[4] = mat[4];
    out[5] = mat[5];
    out[6] = mat[6];
    out[7] = mat[7];
    out[8] = mat[8];
    out[9] = mat[9];
    out[10] = mat[10];
    out[11] = mat[11];
    out[12] = 0;
    out[13] = 0;
    out[14] = 0;
    out[15] = 1;

    return out;

}
*/
//mat3_t WA_API mat4f_toMat3(mat4f_t mat, mat3_t out)
//{

//}
//mat3_t WA_API mat4f_toInverseMat3(mat4f_t mat, mat3_t out);
//{

//}

/**--------------------------------------------------------------------------
@brief 4x4 행렬끼리의 곱셈
@param [in ] mat1 곱셈 대상 행렬1
@param [in ] mat2 곱셈 대상 행렬2
@param [in,out] out 계산 결과 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_multiply(mat4f_t mat1, const mat4f_t mat2, mat4f_t out)
{
  float a00 = mat1[0],  a01 = mat1[1],  a02 = mat1[2],  a03 = mat1[3],
        a10 = mat1[4],  a11 = mat1[5],  a12 = mat1[6],  a13 = mat1[7],
        a20 = mat1[8],  a21 = mat1[9],  a22 = mat1[10], a23 = mat1[11],
        a30 = mat1[12], a31 = mat1[13], a32 = mat1[14], a33 = mat1[15],

        b0 = mat2[0], b1 = mat2[1], b2 = mat2[2], b3 = mat2[3];

  if (!out) {
    out = mat1;
  }

  // Cache only the current line of the second matrix
  out[0] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
  out[1] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
  out[2] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
  out[3] = b0*a03 + b1*a13 + b2*a23 + b3*a33;

  b0 = mat2[4]; b1 = mat2[5]; b2 = mat2[6]; b3 = mat2[7];
  out[4] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
  out[5] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
  out[6] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
  out[7] = b0*a03 + b1*a13 + b2*a23 + b3*a33;

  b0 = mat2[8]; b1 = mat2[9]; b2 = mat2[10]; b3 = mat2[11];
  out[8] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
  out[9] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
  out[10] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
  out[11] = b0*a03 + b1*a13 + b2*a23 + b3*a33;

  b0 = mat2[12]; b1 = mat2[13]; b2 = mat2[14]; b3 = mat2[15];
  out[12] = b0*a00 + b1*a10 + b2*a20 + b3*a30;
  out[13] = b0*a01 + b1*a11 + b2*a21 + b3*a31;
  out[14] = b0*a02 + b1*a12 + b2*a22 + b3*a32;
  out[15] = b0*a03 + b1*a13 + b2*a23 + b3*a33;

  return out;
}

//mat4f_t WA_API mat4f_multiplyVec4f(mat4f_t mat, vec4_t v, mat4f_t out)
//{

//}
/**--------------------------------------------------------------------------
@brief 4x4 행렬을 x, y, z 방향으로 수평이동했을 때의 행렬을 구함
@param [in ] mat 수평 이동 대상 행렬
@param [in ] x x방향 이동 값
@param [in ] y y방향 이동 값
@param [in ] z z방향 이동 값
@param [in,out] out 계산 결과 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_translateByComponents(mat4f_t mat, float x, float y, float z, mat4f_t out)
{
  float a00, a01, a02, a03,
        a10, a11, a12, a13,
        a20, a21, a22, a23;

  if (!out || mat == out) {
    mat[12] = mat[0] * x + mat[4] * y + mat[8] * z + mat[12];
    mat[13] = mat[1] * x + mat[5] * y + mat[9] * z + mat[13];
    mat[14] = mat[2] * x + mat[6] * y + mat[10] * z + mat[14];
    mat[15] = mat[3] * x + mat[7] * y + mat[11] * z + mat[15];
    return mat;
  }

  a00 = mat[0]; a01 = mat[1]; a02 = mat[2]; a03 = mat[3];
  a10 = mat[4]; a11 = mat[5]; a12 = mat[6]; a13 = mat[7];
  a20 = mat[8]; a21 = mat[9]; a22 = mat[10]; a23 = mat[11];

  out[0] = a00; out[1] = a01; out[2] = a02; out[3] = a03;
  out[4] = a10; out[5] = a11; out[6] = a12; out[7] = a13;
  out[8] = a20; out[9] = a21; out[10] = a22; out[11] = a23;

  out[12] = a00 * x + a10 * y + a20 * z + mat[12];
  out[13] = a01 * x + a11 * y + a21 * z + mat[13];
  out[14] = a02 * x + a12 * y + a22 * z + mat[14];
  out[15] = a03 * x + a13 * y + a23 * z + mat[15];

  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 행렬을 x, y, z 축 방향으로 스케일링한 행렬 구함
@param [in ] mat 스케일링 대상 행렬
@param [in ] x x축 스케일링 값
@param [in ] y y축 스케일링 값
@param [in ] z z축 스케일링 값
@param [in,out] out 계산 결과 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_scaleByComponents(mat4f_t mat, float x, float y, float z, mat4f_t out)
{
  if (!out || mat == out) {
    mat[0]  *= x;
    mat[1]  *= x;
    mat[2]  *= x;
    mat[3]  *= x;
    mat[4]  *= y;
    mat[5]  *= y;
    mat[6]  *= y;
    mat[7]  *= y;
    mat[8]  *= z;
    mat[9]  *= z;
    mat[10] *= z;
    mat[11] *= z;
    return mat;
  }

  out[0]  = mat[0]  * x;
  out[1]  = mat[1]  * x;
  out[2]  = mat[2]  * x;
  out[3]  = mat[3]  * x;
  out[4]  = mat[4]  * y;
  out[5]  = mat[5]  * y;
  out[6]  = mat[6]  * y;
  out[7]  = mat[7]  * y;
  out[8]  = mat[8]  * z;
  out[9]  = mat[9]  * z;
  out[10] = mat[10] * z;
  out[11] = mat[11] * z;
  out[12] = mat[12];
  out[13] = mat[13];
  out[14] = mat[14];
  out[15] = mat[15];
  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 행렬을 x, y, z 값으로 정의한 축을 중심으로 회전한 행렬 구함
@param [in ] mat 회전 대상 행렬
@param [in ] angle 회전 각도(라디안)
@param [in ] x 회전축의 x값
@param [in ] y 회전축의 y값
@param [in ] z 회전축의 z값
@param [in,out] out 계산 결과 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_rotateByComponents(mat4f_t mat, float angle, float x, float y, float z, mat4f_t out)
{
  float len = (float)sqrt(x * x + y * y + z * z),
      s, c, t,
      a00, a01, a02, a03,
      a10, a11, a12, a13,
      a20, a21, a22, a23,
      b00, b01, b02,
      b10, b11, b12,
      b20, b21, b22;

  if (float_is_zero(len)) {
    return NULL;
  }

  if (!float_is_one(len)) {
    len = 1.0f / len;
    x *= len;
    y *= len;
    z *= len;
  }

  s = (real_t)sin(angle);
  c = (real_t)cos(angle);
  t = (real_t)(1.0 - c);

  a00 = mat[0]; a01 = mat[1]; a02 = mat[2];  a03 = mat[3];
  a10 = mat[4]; a11 = mat[5]; a12 = mat[6];  a13 = mat[7];
  a20 = mat[8]; a21 = mat[9]; a22 = mat[10]; a23 = mat[11];

  // Construct the elements of the rotation matrix
  b00 = x * x * t + c;     b01 = y * x * t + z * s; b02 = z * x * t - y * s;
  b10 = x * y * t - z * s; b11 = y * y * t + c;     b12 = z * y * t + x * s;
  b20 = x * z * t + y * s; b21 = y * z * t - x * s; b22 = z * z * t + c;

  if (!out) {
    out = mat;
  } else if (mat != out) { // If the source and destination differ, copy the unchanged last row
    out[12] = mat[12];
    out[13] = mat[13];
    out[14] = mat[14];
    out[15] = mat[15];
  }

  // Perform rotation-specific matrix multiplication
  out[0]  = a00 * b00 + a10 * b01 + a20 * b02;
  out[1]  = a01 * b00 + a11 * b01 + a21 * b02;
  out[2]  = a02 * b00 + a12 * b01 + a22 * b02;
  out[3]  = a03 * b00 + a13 * b01 + a23 * b02;

  out[4]  = a00 * b10 + a10 * b11 + a20 * b12;
  out[5]  = a01 * b10 + a11 * b11 + a21 * b12;
  out[6]  = a02 * b10 + a12 * b11 + a22 * b12;
  out[7]  = a03 * b10 + a13 * b11 + a23 * b12;

  out[8]  = a00 * b20 + a10 * b21 + a20 * b22;
  out[9]  = a01 * b20 + a11 * b21 + a21 * b22;
  out[10] = a02 * b20 + a12 * b21 + a22 * b22;
  out[11] = a03 * b20 + a13 * b21 + a23 * b22;

  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 행렬을 x축을 중심으로 회전한 행렬 구함
@param [in ] mat 회전 대상 행렬
@param [in ] angle 회전 각도(라디안)
@param [in,out] out 계산 결과 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_rotateX(mat4f_t mat, float angle, mat4f_t out)
{
  float s = (real_t)sin(angle),
        c = (real_t)cos(angle),
        a10 = mat[4],
        a11 = mat[5],
        a12 = mat[6],
        a13 = mat[7],
        a20 = mat[8],
        a21 = mat[9],
        a22 = mat[10],
        a23 = mat[11];

  if (!out) {
    out = mat;
  } else if (mat != out) { // If the source and destination differ, copy the unchanged rows
    out[0] = mat[0];
    out[1] = mat[1];
    out[2] = mat[2];
    out[3] = mat[3];

    out[12] = mat[12];
    out[13] = mat[13];
    out[14] = mat[14];
    out[15] = mat[15];
  }

  // Perform axis-specific matrix multiplication
  out[4] = a10 * c + a20 * s;
  out[5] = a11 * c + a21 * s;
  out[6] = a12 * c + a22 * s;
  out[7] = a13 * c + a23 * s;

  out[8] = a10 * -s + a20 * c;
  out[9] = a11 * -s + a21 * c;
  out[10] = a12 * -s + a22 * c;
  out[11] = a13 * -s + a23 * c;
  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 행렬을 y축을 중심으로 회전한 행렬 구함
@param [in ] mat 회전 대상 행렬
@param [in ] angle 회전 각도(라디안)
@param [in,out] out 계산 결과 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_rotateY(mat4f_t mat, float angle, mat4f_t out)
{
  float s = (real_t)sin(angle),
        c = (real_t)cos(angle),
        a00 = mat[0],
        a01 = mat[1],
        a02 = mat[2],
        a03 = mat[3],
        a20 = mat[8],
        a21 = mat[9],
        a22 = mat[10],
        a23 = mat[11];

  if (!out) {
    out = mat;
  } else if (mat != out) { // If the source and destination differ, copy the unchanged rows
    out[4] = mat[4];
    out[5] = mat[5];
    out[6] = mat[6];
    out[7] = mat[7];

    out[12] = mat[12];
    out[13] = mat[13];
    out[14] = mat[14];
    out[15] = mat[15];
  }

  // Perform axis-specific matrix multiplication
  out[0] = a00 * c + a20 * -s;
  out[1] = a01 * c + a21 * -s;
  out[2] = a02 * c + a22 * -s;
  out[3] = a03 * c + a23 * -s;

  out[8] = a00 * s + a20 * c;
  out[9] = a01 * s + a21 * c;
  out[10] = a02 * s + a22 * c;
  out[11] = a03 * s + a23 * c;
  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 행렬을 z축을 중심으로 회전한 행렬 구함
@param [in ] mat 회전 대상 행렬
@param [in ] angle 회전 각도(라디안)
@param [in,out] out 계산 결과 행렬
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_rotateZ(mat4f_t mat, float angle, mat4f_t out)
{
  float s = (real_t)sin(angle),
        c = (real_t)cos(angle),
        a00 = mat[0],
        a01 = mat[1],
        a02 = mat[2],
        a03 = mat[3],
        a10 = mat[4],
        a11 = mat[5],
        a12 = mat[6],
        a13 = mat[7];

  if (!out) {
    out = mat;
  } else if (mat != out) { // If the source and destination differ, copy the unchanged last row
    out[8] = mat[8];
    out[9] = mat[9];
    out[10] = mat[10];
    out[11] = mat[11];

    out[12] = mat[12];
    out[13] = mat[13];
    out[14] = mat[14];
    out[15] = mat[15];
  }

  // Perform axis-specific matrix multiplication
  out[0] = a00 * c + a10 * s;
  out[1] = a01 * c + a11 * s;
  out[2] = a02 * c + a12 * s;
  out[3] = a03 * c + a13 * s;

  out[4] = a00 * -s + a10 * c;
  out[5] = a01 * -s + a11 * c;
  out[6] = a02 * -s + a12 * c;
  out[7] = a03 * -s + a13 * c;

  return out;
}
/**--------------------------------------------------------------------------
@brief 4x4 크기의 원근 투영 행렬을 구함, 좌우, 상하, 전후 보이는 값으로부터 계산
@param [out] out 계산 결과 행렬
@param [in ] left 왼쪽 보이는 범위
@param [in ] right 오른쪽 보이는 범위
@param [in ] bottom 아랫쪽 보이는 범위
@param [in ] top 윗쪽 보이는 범위
@param [in ] near_ 가까운 쪽 보이는 범위
@param [in ] far_ 먼쪽 보이는 범위
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_frustum(mat4f_t out, float left, float right, float bottom, float top, float near_, float far_)
{
  float rl = (right - left),
        tb = (top - bottom),
        fn = (far_ - near_);
  out[0] = (near_ * 2) / rl;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = (near_ * 2) / tb;
  out[6] = 0;
  out[7] = 0;
  out[8] = (right + left) / rl;
  out[9] = (top + bottom) / tb;
  out[10] = -(far_ + near_) / fn;
  out[11] = -1;
  out[12] = 0;
  out[13] = 0;
  out[14] = -(far_ * near_ * 2) / fn;
  out[15] = 0;
  return out;

}

/**--------------------------------------------------------------------------
@brief 4x4 크기의 원근 투영 행렬을 구함, 세로 화각과 aspect ratio, 전후 보이는 값으로부터 계산
@param [out] out 계산 결과 행렬
@param [in ] fovy 투영 대상의 세로 화각
@param [in ] aspect aspect ratio, 세로 화각/가로 화각
@param [in ] near_ 가까운 쪽 보이는 범위
@param [in ] far_ 먼쪽 보이는 범위
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_perspective(mat4f_t out, float fovy, float aspect, float near_, float far_)
{
  float top = near_ * (real_t)tan(fovy * M_PI / 360.0),
      right = top * aspect;
  return mat4f_frustum(out, -right, right, -top, top, near_, far_);
}
/**--------------------------------------------------------------------------
@brief 4x4 크기의 정투영 행렬을 구함, 좌우, 상화, 전후 보이는 범위로부터 구함
@param [out] out 계산 결과 행렬
@param [in ] left 왼쪽 보이는 범위
@param [in ] right 오른쪽 보이는 범위
@param [in ] bottom 아랫쪽 보이는 범위
@param [in ] top 윗쪽 보이는 범위
@param [in ] near_ 가까운 쪽 보이는 범위
@param [in ] far_ 먼쪽 보이는 범위
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_ortho(mat4f_t out, float left, float right, float bottom, float top, float near_, float far_)
{
  float rl = (right - left),
        tb = (top - bottom),
        fn = (far_ - near_);
  out[0] = 2 / rl;
  out[1] = 0;
  out[2] = 0;
  out[3] = 0;
  out[4] = 0;
  out[5] = 2 / tb;
  out[6] = 0;
  out[7] = 0;
  out[8] = 0;
  out[9] = 0;
  out[10] = -2 / fn;
  out[11] = 0;
  out[12] = -(left + right) / rl;
  out[13] = -(top + bottom) / tb;
  out[14] = -(far_ + near_) / fn;
  out[15] = 1;
  return out;

}
/**--------------------------------------------------------------------------
@brief 4x4 크기의 원근 투영 행렬을 구함, 보는 위치와 대상의 센터, 위 벡터의 방향으로 구함
@param [out] out 계산 결과 행렬
@param [in ] eye 눈의 좌표
@param [in ] center 바라보는 대상의 중심
@param [in ] up 윗쪽 방향 벡터
@return mat4f_t 타입 행렬
@author ksg
*/
mat4f_t WA_API mat4f_lookAt(mat4f_t out, vec3f_t eye, vec3f_t center, vec3f_t up)
{
  float x0, x1, x2, y0, y1, y2, z0, z1, z2, len,
        eyex = eye[0],
        eyey = eye[1],
        eyez = eye[2],
        upx = up[0],
        upy = up[1],
        upz = up[2],
        centerx = center[0],
        centery = center[1],
        centerz = center[2];

  if (eyex == centerx && eyey == centery && eyez == centerz) {
    return mat4f_identity(out);
  }
  //vec3.direction(eye, center, z);
  z0 = eyex - centerx;
  z1 = eyey - centery;
  z2 = eyez - centerz;

  // normalize (no check needed for 0 because of early return)
  len = 1.0f / (float)sqrt(z0 * z0 + z1 * z1 + z2 * z2);
  z0 *= len;
  z1 *= len;
  z2 *= len;

  //vec3.normalize(vec3.cross(up, z, x));
  x0 = upy * z2 - upz * z1;
  x1 = upz * z0 - upx * z2;
  x2 = upx * z1 - upy * z0;
  len = (float)sqrt(x0 * x0 + x1 * x1 + x2 * x2);

  if (!len) {
    x0 = 0;
    x1 = 0;
    x2 = 0;
  } else {
    len = 1.0f / len;
    x0 *= len;
    x1 *= len;
    x2 *= len;
  }

  //vec3.normalize(vec3.cross(z, x, y));
  y0 = z1 * x2 - z2 * x1;
  y1 = z2 * x0 - z0 * x2;
  y2 = z0 * x1 - z1 * x0;

  len = (float)sqrt(y0 * y0 + y1 * y1 + y2 * y2);
  if (!len) {
    y0 = 0;
    y1 = 0;
    y2 = 0;
  } else {
    len = 1 / len;
    y0 *= len;
    y1 *= len;
    y2 *= len;
  }

  out[0] = x0;
  out[1] = y0;
  out[2] = z0;
  out[3] = 0;
  out[4] = x1;
  out[5] = y1;
  out[6] = z1;
  out[7] = 0;
  out[8] = x2;
  out[9] = y2;
  out[10] = z2;
  out[11] = 0;
  out[12] = -(x0 * eyex + x1 * eyey + x2 * eyez);
  out[13] = -(y0 * eyex + y1 * eyey + y2 * eyez);
  out[14] = -(z0 * eyex + z1 * eyey + z2 * eyez);
  out[15] = 1;

  return out;
}
//mat4f_t WA_API mat4f_fromRotationTranslation(quat_t quat, vec3f_t v, mat4f_t out)
//{

//}








/////////////////////////////////////////////////////////////////////////////

/**--------------------------------------------------------------------------
@brief double형 행렬 할당, 값은 0으로 초기화
@param [in ] row row(행) 수
@param [in ] col column(열) 수
@return double형 행렬 포인터
@author ksg
*/
matd* matd_new(uint16_t row, uint16_t col){
  matd* m;
  int i;

  m = (matd*)allocMemory(sizeof(matd));
  m->row = row;
  m->col = col;

  m->var = (double**)allocZeroMemory(sizeof(double*), row);
  for( i = 0 ; i < row ; i++ ){
    m->var[i] = (double*)allocZeroMemory(sizeof(double), col);
    memset(m->var[i], 0, sizeof(double)*col );
  }

  return m;
}
/**--------------------------------------------------------------------------
@brief double형 행렬 해제
@param [in,out] m 해제할 행렬
@return 없음
@author ksg
*/
void matd_free( matd* m )
{
  int i;
  for( i = 0 ; i < m->row ; i++ ){
    freeMemory(m->var[i]);
  }
  freeMemory(m->var);
  freeMemory(m);
}
/**--------------------------------------------------------------------------
@brief double형 배열을 받아서 double형 행렬 값을 채움
@param [out] m 값을 채울 행렬
@param [in ] src 행렬의 맴버값이 저장된 배열
@return 없음
@author ksg
*/
void matd_import(matd* m, double* src )
{
  int i, j;
  int width = m->col;

  for( j = 0 ; j < m->row ; j++ ) {
    for( i = 0 ; i < width ; i++ ) {
      m->var[j][i] = src[j*width+i];
    }
  }

}
/**--------------------------------------------------------------------------
@brief double형 행렬의 맴버값을 double 형 배열로 받아옴
@param [out] dst 행렬의 값을 받아갈 double 형 배열
@param [in ] m 값을 export할 행렬
@return 없음
@author ksg
*/
void matd_export(double* dst, matd* m )
{
  int i, j;
  int width = m->col;

  for( j = 0 ; j < m->row ; j++ ) {
    for( i = 0 ; i < width ; i++ ) {
      dst[j*width+i] = m->var[j][i];
    }
  }
}
/**--------------------------------------------------------------------------
@brief double형 행렬을 단위 행렬로 세팅함
@param [out] m 단위 행렬로 세팅할 대상 행렬
@return 없음
@author ksg
*/
void matd_identity( matd* m )
{
  int i, j, iter;

  if( m->col != m->row )
    return;

  iter = m->col;
  for( j = 0 ; j < iter ; j++ ) {
    for( i = 0 ; i < iter ; i++ ) {
      if( i == j ) {
        m->var[j][i] = 1.0;
      } else {
        m->var[j][i] = 0.0;
      }
    }
  }
}
/**--------------------------------------------------------------------------
@brief double형 행렬의 traspose 연산
@param [in] m transpose 연산을 할 대상 행렬
@return traspose 연산의 결과 행렬
@author ksg
*/
matd* matd_transpose( matd* m ){

  int i, j;
  matd* t;

  t = matd_new( m->col, m->row );
  for( j = 0 ; j < m->row ; j++ ){
    for( i = 0 ; i < m->col ; i++ ){
      t->var[i][j] = m->var[j][i];
    }
  }
  return t;
}
/**--------------------------------------------------------------------------
@brief double형 행렬의 곱연산
@param [in] a 곱연산 대상 행렬1
@param [in] b 곱연산 대상 행렬2
@return 곱연산의 결과 행렬
@author ksg
*/
matd* matd_multiply( matd* a, matd* b ){

  matd* m;
  int col, row, iter;
  int i, j, k;

  if( a->col != b->row )
    return NULL;

  row = a->row;
  col = b->col;

  iter = a->col;

  m = matd_new( row, col );
  for( j = 0 ; j < row ; j++ ){
    for( i = 0 ; i < col ; i++ ){
      for( k = 0 ; k < iter ; k++ ){
        m->var[j][i] += a->var[j][k]*b->var[k][i];
      }
    }
  }
  return m;

}
/**--------------------------------------------------------------------------
@brief double형 행렬의 inverse 연산
@param [in] m inverse 연산 대상 행렬1
@return inverse 연산의 결과 행렬
@author ksg
*/
matd* matd_inverse( matd* m ){

  matd *inv, *n;
  int iter, i, j, k;

  double v;

  double tmp;
  int    max_key;

  if( m->col != m->row )
    return NULL;

  iter = m->row;
  inv = matd_new( m->row, m->col );
  n = matd_new( m->row, m->col*2 );

  // copy it
  for( j = 0 ; j < iter ; j++ ) {
    for( i = 0 ; i < iter ; i++ ) {
      n->var[j][i] = m->var[j][i];
    }
  }

  // insert identity matd
  for( i = 0 ; i < iter ; i++ )
    n->var[i][i+iter] = 1.0;

  // start gauss elimination
  for( i = 0 ; i < iter ; i++ ){

    // find max
    max_key = i;
    for( j = i+1 ; j < iter ; j++ )
      if( n->var[j][i] * n->var[j][i] > n->var[max_key][i] * n->var[max_key][i])
        max_key = j;

    // swap with current row
    if( max_key != i ){
      for( j = 0 ; j < iter*2 ; j++ ){
        tmp = n->var[i][j];
        n->var[i][j] = n->var[max_key][j];
        n->var[max_key][j] = tmp;
      }
    }

    // normalize
    v = n->var[i][i];
    for( j = i+1 ; j < iter*2 ; j++ ) {
      if (v * v < 1.0E-30) {
        matd_free(inv);
        matd_free(n);
        return NULL;
      } else {
        n->var[i][j] /= v;
      }
    }

    for( j = i+1 ; j < iter ; j++ ){
      v = n->var[j][i];
      n->var[j][i] = 0.0;
      for( k = i+1 ; k < iter*2 ; k++ ){
        n->var[j][k] -= n->var[i][k]*v;
      }
    }

  }
  for( i = iter-2 ; i >= 0 ; i-- ){

    for( j = i ; j >= 0 ; j-- ){
      v = n->var[j][i+1];
      for( k = 0 ; k < iter*2 ; k++ ){
        n->var[j][k] -= n->var[i+1][k]*v;
      }
    }
  }

  // copy it
  for( j = 0 ; j < iter ; j++ )
    for( i = 0 ; i < iter ; i++ )
      inv->var[j][i] = n->var[j][i+iter];

  matd_free(n);

  return inv;
}



/*
vec3_t vec3_unproject(vec3_t vec, mat4_t view, mat4_t proj, vec4_t viewport, vec3_t dest) {

    mat4_t m;
    double *v;
    m = mat4_create(NULL);
    v = WA_Alloc(sizeof(double) * 4);

    if (!dest) { dest = vec; }

    v[0] = (vec[0] - viewport[0]) * 2.0 / viewport[2] - 1.0;
    v[1] = (vec[1] - viewport[1]) * 2.0 / viewport[3] - 1.0;
    v[2] = 2.0 * vec[2] - 1.0;
    v[3] = 1.0;

    mat4_multiply(proj, view, m);
    if(!mat4_inverse(m, NULL)) { return NULL; }

    mat4_multiplyVec4(m, v, NULL);
    if(v[3] == 0.0) { return NULL; }

    dest[0] = v[0] / v[3];
    dest[1] = v[1] / v[3];
    dest[2] = v[2] / v[3];

    return dest;
}
*/


/**--------------------------------------------------------------------------
@brief 두 쌍의 4개의 포인트 좌표를 바탕으로 원근 변환 행렬을 계산함

역행렬을 구하지 못하는 경우에는 실패한다.

@param [in ] dst  결과를 저장할 행렬 포인터
@param [in ] from 변환 전의 점의 좌표(4개)
@param [in ] to   변환 후의 점의 좌표(4개)
@return 성공하면 1, 실패하면 0

@author ksg
*/
int GetPerspectiveMatrixFrom4Points(mat4f_t dst, const vec2 *from, const vec2 *to)
{
  matd *pR= matd_new(8,8), *pRi, *pL, *pC;
  double **v = pR->var;

  v[0][0] = from[0].x; v[0][1] = from[0].y; v[0][2] = 1; v[0][6] = -from[0].x * to[0].x; v[0][7] = -to[0].x * from[0].y;
  v[1][3] = from[0].x; v[1][4] = from[0].y; v[1][5] = 1; v[1][6] = -from[0].x * to[0].y; v[1][7] = -to[0].y * from[0].y;
  v[2][0] = from[1].x; v[2][1] = from[1].y; v[2][2] = 1; v[2][6] = -from[1].x * to[1].x; v[2][7] = -to[1].x * from[1].y;
  v[3][3] = from[1].x; v[3][4] = from[1].y; v[3][5] = 1; v[3][6] = -from[1].x * to[1].y; v[3][7] = -to[1].y * from[1].y;
  v[4][0] = from[2].x; v[4][1] = from[2].y; v[4][2] = 1; v[4][6] = -from[2].x * to[2].x; v[4][7] = -to[2].x * from[2].y;
  v[5][3] = from[2].x; v[5][4] = from[2].y; v[5][5] = 1; v[5][6] = -from[2].x * to[2].y; v[5][7] = -to[2].y * from[2].y;
  v[6][0] = from[3].x; v[6][1] = from[3].y; v[6][2] = 1; v[6][6] = -from[3].x * to[3].x; v[6][7] = -to[3].x * from[3].y;
  v[7][3] = from[3].x; v[7][4] = from[3].y; v[7][5] = 1; v[7][6] = -from[3].x * to[3].y; v[7][7] = -to[3].y * from[3].y;

  pRi = matd_inverse(pR);

  if (pRi == NULL) {
    Err_("Cannot calculate inverse matrix while building perspective matrix");
    matd_free(pR);
    return 0;
  }

  pL = matd_new(8,1);
  v = pL->var;
  v[0][0] = to[0].x; v[1][0] = to[0].y;
  v[2][0] = to[1].x; v[3][0] = to[1].y;
  v[4][0] = to[2].x; v[5][0] = to[2].y;
  v[6][0] = to[3].x; v[7][0] = to[3].y;

  pC = matd_multiply(pRi, pL);
  dst[0] = (float)pC->var[0][0];
  dst[1] = (float)pC->var[3][0];
  dst[2] = (float)0;
  dst[3] = (float)pC->var[6][0];

  dst[4] = (float)pC->var[1][0];
  dst[5] = (float)pC->var[4][0];
  dst[6] = (float)0;
  dst[7] = (float)pC->var[7][0];

  dst[8] = (float)0;
  dst[9] = (float)0;
  dst[10] = (float)1;
  dst[11] = (float)0;

  dst[12] = (float)pC->var[2][0];
  dst[13] = (float)pC->var[5][0];
  dst[14] = (float)0;
  dst[15] = (float)1;

  matd_free(pC);
  matd_free(pL);
  matd_free(pRi);
  matd_free(pR);

  return 1;
}

