/**--------------------------------------------------------------------------
@file gl_matrix.h
@brief gl_matrix.c 파일의 헤더
@author ksg
*/
//---------------------------------------------------------------------------

#ifndef gl_matrixH
#define gl_matrixH
//---------------------------------------------------------------------------
#define _USE_MATH_DEFINES  // VC에서는 M_PI를 사용하기 위해 선언해야 함

#include <math.h>
#include <float.h>
#include "system/mem_dynamic.h"

#if __cplusplus
#define WA_OPTIONAL(name, default) name = default
extern "C" {
#else //#if __cplusplus
#define WA_OPTIONAL(name, default) name
#endif //#else __cplusplus

typedef float  *vec2f_t;  ///< float형 2차원 배열 타입
typedef float  *vec3f_t;  ///< float형 3차원 배열 타입
typedef float  *vec4f_t;  ///< float형 4차원 배열 타입

typedef double *vec2d_t;  ///< double형 2차원 배열 타입
typedef double *vec3d_t;  ///< double형 3차원 배열 타입
typedef double *vec4d_t;  ///< double형 4차원 배열 타입
typedef double *vec8d_t;  ///< double형 8차원 행렬 타입

typedef float  *mat3f_t;  ///< float형 3차원 행렬 타입
typedef double *mat3d_t;  ///< double형 3차원 행렬 타입

typedef float  *mat4f_t;  ///< float형 4차원 행렬 타입
typedef double *mat4d_t;  ///< double형 4차원 행렬 타입

#define float_is_equal(value, ref) ((value) >= (ref - FLT_EPSILON) && (value) < (ref + FLT_EPSILON))
#define float_is_zero(value) ((value) >= -FLT_EPSILON && (value) < FLT_EPSILON)
#define float_is_one(value) ((value) >= (1.0f - FLT_EPSILON) && (value) < (1.0f + FLT_EPSILON))

#define double_is_equal(value, ref) ((value) >= (ref - DBL_EPSILON) && (value) < (ref + DBL_EPSILON))
#define double_is_zero(value) ((value) >= -DBL_EPSILON && (value) < DBL_EPSILON)
#define double_is_one(value) ((value) >= (1.0 - DBL_EPSILON) && (value) < (1.0 + DBL_EPSILON))

extern vec2f_t WA_API vec2f_init               (void);
extern vec2f_t WA_API vec2f_initWithScalar     (float scalar);
extern vec2f_t WA_API vec2f_initWithComponents (float s1, float s2);
extern vec2f_t WA_API vec2f_initWithVec2f      (const vec2f_t src);
extern vec2f_t WA_API vec2f_set                (vec2f_t out, const vec2f_t v);
extern vec2f_t WA_API vec2f_setScalar          (vec2f_t out, float v);
extern vec2f_t WA_API vec2f_setComponents      (vec2f_t out, float s1, float s2);
extern vec2f_t WA_API vec2f_add                (vec2f_t v1, const vec2f_t v2, WA_OPTIONAL(vec2f_t out, NULL));
extern vec2f_t WA_API vec2f_addScalar          (vec2f_t v, float scalar, WA_OPTIONAL(vec2f_t out, NULL));
extern vec2f_t WA_API vec2f_subtract           (vec2f_t v1, const vec2f_t v2, WA_OPTIONAL(vec2f_t out, NULL));
static inline     vec2f_t        vec2f_subtractScalar     (vec2f_t v, float scalar, WA_OPTIONAL(vec2f_t out, NULL)) {return vec2f_addScalar(v, -scalar, out);}
extern vec2f_t WA_API vec2f_multiply           (vec2f_t v1, const vec2f_t v2, WA_OPTIONAL(vec2f_t out, NULL));
extern vec2f_t WA_API vec2f_multiplyScalar     (vec2f_t v, float scalar, WA_OPTIONAL(vec2f_t out, NULL));
extern vec2f_t WA_API vec2f_divide             (vec2f_t v1, const vec2f_t v2, WA_OPTIONAL(vec2f_t out, NULL));
static inline     vec2f_t        vec2f_divideScalar       (vec2f_t v, float scalar, WA_OPTIONAL(vec2f_t out, NULL)) {return vec2f_multiplyScalar(v, 1.0f / scalar, out);}
static inline     vec2f_t        vec2f_scale              (vec2f_t v, float scale, WA_OPTIONAL(vec2f_t out, NULL)) {return vec2f_multiplyScalar(v, scale, out);}
extern vec2f_t WA_API vec2f_normalize          (vec2f_t v, WA_OPTIONAL(vec2f_t out, NULL));
extern real_t   WA_API vec2f_squaredLength      (const vec2f_t v);
static inline     real_t          vec2f_length             (const vec2f_t v) {return (real_t)sqrt(vec2f_squaredLength(v));}
extern real_t   WA_API vec2f_dot                (const vec2f_t v1, const vec2f_t v2);
extern vec2f_t WA_API vec2f_direction          (vec2f_t v1, const vec2f_t v2, WA_OPTIONAL(vec2f_t out, NULL));
extern vec2f_t WA_API vec2f_lerp               (vec2f_t v1, const vec2f_t v2, float lerp, WA_OPTIONAL(vec2f_t out, NULL));
extern real_t   WA_API vec2f_squaredDistance    (const vec2f_t v1, const vec2f_t v2);
static inline     real_t          vec2f_distance           (const vec2f_t v1, const vec2f_t v2) {return (real_t)sqrt(vec2f_squaredDistance(v1, v2));}
extern vec2f_t WA_API vec2f_negate             (vec2f_t v, WA_OPTIONAL(vec2f_t out, NULL));
extern vec2f_t WA_API vec2f_inverse            (vec2f_t v, WA_OPTIONAL(vec2f_t out, NULL));


extern vec2f_t WA_API vec2f_transformMat4f     (vec2f_t v, const mat4f_t mat, WA_OPTIONAL(vec2f_t out, NULL));

extern vec3f_t WA_API vec3f_init               (void);
extern vec3f_t WA_API vec3f_initWithScalar     (float scalar);
extern vec3f_t WA_API vec3f_initWithComponents (float s1, float s2, float s3);
extern vec3f_t WA_API vec3f_initWithVec3f      (const vec3f_t src);
extern vec3f_t WA_API vec3f_set                (vec3f_t out, const vec3f_t v);
extern vec3f_t WA_API vec3f_setScalar          (vec3f_t out, float v);
extern vec3f_t WA_API vec3f_setComponents      (vec3f_t out, float s1, float s2, float s3);
extern vec3f_t WA_API vec3f_add                (vec3f_t v1, const vec3f_t v2, WA_OPTIONAL(vec3f_t out, NULL));
extern vec3f_t WA_API vec3f_addScalar          (vec3f_t v, float scalar, WA_OPTIONAL(vec3f_t out, NULL));
extern vec3f_t WA_API vec3f_subtract           (vec3f_t v1, const vec3f_t v2, WA_OPTIONAL(vec3f_t out, NULL));
static inline     vec3f_t        vec3f_subtractScalar     (vec3f_t v, float scalar, WA_OPTIONAL(vec3f_t out, NULL)) {return vec3f_addScalar(v, -scalar, out);}
extern vec3f_t WA_API vec3f_multiply           (vec3f_t v1, const vec3f_t v2, WA_OPTIONAL(vec3f_t out, NULL));
extern vec3f_t WA_API vec3f_multiplyScalar     (vec3f_t v, float scalar, WA_OPTIONAL(vec3f_t out, NULL));
extern vec3f_t WA_API vec3f_divide             (vec3f_t v1, const vec3f_t v2, WA_OPTIONAL(vec3f_t out, NULL));
static inline     vec3f_t        vec3f_divideScalar       (vec3f_t v, float scalar, WA_OPTIONAL(vec3f_t out, NULL)) {return vec3f_multiplyScalar(v, 1.0f / scalar, out);}
static inline     vec3f_t        vec3f_scale              (vec3f_t v, float scale, WA_OPTIONAL(vec3f_t out, NULL)) {return vec3f_multiplyScalar(v, scale, out);}
extern vec3f_t WA_API vec3f_normalize          (vec3f_t v, WA_OPTIONAL(vec3f_t out, NULL));
extern vec3f_t WA_API vec3f_cross              (vec3f_t v1, const vec3f_t v2, WA_OPTIONAL(vec3f_t out, NULL));
extern real_t  WA_API vec3f_squaredLength      (const vec3f_t v);
static inline     real_t         vec3f_length             (const vec3f_t v) {return (real_t)sqrt(vec3f_squaredLength(v));}
extern real_t  WA_API vec3f_dot                (const vec3f_t v1, const vec3f_t v2);
extern vec3f_t WA_API vec3f_direction          (vec3f_t v1, const vec3f_t v2, WA_OPTIONAL(vec3f_t out, NULL));
extern vec3f_t WA_API vec3f_lerp               (vec3f_t v1, const vec3f_t v2, float lerp, WA_OPTIONAL(vec3f_t out, NULL));
extern real_t  WA_API vec3f_squaredDistance    (const vec3f_t v1, const vec3f_t v2);
static inline     real_t         vec3f_distance           (const vec3f_t v1, const vec3f_t v2) {return (real_t)sqrt(vec3f_squaredDistance(v1, v2));}
extern vec3f_t WA_API vec3f_negate             (vec3f_t v, WA_OPTIONAL(vec3f_t out, NULL));
extern vec3f_t WA_API vec3f_inverse            (vec3f_t v, WA_OPTIONAL(vec3f_t out, NULL));


extern vec3f_t WA_API vec3f_transformMat4f     (vec3f_t v, const mat4f_t mat, WA_OPTIONAL(vec3f_t out, NULL));

extern mat3d_t WA_API mat3d_init(void);
extern vec4d_t WA_API vec4d_init(void);
extern vec8d_t WA_API vec8d_init(void);


/*
#define vec3f_sub       vec3f_subtract
#define vec3f_subS      vec3f_subtractScalar
#define vec3f_mlt       vec3f_multiply
#define vec3f_mltS      vec3f_multiplyScalar
#define vec3f_div       vec3f_divide
#define vec3f_divS      vec3f_divideScalar

#define vec3f_len       vec3f_length
#define vec3f_sqrLen    vec3f_squaredLength
#define vec3f_dist      vec3f_distance
#define vec3f_sqrDist   vec3f_squaredDistance
#define vec3f_neg       vec3f_negate
#define vec3f_inv       vec3f_inverse
*/
extern vec4f_t WA_API vec4f_transformMat4f(vec4f_t v, const mat4f_t mat, WA_OPTIONAL(vec4f_t out, NULL));

//mat4f_t mat3_toMat4(mat3_t mat, mat4f_t out);
extern mat4f_t WA_API mat4f_init(void);
extern mat4f_t WA_API mat4f_initWithMat4f(const mat4f_t mat);
extern mat4f_t WA_API mat4f_set(mat4f_t out, const mat4f_t mat);
extern mat4f_t WA_API mat4f_identity(mat4f_t out);
extern mat4f_t WA_API mat4f_transpose(mat4f_t mat, WA_OPTIONAL(mat4f_t out, NULL));
extern float   WA_API mat4f_determinant(const mat4f_t mat);
extern mat4f_t WA_API mat4f_inverse(mat4f_t mat, WA_OPTIONAL(mat4f_t out, NULL));
//extern mat4f_t WA_API mat4f_toRotationMat(mat4f_t mat, mat4f_t out);
//mat3_t mat4f_toMat3(mat4f_t mat, mat3_t out);
//mat3_t mat4f_toInverseMat3(mat4f_t mat, mat3_t out);
extern mat4f_t WA_API mat4f_multiply(mat4f_t m1, const mat4f_t m2, WA_OPTIONAL(mat4f_t out, NULL));
//extern mat4f_t WA_API mat4_multiplyVec4(mat4f_t mat, vec4_t v, mat4f_t out);
extern mat4f_t WA_API mat4f_translateByComponents(mat4f_t mat, float x, float y, float z, WA_OPTIONAL(mat4f_t out, NULL));
static inline     mat4f_t        mat4f_translate(mat4f_t mat, const vec3f_t v, WA_OPTIONAL(mat4f_t out, NULL)) {return mat4f_translateByComponents(mat, v[0], v[1], v[2], out);}
extern mat4f_t WA_API mat4f_scaleByComponents(mat4f_t mat, float x, float y, float z, WA_OPTIONAL(mat4f_t out, NULL));
static inline     mat4f_t        mat4f_scale(mat4f_t mat, const vec3f_t v, WA_OPTIONAL(mat4f_t out, NULL)) {return mat4f_scaleByComponents(mat, v[0], v[1], v[2], out);}
extern mat4f_t WA_API mat4f_rotateByComponents(mat4f_t mat, float angle, float axis_x, float axis_y, float axis_z, WA_OPTIONAL(mat4f_t out, NULL));
static inline     mat4f_t        mat4f_rotate(mat4f_t mat, float angle, const vec3f_t axis, WA_OPTIONAL(mat4f_t out, NULL)) {return mat4f_rotateByComponents(mat, angle, axis[0], axis[1], axis[2], out);}
extern mat4f_t WA_API mat4f_rotateX(mat4f_t mat, float angle, WA_OPTIONAL(mat4f_t out, NULL));
extern mat4f_t WA_API mat4f_rotateY(mat4f_t mat, float angle, WA_OPTIONAL(mat4f_t out, NULL));
extern mat4f_t WA_API mat4f_rotateZ(mat4f_t mat, float angle, WA_OPTIONAL(mat4f_t out, NULL));

extern mat4f_t WA_API mat4f_frustum(mat4f_t out, float left, float right, float bottom, float top, float near_, float far_);
extern mat4f_t WA_API mat4f_perspective(mat4f_t out, float fovy, float aspect, float near_, float far_);
extern mat4f_t WA_API mat4f_ortho(mat4f_t out, float left, float right, float bottom, float top, float near_, float far_);
extern mat4f_t WA_API mat4f_lookAt(mat4f_t out, vec3f_t eye, vec3f_t center, vec3f_t up);
//extern mat4f_t WA_API mat4_fromRotationTranslation(quat_t quat, vec3f_t v, mat4f_t out);

#define mat4f_mlt         mat4f_multiply

#if (REAL_SYSTEM == REAL_SYSTEM_FLOAT)

  #define vec3_t                    vec3f_t
  #define mat4_t                    mat4f_t

/*
  #define vec3_init                 vec3f_init
  #define vec3_initWithScalar       vec3f_initWithScalar
  #define vec3_initWithComponents   vec3f_initWithComponents
  #define vec3_initWithVec3f        vec3f_initWithVec3f
  #define vec3_set                  vec3f_set
  #define vec3_setScalar            vec3f_setScalar
  #define vec3_setComponents        vec3f_setComponents
  #define vec3_add                  vec3f_add
  #define vec3_addScalar            vec3f_addScalar
  #define vec3_subtract             vec3f_subtract
  #define vec3_subtractScalar       vec3f_subtractScalar
  #define vec3_multiply             vec3f_multiply
  #define vec3_multiplyScalar       vec3f_multiplyScalar
  #define vec3_divide               vec3f_divide
  #define vec3_divideScalar         vec3f_divideScalar
  #define vec3_scale                vec3f_scale
  #define vec3_normalize            vec3f_normalize
  #define vec3_cross                vec3f_cross
  #define vec3_squaredLength        vec3f_squaredLength
  #define vec3_length               vec3f_length
  #define vec3_dot                  vec3f_dot
  #define vec3_direction            vec3f_direction
  #define vec3_lerp                 vec3f_lerp
  #define vec3_squaredDistance      vec3f_squaredDistance
  #define vec3_distance             vec3f_distance
  #define vec3_negate               vec3f_negate
  #define vec3_inverse              vec3f_inverse
  #define vec3_transformMat4        vec3f_transformMat4f
*/


#elif (REAL_SYSTEM == REAL_SYSTEM_DOUBLE)

#endif

#define VFMT_VEC3 "%f,%f,%f"
#define VARG_VEC3(x) x[0], x[1], x[2]

/**--------------------------------------------------------------------------
@brief double형 m by n 크기의 행렬 타입 구조체
*/
typedef struct {
  uint16_t col;  ///< column(열)의 수
  uint16_t row;  ///< row(행)의 수
  double** var;  ///< 데이터 포인터
} matd;

matd* matd_new(uint16_t row, uint16_t col );
void matd_free(matd* m);
matd* matd_multiply( matd* a, matd* b);
matd* matd_inverse(matd* m);
matd* matd_transpose(matd* m);
void matd_import(matd* m, double* src);
void matd_export(double* dst, matd* m);
void matd_identity(matd* m);

#ifndef to_deg_
	#define to_deg_(rad) ((rad) * 180.0 / M_PI)
#endif //#ifndef to_deg_
#ifndef to_rad_
	#define to_rad_(deg) ((deg) * M_PI / 180.0)
#endif //#ifndef to_rad_


extern int GetPerspectiveMatrixFrom4Points(mat4f_t dst, const vec2 *from, const vec2 *to);

#if __cplusplus
}
#endif //#if __cplusplus

#endif

