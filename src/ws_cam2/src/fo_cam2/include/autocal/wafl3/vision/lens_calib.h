//---------------------------------------------------------------------------

#ifndef lens_calibH
#define lens_calibH
//---------------------------------------------------------------------------
#include "system/string_base.h"
#include "gl/gl_matrix.h"

#define TLENSCALIB_TAG_HEADER       0x0c ///< 뒤에 4바이트 fourcc
#define TLENSCALIB_TAG_TYPE         0x01
#define TLENSCALIB_TAG_SRC_SIZE     0x02
#define TLENSCALIB_TAG_DATA_SIZE    0x03
#define TLENSCALIB_TAG_DATA         0x04
#define TLENSCALIB_TAG_SENSOR_SIZE  0x05
#define TLENSCALIB_TAG_CALPNL_SIZE  0x06
#define TLENSCALIB_TAG_CALPNL_DIST  0x07
#define TLENSCALIB_TAG_BITFIELDS    0x08
#define TLENSCALIB_TAG_CAMMATRIX    0x09
#define TLENSCALIB_TAG_DISTORTION   0x0A
#define TLENSCALIB_TAG_DISTORTMAT   0x0B
#define TLENSCALIB_TAG_INVMAT       0x0D
#define TLENSCALIB_TAG_DISTORTION8   0x0E

/// 렌즈 캘리브레이션 원본 데이터(바이너리 형식)을 관리하기 위한 클래스
/// 렌즈 캘리브레이션은 기본적으로 영상 좌표계를 사용한다.
struct TLensCalib {
//private:
public:
	fourcc_t type;
	void *data;         ///< 캘리브레이션 데이터
	size2u16 src_size;  ///< 렌즈 캘리브레이션을 수행할 때 사용한 영상 크기
	size2u16 data_size; ///< 렌즈 캘리브레이션 결과 영상의 크기
	uint32_t bit_fields; ///< 데이터가 수평 방향, 수직 방향, 대각선 방향 중 어느 방향 기준인지
	size2u16 sensor_size; ///< 상이 맺히는 센서 크기
	vec2f    calpnl_size; ///< 데이터 샘플링할 패널 크기
	float    calpnl_dist; ///< 데이터 샘플링할 패널까지의 거리
	mat3d_t K;
	//vec4d_t D;
	vec8d_t D;
	mat3d_t K_new;
	mat3d_t giR;

	int lenstype;

	size_t LoadV1(FILE *fp);
	size_t LoadV2(FILE *fp);
	size_t LoadV1_Stream(DVEC buf);
	size_t LoadV2_Stream(DVEC buf);
  
	struct {
		uint32_t is_normalized : 1;
		uint32_t is_modified : 1;
	};

	real_t ax, bx, ay, by;

	real_t scale_x;
	real_t scale_y;
	real_t p1, p2; ///< tangential 에러 보정을 위한 파라미터

	size2i16 input_image_size;
	rect2    input_image_region;  ///< 0~1 사이의 Normalized 된 범위로 영.상.좌표계를 기반으로 한다.

	int SampleBezierToArray(vec2 *dst, vec2u16 segments); // Bezier곡면 방식
	int SamplePoly1ToArray(vec2 *dst, vec2u16 segments); // 렌즈 프로파일 방식
	int SampleLUTToArray(vec2 *dst, vec2u16 segments); // 김태호 책임 LUT 방식
	int SampleFisheyeToArray(vec2 *dst, vec2u16 segments); //OpenCV fisheye 모델
	int SampleWideToArray(vec2 *dst, vec2u16 segments); //OpenCV fisheye 모델

	int DistortBezierPoints(vec2 *dst, vec2 const *src, int count);
	int UndistortBezierPoints(vec2 *dst, vec2 const *src, int count);
	int DistortPoly1Points(vec2 *dst, vec2 const *src, int count);
	int UndistortPoly1Points(vec2 *dst, vec2 const *src, int count);

	int DistortFisheyePoints(vec2 *dst, vec2 const *src, int count);
	int UndistortFisheyePoints(vec2 *dst, vec2 const *src, int count);

	//int DistortFisheyePoints(vec2 *dst, vec2 const *src, int count);
	int tangencialDistort(float32_t x_in, float32_t y_in, float32_t *x_out, float32_t *y_out);
	int tangencialUndistort(float32_t x_in, float32_t y_in, float32_t *x_out, float32_t *y_out);
  int DistortWAPoints(
	  vec2 *dst,          ///< 렌즈 왜곡된 점의 좌표
	  vec2 const *src,    ///< 렌즈 왜곡 보정된 점의 좌표
	  int count           ///< 좌표 변환할 점의 수
	);

//public:
	TLensCalib();
	~TLensCalib();
	fourcc_t id;        ///< ID
	UString file_name;  ///< 바이너리 파일명
	DVEC dvecStream;


//2022.02.18, kjkim to use 2D calib
	int ProjectVectors(vec2 *dst, vec3 const *src, int count, mat4f_t cam_pos, mat4f_t src_pos = NULL);
	int ProjectFeyeVectors(vec2 *dst, vec3 const *src, int count, mat4f_t cam_pos, mat4f_t src_pos = NULL);

	int ProjectPoly1Vectors(vec2 *dst, vec3 const *src, int count, mat4f_t cam_pos, mat4f_t src_pos = NULL);
	int ProjectOnePoint(vec2 *dst, vec3 const src, mat4f_t cam_pos, mat4f_t src_pos = NULL);

	void Init(void);
	void Free(void);

	/// DistortPoints와 동일한 역할 수행, 단, 영상 좌표를 0~1 사이로 Normalize한다.
	int SampleToArray(vec2 *dst, vec2u16 segments);

	/// src로 제공되는 u,v 좌표값을 영상 좌표 (x,y)로 변환한다. x,y도 0~1 범위로 Normalize된다.
	int DistortPoints(vec2 *dst, vec2 const *src, int count);

	/// src로 제공되는 x,y 좌표(영상좌표, Normalize된)를 보정 좌표 (u,v)로 변환한다. u,v는 0~1 사이로 Normalize된다.
	int UndistortPoints(vec2 *dst, vec2 const *src, int count);

	size_t Load(void);
  int32_t getLensType();

	void ChangeScale(size2i16 new_src_size);

	int setParameter(real_t v1, real_t v2);
	void getParameter(real_t *v1, real_t *v2);

	inline const void *GetData(void) {return Load() ? data : NULL;}
//  inline size2u16 GetSourceSize(void) {Load(); return src_size;}
	inline size2u16 GetDataSize(void) {Load(); return data_size;}
	size2i16       &InputImageSize(void) {is_modified = 1; return input_image_size;}
	size2i16 const &InputImageSize(void) const {return input_image_size;}

	// 영상 좌하단을 원점으로 한 영역
	rect2          &InputImageRegion(void) {is_modified = 1; return input_image_region;}
	rect2 const    &InputImageRegion(void) const {return input_image_region;}
};
#endif
