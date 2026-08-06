//---------------------------------------------------------------------------
#define _USE_MATH_DEFINES  // VC에서는 M_PI를 사용하기 위해 선언해야 함
#include <math.h>
#include <stdlib.h>

#if (defined(CCS) || defined(__BORLANDC__))
#pragma hdrstop
#endif //(defined(CCS) || defined(__BORLANDC__))

#include "lens_calib.h"
#include "geo/bezier.h"
#include "geo/geo_linear.h"
#include "system/sys_logger.h"
#include "system/cstr_util.h"
#include "system/file_serialize_tag.h"
#include "system/dvec_serialize_tag.h"
#include "system/sys_core.h"
//---------------------------------------------------------------------------
#if defined(__BORLANDC__)
#pragma package(smart_init)
#endif //defined(__BORLANDC__)

/**--------------------------------------------------------------------------
@brief 기본 생성자

Init() 메소드를 호출한다.

@author ksg
*/
/////////////////////////////////////////////////////////////////////////////
TLensCalib::TLensCalib()
/////////////////////////////////////////////////////////////////////////////
{
	Init();
}
/**--------------------------------------------------------------------------
@brief 소멸자

Free() 메소드를 호출한다.

@author ksg
*/
TLensCalib::~TLensCalib()
{
	Free();
}
/**--------------------------------------------------------------------------
@brief 구조체를 초기화 한다.

내부를 모두 0로 채움

@author ksg
*/
void TLensCalib::Init(void)
{
	memset(this, 0, sizeof(TLensCalib));
	sensor_size = make_size2u16(1280,800);
	calpnl_size = make_vec2f(36*15, 12*15);
	calpnl_dist = 40;
	bit_fields = 1;
	scale_x = 1.0 / 1280.0;
	scale_y = 1.0 / 800.0;
	input_image_region = make_rect2(0.0, 0.0, 1.0, 1.0); // 영상 좌표계이므로 top이 0, bottom이 1이 된다.

	//K = mat3d_init();
	//D = vec4d_init();
	//K_new = mat3d_init();

	dvecStream = 0;

}
/**--------------------------------------------------------------------------
@brief 구조체에 할당된 메모리를 반환한다.

UString 타입인 file_name에 널문자를 대입하여 메모리를 반환하고 data 멤버를
삭제함

@author ksg
*/
void TLensCalib::Free(void)
{
	file_name = "";
	DVEC_FREE(&data);
}

size_t TLensCalib::LoadV1(FILE *fp)
{
	size_t nRead = 0, nTotalRead = 0, nCount = 0;

	nTotalRead += nRead = fread(&data_size, 1, sizeof(data_size), fp);
	if (nRead != sizeof(data_size)) {
		Err_("Cannot read data size");
		return 0;
	}
	nTotalRead += nRead = fread(&src_size, 1, sizeof(src_size), fp);
	if (nRead != sizeof(src_size)) {
		Err_("Cannot read image source size");
		return 0;
	}
	if (data_size.width == 0 || data_size.height == 0) {
		Err_("Invalid data size [%d,%d]", data_size.width, data_size.height);
		return 0;
	}
	nCount = (int)data_size.width * (int)data_size.height;
	DVEC_SETLENGTH(vec2, &data, nCount);

	nCount *= sizeof(vec2);
	if (data == NULL) {
		Err_("Cannot allocate memory (%d bytes) for Lens Calibration data", nCount);
	}
	nTotalRead += nRead = fread(data, 1, nCount, fp);
	if (nRead != nCount) {
		Err_("Cannot read data, only %d bytes of %d read.", nRead, nCount);
	}
	return nTotalRead;
}

size_t TLensCalib::LoadV2(FILE *fp)
{
	fourcc_t hdr = 0;
	uint8_t tag = 0;
	uint32_t size = 0;
	 double giR2[9];

	for (tag = fread_token(fp, &size); tag != 0; tag = fread_token(fp, &size)) {
		switch (tag) {
			READ_SIMPLE_TAG(TLENSCALIB_TAG_TYPE, type)
			READ_SIMPLE_TAG(TLENSCALIB_TAG_SRC_SIZE, src_size)
			READ_SIMPLE_TAG(TLENSCALIB_TAG_DATA_SIZE, data_size)
			READ_SIMPLE_TAG(TLENSCALIB_TAG_SENSOR_SIZE, sensor_size)
			READ_SIMPLE_TAG(TLENSCALIB_TAG_CALPNL_SIZE, calpnl_size)
			READ_SIMPLE_TAG(TLENSCALIB_TAG_CALPNL_DIST, calpnl_dist)
			READ_SIMPLE_TAG(TLENSCALIB_TAG_BITFIELDS, bit_fields)
		READ_MATRIX_TAG_D9(TLENSCALIB_TAG_CAMMATRIX, K)
		READ_ARRAY_TAG_D4(TLENSCALIB_TAG_DISTORTION, D)
		READ_ARRAY_TAG_D8(TLENSCALIB_TAG_DISTORTION8, D)
		READ_MATRIX_TAG_D9(TLENSCALIB_TAG_DISTORTMAT, K_new)
		READ_MATRIX_TAG_D9(TLENSCALIB_TAG_INVMAT, giR)

			case TLENSCALIB_TAG_HEADER:
				if (size != sizeof(hdr) || fread(&hdr, 1, size, fp) != size) {
					Err_(gsErrIOSizeMismatch);
					return -1;
				}
				if (hdr != make_fourcc('-','c','a','l')) {
					return -1;
				}
				break;
			case TLENSCALIB_TAG_DATA:
				if (fread_DVEC(&data, fp, size) != size) {
					Err_("IO error - cannot read tex_coord");
					return -1;
				}
				break;

	
	//  case TLENSCALIB_TAG_CAMMATRIX:
	//	if (fread_DVEC(&K, fp, size) != size) {
 //         Err_("Fisheye model K read error");
 //         return -1;
 //       }
 //       break;

	//  case TLENSCALIB_TAG_DISTORTION:
	//	if (fread_DVEC(&D, fp, size) != size) {
 //         Err_("Fisheye model D read error");
 //         return -1;
 //       }
 //       break;

	//case TLENSCALIB_TAG_DISTORTMAT:
	//	if (fread_DVEC(&K_new, fp, size) != size) {
 //         Err_("Fisheye model K_new read error");
 //         return -1;
 //       }
 //       break;



			default:
				break;
		}
	}

	// Bezier 곡면의 좌표계를 일단 0~1 범위로 Normalize하고 영상 좌표계(좌상단이 원점)로 바꾼다.
	switch (type) {
		case make_fourcc('b','z','r','B'): {
			BEZIER_INFO bi;
			BEZIER2_Info(&bi, data);
			vec2 vScale = make_vec2(1.0f / (float)(src_size.width), 1.0f / (float)(src_size.height));
			switch (bi.cp_dim) {
				case 2: {
					vec2 *pCP = (vec2*)data;
					for (int i = 0; i < bi.cp_cnt; i++) {
						pCP[i].x *= vScale.x;
						pCP[i].y = 1.0 - pCP[i].y * vScale.y;
					}
					break;
				}
				case 3: {
					vec3 *pCP = (vec3*)data;
					for (int i = 0; i < bi.cp_cnt; i++) {
						pCP[i].x *= vScale.x;
						pCP[i].y = 1.0 - pCP[i].y * vScale.y;
					}
					break;
				}
				default: {
					Err_("Cannot support %d dimension control points", bi.cp_dim);
					return 0;
				}
			}
			break;
		}
		case make_fourcc('p','l','y','1'):
			//return SamplePoly1ToArray(dst, segments);
			break;

	case make_fourcc('f','e','y','e'):
		DVEC_SetLength(&data, 1, 1); //data를 NULL로 놓아두면 LoadV1이 호출됨
		lenstype = 1; //LENSTYPE_FISHEYE
		break;

	case make_fourcc('w','i','d','e'):
		DVEC_SetLength(&data, 1, 1); //data를 NULL로 놓아두면 LoadV1이 호출됨
		lenstype = 0; //LENSTYPE_NORMAL
		break;
		default:
			break;
	}

	return 1;
}

int32_t TLensCalib::getLensType()
{
	return lenstype;
}

size_t TLensCalib::LoadV1_Stream(DVEC buf)
{
	size_t nRead = 0, nTotalRead = 0, nCount = 0;
	FILE *fp;  //// todo: DVEC으로 읽기 구현해야 하는데 LoadV!_Stream은 우선 호출 안하므로 나중에 구현하자, LJP 2017-10-17

	nTotalRead += nRead = fread(&data_size, 1, sizeof(data_size), fp);
	if (nRead != sizeof(data_size)) {
		Err_("Cannot read data size");
		return 0;
	}
	nTotalRead += nRead = fread(&src_size, 1, sizeof(src_size), fp);
	if (nRead != sizeof(src_size)) {
		Err_("Cannot read image source size");
		return 0;
	}
	if (data_size.width == 0 || data_size.height == 0) {
		Err_("Invalid data size [%d,%d]", data_size.width, data_size.height);
		return 0;
	}
	nCount = (int)data_size.width * (int)data_size.height;
	DVEC_SETLENGTH(vec2, &data, nCount);

	nCount *= sizeof(vec2);
	if (data == NULL) {
		Err_("Cannot allocate memory (%d bytes) for Lens Calibration data", nCount);
	}
	nTotalRead += nRead = fread(data, 1, nCount, fp);
	if (nRead != nCount) {
		Err_("Cannot read data, only %d bytes of %d read.", nRead, nCount);
	}
	return nTotalRead;
}

/**
	@brief DVEC buffer로부터 calib 파일 읽기
*/
size_t TLensCalib::LoadV2_Stream(DVEC buf)
{
	fourcc_t hdr = 0;
	uint8_t tag = 0;
	uint32_t size = 0;

	WADVECStream *ds = new WADVECStream();

	int size_buf = DVEC_Length(buf);

	ds->Write(buf, size_buf);
	ds->SetPosition(0);
 

	for (tag = fread_token_STREAM(ds, &size); tag != 0; tag = fread_token_STREAM(ds, &size)) {
  
		switch (tag) {
			READ_SIMPLE_TAG_STREAM(TLENSCALIB_TAG_TYPE, type)
			READ_SIMPLE_TAG_STREAM(TLENSCALIB_TAG_SRC_SIZE, src_size)
			READ_SIMPLE_TAG_STREAM(TLENSCALIB_TAG_DATA_SIZE, data_size)
			READ_SIMPLE_TAG_STREAM(TLENSCALIB_TAG_SENSOR_SIZE, sensor_size)
			READ_SIMPLE_TAG_STREAM(TLENSCALIB_TAG_CALPNL_SIZE, calpnl_size)
			READ_SIMPLE_TAG_STREAM(TLENSCALIB_TAG_CALPNL_DIST, calpnl_dist)
			READ_SIMPLE_TAG_STREAM(TLENSCALIB_TAG_BITFIELDS, bit_fields)
		READ_MATRIX_TAG_D9_STREAM(TLENSCALIB_TAG_CAMMATRIX, K)
		READ_ARRAY_TAG_D4_STREAM(TLENSCALIB_TAG_DISTORTION, D)
		READ_ARRAY_TAG_D8_STREAM(TLENSCALIB_TAG_DISTORTION8, D)
		READ_MATRIX_TAG_D9_STREAM(TLENSCALIB_TAG_DISTORTMAT, K_new)
		READ_MATRIX_TAG_D9_STREAM(TLENSCALIB_TAG_INVMAT, giR)

			case TLENSCALIB_TAG_HEADER:
				if (size != sizeof(hdr) || ds->Read(&hdr, size) != size) {
					Err_(gsErrIOSizeMismatch);
					return -1;
				}
				if (hdr != make_fourcc('-','c','a','l')) {
					return -1;
				}
				break;
			case TLENSCALIB_TAG_DATA:
				if (fread_DVEC_STREAM(&data, ds, size) != size) {
					Err_("IO error - cannot read tex_coord");
					return -1;
				}
				break;

	

			default:
				break;
		}
	}

	delete ds;

	// Bezier 곡면의 좌표계를 일단 0~1 범위로 Normalize하고 영상 좌표계(좌상단이 원점)로 바꾼다.
	switch (type) {
		case make_fourcc('b','z','r','B'): {
			BEZIER_INFO bi;
			BEZIER2_Info(&bi, data);
			vec2 vScale = make_vec2(1.0f / (float)(src_size.width), 1.0f / (float)(src_size.height));
			switch (bi.cp_dim) {
				case 2: {
					vec2 *pCP = (vec2*)data;
					for (int i = 0; i < bi.cp_cnt; i++) {
						pCP[i].x *= vScale.x;
						pCP[i].y = 1.0 - pCP[i].y * vScale.y;
					}
					break;
				}
				case 3: {
					vec3 *pCP = (vec3*)data;
					for (int i = 0; i < bi.cp_cnt; i++) {
						pCP[i].x *= vScale.x;
						pCP[i].y = 1.0 - pCP[i].y * vScale.y;
					}
					break;
				}
				default: {
					Err_("Cannot support %d dimension control points", bi.cp_dim);
					return 0;
				}
			}
			break;
		}
		case make_fourcc('p','l','y','1'):
			//return SamplePoly1ToArray(dst, segments);
			break;

	case make_fourcc('f','e','y','e'):
		DVEC_SetLength(&data, 1, 1);  //data를 NULL로 놓아두면 LoadV1이 호출됨
		lenstype = 1; //LENSTYPE_FISHEYE
		break;

	case make_fourcc('w','i','d','e'):
		DVEC_SetLength(&data, 1, 1);  //data를 NULL로 놓아두면 LoadV1이 호출됨
		lenstype = 0; //LENSTYPE_NORMAL
		break;

		default:
			break;
	}

	return 1;
}


/**--------------------------------------------------------------------------
@brief file_name 멤버에 지정된 파일을 불러온다.

바이너리 형식의 Calibration 데이터는 다음과 같은 구조로 저장되어 있다.
data_size(size2u16): 4byte
src_size(size2u16): 4byte
data(vec2): data_size.width * data_size.height * 8byte

읽어들이는 과정에서 발생하는 오류는 Err_ 함수로 그 내용을 출력한다.

@return 읽어들인 데이터의 바이트 수

@author ksg
*/
size_t TLensCalib::Load(void)
{
	if (data) {
		return 1;
	}
	size_t ret = 0;

	if(dvecStream) //스트림으로 읽을 때
	{
		char pHeader[] = "     ";
		memcpy(pHeader, dvecStream, 5);

		if (strisame(pHeader,"l-cal")) {
		ret = LoadV2_Stream(dvecStream);
		} else {
		ret = LoadV1_Stream(dvecStream);
		}

	}
	else
	{
		FILE *fp = fopen(file_name.c_str(), "rb");
		char pHeader[] = "     ";
		if (fp == NULL) {
		Err_("Cannot open the file \'%s\'", file_name.c_str());
		return 0;
		}
		fread(pHeader, 1, 5, fp);
		fseek(fp, 0, SEEK_SET);
	  

		if (strisame(pHeader,"l-cal")) {
		// 새로운 버전의 Calibration data
		ret = LoadV2(fp);
		} else {
		ret = LoadV1(fp);
		}

		scale_x = 1.0 / (float)(src_size.width);
		scale_y = 1.0 / (float)(src_size.height);
/* 2021.11.01, kjkim test
if (K != NULL)
{
    Log_("cx cy fx fy =%f %f %f %f", K[2], K[5], K[0], K[4]);
    Log_("D = %f %f %f %f %f", D[0], D[1], D[2], D[3], D[4]);
    Log_("K_new =%f %f %f %f", K_new[2], K_new[5], K_new[0], K_new[4]);
}
*/
		input_image_size = make_size2i16(src_size.width, src_size.height);

		fclose(fp);
	}
	return ret;
}

/**--------------------------------------------------------------------------
@brief Bezier 곡면 기반 렌즈 캘리브레이션 데이터를 바탕으로 지정한 segment
간격으로 위치를 샘플링한다.

@param [in ] dst      샘플링 된 정점 벡터를 저장할 배열. 배열은
											(segments.x + 1) * (segments.y + 1)의 크기로 준비되어야
											한다. 정점 벡터는 0~1로 정규화 되어 저장된다.
@param [in ] segments 가로 세로 방향으로 분할할 구간 수
@return 생성된 정점의 수

@author ksg
*/
int TLensCalib::SampleBezierToArray(vec2 *dst, vec2u16 segments)
{
	return BEZIER2_SampleWithParams((DVEC*)&dst, 2, data, 0, 1, segments.x, 0, 1, segments.y);
}
int TLensCalib::DistortBezierPoints(vec2 *dst, vec2 const *src, int count)
{
	return BEZIER2_Sample((real_t*)dst, 2, data, src, count);
}

int TLensCalib::UndistortBezierPoints(vec2 *dst, vec2 const *src, int count)
{
	for (int i = 0; i < count; ++i, ++dst) {
		BEZIER2_GetUV((real_t *)dst, data, make_vec3f(src[i].x, src[i].y, 0));
//    dst->y = 1.0f - dst->y;
	}
	return count;
}

int TLensCalib::DistortPoints(vec2 *dst, vec2 const *src, int count)
{
	if (GetData() == NULL) {
		Err_("Cannot get calibration data. file_name: %s", file_name.c_str());
		return 0;
	}
	switch (type) {
		case make_fourcc('b','z','r','B'):
			count = DistortBezierPoints(dst, src, count);
			break;
		case make_fourcc('p','l','y','1'):
		count = DistortPoly1Points(dst, src, count);
			break;
	case make_fourcc('f','e','y','e'):
		count = DistortFisheyePoints(dst, src, count);
		break;

	case make_fourcc('w','i','d','e'):
		count = DistortWAPoints(dst, src, count);
		break;

		default:
			//ret = SampleLUTToArray(dst, segments);
			break;
	}
	return count;
}

int TLensCalib::setParameter(real_t v1, real_t v2)
{
	p1 = v1;
	p2 = v2;
	return 0;
}

void TLensCalib::getParameter(real_t *v1, real_t *v2)
{
	*v1 = p1;
	*v2 = p2;
}


int TLensCalib::UndistortPoints(vec2 *dst, vec2 const *src, int count)
{
	if (GetData() == NULL) {
		Err_("Cannot get calibration data. file_name: %s", file_name.c_str());
		return 0;
	}
	switch (type) {
		case make_fourcc('b','z','r','B'):
			count = UndistortBezierPoints(dst, src, count);
			break;
		case make_fourcc('p','l','y','1'):
		 count = UndistortPoly1Points(dst, src, count);
			break;
	case make_fourcc('f','e','y','e'):
		count = UndistortFisheyePoints(dst, src, count);
		break;
	case make_fourcc('w','i','d','e'):
		count = UndistortFisheyePoints(dst, src, count);
		break;
		default:
			//ret = SampleLUTToArray(dst, segments);
			break;
	}
	return count;
}

void tiltXY(int n_x, int n_y, float32_t x, float32_t y, float32_t *newx, float32_t *newy)
{
	float32_t p1, p2;
	int32_t i;
	if(n_x < 0) {n_x = -n_x; p1 = -0.001f;}
	else {p1 = 0.001f;}
	if(n_y < 0) {n_y = -n_y; p2 = -0.001f;}
	else {p2 = 0.001f;}

	float32_t xx, yy, r2, x_corrected, y_corrected;
	xx = x;
	yy = y;

	for(i = 0; i < n_x; i++) {
		r2 = (xx * xx) + (yy * yy);
		x_corrected = xx + (2 * p1 * xx * yy);
		y_corrected = yy + (p1 * (r2 + 2 * yy * yy));
		xx = x_corrected;
		yy = y_corrected;
	}

	for(i = 0; i < n_y; i++) {
		r2 = (xx * xx) + (yy * yy);
		x_corrected = xx + (p2 * (r2 + 2 * xx * xx));
		y_corrected = yy + (2 * p2 * xx * yy);
		xx = x_corrected;
		yy = y_corrected;
	}
	if(n_x == 0 && n_y == 0) {
		*newx = x;
		*newy = y;
	} else {
		*newx = x_corrected;
		*newy = y_corrected;
	}

}


int TLensCalib::DistortPoly1Points(vec2 *dst, vec2 const *src, int count)
{
	// src에 들어오는 좌표값은 캘리브레이션 패널의 가로 세로를 각각 1로 놨을 때
	// Normalize된 좌표로 영상 좌표계를 사용한다 (좌상단이 원점)

	vec2 *pPair = (vec2*)data;
	vec2 vSrcSize = make_vec2(src_size.width, src_size.height);
	vec2 vSrcHalf = make_vec2(vSrcSize.x * 0.5f, vSrcSize.y * 0.5f);
	vec2 vSnsSize = make_vec2(sensor_size.width, sensor_size.height);

	int data_cnt = DVEC_Length(data);

	vec2   vClbOrgn = make_vec2(calpnl_size.x * 0.5f, calpnl_size.y * 0.5f); // 패널 중심 위치

	real_t fDistMax = pPair[data_cnt-1].y;
	real_t fScale = 0;
	switch (bit_fields & 0x03) {
		case 0: // Diagonal
			fScale = (real_t)sqrt(vec2_sqlen(vSnsSize)) / (fDistMax * 2.0f); // 센서 픽셀 수 기준 (대각)
			break;
		case 1: // Horizontal
			fScale = vSnsSize.x / (fDistMax * 2.0f); // 수평 기준
			break;
		case 2: // Vertical
			fScale = vSnsSize.y / (fDistMax * 2.0f); // 수직 기준
			break;
	default:
		;
	}
	//double k[4] = {-0.035458229399265073, -0.011491268689424418, -0.00021135230522860470, 0.00069747562089634749};

	//double p1, p2;
	//p1 = 0.01;
	//p2 = 0.01;

	vec2   vPhyPos, vSnsPos;
	for (int i = 0; i < count; i++) {
		vPhyPos = src[i];

		vPhyPos.x = vPhyPos.x * calpnl_size.x - vClbOrgn.x;
		vPhyPos.y = vPhyPos.y * calpnl_size.y - vClbOrgn.y;

		if (vPhyPos.x == 0.0 && vPhyPos.y == 0.0) {
			vSnsPos = make_vec2(0.5,0.5);
		} else {
			real_t fRadZ = (real_t)atan2((double)vPhyPos.y, (double)vPhyPos.x);
			real_t fRadL = (real_t)atan2((double)sqrt((double)vec2_sqlen(vPhyPos)), (double)calpnl_dist);
			real_t fDegL = (real_t)fRadL * 180.0f / (real_t)M_PI;
			int idx;
			for (idx = 0; idx < data_cnt - 1; ++idx) {
				if (fDegL < pPair[idx+1].x) {
					break;
				}
			}

#if 0
			double theta = fRadL;
			double r = tan(theta);

			double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
			double theta_d = theta * (1 + k[0] * theta2 + k[1] * theta4 + k[2] * theta6 + k[3] * theta8);

			//double scale = (r == 0) ? 1.0 : theta_d / r;

			real_t fSnsLen = theta_d;
			vSnsPos.x = (fSnsLen * fScale * (real_t)cos((double)fRadZ) + vSrcHalf.x) / vSrcSize.x;
			vSnsPos.y = (fSnsLen * fScale * (real_t)sin((double)fRadZ) + vSrcHalf.y) / vSrcSize.y;
#else

			if (idx < data_cnt) {
				// 선형보간
				real_t fdx = (fDegL - pPair[idx].x) / (pPair[idx+1].x - pPair[idx].x);
				real_t fSnsLen = pPair[idx].y * (1.0f - fdx) + pPair[idx+1].y * fdx;

				// 센서상의 위치 계산
				/// tangencial 오차 보정
				float32_t x, y;
				x = (fSnsLen * fScale * (real_t)cos((double)fRadZ) + vSrcHalf.x) / (vSrcSize.x - 1.0f);//- 0.5f;
				y = (fSnsLen * fScale * (real_t)sin((double)fRadZ) + vSrcHalf.y) / (vSrcSize.y - 1.0f);//- 0.5f);

				tangencialDistort(x, y, &vSnsPos.x, &vSnsPos.y);

			} else {
				Err_("Invalid idx");
			}

	
#endif

		}
		dst[i] = vSnsPos;
	}

/*
	vec2 *test = NULL;
	DVEC_SETLENGTH(vec2, &test, count);
	UndistortPoly1Points(test, dst, count);
	DVEC_FREE(&test);
*/
	return count;
}

/**
	@brief 렌즈와 센서 사이의 tangencial 에러 보정
*/
int TLensCalib::tangencialDistort(
	float32_t x_in,    ///< 입력 x 좌표, 0.0 ~ 1.0으로 normalize
	float32_t y_in,    ///< 입력 y 좌표, 0.0 ~ 1.0으로 normalize
	float32_t *x_out,  ///< 출력 x 좌표, 0.0 ~ 1.0으로 normalize
	float32_t *y_out   ///< 출력 y 좌표, 0.0 ~ 1.0으로 normalize
)
{
	x_in -= 0.5f;
	y_in -= 0.5f;
	float32_t r2;
	float32_t x_corrected, y_corrected;

	r2 = (x_in * x_in) + (y_in * y_in);
	x_corrected = x_in + ((2 * p1 * x_in * y_in) + (p2 * (r2 + 2 * x_in * x_in)));
	y_corrected = y_in + ((p1 * (r2 + 2 * y_in * y_in)) + (2 * p2 * x_in * y_in));
	*x_out = x_corrected + 0.5f;
	*y_out = y_corrected + 0.5f;

	return 0;
}

/**
	@brief 렌즈와 센서 사이의 tangencial 에러 보정, Undistort 방향
*/
int TLensCalib::tangencialUndistort(
	float32_t x_in,    ///< 입력 x 좌표, 0.0 ~ 1.0으로 normalize
	float32_t y_in,    ///< 입력 y 좌표, 0.0 ~ 1.0으로 normalize
	float32_t *x_out,  ///< 출력 x 좌표, 0.0 ~ 1.0으로 normalize
	float32_t *y_out   ///< 출력 y 좌표, 0.0 ~ 1.0으로 normalize
)
{
	x_in -= 0.5f;
	y_in -= 0.5f;
	float32_t r2;
	float32_t x_corrected, y_corrected;

	r2 = (x_in * x_in) + (y_in * y_in);
	x_corrected = x_in + ((2 * (-p1) * x_in * y_in) + ((-p2) * (r2 + 2 * x_in * x_in)));
	y_corrected = y_in + (((-p1) * (r2 + 2 * y_in * y_in)) + (2 * (-p2) * x_in * y_in));

	x_in = x_corrected;
	y_in = y_corrected;

	r2 = (x_in * x_in) + (y_in * y_in);
	x_corrected = x_in + ((2 * (-p1 / 10.0f) * x_in * y_in) + ((-p2 / 10.0f) * (r2 + 2 * x_in * x_in)));
	y_corrected = y_in + (((-p1 / 10.0f) * (r2 + 2 * y_in * y_in)) + (2 * (-p2 / 10.0f) * x_in * y_in));

	*x_out = x_corrected + 0.5f;
	*y_out = y_corrected + 0.5f;

	return 0;
}

/**
	@brief	어안렌즈 모델로 왜곡 보정된 좌표에서 렌즈 왜곡된 점의 좌표 구하기
*/
int TLensCalib::DistortFisheyePoints(
	vec2 *dst,          ///< 렌즈 왜곡된 점의 좌표
	vec2 const *src,    ///< 렌즈 왜곡 보정된 점의 좌표
	int count           ///< 좌표 변환할 점의 수
)
{
	int height = sensor_size.height - 1; //왜곡 보정한 이미지의 크기, OpenCV의 fisheye 모델에서는 좌표 변화에 카메라 모델을 사용하므로 변환 전 후 이미지 크기를 알아야 함
	int width = sensor_size.width - 1;
	float64_t f[2];
	float64_t c[2];

	f[0] = K[0];
	f[1] = K[4];
	c[0] = K[2];
	c[1] = K[5];
	vec2 vSnsPos;

	float64_t _x, _y, _w;
	float64_t sx, sy;

	for(int i = 0; i < count; i++) {
		sx = (float64_t)src[i].x * (float64_t)width;  //픽셀 좌표계로 변환
		sy = (float64_t)src[i].y * (float64_t)height;
		_x = sx * giR[0] + sy * giR[1] + giR[2];
		_y = sx * giR[3] + sy * giR[4] + giR[5];
		_w = sx * giR[6] + sy * giR[7] + giR[8];
		float64_t x = _x / _w, y = _y / _w; //inverse camera matrix를 곱해서 각도 좌표계로 변환

		/// fisheye 모델 적용
		float64_t r = sqrt(x*x + y*y);
		float64_t theta = atan(r);

		float64_t theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
		float64_t theta_d = theta * (1 + D[0] * theta2 + D[1] * theta4 + D[2] * theta6 + D[3] * theta8);

		float64_t scale = (r == 0) ? 1.0 : theta_d / r;
		float64_t u = f[0] * x*scale + c[0];
		float64_t v = f[1] * y*scale + c[1];

		/// tangencial 왜곡 보정

		float32_t xn, yn;
		xn = (float32_t)u / (float32_t)(src_size.width - 1);
		yn = (float32_t)v / (float32_t)(src_size.height - 1);
		tangencialDistort(xn, yn, &vSnsPos.x, &vSnsPos.y);

		dst[i] = vSnsPos;

		
	}


	
	return count;
}

/**
	@brief	어안렌즈 모델로 왜곡 보정된 좌표에서 렌즈 왜곡된 점의 좌표 구하기
*/

// 코드 최적화 필요함
int TLensCalib::DistortWAPoints(
  vec2 *dst,          ///< 렌즈 왜곡된 점의 좌표
  vec2 const *src,    ///< 렌즈 왜곡 보정된 점의 좌표
  int count           ///< 좌표 변환할 점의 수
)
{

	const int height = sensor_size.height - 1; //왜곡 보정한 이미지의 크기, OpenCV의 fisheye 모델에서는 좌표 변화에 카메라 모델을 사용하므로 변환 전 후 이미지 크기를 알아야 함
	const int width = sensor_size.width - 1;
	const int stripe_size = 6; // from lens calibration;
	int counter = 0;
	float64_t f[2];
	float64_t c[2];

	f[0] = K[0]; // K(0, 0)
	f[1] = K[4]; // K(1, 1)
	c[0] = K[2]; // K(0, 2)
	c[1] = K[5]; // K(1, 2)
	vec2 vSnsPos;

	//double u0 = A(0, 2), v0 = A(1, 2);
	//double fx = A(0, 0), fy = A(1, 1);

	//CV_Assert(distCoeffs.size() == Size(1, 4) || distCoeffs.size() == Size(4, 1) ||
	//	distCoeffs.size() == Size(1, 5) || distCoeffs.size() == Size(5, 1) ||
	//	distCoeffs.size() == Size(1, 8) || distCoeffs.size() == Size(8, 1) ||
	//	distCoeffs.size() == Size(1, 12) || distCoeffs.size() == Size(12, 1) ||
	//	distCoeffs.size() == Size(1, 14) || distCoeffs.size() == Size(14, 1));

	//if (distCoeffs.rows != 1 && !distCoeffs.isContinuous())
	//	distCoeffs = distCoeffs.t();

	const double* const distPtr = (const double*)D;
	double k1 = distPtr[0];
	double k2 = distPtr[1];
	double p1 = distPtr[2];
	double p2 = distPtr[3];
	double k3 = 1 + 5 - 1 >= 5 ? distPtr[4] : 0.;
	double k4 = 1 + 5 - 1 >= 8 ? distPtr[5] : 0.;
	double k5 = 1 + 5 - 1 >= 8 ? distPtr[6] : 0.;
	double k6 = 1 + 5 - 1 >= 8 ? distPtr[7] : 0.;
	double s1 = 1 + 5 - 1 >= 12 ? distPtr[8] : 0.;
	double s2 = 1 + 5 - 1 >= 12 ? distPtr[9] : 0.;
	double s3 = 1 + 5 - 1 >= 12 ? distPtr[10] : 0.;
	double s4 = 1 + 5 - 1 >= 12 ? distPtr[11] : 0.;
	double tauX = 1 + 5 - 1 >= 14 ? distPtr[12] : 0.;
	double tauY = 1 + 5- 1 >= 14 ? distPtr[13] : 0.;

//  Matrix for trapezoidal distortion of tilted image sensor
//	cv::Matx33d matTilt = cv::Matx33d::eye();
//	mat3d_t matTilt = mat3d_t(1., 0., 0., 0., 1., 0., 0., 0., 1.);//(mat3d_t)calloc(sizeof(double), 9);
//	cv::detail::computeTiltProjectionMatrix(tauX, tauY, &matTilt);
	//computeTiltProjectionMatrix(tauX, tauY, &matTilt);

	float64_t _x=0, _y, _w;
	float64_t sx, sy;
	sx = (float64_t)src[0].x * (float64_t)width;  //픽셀 좌표계로 변환
	sy = (float64_t)src[0].y * (float64_t)height;

	//int yi = (int)sy;
	//int xi = (int)sx;
	float64_t yi = sy;
	float64_t xi = sx;
//	if ( yi != 0 || xi != 0)
	{
		_x = yi*giR[1] + giR[2], _y = yi*giR[4] + giR[5], _w = yi*giR[7] + giR[8];
	}
	if (xi != 0) {
		_x += (xi*giR[0]), _y += (xi*giR[3]), _w += (xi*giR[6]);
	}

	/// 샘플링을 정해진 범위와 간격으로만 하게끔 되어 있어서 넣은 코드
    /// 의도한 샘플링 영역을 구하고 함수 호출 시 요구한 좌표를 반영하게 함
	float64_t sample_sx, sample_ex, sample_sy, sample_ey, sample_w, sample_h;
	sample_sx = _x;
	sample_sy = _y;
	sample_ex = sample_sx + 511 * giR[0];
	sample_ey = sample_sy + 255 * giR[4];
	sample_w = sample_ex - sample_sx;
    sample_h = sample_ey - sample_sy;

	//FILE *fp;
	//fp = fopen("log1_xy2.txt", "w+");

//for(int i = sy; i < sensor_size.height; i++) {
	for (int i = 0; i < count; i++, xi++, _x += giR[0], _y += giR[3], _w += giR[6]) {
		if (xi > width) {
			yi++; xi = 0;
			_x = yi*giR[1] + giR[2], _y = yi*giR[4] + giR[5], _w = yi*giR[7] + giR[8];
		}

		//fprintf(fp, "[%d]%f,%f\n", i, _x, _y);

		_x = src[i].x * sample_w + sample_sx; //src 값의 의도대로 샘플링 좌표 구하기
		_y = src[i].y * sample_h + sample_sy;
//	if ( (i%6) == 0) {
//		giR[5] += 0.048666126;
//	}
//	for(int j = sx; j < sensor_size.width; j++,  _x += giR[0], _y += giR[3], _w += giR[6]) {

		float64_t w = 1. / _w, x = _x*w, y = _y*w;

		float64_t x2 = x*x, y2 = y*y;
		float64_t r2 = x2 + y2, _2xy = 2 * x*y;
		float64_t kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2) / (1 + ((k6*r2 + k5)*r2 + k4)*r2);
		float64_t xd = (x*kr + p1*_2xy + p2*(r2 + 2 * x2) + s1*r2 + s2*r2*r2);
		float64_t yd = (y*kr + p1*(r2 + 2 * y2) + p2*_2xy + s3*r2 + s4*r2*r2);

//		vec3 vecTilt = vec3f(xd,yd,1);//NULL;//matTilt*cv::Vec3d(xd, yd, 1);
		//float64_t invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;
		float64_t invProj = 1;
//		float64_t scale = (r == 0) ? 1.0 : theta_d / r;
		//float64_t u = fx*invProj*vecTilt(0) + u0;
		//float64_t v = fy*invProj*vecTilt(1) + v0;
		//float64_t u = x*1 + c[0];
		//float64_t v = y*1 + c[1];
		float64_t u = f[0] *xd + c[0];
		float64_t v = f[1] *yd + c[1];

		/// tangencial 왜곡 보정
		float32_t xn, yn;
		xn = (float32_t)u / (float32_t)(src_size.width - 1);
		yn = (float32_t)v / (float32_t)(src_size.height - 1);
		//if (xn < 0) xn = -1 * xn;
		//if (yn < 0) yn = -1 * yn;
		tangencialDistort(xn, yn, &vSnsPos.x, &vSnsPos.y);
//		vSnsPos.x = xn;
//		vSnsPos.y = yn;

		//vSnsPos.x = vSnsPos.x * 2.0f - 1.0f;
		//vSnsPos.y = vSnsPos.y * 2.0f - 1.0f;
		dst[i] = vSnsPos;

        //fprintf(fp, "[%d]%f,%f\n", i, vSnsPos.x, vSnsPos.y);
	}

	//fclose(fp);

DistortFinsih:
	return count;
}


#define real_is_(v, o) (((v) < ((o)+1.0E-7)) && ((v) > ((o)-1.0E-7)))
/**
@params src Vertex 배열
*/

int TLensCalib::ProjectPoly1Vectors(vec2 *dst, vec3 const *src, int count, mat4f_t cam_pos, mat4f_t src_pos)
{
// 2022.03.04, kjkim parameter test
/*
    calpnl_size.x = sensor_size.width = src_size.width = 1280;
    calpnl_size.y = sensor_size.height = src_size.height = 720;
    calpnl_dist = 1;
*/
	vec2 *pPair = (vec2*)data;
	vec2 vSnsSize = make_vec2(sensor_size.width, sensor_size.height);

	vec2 vSrcSize = make_vec2(src_size.width, src_size.height);
	vec2 vSrcHalf = make_vec2(vSrcSize.x * 0.5f, vSrcSize.y * 0.5f);

	vec2 vSnsPos;

	vec2   vClbOrgn = make_vec2(calpnl_size.x * 0.5f, calpnl_size.y * 0.5f); // 패널 중심 위치

	int data_cnt = DVEC_Length(data);

	real_t fDistMax = pPair[data_cnt-1].y;
//2022.02.18, kjkim to use 2D calib
	real_t fScale = 0;
	real_t fScale_x = 0;
	real_t fScale_y = 0;

	switch (bit_fields & 0x03) {
		case 0: // Diagonal
			fScale = (real_t)sqrt(vec2_sqlen(vSnsSize)) / (fDistMax * 2.0f); // 센서 픽셀 수 기준 (대각)
			break;
		case 1: // Horizontal
//2022.02.18, kjkim to use 2D calib
//			fScale = vSnsSize.x / (fDistMax * 2.0f); // 수평 기준
			fScale_x = vSnsSize.x / (fDistMax * 2.0f); // 수평 기준
//			fScale_y = vSnsSize.x / (fDistMax * 2.0f)*sensor_size.width/sensor_size.height; // 수직 기준 비율 맞춤
//			fScale_y = vSnsSize.x / (fDistMax * 2.0f)*1.5f; // 수직 기준 비율 맞춤  (메뉴얼)
			fScale_y = vSnsSize.x / (fDistMax * 2.0f); // 수직 기준 비율 맞춤  (메뉴얼)
			break;
		case 2: // Vertical
			fScale = vSnsSize.y / (fDistMax * 2.0f); // 수직 기준
			break;
	default:
		;
	}

	vec2 v2d;
	vec3 vRay, vAxis = {0,1,0}; // y축
	float *pRay = (float*)&vRay, *pAxis = (float*)&vAxis;
	mat4f_t cam_inv = mat4f_init();
	mat4f_inverse(cam_pos, cam_inv);
	if (src_pos) {
		mat4f_multiply(cam_inv, src_pos, cam_inv);
	}

	for (int i = 0; i < count; i++) {
		// 로컬좌표로 변환
		vec3f_transformMat4f((float*)(src + i), cam_inv, pRay);
		v2d = make_vec2f(vRay.x, vRay.z);
		vec3f_normalize(pRay); // Normalize

		real_t dot = vec3f_dot(pRay, pAxis);
		//real_t dot = vRay.x * vAxis.x + vRay.y * vAxis.y + vRay.z * vAxis.z;
		real_t rad_r = acos(dot); // Ray와 카메라 광축(Y)과의 각도


		if (real_is_(rad_r, 0.0)) {
			vSnsPos = make_vec2(0.5,0.5);
		} else {
			real_t deg_r = to_deg_(rad_r);
			vec2f_normalize((float*)&v2d); // 이제 v2d.x == cos이고 v2d.y는 sin값이 된다.

/*
			real_t rad_y = (real_t)atan2((double)vRay.z, (double)vRay.x); // y축 중심으로 회전각
			real_t deg_y = to_deg_(rad_y);
*/
			int idx;
			for (idx = 0; idx < data_cnt - 1; ++idx) {
				if (deg_r < pPair[idx+1].x) {
					break;
				}
			}

			if (idx < data_cnt) {
				// 선형보간
				real_t fdx = (deg_r - pPair[idx].x) / (pPair[idx+1].x - pPair[idx].x);
				real_t fSnsLen = pPair[idx].y * (1.0f - fdx) + pPair[idx+1].y * fdx;

				// 센서상의 위치 계산
//2022.02.18, kjkim to use 2D calib
//				vSnsPos.x = (fSnsLen * fScale * v2d.x + vSrcHalf.x) / (real_t)(vSrcSize.x - 1);
//				vSnsPos.y = (fSnsLen * fScale * v2d.y + vSrcHalf.y) / (real_t)(vSrcSize.y - 1);

				vSnsPos.x = (fSnsLen * fScale_x * v2d.x + vSrcHalf.x) / (real_t)(vSrcSize.x - 1);
				vSnsPos.y = (fSnsLen * fScale_y * v2d.y + vSrcHalf.y) / (real_t)(vSrcSize.y - 1);

				/// tangencial 오차 보정
				//tangencialDistort(vSnsPos.x, vSnsPos.y, &vSnsPos.x, &vSnsPos.y);
			} else {
				Err_("Invalid idx");
			}
		}

		dst[i] = vSnsPos;
	}

	free(cam_inv);
	return 1;
}

/**
@params src Vertex 배열
*/
//2022.02.18, kjkim to use 2D calib
int TLensCalib::ProjectVectors(vec2 *dst, vec3 const *src, int count, mat4f_t cam_pos, mat4f_t src_pos)
{

    switch (type) {
		case make_fourcc('b','z','r','B'):
			break;
		case make_fourcc('p','l','y','1'):
		    ProjectPoly1Vectors(dst, src, count, cam_pos, src_pos);
//            pCalib->ProjectVectors((vec2*)(pTX->data), (vec3*)(pVB->data), pVB->GetLength(), pCI->P);
			break;
#if (USE_SAVM_HDI == 1)
//2023.11.21, kjkim calibration.data 가 있는 경우에는 type을 읽지 않아 0으로 온다.
//		case 0:
#endif
        case make_fourcc('f','e','y','e'):
		    ProjectFeyeVectors(dst, src, count, cam_pos, src_pos);
            break;
        case make_fourcc('w','i','d','e'):
            break;
		default:
			break;
	}
	return 1;
}

#if 1 //2022.03.03, test ksj's ftn
int TLensCalib::ProjectFeyeVectors(vec2 *dst, vec3 const *src, int count, mat4f_t cam_pos, mat4f_t src_pos)
{
	int height = sensor_size.height - 1; //왜곡 보정한 이미지의 크기, OpenCV의 fisheye 모델에서는 좌표 변화에 카메라 모델을 사용하므로 변환 전 후 이미지 크기를 알아야 함
	int width = sensor_size.width - 1;
	float64_t f[2];
	float64_t c[2];

	f[0] = K[0];
	f[1] = K[4];
	c[0] = K[2];
	c[1] = K[5];
	vec2 vSnsPos;

	float64_t _x, _y, _w;
	float64_t sx, sy, sz;

    vec2 v2d;
	vec3 vRay, vAxis = {0,1,0}; // y축

	float *pRay = (float*)&vRay, *pAxis = (float*)&vAxis;
	mat4f_t cam_inv = mat4f_init();
	mat4f_inverse(cam_pos, cam_inv);
	if (src_pos) {
		mat4f_multiply(cam_inv, src_pos, cam_inv);
	}

	for(int i = 0; i < count; i++) {
        vec3f_transformMat4f((float*)(src + i), cam_inv, pRay);
//		v2d = make_vec2f(vRay.x, vRay.z);
		vec3f_normalize(pRay); // Normalize
        //pRay[0] = pRay[0] * 0.5f;
        //pRay[1] = pRay[1] * 0.5f;
        real_t dot = vec3f_dot(pRay, pAxis);
        real_t rad_r = acos(dot);

/*
		sx = (float64_t)pRay[0] * (float64_t)width;  //픽셀 좌표계로 변환
		sy = (float64_t)pRay[1] * (float64_t)height;

		_x = sx * giR[0] + sy * giR[1] + giR[2];
		_y = sx * giR[3] + sy * giR[4] + giR[5];
		_w = sx * giR[6] + sy * giR[7] + giR[8];
		float64_t x = _x / _w, y = _y / _w; //inverse camera matrix를 곱해서 각도 좌표계로 변환
*/
		float64_t x = pRay[0];
        float64_t y = pRay[2];

		/// fisheye 모델 적용
		float64_t r = sqrt(x*x + y*y);
		float64_t theta = rad_r;//atan(r); //cal_panel_dist 를 1로 뒀을때 fDegL과 같음, 그러므로 projectpoly에서 rad_r과 같다 할 수 있음
//		float64_t theta = atan(r);

		float64_t theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
		float64_t theta_d = theta * (1 + D[0] * theta2 + D[1] * theta4 + D[2] * theta6 + D[3] * theta8);

		float64_t scale = (r == 0) ? 1.0 : theta_d / r;
		float64_t u = f[0] * x*scale + c[0];
		float64_t v = f[1] * y*scale + c[1];

		/// tangencial 왜곡 보정

		float32_t xn, yn;
		xn = (float32_t)u / (float32_t)(src_size.width - 1);
		yn = (float32_t)v / (float32_t)(src_size.height - 1);

		tangencialDistort(xn, yn, &vSnsPos.x, &vSnsPos.y);

		dst[i] = vSnsPos;
	}

	return count;
}
#else //2022.03.03, test ksj's ftn
int TLensCalib::ProjectFeyeVectors(vec2 *dst, vec3 const *src, int count, mat4f_t cam_pos, mat4f_t src_pos)
{
	int height = sensor_size.height - 1; //왜곡 보정한 이미지의 크기, OpenCV의 fisheye 모델에서는 좌표 변화에 카메라 모델을 사용하므로 변환 전 후 이미지 크기를 알아야 함
	int width = sensor_size.width - 1;
	float64_t f[2];
	float64_t c[2];

	f[0] = K[0];
	f[1] = K[4];
	c[0] = K[2];
	c[1] = K[5];

	vec2 vSnsSize = make_vec2(sensor_size.width, sensor_size.height);
	vec2 vSrcSize = make_vec2(src_size.width, src_size.height);
	vec2 vSrcHalf = make_vec2(vSrcSize.x * 0.5f, vSrcSize.y * 0.5f);
    vec2 vSnsPos;

	vec2 v2d;
	vec3 vRay, vAxis = {0,1,0}; // y축
	float *pRay = (float*)&vRay, *pAxis = (float*)&vAxis;
	mat4f_t cam_inv = mat4f_init();
	mat4f_inverse(cam_pos, cam_inv);
	if (src_pos) {
		mat4f_multiply(cam_inv, src_pos, cam_inv);
	}

    float64_t _x, _y, _w;
	float64_t sx, sy;

	for(int i = 0; i < count; i++) {
		// 로컬좌표로 변환
		vec3f_transformMat4f((float*)(src + i), cam_inv, pRay);
		v2d = make_vec2f(vRay.x, vRay.z);
		vec3f_normalize(pRay); // Normalize

		real_t dot = vec3f_dot(pRay, pAxis);
		//real_t dot = vRay.x * vAxis.x + vRay.y * vAxis.y + vRay.z * vAxis.z;
		real_t rad_r = acos(dot); // Ray와 카메라 광축(Y)과의 각도


		if (real_is_(rad_r, 0.0)) {
			vSnsPos = make_vec2(0.5,0.5);
		} else {
			real_t deg_r = to_deg_(rad_r);
			vec2f_normalize((float*)&v2d); // 이제 v2d.x == cos이고 v2d.y는 sin값이 된다.
            sx = (float64_t)src[i].x * (float64_t)width;  //픽셀 좌표계로 변환
            sy = (float64_t)src[i].y * (float64_t)height;
            _x = sx * giR[0] + sy * giR[1] + giR[2];
            _y = sx * giR[3] + sy * giR[4] + giR[5];
            _w = sx * giR[6] + sy * giR[7] + giR[8];
            float64_t x = _x / _w, y = _y / _w; //inverse camera matrix를 곱해서 각도 좌표계로 변환

            /// fisheye 모델 적용
            float64_t theta = deg_r;
			float64_t r = tan(theta);

            float64_t theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
            float64_t theta_d = theta * (1 + D[0] * theta2 + D[1] * theta4 + D[2] * theta6 + D[3] * theta8);

            float64_t scale = (r == 0) ? 1.0 : theta_d / r;
            float64_t u = f[0] * x*scale + c[0];
            float64_t v = f[1] * y*scale + c[1];

            //double scale = (r == 0) ? 1.0 : theta_d / r;

            real_t fSnsLen = theta_d;
            // 센서상의 위치 계산
			vSnsPos.x = (fSnsLen * scale * v2d.x + vSrcHalf.x) / (real_t)(vSrcSize.x - 1);
			vSnsPos.y = (fSnsLen * scale * v2d.y + vSrcHalf.y) / (real_t)(vSrcSize.y - 1);
        }

		dst[i] = vSnsPos;
	}

	free(cam_inv);
	return 1;
}
/////////////////////////////////////////////////////2022.02.18
#endif //2022.03.03, test ksj's ftn

int TLensCalib::ProjectOnePoint(vec2 *dst, vec3 const src, mat4f_t cam_pos, mat4f_t src_pos)
{

	//Log_("ProjectPoly1Vectors 0");
	vec2 *pPair = (vec2*)data;
	//Log_("ProjectPoly1Vectors 0-1");
	vec2 vSnsSize = make_vec2(sensor_size.width, sensor_size.height);
	//Log_("ProjectPoly1Vectors 0-2");
	vec2 vSrcSize = make_vec2(src_size.width, src_size.height);
	//Log_("ProjectPoly1Vectors 0-3");
	vec2 vSrcHalf = make_vec2(vSrcSize.x * 0.5f, vSrcSize.y * 0.5f);
	//Log_("ProjectPoly1Vectors 0-4");
	vec2 vSnsPos;

	vec2   vClbOrgn = make_vec2(calpnl_size.x * 0.5f, calpnl_size.y * 0.5f); // 패널 중심 위치
	//Log_("ProjectPoly1Vectors 0-5");
	int data_cnt = DVEC_Length(data);
	//Log_("ProjectPoly1Vectors 0-6 data_cnt = %d", data_cnt);
	real_t fDistMax = pPair[data_cnt-1].y;
	//Log_("ProjectPoly1Vectors 0-7");
	real_t fScale = 0;
	//Log_("ProjectPoly1Vectors 1 bit_fields = %d", bit_fields);
	switch (bit_fields & 0x03) {
		case 0: // Diagonal
			fScale = (real_t)sqrt(vec2_sqlen(vSnsSize)) / (fDistMax * 2.0f); // 센서 픽셀 수 기준 (대각)
			break;
		case 1: // Horizontal
			fScale = vSnsSize.x / (fDistMax * 2.0f); // 수평 기준
			break;
		case 2: // Vertical
			fScale = vSnsSize.y / (fDistMax * 2.0f); // 수평 기준
			break;
	default:
		;
	}
	//Log_("ProjectPoly1Vectors 2");
	vec2 v2d;
	vec3 vRay, vAxis = {0,1,0}; // y축
	float *pRay = (float*)&vRay, *pAxis = (float*)&vAxis;
	mat4f_t cam_inv = mat4f_init();
	mat4f_inverse(cam_pos, cam_inv);
	//Log_("ProjectPoly1Vectors 3");
	if (src_pos) {
		mat4f_multiply(cam_inv, src_pos, cam_inv);
	}
	//Log_("ProjectPoly1Vectors 4 count = %d", count);
	//for (int i = 0; i < count; i++) {
		// 로컬좌표로 변환
		vec3f_transformMat4f((float*)(&src), cam_inv, pRay);
		v2d = make_vec2f(vRay.x, vRay.z);
		vec3f_normalize(pRay); // Normalize
		real_t dot = vec3f_dot(pRay, pAxis);
		//real_t dot = vRay.x * vAxis.x + vRay.y * vAxis.y + vRay.z * vAxis.z;
		real_t rad_r = acos(dot); // Ray와 카메라 광축(Y)과의 각도

		if (real_is_(rad_r, 0.0)) {
			vSnsPos = make_vec2(0.5,0.5);
		} else {
			real_t deg_r = to_deg_(rad_r);
			vec2f_normalize((float*)&v2d); // 이제 v2d.x == cos이고 v2d.y는 sin값이 된다.

			int idx;
			for (idx = 0; idx < data_cnt - 1; ++idx) {
				if (deg_r < pPair[idx+1].x) {
					break;
				}
			}

			if (idx < data_cnt) {
				// 선형보간
				real_t fdx = (deg_r - pPair[idx].x) / (pPair[idx+1].x - pPair[idx].x);
				real_t fSnsLen = pPair[idx].y * (1.0f - fdx) + pPair[idx+1].y * fdx;

				// 센서상의 위치 계산
				vSnsPos.x = (fSnsLen * fScale * v2d.x + vSrcHalf.x) / (real_t)(vSrcSize.x - 1);
				vSnsPos.y = (fSnsLen * fScale * v2d.y + vSrcHalf.y) / (real_t)(vSrcSize.y - 1);

				/// tangencial 오차 보정
				//tangencialDistort(vSnsPos.x, vSnsPos.y, &vSnsPos.x, &vSnsPos.y);
			} else {
				Err_("Invalid idx");
			}
		}
		//dst[i] = vSnsPos;
        *dst = vSnsPos;
	//}
	//Log_("ProjectPoly1Vectors 5");
	free(cam_inv);
	//Log_("ProjectPoly1Vectors 6");
	return 1;
}


int TLensCalib::UndistortPoly1Points(vec2 *dst, vec2 const *src, int count)
{
	vec2 *pPair = (vec2*)data;
	vec2 vSrcSize = make_vec2(src_size.width, src_size.height);
	vec2 vSrcHalf = make_vec2(vSrcSize.x * 0.5f, vSrcSize.y * 0.5f);
	vec2 vSnsSize = make_vec2(sensor_size.width, sensor_size.height);

	vec2   vClbOrgn = make_vec2(calpnl_size.x * 0.5f, calpnl_size.y * 0.5f); // 패널 중심 위치

	int data_cnt = DVEC_Length(data);

	real_t fDistMax = pPair[data_cnt-1].y;
	real_t fScale = 0;
	switch (bit_fields & 0x03) {
		case 0: // Diagonal
			fScale = (fDistMax * 2.0f) / (real_t)sqrt(vec2_sqlen(vSnsSize)); // 센서 픽셀 수 기준 (대각)
			break;
		case 1: // Horizontal
			fScale = (fDistMax * 2.0f) / vSnsSize.x; // 수평 기준
			break;
		case 2: // Vertical
			fScale = (fDistMax * 2.0f) / vSnsSize.y; // 수평 기준
			break;
	default:
		;
	}

	vec2   vUV, vSnsPos, vPhyPos;
	for (int i = 0; i < count; i++) {
		vSnsPos = src[i];

		/// tangencial 오차 보정
		tangencialUndistort(vSnsPos.x, vSnsPos.y, &vSnsPos.x, &vSnsPos.y);

		if (vSnsPos.x == 0.5 && vSnsPos.y == 0.5) {
			vUV = make_vec2(0.5,0.5);
			continue;
		}
		vSnsPos.x = vSnsPos.x * vSrcSize.x - vSrcHalf.x;
		vSnsPos.y = vSnsPos.y * vSrcSize.y - vSrcHalf.y;



		real_t fRadZ = (real_t)atan2((double)vSnsPos.y, (double)vSnsPos.x);
		real_t fSnsLen = sqrt(vec2_sqlen(vSnsPos)) * fScale;

		// 계산된 거리를 바탕으로 각도 역산출
		int idx;
		for (idx = 0; idx < data_cnt - 1; ++idx) {
			if (fSnsLen < pPair[idx+1].y) {
				break;
			}
		}
		if (idx < data_cnt) {
			// 선형보간
			real_t fdy = (fSnsLen - pPair[idx].y) / (pPair[idx+1].y - pPair[idx].y);
			real_t fDegL = pPair[idx].x * (1.0f - fdy) + pPair[idx+1].x * fdy;
			real_t fRadL = fDegL * M_PI / 180.0;

			real_t fPhyLen = tan((double)fRadL) * calpnl_dist;
			vUV.x = fPhyLen * cos((double)fRadZ);
			vUV.y = fPhyLen * sin((double)fRadZ);


//      real_t fRadZ = (real_t)atan2(vPhyPos.y, vPhyPos.x);



/*

			vSnsPos.x = (fSnsLen * fScale * (real_t)cos(fRadZ) + vSrcHalf.x) / vSrcSize.x;
			vSnsPos.y = (fSnsLen * fScale * (real_t)sin(fRadZ) + vSrcHalf.y) / vSrcSize.y;
*/
			// 물리 좌표를 0~1 범위로 Normalize
			vUV.x = (vUV.x + vClbOrgn.x) / calpnl_size.x;
			vUV.y = (vUV.y + vClbOrgn.y) / calpnl_size.y;


		} else {
			Err_("Invalid idx");
		}
		dst[i] = vUV;
	}

	return count;
}


int TLensCalib::UndistortFisheyePoints(vec2 *dst, vec2 const *src, int count)
{
	double f[2];// = {228.28224489392343, 223.75652841334494};
	double c[2];// = {317.84618409608305, 251.77959922522803};

	f[0] = K[0];
	f[1] = K[4];
	c[0] = K[2];
	c[1] = K[5];

		for(int32_t i = 0; i < count; i++ )
		{
				vec2 pi = src[i];  // image point
		

		/// tangencial 오차 보정
		tangencialUndistort(pi.x, pi.y, &pi.x, &pi.y);

		pi.x *= (float32_t)(src_size.width - 1);
		pi.y *= (float32_t)(src_size.height - 1);


				vec2 pw = make_vec2((pi.x - c[0])/f[0], (pi.y - c[1])/f[1]);      // world point

				double scale = 1.0;

				double theta_d = sqrt(pw.x * pw.x + pw.y * pw.y);

				// the current camera model is only valid up to 180° FOV
				// for larger FOV the loop below does not converge
				// clip values so we still get plausible results for super fisheye images > 180°
				theta_d = Min(Max(-M_PI/2., theta_d), M_PI/2.);

				if (theta_d > 1e-8)
				{
						// compensate distortion iteratively
						double theta = theta_d;
						for(int j = 0; j < 10; j++ )
						{
								double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta6*theta2;
								theta = theta_d / (1 + D[0] * theta2 + D[1] * theta4 + D[2] * theta6 + D[3] * theta8);
						}

						scale = tan(theta) / theta_d;
				}

				vec2 pu = make_vec2(pw.x * scale, pw.y * scale); //undistorted point

		double _x, _y, _w;
		_x = K_new[0] * pu.x + K_new[1] * pu.y + K_new[2] * 1.0;
		_y = K_new[3] * pu.x + K_new[4] * pu.y + K_new[5] * 1.0;
		_w = K_new[6] * pu.x + K_new[7] * pu.y + K_new[8] * 1.0;

				// reproject
		vec2 fi;
		fi = make_vec2((float)(_x / _w), (float)(_y / _w));

		fi.x /= (float32_t)(sensor_size.width - 1);
		fi.y /= (float32_t)(sensor_size.height - 1);

		dst[i] = fi;

		}

	return count;
}

int TLensCalib::SamplePoly1ToArray(vec2 *dst, vec2u16 segments)
{
	vec2 *src = NULL, *pS;
	vec2 vUV;

	vec2 vInc = make_vec2(1.0 / (float)(segments.x), 1.0 / (float)(segments.y));

	int count = (segments.x + 1) * (segments.y + 1);
	DVEC_SETLENGTH(vec2, &src, count);

	pS = src;
	for (uint16_t iy = 0; iy <= segments.y; ++iy) {
		vUV.y = vInc.y * (float)iy;
		for (uint16_t ix = 0; ix <= segments.x; ++ix, pS++) {
			vUV.x = vInc.x * (float)ix;
			*pS = vUV;
		}
	}
	count = DistortPoly1Points(dst, src, count);

  
	DVEC_FREE(&src);
	return count;
}

int TLensCalib::SampleFisheyeToArray(vec2 *dst, vec2u16 segments)
{
	vec2 *src = NULL, *pS;
	vec2 vUV;

	vec2 vInc = make_vec2(1.0 / (float)(segments.x), 1.0 / (float)(segments.y));

	int count = (segments.x + 1) * (segments.y + 1);
	DVEC_SETLENGTH(vec2, &src, count);

	pS = src;
	for (uint16_t iy = 0; iy <= segments.y; ++iy) {
		vUV.y = vInc.y * (float)iy;
		for (uint16_t ix = 0; ix <= segments.x; ++ix, pS++) {
			vUV.x = vInc.x * (float)ix;
			*pS = vUV;
		}
	}
	count = DistortFisheyePoints(dst, src, count);

	DVEC_FREE(&src);
	return count;
}

int TLensCalib::SampleWideToArray(vec2 *dst, vec2u16 segments)
{
  vec2 *src = NULL, *pS;
  vec2 vUV;

  vec2 vInc = make_vec2(1.0 / (float)(segments.x), 1.0 / (float)(segments.y));

  int count = (segments.x + 1) * (segments.y + 1);
  DVEC_SETLENGTH(vec2, &src, count);

  pS = src;
  for (uint16_t iy = 0; iy <= segments.y; ++iy) {
    vUV.y = vInc.y * (float)iy;
    for (uint16_t ix = 0; ix <= segments.x; ++ix, pS++) {
      vUV.x = vInc.x * (float)ix;
      *pS = vUV;
    }
  }
  count = DistortWAPoints(dst, src, count);

  DVEC_FREE(&src);
  return count;
}
int TLensCalib::SampleLUTToArray(vec2 *dst, vec2u16 segments)
{
	const vec2 *src = (vec2*)GetData();
	vec2i32 map_pos;
	vec2 step, pos, P1, P2, P3, P4, uv, result;

	step.x = (float)(data_size.width-1) / segments.x;
	step.y = (float)(data_size.height-1) / segments.y;

	for (int iy = 0; iy <= segments.y; iy ++) {
		pos.y = iy * step.y;
		map_pos.y = (int)pos.y;

		for (int ix = 0; ix <= segments.x; ix ++) {
			pos.x = ix * step.x;
			map_pos.x = (int)pos.x;

			// 4점을 이용한 선형 보간
			P1 = src[map_pos.y * data_size.width + map_pos.x];
			if (ix == segments.x && iy == segments.y) {
				result = P1;
			} else if (ix == segments.x) {
				P2 = P1;
				P3 = src[(map_pos.y + 1) * data_size.width + map_pos.x];
				P4 = P3;
				uv.x = 0;
				uv.y = (pos.y - (float)map_pos.y) / step.y;
			} else if (iy == segments.y) {
				P3 = P1;
				P2 = src[map_pos.y * data_size.width + map_pos.x + 1];
				P4 = P2;
				uv.x = (pos.x - (float)map_pos.x) / step.x;
				uv.y = 0;
			} else {
				P2 = src[map_pos.y * data_size.width + map_pos.x + 1];
				P3 = src[(map_pos.y + 1) * data_size.width + map_pos.x];
				P4 = src[(map_pos.y + 1) * data_size.width + map_pos.x + 1];
				uv.x = (pos.x - (float)map_pos.x) / step.x;
				uv.y = (pos.y - (float)map_pos.y) / step.y;
			}

			result = GetXYFrom4CP(P1, P2, P3, P4, uv);

			//결과 저장 (정규화)
			result.x *= scale_x;
			result.y *= scale_y;

			dst[iy * (segments.x + 1) + ix] = result;
		}
	}
	return (segments.x + 1) * (segments.y + 1);
}

/**--------------------------------------------------------------------------
@brief 렌즈 캘리브레이션 데이터를 바탕으로 지정한 segment 간격으로 위치를
샘플링한다.

PNRV에서 사용하는 Polygon 형식의 렌즈 캘리브레이션 데이터를 생성하기 위해
픽셀 단위로 저장된 캘리브레이션 데이터로부터 일정 간격으로 건너뛰면서
포인트 클라우드를 생성한다. 서브픽셀 단위의 좌표는 주변 4점을 이용한
선형 보간을 통해 좌표를 계산한다.

@param [in ] dst      샘플링 된 정점 벡터를 저장할 배열. 배열은
											(segments.x + 1) * (segments.y + 1)의 크기로 준비되어야
											한다. 정점 벡터는 0~1로 정규화 되어 저장된다.
@param [in ] segments 가로 세로 방향으로 분할할 구간 수
@return 생성된 정점의 수

@author ksg
*/
int TLensCalib::SampleToArray(vec2 *dst, vec2u16 segments)
{
	//int i, i_end;
	int ret;

	if (GetData() == NULL) {
		return 0;
	}

	switch (type) {
		case make_fourcc('b','z','r','B'):
			return SampleBezierToArray(dst, segments);
		case make_fourcc('p','l','y','1'):
			return SamplePoly1ToArray(dst, segments);

	case make_fourcc('f','e','y','e'):
			return SampleFisheyeToArray(dst, segments);

	case make_fourcc('w','i','d','e'):
		return SampleWideToArray(dst, segments);
		//return SampleFisheyeToArray(dst, segments);

		default:
			ret = SampleLUTToArray(dst, segments);
			break;
	}

#if 0
	// uv_range에 따라 좌표 재조정
	int i, i_end;
	bool lFlipX, lFlipY;
	float ax, bx, ay, by;
	const vec2i32 tex_coord_size = {segments.x + 1, segments.y + 1};
	const int tex_coord_length = tex_coord_size.x * tex_coord_size.y;

	lFlipX = range.left > range.right;
	lFlipY = range.top > range.bottom;

	// 반전 여부와 상관 없이 스케일 변환이 제대로 이루어 지도록
	if (lFlipX) {
		ax = range.left - range.right;
		bx = range.right;
	} else {
		ax = range.right - range.left;
		bx = range.left;
	}

	// 반전 여부와 상관 없이 스케일 변환이 제대로 이루어 지도록
	if (lFlipY) {
		ay = range.top - range.bottom;
		by = range.bottom;
	} else {
		ay = range.bottom - range.top;
		by = range.top;
	}
		// 좌표 스케일 변환
		for (i = 0, i_end = DVEC_Length(tex_coord); i < i_end; i++) {
			tex_coord[i].x = ax * tex_coord[i].x + bx;
			tex_coord[i].y = ay * tex_coord[i].y + by;
		}

		if (lFlipX || lFlipY) {
			vec2 *pFinal = NULL;
			DVEC_SETLENGTH(vec2, &pFinal, tex_coord_length);
			int dst_x, dst_y;
			for (int y = 0; y <= segments.y; y++) {
				dst_y = lFlipY ? segments.y - y : y;
				for (int x = 0; x <= segments.x; x++) {
					dst_x = lFlipX ? segments.x - x : x;
					pFinal[dst_y * tex_coord_size.x + dst_x] = tex_coord[y * tex_coord_size.x + x];
				}
			}
			DVEC_FREE(&tex_coord);
			tex_coord = pFinal;
		}
	} else {
		TUVArrayInfo * pInfo = uv_manager.GetUVArrayOf(TUVParam(range, segments.x, segments.y));
		pInfo->AssignTo((DVEC*)&(tex_coord));
//    SetRectangularUVArray((real_t*)tex_coord, range.left, range.right, segments.x, range.top, range.bottom, segments.y, 2);
	}
	return tex_coord_length;
#endif



	if (scale_x < 0) {
		// Flip X
		for (int i = 0, i_end = (segments.x + 1) * (segments.y + 1); i < i_end; i++) {
			dst[i].x += 1.0;
		}
	}
	if (scale_y < 0) {
		// Flip Y
		for (int i = 0, i_end = (segments.x + 1) * (segments.y + 1); i < i_end; i++) {
			dst[i].y += 1.0;
		}
	}

	return ret;
}
