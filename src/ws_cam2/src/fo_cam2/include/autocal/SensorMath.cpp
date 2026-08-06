#include "SensorMath.h"
#include <math.h>


TGuideLine::TGuideLine()
{


}

TGuideLine::~TGuideLine()
{


}

void transCoordMat(double *mat, vec2f *in, vec2f *out, int cnt)
{
	int i;
	double X, Y, Z;
	float tx = 0.0f, ty = 2.0f;
	float px, py;

	for (i = 0; i < cnt; i++)
	{
		tx = in[i].x;
		ty = in[i].y;
		X = (mat[0] * tx) + (mat[1] * ty) + mat[2];
		Y = (mat[3] * tx) + (mat[4] * ty) + mat[5];
		Z = (mat[6] * tx) + (mat[7] * ty) + mat[8];


		if (Z != 0.0f) //0.0으로 나누는 것을 방지
		{

			px = (float32_t)(X / Z);
			py = (float32_t)(Y / Z);
		}
		else {
			px = 0.0f;
			py = 0.0f;
		}
		out[i].x = px;
		out[i].y = py;
	}

}

/**
	@brief 탑뷰 좌표에서 왜곡보정 이미지 좌표로 변환
*/
void TGuideLine::transCoordinateTop2Perspective(vec2f *pin, vec2f *pout, int cnt)
{
	int i;
	/*double X, Y, Z;
	float tx = 0.0f, ty = 2.0f;
	float px, py;*/
	double outmat[9];
	outmat[0] = projMat->var[0][0];
	outmat[1] = projMat->var[0][1];
	outmat[2] = projMat->var[0][2];
	outmat[3] = projMat->var[1][0];
	outmat[4] = projMat->var[1][1];
	outmat[5] = projMat->var[1][2];
	outmat[6] = projMat->var[2][0];
	outmat[7] = projMat->var[2][1];
	outmat[8] = projMat->var[2][2];

	transCoordMat(outmat, pin, pout, cnt);
	
}

/**
@brief 왜곡보정 이미지 좌표에서 탑뷰 좌표로 변환
*/
void TGuideLine::transCoordinatePerspective2Top(vec2f *pin, vec2f *pout, int cnt)
{
	int i;
	/*double X, Y, Z;
	float tx = 0.0f, ty = 2.0f;
	float px, py;*/
	double outmat[9];
	outmat[0] = projMatRev->var[0][0];
	outmat[1] = projMatRev->var[0][1];
	outmat[2] = projMatRev->var[0][2];
	outmat[3] = projMatRev->var[1][0];
	outmat[4] = projMatRev->var[1][1];
	outmat[5] = projMatRev->var[1][2];
	outmat[6] = projMatRev->var[2][0];
	outmat[7] = projMatRev->var[2][1];
	outmat[8] = projMatRev->var[2][2];

	transCoordMat(outmat, pin, pout, cnt);

}

/**
	@brief 왜곡보정이미지 좌표에서 왜곡이 있는 카메라 원본이미지 좌표로 변환
*/
void TGuideLine::transCoordinatePerspective2Cam(vec2f *pin, vec2f *pout, int cnt)
{
	int i;
	vec2f *tempVin;
	vec2f *tempVout;

	tempVin = (vec2f *)malloc(sizeof(vec2f)* cnt);
	tempVout = (vec2f *)malloc(sizeof(vec2f)* cnt);

	for (i = 0; i < cnt; i++)  //normalize
	{
		tempVin[i].x = pin[i].x / (float)UNDISTORT_IMG_W;
		tempVin[i].y = pin[i].y / (float)UNDISTORT_IMG_H;
	}
	lense_calib->DistortPoints(tempVout, tempVin, cnt);
	for (i = 0; i < cnt; i++)  //normalize
	{
		pout[i].x = tempVout[i].x * (float)CAM_W;
		pout[i].y = tempVout[i].y * (float)CAM_H;
	}

	free(tempVin);
	free(tempVout);
}

/**
@brief 카메라 원본이미지 좌표에서 왜곡보정이미지 좌표로 변환
*/
void TGuideLine::transCoordinateCam2Perspective(vec2f *pin, vec2f *pout, int cnt)
{
	int i;
	vec2f *tempVin;
	vec2f *tempVout;

	tempVin = (vec2f *)malloc(sizeof(vec2f)* cnt);
	tempVout = (vec2f *)malloc(sizeof(vec2f)* cnt);

	for (i = 0; i < cnt; i++)  //normalize
	{
		tempVin[i].x = pin[i].x / (float)CAM_W;
		tempVin[i].y = pin[i].y / (float)CAM_H;
	}
	lense_calib->UndistortPoints(tempVout, tempVin, cnt);
	for (i = 0; i < cnt; i++)  //normalize
	{
		pout[i].x = tempVout[i].x * (float)UNDISTORT_IMG_W;
		pout[i].y = tempVout[i].y * (float)UNDISTORT_IMG_H;
	}

	free(tempVin);
	free(tempVout);
}


void TGuideLine::setPerspectiveMatrix(TBoundBox **bb_img, TCuboid **cb_lidar)
{
	vec2d tgt[4], src[4];
#if 0
	/// 탑뷰쪽 좌표
	src[0].x = 10;   //1
	src[0].y = -17.3;  
	src[1].x = 10;  //3
	src[1].y = 17.3;
	src[2].x = 30;  //9
	src[2].y = 10;
	src[3].x = 30;  //7
	src[3].y = -10;

	/// 렌즈왜곡 보정쪽 좌표
	tgt[0].x = 53.76;
	tgt[0].y = 190.96;
	tgt[1].x = 442.42;
	tgt[1].y = 183.12;
	tgt[2].x = 302.33;
	tgt[2].y = 176.42;
	tgt[3].x = 220.06;
	tgt[3].y = 177.94;
#else
	
	
	/// 탑뷰쪽 좌표
	src[0].x = cb_lidar[0]->frontFoot2D.x;   //
	src[0].y = cb_lidar[0]->frontFoot2D.y;
	src[1].x = cb_lidar[1]->frontFoot2D.x;  //
	src[1].y = cb_lidar[1]->frontFoot2D.y;
	src[2].x = cb_lidar[2]->frontFoot2D.x;  //
	src[2].y = cb_lidar[2]->frontFoot2D.y;
	src[3].x = cb_lidar[3]->frontFoot2D.x;  //
	src[3].y = cb_lidar[3]->frontFoot2D.y;

	///// 렌즈왜곡 보정쪽 좌표
	tgt[0].x = bb_img[0]->foot_undistort.x;
	tgt[0].y = bb_img[0]->foot_undistort.y;
	tgt[1].x = bb_img[1]->foot_undistort.x;
	tgt[1].y = bb_img[1]->foot_undistort.y;
	tgt[2].x = bb_img[2]->foot_undistort.x;
	tgt[2].y = bb_img[2]->foot_undistort.y;
	tgt[3].x = bb_img[3]->foot_undistort.x;
	tgt[3].y = bb_img[3]->foot_undistort.y;
#endif

	projMat = projection_matrix2(tgt, src);

	projMatRev = projection_matrix2(src, tgt);

}

void TGuideLine::setLensParameter()
{
	lense_calib = new TLensCalib();
	//lense_calib->file_name = "gvfo.calib";
	lense_calib->file_name = "gvfo_feye.calib";
	lense_calib->Load();
}

void TGuideLine::calcTopviewPoints(float steer_angle)
{

}

vec2f* TGuideLine::getVLine(int index)
{
	if (index == 0 || index == 1)
	{
		return gptVtop[index];
	}
	else
	{
		return NULL;
	}
}

vec2f* TGuideLine::getHLine(int index)
{
	if (index == 0 || index == 1 || index == 2)
	{
		return gptHtop[index];
	}
	else
	{
		return NULL;
	}
}

vec2f* TGuideLine::getPerspectiveVLine(int index)
{
	if (index == 0 || index == 1)
	{
		return gptVperspective[index];
	}
	else
	{
		return NULL;
	}
}

vec2f* TGuideLine::getPerspectiveHLine(int index)
{
	if (index == 0 || index == 1 || index == 2)
	{
		return gptHperspective[index];
	}
	else
	{
		return NULL;
	}
}

vec2f* TGuideLine::getCamVLine(int index)
{
	if (index == 0 || index == 1)
	{
		return gptVcam[index];
	}
	else
	{
		return NULL;
	}
}

vec2f* TGuideLine::getCamHLine(int index)
{
	if (index == 0 || index == 1 || index == 2)
	{
		return gptHcam[index];
	}
	else
	{
		return NULL;
	}
}


/**
	@brief 생성자
*/
TBoundBox::TBoundBox()
{
	left = 0.0f;
	right = 0.0f;
	top = 0.0f;
	bottom = 0.0f;
	width = 0.0f;
	height = 0.0f;
	center.x = 0.0f;
	center.y = 0.0f;
	foot.x = 0.0f;
	foot.y = 0.0f;
}

/**
@brief
*/
TBoundBox::~TBoundBox()
{

}

/**
	@brief set bound box value with LTRB
*/
void TBoundBox::setByLTRB(float L, float T, float R, float B)
{
	left = L;
	right = R;
	top = T;
	bottom = B;

	width = right - left;
	height = abs(bottom - top);
	center.x = (left + right) / 2.0f;
	center.y = (top + bottom) / 2.0f;
	foot.x = (left + right) / 2.0f;
	foot.y = bottom;

}

/**
@brief set bound box value with LT and width, height
*/
void TBoundBox::setByCornerWH(float L, float T, float W, float H)
{
	left = L;
	top = T;
	width = W;
	height = H;

	right = left + width;
	bottom = top + height;
	center.x = (left + right) / 2.0f;
	center.y = (top + bottom) / 2.0f;
	foot.x = (left + right) / 2.0f;
	foot.y = bottom;
}

/**
	@brief 중심점 좌표 반환
*/
vec2f TBoundBox::getCenter()
{
	return center;
}

/**
	@brief bottom의 중심점 좌표 반환
*/
vec2f TBoundBox::getFoot()
{
	return foot;
}

/**
	@brief 생성자
*/
TCuboid::TCuboid()
{
	center.x = 0.0f;
	center.y = 0.0f;
	center.z = 0.0f;
	size.x = 0.0f;
	size.y = 0.0f;
	size.z = 0.0f;
}

TCuboid::~TCuboid()
{

}

/**
	@brief cuboid의 중심 좌표 설정
*/
void TCuboid::setCenter(float px, float py, float pz)
{
	center.x = px;
	center.y = py;
	center.z = pz;
	frontFoot.x = center.x - (size.x / 2.0f);
	frontFoot.y = center.y;
	frontFoot.z = center.z - (size.z / 2.0f);

	//영점 위치에서 바라보는 각도 계산, 자동 캘리브레이션 시 정렬 위해
	angle = atan2(center.y, center.x);

}

/**
	@brief cuboid의 크기 설정
*/
void TCuboid::setSize(float sx, float sy, float sz)
{
	size.x = sx;
	size.y = sy;
	center.z = sz;
	frontFoot.x = center.x - (size.x / 2.0f);
	frontFoot.y = center.y;
	frontFoot.z = center.z - (size.z / 2.0f);

	frontFoot2D.x = frontFoot.x;
	frontFoot2D.y = frontFoot.y;
}

/**
	@brief 정면 바닥 중심 좌표 반환
*/
vec3f TCuboid::getFrontFoot()
{
	return frontFoot;
}

/**
	@brief 정면 바닥 중심 좌표 2차원으로 반환(z 제외)
*/
vec2f TCuboid::getFrontFoot2D()
{
	return frontFoot2D;
}


