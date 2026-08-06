#ifndef _GUIDELINE_H_
#define _GUIDELINE_H_

#include "wafl3.h"
#include "math/Projection.h"
#include "vision/lens_calib.h"
#define UNDISTORT_IMG_W 512
#define UNDISTORT_IMG_H 256
#define CAM_W 1920
#define CAM_H 1080

#define NP 20 //세로선 segment 수
#define NH 10 //가로선 segment 수

/**
@brief 2차원 bounding box의 클래스
*/
class TBoundBox
{
private:

protected:

public:
	float left;
	float right;
	float top;
	float bottom;
	float width;
	float height;
	vec2f center;
	vec2f foot;
	vec2f foot_undistort;
	vec2f foot_topview;

	TBoundBox();
	~TBoundBox();
	void setByLTRB(float left, float top, float right, float bottom);
	void setByCornerWH(float left, float top, float width, float height);
	vec2f getCenter();
	vec2f getFoot();

};

/**
@brief 3차원 cuboid의 클래스
*/
class TCuboid
{
private:

protected:

public:
	vec3f center;
	vec3f frontFoot;
	vec3f size;
	vec2f frontFoot2D;
	vec2f frontFoot2D_undistort;
	vec2f frontFoot2D_image;
	float angle;

	TCuboid();
	~TCuboid();
	void setCenter(float x, float y, float z);
	void setSize(float sx, float sy, float sz);
	vec3f getFrontFoot();
	vec2f getFrontFoot2D();

};


class TGuideLine
{
private:
		
protected:
	vec2f gptVtop[2][NP + 1];  //탑뷰 세로선 위의 점들 좌표
	vec2f gptHtop[3][NH + 1]; //탑뷰 가로선 위의 점들 좌표
	vec2f gptVperspective[2][NP + 1];  //렌즈왜곡보정뷰 세로선 위의 점들 좌표
	vec2f gptHperspective[3][NH + 1]; //렌즈왜곡보정뷰 가로선 위의 점들 좌표
	vec2f gptVcam[2][NP + 1];  //카메라원본이미지 세로선 위의 점들 좌표
	vec2f gptHcam[3][NH + 1]; //카메라원본이미지 가로선 위의 점들 좌표
	vec2f gptVscreen[2][NP + 1];  //출력 화면 세로선 위의 점들 좌표
	vec2f gptHscreen[3][NH + 1];  //출력 화면 가로선 위의 점들 좌표
	vec2f tempVin[NP + 1];
	vec2f tempVout[NP + 1];
	vec2f tempHin[NH + 1];
	vec2f tempHout[NH + 1];
	matrix *projMat;
	matrix *projMatRev;
	TLensCalib *lense_calib;

public:
	TGuideLine();
	~TGuideLine();

	float steer;

	virtual void calcTopviewPoints(float steer_angle);
	vec2f* getVLine(int index);
	vec2f* getHLine(int index);
	vec2f* getPerspectiveVLine(int index);
	vec2f* getPerspectiveHLine(int index);
	vec2f* getCamVLine(int index);
	vec2f* getCamHLine(int index);
	void setPerspectiveMatrix(TBoundBox **bb_img, TCuboid **cb_lidar);
	void setLensParameter();
	void transCoordinateTop2Perspective(vec2f *pin, vec2f *pout, int cnt);
	void transCoordinatePerspective2Top(vec2f *pin, vec2f *pout, int cnt);
	
	void transCoordinatePerspective2Cam(vec2f *pin, vec2f *pout, int cnt);
	void transCoordinateCam2Perspective(vec2f *pin, vec2f *pout, int cnt);


};



#endif

