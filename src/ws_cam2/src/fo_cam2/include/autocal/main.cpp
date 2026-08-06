// OpenCV_Sample.cpp : �ܼ� ���� ���α׷��� ���� �������� �����մϴ�.
//

//#include "stdafx.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "SensorMath.h"

using namespace cv;
using namespace std;
TGuideLine *guideline;

TBoundBox *bb[4];
TCuboid *cb[4];

TBoundBox bb_fromLidar[4];

int openWindow();
int makeLUT();
int readLUT();
int drawTopview();
int getTopviewCoordinate();
int drawCam();
float *lut;
float *rev_lut;

int main(int argc, char* argv[])
{

	guideline = new TGuideLine();

	guideline->setLensParameter();

	bb[0] = new TBoundBox();
	bb[1] = new TBoundBox();
	bb[2] = new TBoundBox();
	bb[3] = new TBoundBox();

	cb[0] = new TCuboid();
	cb[1] = new TCuboid();
	cb[2] = new TCuboid();
	cb[3] = new TCuboid();

	bb[3]->setByLTRB(120, 600, 170, 763);
	bb[0]->setByLTRB(688, 694, 706, 724);
	bb[2]->setByLTRB(1512, 626, 1547, 713);
	bb[1]->setByLTRB(1853, 564, 1903, 721);

	cb[0]->setCenter(6.122f, 10.361f, 0.604f);
	cb[0]->setSize(0.727f, 0.705f, 1.806f);

	cb[1]->setCenter(26.071f, 10.095f, 0.659f);
	cb[1]->setSize(2.195f, 0.71f, 1.353f);

	cb[2]->setCenter(16.599f, -9.207f, 0.347f);
	cb[2]->setSize(2.118f, 0.703f, 1.377f);

	cb[3]->setCenter(7.65f, -7.326f, 0.234f);
	cb[3]->setSize(0.413f, 0.399f, 1.079f);

	//TCuboid **a; 
	//a = cb;
	//a[0]->frontFoot2D.x = 4.2f;
	//sort by left ��ǥ, ��������
	TBoundBox *temp;
	for (int i = 3; i >= 0; i--)
	{
 		for (int j = 0; j < i; j++)
		{
			if (bb[j]->left > bb[j + 1]->left)
			{
				temp = bb[j];
				bb[j] = bb[j + 1];
				bb[j + 1] = temp;
			}
		}
	}
	for (int i = 0; i < 4; i++)
	{
		cout << "bb[" << i << "].left = " << bb[i]->left << endl;
		guideline->transCoordinateCam2Perspective(&bb[i]->foot, &bb[i]->foot_undistort, 1);
		cout << "foot=" << bb[i]->foot.x << "," << bb[i]->foot.y << " foot_undistort=" << bb[i]->foot_undistort.x << "," << bb[i]->foot_undistort.y << endl;
	}

	//���̴� �ν� ������ ����
	TCuboid *temp_c;
	for (int i = 3; i >= 0; i--)
	{
		for (int j = 0; j < i; j++)
		{
			if (cb[j]->angle < cb[j + 1]->angle)
			{
				temp_c = cb[j];
				cb[j] = cb[j + 1];
				cb[j + 1] = temp_c;
			}
		}
	}
	for (int i = 0; i < 4; i++)
	{
		cout << "cb[" << i << "].x = " << cb[i]->center.x << endl;
		cout << "foot = " << cb[i]->frontFoot2D.x << "," << cb[i]->frontFoot2D.y << endl;
	}


	guideline->setPerspectiveMatrix(bb, cb);

	bb_fromLidar[0].foot_topview.x = 5.759f;
	bb_fromLidar[0].foot_topview.y = 10.361f;
	bb_fromLidar[1].foot_topview.x = 24.973f;
	bb_fromLidar[1].foot_topview.y = 10.095f;
	bb_fromLidar[2].foot_topview.x = 15.54f;
	bb_fromLidar[2].foot_topview.y = -9.207f;
	bb_fromLidar[3].foot_topview.x = 7.444f;
	bb_fromLidar[3].foot_topview.y = -7.326f;

	for (int i = 0; i < 4; i++)
	{
		guideline->transCoordinateTop2Perspective(&bb_fromLidar[i].foot_topview, &bb_fromLidar[i].foot_undistort, 1);
		guideline->transCoordinatePerspective2Cam(&bb_fromLidar[i].foot_undistort, &bb_fromLidar[i].foot, 1);
	}

	drawCam();

#if 0 //ī�޶� to ž�� ����

	pFrom[0].x = 244.0f;
	pFrom[0].y = 782.0f;
	pFrom[1].x = 1790.0f;
	pFrom[1].y = 775.0f;
	pFrom[2].x = 1255.0f;
	pFrom[2].y = 823.0f;
	pFrom[3].x = 747.0f;
	pFrom[3].y = 825.0f;

	guideline->transCoordinateCam2Perspective(pFrom, pTo, 4);

	guideline->transCoordinatePerspective2Top(pTo, pTopview, 4);
#endif

#if 0 //ž�� to ī�޶� ����

	pGround[0].x = -17.3f;
	pGround[0].y = 10.0f;
	pGround[1].x = 17.3f;
	pGround[1].y = 10.0f;
	pGround[2].x = 10.0f;
	pGround[2].y = 30.0f;
	pGround[3].x = -10.0f;
	pGround[3].y = 30.0f;

	int i;
	for (i = 0; i < 4; i++)
	{
		pTopview[i].x = pGround[i].x * 20.0f + 400.0f;
		pTopview[i].y = 700 - (pGround[i].y * 20.0f);
	}
	guideline->transCoordinateTop2Perspective(pTopview, pUndistort, 4);
	guideline->transCoordinatePerspective2Cam(pUndistort, pCam, 4);


#endif
	//makeLUT(); //LUT ���� ����
	
	//readLUT();
	//drawTopview();
	//getTopviewCoordinate();

	//free(lut);
	//free(rev_lut);

	getchar();
	return 0;
}

void drawNPoints(cv::Mat img, vec2f *p, int radius, int thickness, cv::Scalar color, int cnt)
{
	int i;
	float x, y;
	int px, py;
	if (p != NULL)
	{
		for (i = 0; i < cnt; i++)
		{
			x = p[i].x;
			y = p[i].y;
			px = (int)(x + 0.5f);
			py = (int)(y + 0.5f);
			circle(img, Point(px, py), radius, color, thickness, 8, 0);
		}
	}

}

#define LUT_COLS  81
#define LUT_ROWS  61
#define REV_LUT_COLS 193
#define REV_LUT_ROWS 31
/**
	@brief LUT ���� ����
*/
int makeLUT()
{

#if 0  //ž�信�� ī�޶� ��ǥ�� ��ȯ�ϴ� LUT
	int i, j;
	vec2f pin, pout;
	vec2f ptemp;
	FILE *fp;
	fp = fopen("top2cam.txt", "w");

	for (i = 0; i < LUT_ROWS; i++)
	{
		for (j = 0; j < LUT_COLS; j++)
		{
			pin.x = j * 10.0f;
			pin.y = i * 10.0f;
			guideline->transCoordinateTop2Perspective(&pin, &ptemp, 1);
			guideline->transCoordinatePerspective2Cam(&ptemp, &pout, 1);
			fprintf(fp, "%.2f,%.2f\n", pout.x, pout.y);
		}
	}
	fclose(fp);
#endif

#if 0 //ī�޶� ��ǥ���� ž�� ��ǥ�� ��ȯ�ϴ� LUT
	int i, j;
	vec2f pin, pout;
	vec2f ptemp;
	FILE *fp;
	fp = fopen("cam2top.txt", "w");

	for (i = 0; i < REV_LUT_ROWS; i++)
	{
		for (j = 0; j < REV_LUT_COLS; j++)
		{
			pin.x = j * 10.0f;
			pin.y = i * 10.0f + 700;
			guideline->transCoordinateCam2Perspective(&pin, &ptemp, 1);
			guideline->transCoordinatePerspective2Top(&ptemp, &pout, 1);
			fprintf(fp, "%.2f,%.2f\n", pout.x, pout.y);
		}
	}
	fclose(fp);
#endif

	return 0;
}
/**
	@brief LUT ���� �б�
*/
int readLUT()
{
	lut = (float *)malloc(sizeof(float)* LUT_COLS * LUT_ROWS * 2);
	FILE *fp;
	fp = fopen("top2cam.txt", "r");
	int i;
	float x, y;
	for (i = 0; i < LUT_ROWS * LUT_COLS; i++)
	{
		fscanf(fp, "%f,%f\n", &x, &y);
		lut[i * 2] = x;
		lut[i * 2 + 1] = y;
	}
	fclose(fp);
	fp = 0;

	rev_lut = (float *)malloc(sizeof(float)* REV_LUT_COLS * REV_LUT_ROWS * 2);
	fp = fopen("cam2top.txt", "r");
	for (i = 0; i < REV_LUT_ROWS * REV_LUT_COLS; i++)
	{
		fscanf(fp, "%f,%f\n", &x, &y);
		rev_lut[i * 2] = x;
		rev_lut[i * 2 + 1] = y;
	}
	fclose(fp);

	return 0;
}
/**
	@brief LUT �����͸� �̿��ؼ� ž�� �̹��� ����
*/
int drawTopview()
{
	Mat img = imread("input.png", 1);
	Mat frame = Mat::zeros(600, 800, CV_8UC3);

	int i, j;
	int idx_x, idx_y;
	int lut_idx[4];
	unsigned char r[4], g[4], b[4];
	float xr, yr;
	
	float ox, oy;
	int oix, oiy;
	int oindex[4];
	float ox_r, oy_r;
	int oR[4], oG[4], oB[4];
	

	for (i = 0; i < 600; i++)
	{
		for (j = 0; j < 800; j++)
		{
			//frame.data[(i * 800 + j) * 3] = 255;
			//frame.data[(i * 800 + j) * 3 + 1] = 255;
			//frame.data[(i * 800 + j) * 3 + 2] = 0;
			idx_x = j / 10;
			idx_y = i / 10;
			lut_idx[0] = idx_y * LUT_COLS + idx_x;
			lut_idx[1] = idx_y * LUT_COLS + idx_x + 1;
			lut_idx[2] = (idx_y + 1) * LUT_COLS + idx_x;
			lut_idx[3] = (idx_y + 1) * LUT_COLS + idx_x + 1;
			xr = (j - (idx_x * 10)) / 10.0f;
			yr = (i - (idx_y * 10)) / 10.0f;


			//// 0
			ox = lut[lut_idx[0] * 2];
			oy = lut[lut_idx[0] * 2 + 1];

			oix = (int)ox;
			ox_r = ox - oix;
			oiy = (int)oy;
			oy_r = oy - oiy;
			
			oindex[0] = oiy * CAM_W + oix;
			oindex[1] = oiy * CAM_W + oix + 1;
			oindex[2] = (oiy + 1) * CAM_W + oix;
			oindex[3] = (oiy + 1) * CAM_W + oix + 1;

			oR[0] = (int)img.data[oindex[0] * 3];
			oG[0] = (int)img.data[oindex[0] * 3 + 1];
			oB[0] = (int)img.data[oindex[0] * 3 + 2];
			oR[1] = (int)img.data[oindex[1] * 3];
			oG[1] = (int)img.data[oindex[1] * 3 + 1];
			oB[1] = (int)img.data[oindex[1] * 3 + 2];
			oR[2] = (int)img.data[oindex[2] * 3];
			oG[2] = (int)img.data[oindex[2] * 3 + 1];
			oB[2] = (int)img.data[oindex[2] * 3 + 2];
			oR[3] = (int)img.data[oindex[3] * 3];
			oG[3] = (int)img.data[oindex[3] * 3 + 1];
			oB[3] = (int)img.data[oindex[3] * 3 + 2];

			r[0] = (oR[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oR[1] * (ox_r) * (1.0f - oy_r)) +
				(oR[2] * (1.0f - ox_r) * (oy_r)) +
				(oR[3] * (ox_r) * (oy_r));
			g[0] = (oG[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oG[1] * (ox_r)* (1.0f - oy_r)) +
				(oG[2] * (1.0f - ox_r) * (oy_r)) +
				(oG[3] * (ox_r)* (oy_r));
			b[0] = (oB[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oB[1] * (ox_r)* (1.0f - oy_r)) +
				(oB[2] * (1.0f - ox_r) * (oy_r)) +
				(oB[3] * (ox_r)* (oy_r));


			//// 1
			ox = lut[lut_idx[1] * 2];
			oy = lut[lut_idx[1] * 2 + 1];

			oix = (int)ox;
			ox_r = ox - oix;
			oiy = (int)oy;
			oy_r = oy - oiy;

			oindex[0] = oiy * CAM_W + oix;
			oindex[1] = oiy * CAM_W + oix + 1;
			oindex[2] = (oiy + 1) * CAM_W + oix;
			oindex[3] = (oiy + 1) * CAM_W + oix + 1;

			oR[0] = (int)img.data[oindex[0] * 3];
			oG[0] = (int)img.data[oindex[0] * 3 + 1];
			oB[0] = (int)img.data[oindex[0] * 3 + 2];
			oR[1] = (int)img.data[oindex[1] * 3];
			oG[1] = (int)img.data[oindex[1] * 3 + 1];
			oB[1] = (int)img.data[oindex[1] * 3 + 2];
			oR[2] = (int)img.data[oindex[2] * 3];
			oG[2] = (int)img.data[oindex[2] * 3 + 1];
			oB[2] = (int)img.data[oindex[2] * 3 + 2];
			oR[3] = (int)img.data[oindex[3] * 3];
			oG[3] = (int)img.data[oindex[3] * 3 + 1];
			oB[3] = (int)img.data[oindex[3] * 3 + 2];

			r[1] = (oR[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oR[1] * (ox_r)* (1.0f - oy_r)) +
				(oR[2] * (1.0f - ox_r) * (oy_r)) +
				(oR[3] * (ox_r)* (oy_r));
			g[1] = (oG[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oG[1] * (ox_r)* (1.0f - oy_r)) +
				(oG[2] * (1.0f - ox_r) * (oy_r)) +
				(oG[3] * (ox_r)* (oy_r));
			b[1] = (oB[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oB[1] * (ox_r)* (1.0f - oy_r)) +
				(oB[2] * (1.0f - ox_r) * (oy_r)) +
				(oB[3] * (ox_r)* (oy_r));

			//// 2
			ox = lut[lut_idx[2] * 2];
			oy = lut[lut_idx[2] * 2 + 1];

			oix = (int)ox;
			ox_r = ox - oix;
			oiy = (int)oy;
			oy_r = oy - oiy;

			oindex[0] = oiy * CAM_W + oix;
			oindex[1] = oiy * CAM_W + oix + 1;
			oindex[2] = (oiy + 1) * CAM_W + oix;
			oindex[3] = (oiy + 1) * CAM_W + oix + 1;

			oR[0] = (int)img.data[oindex[0] * 3];
			oG[0] = (int)img.data[oindex[0] * 3 + 1];
			oB[0] = (int)img.data[oindex[0] * 3 + 2];
			oR[1] = (int)img.data[oindex[1] * 3];
			oG[1] = (int)img.data[oindex[1] * 3 + 1];
			oB[1] = (int)img.data[oindex[1] * 3 + 2];
			oR[2] = (int)img.data[oindex[2] * 3];
			oG[2] = (int)img.data[oindex[2] * 3 + 1];
			oB[2] = (int)img.data[oindex[2] * 3 + 2];
			oR[3] = (int)img.data[oindex[3] * 3];
			oG[3] = (int)img.data[oindex[3] * 3 + 1];
			oB[3] = (int)img.data[oindex[3] * 3 + 2];

			r[2] = (oR[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oR[1] * (ox_r)* (1.0f - oy_r)) +
				(oR[2] * (1.0f - ox_r) * (oy_r)) +
				(oR[3] * (ox_r)* (oy_r));
			g[2] = (oG[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oG[1] * (ox_r)* (1.0f - oy_r)) +
				(oG[2] * (1.0f - ox_r) * (oy_r)) +
				(oG[3] * (ox_r)* (oy_r));
			b[2] = (oB[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oB[1] * (ox_r)* (1.0f - oy_r)) +
				(oB[2] * (1.0f - ox_r) * (oy_r)) +
				(oB[3] * (ox_r)* (oy_r));

			//// 3
			ox = lut[lut_idx[3] * 2];
			oy = lut[lut_idx[3] * 2 + 1];

			oix = (int)ox;
			ox_r = ox - oix;
			oiy = (int)oy;
			oy_r = oy - oiy;

			oindex[0] = oiy * CAM_W + oix;
			oindex[1] = oiy * CAM_W + oix + 1;
			oindex[2] = (oiy + 1) * CAM_W + oix;
			oindex[3] = (oiy + 1) * CAM_W + oix + 1;

			oR[0] = (int)img.data[oindex[0] * 3];
			oG[0] = (int)img.data[oindex[0] * 3 + 1];
			oB[0] = (int)img.data[oindex[0] * 3 + 2];
			oR[1] = (int)img.data[oindex[1] * 3];
			oG[1] = (int)img.data[oindex[1] * 3 + 1];
			oB[1] = (int)img.data[oindex[1] * 3 + 2];
			oR[2] = (int)img.data[oindex[2] * 3];
			oG[2] = (int)img.data[oindex[2] * 3 + 1];
			oB[2] = (int)img.data[oindex[2] * 3 + 2];
			oR[3] = (int)img.data[oindex[3] * 3];
			oG[3] = (int)img.data[oindex[3] * 3 + 1];
			oB[3] = (int)img.data[oindex[3] * 3 + 2];

			r[3] = (oR[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oR[1] * (ox_r)* (1.0f - oy_r)) +
				(oR[2] * (1.0f - ox_r) * (oy_r)) +
				(oR[3] * (ox_r)* (oy_r));
			g[3] = (oG[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oG[1] * (ox_r)* (1.0f - oy_r)) +
				(oG[2] * (1.0f - ox_r) * (oy_r)) +
				(oG[3] * (ox_r)* (oy_r));
			b[3] = (oB[0] * (1.0f - ox_r) * (1.0f - oy_r)) +
				(oB[1] * (ox_r)* (1.0f - oy_r)) +
				(oB[2] * (1.0f - ox_r) * (oy_r)) +
				(oB[3] * (ox_r)* (oy_r));


			///// ���� RGB
			frame.data[(i * 800 + j) * 3] = (r[0] * (1.0f - xr) * (1.0f - yr)) +
				(r[1] * (xr) * (1.0f - yr)) +
				(r[2] * (1.0f - xr) * (yr)) +
				(r[3] * (xr) * (yr));

			frame.data[(i * 800 + j) * 3 + 1] = (g[0] * (1.0f - xr) * (1.0f - yr)) +
				(g[1] * (xr)* (1.0f - yr)) +
				(g[2] * (1.0f - xr) * (yr)) +
				(g[3] * (xr)* (yr));

			frame.data[(i * 800 + j) * 3 + 2] = (b[0] * (1.0f - xr) * (1.0f - yr)) +
				(b[1] * (xr)* (1.0f - yr)) +
				(b[2] * (1.0f - xr) * (yr)) +
				(b[3] * (xr)* (yr));

		}
	}
	
	while (1)
	{

		imshow("Frame", frame);
		imshow("image", img);

		char c = waitKey(25);
		if (c == 27) break; //ESC
	}
	return 0;
}

int drawCam()
{
	int i;
	Mat img = imread("s2.png", 1);

	for (i = 0; i < 4; i++)
	{
		int px, py;
		px = (int)(bb_fromLidar[i].foot.x + 0.5f);
		py = (int)(bb_fromLidar[i].foot.y + 0.5f);
		circle(img, Point(px, py), 5, Scalar(255, 255, 0), 2, 8, 0);
	}

	while (1)
	{
		imshow("image", img);

		char c = waitKey(25);
		if (c == 27) break; //ESC
	}
	return 0;
}

int getTopviewCoordinate()
{
	float img_X, img_Y;
	int idx_x, idx_y;
	int lut_idx[4];
	float xr, yr;
	vec2f ptop[4];
	vec2f dtop;
	vec2f pmeter;

	img_X = 1256.0f;
	img_Y = 828.0f;


	idx_x = (int)(img_X / 10.0f);
	idx_y = (int)((img_Y - 700.0f) / 10.0f);
	xr = (img_X - (idx_x * 10.0f)) / 10.0f;
	yr = (img_Y - 700.0f - (idx_y * 10.0f)) / 10.0f;
	lut_idx[0] = idx_y * REV_LUT_COLS + idx_x;
	lut_idx[1] = idx_y * REV_LUT_COLS + idx_x + 1;
	lut_idx[2] = (idx_y + 1) * REV_LUT_COLS + idx_x;
	lut_idx[3] = (idx_y + 1) * REV_LUT_COLS + idx_x + 1;

	ptop[0].x = rev_lut[lut_idx[0] * 2];
	ptop[0].y = rev_lut[lut_idx[0] * 2 + 1];
	ptop[1].x = rev_lut[lut_idx[1] * 2];
	ptop[1].y = rev_lut[lut_idx[1] * 2 + 1];
	ptop[2].x = rev_lut[lut_idx[2] * 2];
	ptop[2].y = rev_lut[lut_idx[2] * 2 + 1];
	ptop[3].x = rev_lut[lut_idx[3] * 2];
	ptop[3].y = rev_lut[lut_idx[3] * 2 + 1];

	dtop.x = ptop[0].x * (1.0f - xr) * (1.0f - yr) +
		ptop[1].x * (xr) * (1.0f - yr) +
		ptop[2].x * (1.0f - xr) * (yr) +
		ptop[3].x * (xr) * (yr);
	dtop.y = ptop[0].y * (1.0f - xr) * (1.0f - yr) +
		ptop[1].y * (xr)* (1.0f - yr) +
		ptop[2].y * (1.0f - xr) * (yr) +
		ptop[3].y * (xr)* (yr);

	pmeter.x = (dtop.x - 400.0f) / 20.0f;
	pmeter.y = (700.0f - dtop.y) / 20.0f;


	return 0;
}


