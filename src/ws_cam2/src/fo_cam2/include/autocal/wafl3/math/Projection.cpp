#ifdef WIN32
//include "stdafx.h"
#endif
#include "Projection.h"



matrix* projection_matrix3( double* x, double* y, double* X, double* Y)
{
	
	matrix* a;
	matrix* b;
	matrix* c;
	       
	matrix* a_inv;
	matrix* projection;
	       
	a = matrix_new( 8, 8 );
	b = matrix_new( 8, 1 );
	
	a->var[0][0] = x[0];
	a->var[0][1] = y[0];
	a->var[0][2] = 1.0;
	a->var[0][6] = -1 * x[0] * X[0];
	a->var[0][7] = -1 * X[0] * y[0];
	       
	a->var[1][3] = x[0];
	a->var[1][4] = y[0];
	a->var[1][5] = 1.0;
	a->var[1][6] = -1 * x[0] * Y[0];
	a->var[1][7] = -1 * y[0] * Y[0];
	       
	a->var[2][0] = x[1];
	a->var[2][1] = y[1];
	a->var[2][2] = 1.0;
	a->var[2][6] = -1 * x[1] * X[1];
	a->var[2][7] = -1 * X[1] * y[1];
	       
	a->var[3][3] = x[1];
	a->var[3][4] = y[1];
	a->var[3][5] = 1.0;
	a->var[3][6] = -1 * x[1] * Y[1];
	a->var[3][7] = -1 * y[1] * Y[1];
	       
	a->var[4][0] = x[2];
	a->var[4][1] = y[2];
	a->var[4][2] = 1.0;
	a->var[4][6] = -1 * x[2] * X[2];
	a->var[4][7] = -1 * X[2] * y[2];
	
	a->var[5][3] = x[2];
	a->var[5][4] = y[2];
	a->var[5][5] = 1.0;
	a->var[5][6] = -1 * x[2] * Y[2];
	a->var[5][7] = -1 * y[2] * Y[2];
	
	a->var[6][0] = x[3];
	a->var[6][1] = y[3];
	a->var[6][2] = 1.0;
	a->var[6][6] = -1 * x[3] * X[3];
	a->var[6][7] = -1 * X[3] * y[3];
	
	a->var[7][3] = x[3];
	a->var[7][4] = y[3];
	a->var[7][5] = 1.0;
	a->var[7][6] = -1 * x[3] * Y[3];
	a->var[7][7] = -1 * y[3] * Y[3];
	       
	b->var[0][0] = X[0];
	b->var[1][0] = Y[0];
	b->var[2][0] = X[1];
	b->var[3][0] = Y[1];
	b->var[4][0] = X[2];
	b->var[5][0] = Y[2];
	b->var[6][0] = X[3];
	b->var[7][0] = Y[3];
	
	a_inv = matrix_inv(a);
	c = matrix_multiple( a_inv, b );
	
	projection = matrix_new( 3, 3 );
	projection->var[0][0] = c->var[0][0];
	projection->var[0][1] = c->var[1][0];
	projection->var[0][2] = c->var[2][0];
	       
	projection->var[1][0] = c->var[3][0];
	projection->var[1][1] = c->var[4][0];
	projection->var[1][2] = c->var[5][0];
	       
	projection->var[2][0] = c->var[6][0];
	projection->var[2][1] = c->var[7][0];
	projection->var[2][2] = 1.0;
	
	matrix_free(a);
	matrix_free(b);
	matrix_free(c);
	
	matrix_free(a_inv);
	
	return projection;
}

matrix* projection_matrix2(vec2d *src, vec2d *tgt)
{
	double x[4];
	double y[4];
	double X[4];
	double Y[4];

	x[0] = (double)tgt[0].x;
	x[1] = (double)tgt[1].x;
	x[2] = (double)tgt[2].x;
	x[3] = (double)tgt[3].x;
	
	y[0] = (double)tgt[0].y;
	y[1] = (double)tgt[1].y;
	y[2] = (double)tgt[2].y;
	y[3] = (double)tgt[3].y;

	X[0] = (double)src[0].x;
	X[1] = (double)src[1].x;
	X[2] = (double)src[2].x;
	X[3] = (double)src[3].x;

	Y[0] = (double)src[0].y;
	Y[1] = (double)src[1].y;
	Y[2] = (double)src[2].y;
	Y[3] = (double)src[3].y;
	

	matrix* a;
	matrix* b;
	matrix* c;
	       
	matrix* a_inv;
	matrix* projection;
	       
	a = matrix_new( 8, 8 );
	b = matrix_new( 8, 1 );
	
	a->var[0][0] = x[0];
	a->var[0][1] = y[0];
	a->var[0][2] = 1.0;
	a->var[0][6] = -1 * x[0] * X[0];
	a->var[0][7] = -1 * X[0] * y[0];
	       
	a->var[1][3] = x[0];
	a->var[1][4] = y[0];
	a->var[1][5] = 1.0;
	a->var[1][6] = -1 * x[0] * Y[0];
	a->var[1][7] = -1 * y[0] * Y[0];
	       
	a->var[2][0] = x[1];
	a->var[2][1] = y[1];
	a->var[2][2] = 1.0;
	a->var[2][6] = -1 * x[1] * X[1];
	a->var[2][7] = -1 * X[1] * y[1];
	       
	a->var[3][3] = x[1];
	a->var[3][4] = y[1];
	a->var[3][5] = 1.0;
	a->var[3][6] = -1 * x[1] * Y[1];
	a->var[3][7] = -1 * y[1] * Y[1];
	       
	a->var[4][0] = x[2];
	a->var[4][1] = y[2];
	a->var[4][2] = 1.0;
	a->var[4][6] = -1 * x[2] * X[2];
	a->var[4][7] = -1 * X[2] * y[2];
	
	a->var[5][3] = x[2];
	a->var[5][4] = y[2];
	a->var[5][5] = 1.0;
	a->var[5][6] = -1 * x[2] * Y[2];
	a->var[5][7] = -1 * y[2] * Y[2];
	
	a->var[6][0] = x[3];
	a->var[6][1] = y[3];
	a->var[6][2] = 1.0;
	a->var[6][6] = -1 * x[3] * X[3];
	a->var[6][7] = -1 * X[3] * y[3];
	
	a->var[7][3] = x[3];
	a->var[7][4] = y[3];
	a->var[7][5] = 1.0;
	a->var[7][6] = -1 * x[3] * Y[3];
	a->var[7][7] = -1 * y[3] * Y[3];
	       
	b->var[0][0] = X[0];
	b->var[1][0] = Y[0];
	b->var[2][0] = X[1];
	b->var[3][0] = Y[1];
	b->var[4][0] = X[2];
	b->var[5][0] = Y[2];
	b->var[6][0] = X[3];
	b->var[7][0] = Y[3];
	
	a_inv = matrix_inv(a);
	c = matrix_multiple( a_inv, b );
	
	projection = matrix_new( 3, 3 );
	projection->var[0][0] = c->var[0][0];
	projection->var[0][1] = c->var[1][0];
	projection->var[0][2] = c->var[2][0];
	       
	projection->var[1][0] = c->var[3][0];
	projection->var[1][1] = c->var[4][0];
	projection->var[1][2] = c->var[5][0];
	       
	projection->var[2][0] = c->var[6][0];
	projection->var[2][1] = c->var[7][0];
	projection->var[2][2] = 1.0;
	
	matrix_free(a);
	matrix_free(b);
	matrix_free(c);
	
	matrix_free(a_inv);
	
	return projection;


}