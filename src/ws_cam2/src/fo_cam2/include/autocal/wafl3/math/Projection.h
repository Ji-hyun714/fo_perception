#pragma once

#include "wafl3.h"
#include "matrix_proj.h"
//#include "cal_ImageProcess.h"

#if __cplusplus
extern "C" {
#endif //#if __cplusplus


matrix* projection_matrix3(double* x, double* y, double* X, double* Y);
matrix* projection_matrix2(vec2d *, vec2d *);

#if __cplusplus
}
#endif //#if __cplusplus

