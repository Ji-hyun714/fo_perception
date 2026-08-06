//---------------------------------------------------------------------------

#if (defined(CCS) || defined(__BORLANDC__))
#pragma hdrstop
#endif //(defined(CCS) || defined(__BORLANDC__))

#include "geo_linear.h"
//---------------------------------------------------------------------------
#if defined(__BORLANDC__)
#pragma package(smart_init)
#endif //defined(__BORLANDC__)

/**--------------------------------------------------------------------------
@brief 4개의 샘플 포인트로부터 0~1사이로 정규화된 uv좌표의 위치를
선형 보간하여 계산함

@param [in ] P1 좌상단 좌표
@param [in ] P2 우상단 좌표
@param [in ] P3 좌하단 좌표
@param [in ] P4 우하단 좌표
@param [in ] uv 0~1 사이로 정규화 된 uv 좌표
@return 선형 보간된 x,y 좌표

@author ksg
*/
vec2 GetXYFrom4CP(vec2 P1, vec2 P2, vec2 P3, vec2 P4, vec2 uv)
{
  vec2 R;
  R.x = uv.x * uv.y * (P4.x + P1.x - P2.x - P3.x)
      + uv.x * (P2.x - P1.x)
      + uv.y * (P3.x - P1.x)
      + P1.x;
  R.y = uv.x * uv.y * (P4.y + P1.y - P2.y - P3.y)
      + uv.x * (P2.y - P1.y)
      + uv.y * (P3.y - P1.y)
      + P1.y;
  return R;
}