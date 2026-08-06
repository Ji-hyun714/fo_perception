/**--------------------------------------------------------------------------
@file file_serialize_tag.cpp
@brief TAG-DATA 방식의 바이너리 직렬화 관련 함수들의 집합
@author ksg
*/
//---------------------------------------------------------------------------

#pragma hdrstop

#include "dvec_serialize_tag.h"
#include "file_serialize_tag.h"
#include "system/sys_logger.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)


/**--------------------------------------------------------------------------
@brief decode_uint32와 동일하나 WASTream으로부터 읽어들임

@param [in ] stm WAStream 포인터
@return 디코딩 된 값

@author ksg
*/
uint32_t WA_API fdecode_uint32_STREAM(WAStream *stm)
{
  uint8_t buf[4];
  const uint8_t *p;
  size_t len = 0;
  size_t szRead;
  szRead = stm->Read(buf, 1);
  if (szRead != 1) {
    Err_("IO error: fread returned %d, expected=%d, offset=%d", szRead, 1, stm->GetPosition());
    return 0;
  }
  if ((buf[0] & 0x80) == 0x00) {
    return (uint32_t)buf[0];
  } else if ((buf[0] & 0xC0) == 0x80) {
    len = 1;
  } else if ((buf[0] & 0xE0) == 0xC0) {
    len = 2;
  } else if ((buf[0] & 0xF0) == 0xE0) {
    len = 3;
  } else {
    Err_("Invalid value encoding");
    return 0;
  }
  szRead = stm->Read(buf+1, 1);
  if (szRead != len) {
    Err_("IO error: fread returned %d, expected=%d, offset=%d", szRead, len, stm->GetPosition());
    return 0;
  }
  p = buf;
  return decode_uint32(&p);
}


/**--------------------------------------------------------------------------
@brief read_token와 동일하나 WAStream으로부터 읽어들임

@param [in]  stm   WAStream 포인터
@param [out] size 뒤따르는 데이터의 크기를 저장할 주소
@return 읽어들인 TAG값

@author ksg
*/
uint8_t WA_API fread_token_STREAM(WAStream *stm, uint32_t *size)
{
  uint8_t token;
  size_t szRead;
	szRead = stm->Read(&token, 1);
  if (szRead != 1) {
    Err_("IO error fread returned %d, expected=1, offset=%d", szRead, stm->GetPosition());
    return 0;
  }
  if ((token & 0x80) == 0) {
    int shift = (int)(token >> 5) - 1;
    *size = (shift == -1) ? (0) : (1 << shift);
    token &= 0x1f;
  } else {
    *size = fdecode_uint32_STREAM(stm);
    token &= 0x7f;
  }
  return token;
}


/**--------------------------------------------------------------------------
@brief read_DVEC과 동일하나 WAStream으로부터 읽어들인다.

@param [in ] dst        복원할 DVEC타입의 리스트 객체
@param [in]  stm   WAStream 포인터
@param [in ] token_size 입력 데이터의 크기. read_token 함수로 얻은 size 정보를 넘겨야 한다.
@return 처리된 입력 데이터의 크기

@author ksg
*/
size_t WA_API fread_DVEC_STREAM(DVEC *dst, WAStream *stm, size_t token_size)
{
  uint32_t context;
  int len;
  size_t szRead;
  szRead = stm->Read(&context, 4);
  if (szRead != 4) {
	  Err_("IO error: fread returned %d, expected=4, offset=%d", szRead, stm->GetPosition());
    return 0;
  }
  token_size -= 4;
  len = token_size / (context >> 16);
  rcm_set_len_ctx(dst, len, context);
  szRead = stm->Read(*dst, token_size);
  if (szRead != token_size) {
    Err_("IO error: fread returned %d, expected=%d, offset=%d", szRead, token_size, stm->GetPosition());
    return 0;
  }
  return token_size + 4;
}



