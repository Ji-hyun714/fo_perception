/**--------------------------------------------------------------------------
@file file_serialize_tag.cpp
@brief TAG-DATA 방식의 바이너리 직렬화 관련 함수들의 집합
@author ksg
*/
//---------------------------------------------------------------------------

#if (defined(CCS) || defined(__BORLANDC__))
#pragma hdrstop
#endif //(defined(CCS) || defined(__BORLANDC__))

#include "file_serialize_tag.h"
#include "system/sys_logger.h"
//---------------------------------------------------------------------------
#if defined(__BORLANDC__)
#pragma package(smart_init)
#endif //defined(__BORLANDC__)

const char gsErrUnknownTag[] = "Unknown tag was found. It would be ignored";
const char gsErrIOSizeMismatch[] = "IO error - size mismatch";


/**--------------------------------------------------------------------------
@brief uint32_t 타입의 값을 가변 길이 인코딩

@param [out] dst   인코딩 된 결과를 저장할 주소
@param [in ] value 인코딩 할 값
@return 인코딩 된 결과가 차지하는 바이트 수

@author ksg
*/
int WA_API encode_uint32(uint8_t **dst, uint32_t value)
{
  int ret = 0;
  uint8_t *p = NULL;

  if (value < 0x80) {
    ret = 1;
  } else if (value < 0x4000) {
    ret = 2;
  } else if (value < 0x200000) {
    ret = 3;
  } else if (value < 0x10000000) {
    ret = 4;
  } else {
    Err_("Cannot encode the value");
    return 0;
  }

  if (dst == NULL || *dst == NULL) {
    return ret;
  }

  p = *dst;
  switch (ret) {
    case 1:
      p[0] = (uint8_t)(value);
      break;
    case 2:
      p[0] = (uint8_t)((value >> 8) | 0x80);
      p[1] = (uint8_t)(value);
      break;
    case 3:
      p[0] = (uint8_t)((value >> 16) | 0xC0);
      p[1] = (uint8_t)(value >> 8);
      p[2] = (uint8_t)(value);
      break;
    case 4:
      p[0] = (uint8_t)((value >> 24) | 0xE0);
      p[1] = (uint8_t)(value >> 16);
      p[2] = (uint8_t)(value >> 8);
      p[3] = (uint8_t)(value);
      break;
    default:
      break;
  }
  *dst = p + ret;
  return ret;
}

/**--------------------------------------------------------------------------
@brief encode_uint32와 동일하나 결과를 FILE 포인터에 저장함

@param [out] fp    결과를 저장할 FILE 포인터
@param [in ] value 인코딩 할 값
@return 저장된 바이트 수

@author ksg
*/
int WA_API fencode_uint32(FILE *fp, uint32_t value)
{
  uint8_t buf[4], *p;
  p = buf;
  return fwrite(buf, 1, encode_uint32(&p, value), fp);
}

/**--------------------------------------------------------------------------
@brief 가변 길이 인코딩 된 데이터로부터 uint32_t 타입의 정수를 복원함

@param [in,out] src 입력 데이터의 포인터. 처리한 만큼 포인터의 값은 증가됨
@return 디코딩 된 값

@author ksg
*/
uint32_t WA_API decode_uint32(const uint8_t **src)
{
  uint32_t value = 0;
  const uint8_t *p = *src;
  value = (uint32_t)(p[0]);
  if ((value & 0x00000080) == 0x00000000) {
    *src = p + 1;
  } else if ((value & 0x000000C0) == 0x00000080) {
    value = (((value & 0x0000003F) << 8) | ((uint32_t)(p[1])));
    *src = p + 2;
  } else if ((value & 0x000000E0) == 0x000000C0) {
    value = (((value & 0x0000001F) << 16) | (((uint32_t)(p[1])) << 8) | ((uint32_t)(p[2])));
    *src = p + 3;
  } else if ((value & 0x000000F0) == 0x000000E0) {
    value = (((value & 0x0000000F) << 24) | (((uint32_t)(p[1])) << 16) | (((uint32_t)(p[2])) << 8) | ((uint32_t)(p[3])));
    *src = p + 4;
  } else {
    Err_("Invalid value encoding");
  }
  return value;
}

/**--------------------------------------------------------------------------
@brief decode_uint32와 동일하나 파일로부터 읽어들임

@param [in ] fp 파일 포인터
@return 디코딩 된 값

@author ksg
*/
uint32_t WA_API fdecode_uint32(FILE *fp)
{
  uint8_t buf[4];
  const uint8_t *p;
  size_t len = 0;
  size_t szRead;
  szRead = fread(buf, 1, 1, fp);
  if (szRead != 1) {
    Err_("IO error: fread returned %d, expected=%d, offset=%d", szRead, 1, ftell(fp));
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
  szRead = fread(buf+1, 1, len, fp);
  if (szRead != len) {
    Err_("IO error: fread returned %d, expected=%d, offset=%d", szRead, len, ftell(fp));
    return 0;
  }
  p = buf;
  return decode_uint32(&p);
}

/**--------------------------------------------------------------------------
@brief TAG 정보를 읽고 TAG의 종류에 따라 뒤따르는 데이터의 크기 정보를 
읽어 반환한다.

@param [in,out] src  읽어들일 데이터에 대한 포인터
@param [out] size 뒤따르는 데이터의 크기를 저장할 주소
@return 읽어들인 TAG값

@author ksg
*/
uint8_t WA_API read_token(const uint8_t **src, uint32_t *size)
{
  uint8_t token;
  const uint8_t *p = *src;
  token = *p;
  p++;
  if ((token & 0x80) == 0) {
    int shift = (int)(token >> 5) - 1;
    *size = (shift == -1) ? (0) : (1 << shift);
    token &= 0x1f;
  } else {
    *size = decode_uint32(&p);
    token &= 0x7f;
  }
  *src = p;
  return token;
}
/**--------------------------------------------------------------------------
@brief read_token와 동일하나 파일로부터 읽어들임

@param [in]  fp   파일 포인터
@param [out] size 뒤따르는 데이터의 크기를 저장할 주소
@return 읽어들인 TAG값

@author ksg
*/
uint8_t WA_API fread_token(FILE *fp, uint32_t *size)
{
  uint8_t token;
  size_t szRead;
  szRead = fread(&token, 1, 1, fp);
  if (szRead != 1) {
    Err_("IO error fread returned %d, expected=1, offset=%d", szRead, ftell(fp));
    return 0;
  }
  if ((token & 0x80) == 0) {
    int shift = (int)(token >> 5) - 1;
    *size = (shift == -1) ? (0) : (1 << shift);
    token &= 0x1f;
  } else {
    *size = fdecode_uint32(fp);
    token &= 0x7f;
  }
  return token;
}

/**--------------------------------------------------------------------------
@brief 지정한 테그와 데이터를 바이너리 형식으로 저장한다.

@param [out] dst  결과를 기록할 주소
@param [in ] tag  테그값. 0은 터미널의 용도로만 사용하고 0~31까지 사용 가능
@param [in ] size 저장할 후속 데이터의 크기. 0이면 단독 테그임을 의미함
@param [in ] data 저장할 후속 데이터
@return 저장한 크기
dst가 NULL이거나 *dst가 NULL인 경우 data까지 저장하는데 필요한 크기를 반환하며
data만 NULL인 경우 tag와 size 정보까지만 기록하고 tag와 size 저장에 소요된 크기만
반환한다.

@author ksg
*/
int WA_API write_token(uint8_t **dst, uint8_t tag, uint32_t size, const void *data)
{
  int ret = 0;
  uint8_t *p;

  if ((tag & 0x1f) != tag) {
    Err_("Too big tag!!");
    return ret;
  }

  ret = 1;
  if (size <= 4 && size != 3) {
    if (dst && (p = *dst) != NULL) {
      switch (size) {
        case 0:
          p[0] = tag | TAG_SINGLE_0;
          break;
        case 1:
          p[0] = tag | TAG_SINGLE_1;
          if (data) {
            p[1] = ((uint8_t*)data)[0];
          }
          break;
        case 2:
          p[0] = tag | TAG_SINGLE_2;
          if (data) {
            ((uint16_t*)p)[0] = ((uint16_t*)data)[0];
          }
          break;
        case 4:
          p[0] = tag | TAG_SINGLE_4;
          if (data) {
            ((uint32_t*)p)[0] = ((uint32_t*)data)[0];
          }
          break;
        default:
          break;
      }
      if (data) {
        *dst = p + 1 + size;
        ret += size;
      } else {
        *dst = p + 1;
      }
    } else {
      ret += size;
    }
  } else {
    if (dst && (p = *dst) != NULL) {
      p[0] = tag | TAG_MULTI;
      p++;
      ret += encode_uint32(&p, size);
      if (data) {
        memcpy(p, data, size);
        *dst = p + size;
        ret += size;
      } else {
        *dst = p;
      }
    } else {
      ret += encode_uint32(NULL, size) + size;
    }
  }
  return ret;
}

/**--------------------------------------------------------------------------
@brief write_token과 동일하나 파일에 결과를 저장

@param [in ] fp   파일 포인터
@param [in ] tag  테그값. 0은 터미널의 용도로만 사용하고 0~31까지 사용 가능
@param [in ] size 저장할 후속 데이터의 크기. 0이면 단독 테그임을 의미함
@param [in ] data 저장할 후속 데이터
@return 저장한 크기

@author ksg
*/
int WA_API fwrite_token(FILE *fp, uint8_t tag, uint32_t size, const void *data)
{
  uint8_t buf[5], *p;
  size_t len;

  p = buf;
  len = write_token(&p, tag, size, NULL);
  if (fwrite(buf, 1, len, fp) != len) {
    Err_("IO Error");
    return 0;
  }
  if (data) {
    if (fwrite(data, 1, size, fp) == size) {
      len += size;
    } else {
      Err_("IO Error");
    }
  }
  return len;
}

/**--------------------------------------------------------------------------
@brief DVEC 타입의 리스트를 저장함

DVEC타입은 내부적으로 write_token 함수를 호출하여 기록하지만 
감춰진 레코드의 맨 첫 4byte(요소의 크기 및 사용자 정의 context)
를 먼저 저장하고 데이터 영역을 이어 저장하도록 구현되어 있다.

@param [in,out] dst  저장할 주소의 포인터
@param [in ] tag  테그값. 0은 터미널의 용도로만 사용하고 0~31까지 사용 가능
@param [in ] data 저장할 DVEC타입 리스트
@return 저장한 크기

@author ksg
*/
int WA_API write_DVEC(uint8_t **dst, uint8_t tag, const DVEC data)
{
  uint32_t *rec, size;
  uint8_t *p;
  size_t len;

  if (data == NULL) {
    return 0;
  }

  rec = (uint32_t*)DVEC_GetRec(data);
  size = (rec[0] >> 16) * rec[2];
  len = write_token(dst, tag, size + sizeof(uint32_t), NULL);
  p = *dst;
  ((uint32_t*)p)[0] = rec[0];
  p += sizeof(uint32_t);
  memcpy(p, data, size);
  *dst = p + size;
  len += size;
  return len;
}
/**--------------------------------------------------------------------------
@brief write_DVEC과 동일하나 결과를 파일에 저장함

@param [in ] fp   파일 포인터
@param [in ] tag  테그값. 0은 터미널의 용도로만 사용하고 0~31까지 사용 가능
@param [in ] data 저장할 DVEC타입 리스트
@return 저장한 크기

@author ksg
*/

size_t WA_API fwrite_DVEC(FILE *fp, uint8_t tag, DVEC data)
{
  uint32_t *rec, size;
  size_t ret;
  if (data == NULL) {
    return 0;
  }
  rec = (uint32_t*)DVEC_GetRec(data);
  size = (rec[0] >> 16) * rec[2];
  ret = fwrite_token(fp, tag, size + 4, NULL);
  if (fwrite(rec, 1, 4, fp) != 4) {
    Err_("IO error");
    return 0;
  }
  ret += 4;
  if (fwrite(data, 1, size, fp) != size) {
    Err_("IO error");
    return 0;
  }
  ret += size;
  return ret;
}

/**--------------------------------------------------------------------------
@brief DVEC 타입의 리스트를 읽어들인다.

DVEC 타입을 write_DVEC으로 기록할 경우 데이터의 길이는 write_token 함수로
저장되는 길이 값을 그대로 사용할 뿐 별도로 기록되어있지 않다.
따라서 read_DVEC을 호출하려면 read_token 함수를 호출할 때 얻게 되는 추가
데이터의 크기, size를 이 함수의 token_size로 전달해야 데이터의 길이를
제대로 설정하여 읽어들일 수 있다.

@param [out] dst        복원할 DVEC타입의 리스트 객체
@param [in,out] src     데이터 원본 주소의 포인터. 처리한 결과 만큼 이동된다.
@param [in ] token_size 입력 데이터의 크기. read_token 함수로 얻은 size 정보를 넘겨야 한다.
@return 처리된 입력 데이터의 크기

@author ksg
*/
int WA_API read_DVEC(DVEC *dst, const uint8_t **src, uint32_t token_size)
{
  const uint8_t *p = *src;
  int length;
  uint32_t context = *((uint32_t*)p);

  length = (token_size - sizeof(uint32_t)) / (context >> 16);

  rcm_set_len_ctx(dst, length, context);
  memcpy(*dst, p + sizeof(uint32_t), length * (context >> 16));

  *src = p + token_size;
  return token_size;
}

/**--------------------------------------------------------------------------
@brief read_DVEC과 동일하나 파일로부터 읽어들인다.

@param [in ] dst        복원할 DVEC타입의 리스트 객체
@param [in ] fp         FILE 포인터
@param [in ] token_size 입력 데이터의 크기. read_token 함수로 얻은 size 정보를 넘겨야 한다.
@return 처리된 입력 데이터의 크기

@author ksg
*/
size_t WA_API fread_DVEC(DVEC *dst, FILE *fp, size_t token_size)
{
  uint32_t context;
  int len;
  size_t szRead;
  szRead = fread(&context, 1, 4, fp);
  if (szRead != 4) {
    Err_("IO error: fread returned %d, expected=4, offset=%d", szRead, ftell(fp));
    return 0;
  }
  token_size -= 4;
  len = token_size / (context >> 16);
  rcm_set_len_ctx(dst, len, context);
  szRead = fread(*dst, 1, token_size, fp);
  if (szRead != token_size) {
    Err_("IO error: fread returned %d, expected=%d, offset=%d", szRead, token_size, ftell(fp));
    return 0;
  }
  return token_size + 4;
}



