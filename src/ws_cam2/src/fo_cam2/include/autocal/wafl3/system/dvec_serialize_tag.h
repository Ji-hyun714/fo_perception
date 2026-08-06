/**--------------------------------------------------------------------------
@file file_serialize_tag.h
@brief file_serialize_tag.cpp 파일의 헤더
@author ksg
*/
//---------------------------------------------------------------------------

#ifndef dvec_serialize_tagH
#define dvec_serialize_tagH
//---------------------------------------------------------------------------
#include "system/string_base.h"
#include "system/sys_core.h"

#define TAG_SINGLE_0 0x00
#define TAG_SINGLE_1 0x20
#define TAG_SINGLE_2 0x40
#define TAG_SINGLE_4 0x60
#define TAG_MULTI    0x80

#define BEGIN_DISPATCH_MAP \
  switch (tag) {

#define READ_SIMPLE_TAG_STREAM(_tag, _member) \
    case _tag: \
      if (size != sizeof(_member) || ds->Read(&(_member), size) != size) { \
        Err_(gsErrIOSizeMismatch); \
        return -1; \
      } \
      break;
#define READ_STATIC_ARRAY_TAG_STREAM(_tag, _member) \
    case _tag: \
      if (size != sizeof(_member) || ds->Read(_member, size) != size) { \
        Err_(gsErrIOSizeMismatch); \
        return -1; \
      } \
      break;
#define READ_BOOLEAN_TAG_STREAM(_tag, _member) \
    case _tag: \
      if (size != 1 || ds->Read(&(_member), size) != size) { \
        Err_(gsErrIOSizeMismatch); \
      } \
      break;
#define READ_MATRIX_TAG_STREAM(_tag, _member) \
    case _tag: \
      if (size == sizeof(float) * 16) { \
        _member = mat4f_init(); \
        if (ds->Read(_member, size) != size) { \
          Err_(gsErrIOSizeMismatch); \
        } \
      } else { \
        Err_(gsErrIOSizeMismatch); \
      } \
      break;
#define READ_MATRIX_TAG_D9_STREAM(_tag, _member) \
    case _tag: \
      if (size == sizeof(double) * 9) { \
        _member = mat3d_init(); \
        if (ds->Read(_member, size) != size) { \
          Err_(gsErrIOSizeMismatch); \
        } \
      } else { \
        Err_(gsErrIOSizeMismatch); \
      } \
      break;
#define READ_ARRAY_TAG_D4_STREAM(_tag, _member) \
    case _tag: \
      if (size == sizeof(double) * 4) { \
        _member = vec4d_init(); \
        if (ds->Read(_member, size) != size) { \
          Err_(gsErrIOSizeMismatch); \
        } \
      } else { \
        Err_(gsErrIOSizeMismatch); \
      } \
	  break;
#define READ_ARRAY_TAG_D8_STREAM(_tag, _member) \
    case _tag: \
      if (size == sizeof(double) * 8) { \
        _member = vec8d_init(); \
        if (ds->Read(_member, size) != size) { \
          Err_(gsErrIOSizeMismatch); \
        } \
      } else { \
        Err_(gsErrIOSizeMismatch); \
      } \
      break;

#define END_DISPATCH_MAP(parent) \
    default: \
      return parent::DispatchLoad(fp, tag, size); \
  } \
  return 1;

#define BEGIN_WRITE_MAP \
  int written = 0;

#define WRITE_SIMPLE_TAG(_tag, _member) \
  written += fwrite_token(fp, _tag, sizeof(_member), &(_member))

#define WRITE_SIMPLE_TAG_IFNOT_DEFAULT(_tag, _member, _default) \
  if ((_member) != (_default)) { \
    written += fwrite_token(fp, (_tag), sizeof(_member), &(_member)); \
  }

#define WRITE_BOOLEAN_TAG(_tag, _member) \
  written += fwrite_token(fp, _tag, 1, &(_member))
#define WRITE_BOOLEAN_TAG_IFNOT_DEFAULT(_tag, _member, _default) \
  if ((_member) != (_default)) { \
    written += fwrite_token(fp, _tag, 1, &(_member)); \
  }
#define WRITE_MATRIX_TAG(_tag, _member) \
  if (_member) { \
    written += fwrite_token(fp, _tag, sizeof(float) * 16, _member); \
  }
#define WRITE_MATRIX_TAG_D9(_tag, _member) \
if (_member) { \
	written += fwrite_token(fp, _tag, sizeof(double) * 9, _member); \
}
#define WRITE_ARRAY_TAG_D4(_tag, _member) \
if (_member) { \
	written += fwrite_token(fp, _tag, sizeof(double) * 4, _member); \
}

#define WRITE_TERMINAL_TAG \
  written += fwrite_token(fp, 0, 0, NULL)


#define END_WRITE_MAP(parent) \
  return written + parent::Save(fp);

#if __cplusplus
extern "C" {
#endif //#if __cplusplus


extern uint32_t WA_API fdecode_uint32_STREAM(WAStream *stm);

extern uint8_t WA_API fread_token_STREAM(WAStream *stm, uint32_t *size);

extern size_t WA_API fread_DVEC_STREAM(DVEC *dst, WAStream *stm, size_t token_size);

#if __cplusplus
}
#endif //#if __cplusplus

#endif
