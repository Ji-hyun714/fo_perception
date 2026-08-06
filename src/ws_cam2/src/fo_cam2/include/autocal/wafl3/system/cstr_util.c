/**--------------------------------------------------------------------------
@file cstr_util.c
@brief C용 기초적인 문자열 처리 함수 모음
@author ksg
*/
//---------------------------------------------------------------------------
#include <stdarg.h>
#include <stdlib.h>

#if (defined(CCS) || defined(__BORLANDC__))
#pragma hdrstop
#endif //(defined(CCS) || defined(__BORLANDC__))

#include "cstr_util.h"
#include "system/sys_logger.h"
//---------------------------------------------------------------------------

#if defined(__BORLANDC__)
#pragma package(smart_init)
#endif //defined(__BORLANDC__)

/**--------------------------------------------------------------------------
@brief 문자열을 정수로 변환

atoi와는 달리 0x로 시작하는 문자열의 경우 hex인코딩 된 것으로 인식하여
16진수에 대해서도 처리할 수 있도록 개선된 버전
또한 true, false로 표현된 boolean값은 각각 1, 0으로 변환한다.

@param [in ] value 문자열
@return 정수값

@author ksg
*/
int str_to_int(const char *value)
{
	int l = strlen(value);
	if (l > 2 && value[0] == '0' && (value[1] == 'x' || value[1] == 'X')) {
		return strtol(value, NULL, 16);
	} else {
		if (strisame(value, "true"))
			return 1;
		if (strisame(value, "false"))
			return 0;
		return atoi(value);
	}
}

/**--------------------------------------------------------------------------
@brief 문자열을 64비트 부호 없는 정수로 변환

atoi와는 달리 0x로 시작하는 문자열의 경우 hex인코딩 된 것으로 인식하여
16진수에 대해서도 처리할 수 있도록 개선된 버전
또한 true, false로 표현된 boolean값은 각각 1, 0으로 변환한다.

@param [in ] value value 문자열
@return 64비트 부호 없는 정수값

@author ksg
*/
uint64_t str_to_uint64(const char *value)
{
	int i = 0, l = strlen(value);
	char c;
	uint64_t ret = 0;
	if (l > 2 && value[0] == '0' && (value[1] == 'x' || value[1] == 'X')) {
		for (i = 2; i < l; i++) {
			c = value[i];
			if (c >= '0' && c <= '9') {
				ret <<= 4;
				ret |= (uint64_t)(c - '0');
			} else if (c >= 'a' && c <= 'f') {
				ret <<= 4;
				ret |= (uint64_t)(c - 'a' + 10);
			} else if (c >= 'A' && c <= 'F') {
				ret <<= 4;
				ret |= (uint64_t)(c - 'A' + 10);
			}
		}
	}
	return ret;
}
/**--------------------------------------------------------------------------
@brief 문자열을 fourcc로 변환

입력 문자열의 길이 외에 딱히 무결성 체크를 하지 않으므로 주의해서 사용
해야 한다.

@param [in ] s 문자열
@return fourcc값

@author ksg
*/
fourcc_t str_to_fourcc(const char *s)
{
	fourcc_t ret = 0x20202020;
	char *p = (char*)(&ret);
	if (s[0]) {
		p[0] = s[0];
		if (s[1]) {
			p[1] = s[1];
			if (s[2]) {
				p[2] = s[2];
				if (s[3]) {
					p[3] = s[3];
				}
			}
		}
	}
	return ret;
}
/**--------------------------------------------------------------------------
@brief 문자열을 twocc로 변환

입력 문자열의 길이 외에 딱히 무결성 체크를 하지 않으므로 주의해서 사용
해야 한다.

@param [in ] s 문자열
@return fourcc값

@author ksg
*/
twocc_t str_to_twocc(const char *s)
{
	twocc_t ret = 0x2020;
	char *p = (char*)(&ret);
	if (s[0]) {
		p[0] = s[0];
		if (s[1]) {
			p[1] = s[1];
		}
	}
	return ret;
}

/**--------------------------------------------------------------------------
@brief 문자열을 지정한 타입으로 변경

입력된 문자열을 bool, int8~64_t, uint8~64_t, float, double형으로 변환함

@param [in ] src  문자열
@param [out] dst  결과를 저장할 주소
@param [in ] type 타입
@return 성공하면 결과 타입의 크기, 실패하면 0

@author ksg
*/
int str_to_numeric(const char *src, void *dst, WA_TYPE type)
{
	if (src == NULL)
		return 0;

	switch (type) {
		case WA_TYPE_NULL:
			break;
		case WA_TYPE_BOOL:
			*((bool*)dst) = (bool)str_to_int(src);
			return sizeof(bool);
		case WA_TYPE_INT8:
			*((int8_t*)dst) = (int8_t)str_to_int(src);
			return sizeof(int8_t);
		case WA_TYPE_INT16:
			*((int16_t*)dst) = (int16_t)str_to_int(src);
			return sizeof(int16_t);
		case WA_TYPE_INT32:
			*((int32_t*)dst) = (int32_t)str_to_int(src);
			return sizeof(int32_t);
		case WA_TYPE_INT64:
			*((int64_t*)dst) = (int64_t)str_to_int(src);
			return sizeof(uint64_t);
		case WA_TYPE_UINT8:
			*((uint8_t*)dst) = (uint8_t)str_to_int(src);
			return sizeof(uint8_t);
		case WA_TYPE_UINT16:
			*((uint16_t*)dst) = (uint16_t)str_to_int(src);
			return sizeof(uint16_t);
		case WA_TYPE_UINT32:
			*((uint32_t*)dst) = (uint32_t)str_to_int(src);
			return sizeof(uint32_t);
		case WA_TYPE_UINT64:
			*((uint64_t*)dst) = (uint64_t)str_to_int(src);
			return sizeof(uint64_t);
		case WA_TYPE_FLOAT: {
			*((float*)dst) = (float)atof(src);
			return sizeof(float);
		}
		case WA_TYPE_DOUBLE:
			*((double*)dst) = (double)atof(src);
			return sizeof(double);
		default:
			break;
	}
	return 0;
}

/**--------------------------------------------------------------------------
@brief 배열형 문자열을 숫자 배열로 저장함

pSrc는 문자열 포인터의 주소로 최종적으로 처리한 위치까지 이동하게 된다.

@param [in,out] pSrc   문자열 포인터에 대한 주소
@param [in ] delimiter 구분자
@param [out] array     결과 배열 주소
@param [in ] length    결과 배열 길이
@param [in ] type      결과 타입
@return 처리에 성공한 요소의 수

@author ksg
*/
int str_to_array(const char **pSrc, const char delimiter, void *array, int length, WA_TYPE type)
{
	const char *pBegin;
	const char *src;
	char *dst;
	int idx = 0, l;

	if (pSrc == NULL || (src = *pSrc) == NULL)
		return 0;

	while(src[0] <= ' ') {// 맨 앞의 공백 문자 제거
		if (src[0] == 0) {
			return 0;
		}
		src++;
	}

	pBegin = src;
	dst = (char*)array;
	while (idx < length) {
		if (*src <= ' ' || *src == delimiter) {
			if ((l = str_to_numeric(pBegin, dst, type)) == 0) {
				*pSrc = src;
				return 0;
			}
			dst += l;
			idx ++;
			if (*src == 0)
				break;
			// 공백문자를 모두 건너뛴다.
			while(src[1] <= ' ')
				src++;
			pBegin = src+1;
		}
		src++;
	}
	*pSrc = src;
	return idx;
}
/**--------------------------------------------------------------------------
@brief 키=값 형식의 문자열 입력으로부터 구조체의 내용을 채워주는 함수

VEC2, RECT_F와 같이 구조체이면서도 특정 타입의 배열로 볼 수 있는 구조체에
대해 입력된 키를 분석하여 구조체 전체를 읽어들이거나 특정 구조체 멤버의
값을 읽어들이는데 사용된다.

예를 들어 다음과 같이 RECT_F 구조체가 선언되고 RECT_F타입의 my_rect를
ini 파일로부터 읽어들이는 경우

RECT_F 선언
~~~~{.c}
typedef struct tagRECT_F {
	float l;
	float t;
	float r;
	float b;
} RECT_F;
~~~~

다음과 같이 읽어들일 수 있다.
~~~~{.c}
RECT_F rect1, rect2;

char lines[5][2] = {
		{"rect1.l", "1"},      // "rect1.l = 1"
		{"rect1.t", "2"},      // "rect1.t = 2"
		{"rect1.r", "3"},      // "rect1.r = 3"
		{"rect1.b", "4"},      // "rect1.b = 4"
		{"rect2",   "1,2,3,4"} // "rect2 = 1,2,3,4"
	};

#define KEY    0
#define VALUE  1
for (int i = 0; i < 5; i++_ {
	// i == 0~4까지 성공(멤버 하나씩 읽어들임), 5에서 실패
	str_to_array_if_match(&rect1, 4, WA_TYPE_FLOAT, ',', lines[i][KEY], lines[i][VALUE], "rect1", "l", "t", "r", "b");
	// i == 0~4까지 실패, 5에서 성공 (4개의 멤버를 순서대로 읽어들임)
	str_to_array_if_match(&rect2, 4, WA_TYPE_FLOAT, ',', lines[i][KEY], lines[i][VALUE], "rect2", "l", "t", "r", "b");
}
~~~~

@param [in ] array     (배열 형식으로 변형 가능한)구조체 주소
@param [in ] length    구조체의 멤버 수
@param [in ] type      구조체 멤버의 타입
@param [in ] delimiter 구분자
@param [in ] key       키
@param [in ] value     값
@param [in ] base_name 구조체의 이름
@param [in ] ...       구조체의 멤버 이름
@return 읽어들이는데 성공한 경우 1, 실패한 경우 0

@author ksg
*/
int __cdecl str_to_array_if_match(void *array, int length, const WA_TYPE type, const char delimiter, const char *key, const char *value, const char *base_name, ...)
{
	int i, elem_size;
	const char *arg;
	char ck=0, cb=0;
	va_list ap;

	elem_size = (1 << (type & 0x03));
	for (;(ck = *key) > 0 && ck < 32; key++) {}
	for (;(ck = *key) != 0 && (cb = *base_name) != 0; key++, base_name++) {
		if (lower_char(ck) != lower_char(cb))
			return 0;
	}
	if (ck == 0 && *base_name == 0) { // 정확히 일치하는 경우. Array타입으로 인식
		return str_to_array(&value, delimiter, array, length, type);
	}
	if (ck != '.') // 구분자가 나타나지 않으므로 -1 반환
		return 0;
	key++;
	va_start(ap, base_name);
	for (i = 0; i < length; i++) {
		arg = va_arg(ap, const char *);
		if (strisame(arg, key)) {
			str_to_numeric(value, (char*)array + i * elem_size, type);
			break;
		}
	}
	va_end(ap);
	return (i < length) ? 1 : 0;
}


