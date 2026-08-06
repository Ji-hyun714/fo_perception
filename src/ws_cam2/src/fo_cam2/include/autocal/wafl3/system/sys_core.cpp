//---------------------------------------------------------------------------

#pragma hdrstop

#include "sys_core.h"

//---------------------------------------------------------------------------
#pragma package(smart_init)

//===========================================================================
WAStream::WAStream()
{

}
WAStream::~WAStream()
{

}

size_t WAStream::GetPosition(void)
{
	return Seek(0, srCurrent);
}

void WAStream::SetPosition(size_t Pos)
{
	Seek(Pos, srStart);
}

size_t WAStream::GetSize(void)
{
	size_t curr = Seek(0, srCurrent);
	size_t ret = Seek(0, srEnd);
	Seek(curr, srStart);
	return ret;
}

bool WAStream::WriteData(size_t &total_written, const void *data, size_t size)
{
	size_t ret = Write(data, size);
	total_written += ret;
	return (ret == size);
}

bool WAStream::ReadData(size_t &total_read, void *data, size_t size)
{
	size_t ret = Read(data, size);
	total_read += ret;
	return (ret == size);
}

bool WAStream::IsActive(void) const
{
	return true;
}

bool WAStream::SetSize(size_t Size)
{
	return false;
}



//===========================================================================
WAFileStream::WAFileStream(const UString &FileName, const UString &Mode)
	: WAStream(), fp(NULL)
{
#if defined(_WIN32) || defined(WIN32)
	UString sFile = FileName, sMode = Mode;
	sFile.ChangeCodePage(CPD_UTF16);
	sMode.ChangeCodePage(CPD_UTF16);
	fp = _wfopen(sFile.w_str(), sMode.w_str());
#else //#if defined(_WIN32) || defined(WIN32)
	fp = fopen(FileName.c_str(), Mode.c_str());
#endif //#else #if defined(_WIN32) || defined(WIN32)
}
WAFileStream::WAFileStream(const ansi_t *file_name, const utf8_t *mode, CPD codepage)
	: WAStream(), fp(NULL)
{
#if defined(_WIN32) || defined(WIN32)
	UString sFile = UString(codepage, file_name), sMode = UString(codepage, mode);
	sFile.ChangeCodePage(CPD_UTF16);
	sMode.ChangeCodePage(CPD_UTF16);
	fp = _wfopen(sFile.w_str(), sMode.w_str());
#else //#if defined(_WIN32) || defined(WIN32)
	fp = fopen(file_name, mode);
#endif //#else #if defined(_WIN32) || defined(WIN32)
}
#if defined(_WIN32) || defined(WIN32)
WAFileStream::WAFileStream(const utf16_t *file_name, const utf16_t *mode)
	: WAStream(), fp(NULL)
{
	fp = _wfopen(file_name, mode);
}
#endif //#if defined(_WIN32) || defined(WIN32)
WAFileStream::~WAFileStream()
{
	fclose(fp);
}
size_t WAFileStream::Write(const void *data, size_t size)
{
	return (size_t)fwrite(data, 1, size, fp);
}
size_t WAFileStream::Read(void *data, size_t size)
{
	return (size_t)fread(data, 1, size, fp);
}
size_t WAFileStream::Seek(size_t Pos, SeekReference Origin)
{
	int32_t origin = SEEK_SET;
	switch (Origin) {
		case srCurrent: origin = SEEK_CUR; break;
		case srEnd: origin = SEEK_END; break;
		default: break;
	}
	fseek(fp, Pos, origin);
	return ftell(fp);
}
size_t WAFileStream::GetPosition(void)
{
	return (size_t)ftell(fp);
}
size_t WAFileStream::GetSize(void)
{
	size_t curr = ftell(fp);
	fseek(fp, 0, SEEK_END);
	size_t ret = ftell(fp);
	fseek(fp, curr, SEEK_SET);
	return ret;
}
bool WAFileStream::IsActive(void) const
{
	return (fp != NULL);
}

//===========================================================================

WADVECStream::WADVECStream()
	: WAStream(), m_vec(NULL), m_idx(0)
{
}
WADVECStream::~WADVECStream()
{
	DVEC_FREE(&m_vec);
}
size_t WADVECStream::Write(const void *data, size_t size)
{
	size_t len = DVEC_Length(m_vec);
	len = m_idx + size - len;
	if (len > 0) {
		DVEC_APPEND(uint8_t, &m_vec, len);
	}
	memcpy(m_vec + m_idx, data, size);
	m_idx += size;
	return size;
}
size_t WADVECStream::Read(void *data, size_t size)
{
	size_t nRead = DVEC_Length(m_vec) - m_idx;
	memcpy(data, m_vec + m_idx, nRead);
	m_idx += nRead;
	return nRead;
}
size_t WADVECStream::Seek(size_t Pos, SeekReference Origin)
{
	size_t len = DVEC_Length(m_vec);
	switch (Origin) {
		case srStart: m_idx = Pos; break;
		case srCurrent: m_idx += Pos; break;
		case srEnd: m_idx = (Pos > len) ? 0 : len - Pos; break;
		default: break;
	}
	if (m_idx > len) {
		DVEC_SETLENGTH(uint8_t, &m_vec, m_idx);
	}
	return m_idx;
}

size_t WADVECStream::GetPosition(void)
{
	return m_idx;
}
size_t WADVECStream::GetSize(void)
{
	return DVEC_Length(m_vec);
}
bool WADVECStream::SetSize(size_t Size)
{
	DVEC_SETLENGTH(uint8_t, &m_vec, Size);
	if (m_idx > Size) {
		m_idx = Size;
	}
	return true;
}
