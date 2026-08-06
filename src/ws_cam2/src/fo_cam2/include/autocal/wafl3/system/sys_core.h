//---------------------------------------------------------------------------

#ifndef sys_coreH
#define sys_coreH
//---------------------------------------------------------------------------
#include "system/string_base.h"


#if defined(__cplusplus)
//===========================================================================
class WAStream
{
protected:
public:
	WAStream();
	virtual ~WAStream();

	enum SeekReference {
		srStart = 0,
		srCurrent,
		srEnd
	};

	virtual size_t Write(const void *data, size_t size) = 0;
	virtual size_t Read(void *data, size_t size) = 0;
	virtual size_t Seek(size_t Pos, SeekReference Origin = srStart) = 0;
	virtual bool SetSize(size_t Size);
	virtual size_t GetSize(void);
	virtual size_t GetPosition(void);
	virtual void SetPosition(size_t Pos);
	virtual bool IsActive(void) const;

	bool WriteData(size_t &total_written, const void *data, size_t size);
	bool ReadData(size_t &total_read, void *data, size_t size);

};

//===========================================================================
class WAFileStream : public WAStream
{
protected:
	FILE *fp;
public:
	WAFileStream(const UString &FileName, const UString &Mode);
	WAFileStream(const ansi_t *file_name, const utf8_t *mode, CPD codepage = CPD_ACP);
#if defined(_WIN32) || defined(WIN32)
	WAFileStream(const utf16_t *file_name, const utf16_t *mode);
#endif //#if defined(_WIN32) || defined(WIN32)
	virtual ~WAFileStream();

	virtual size_t Write(const void *data, size_t size);
	virtual size_t Read(void *data, size_t size);
	virtual size_t Seek(size_t Pos, SeekReference Origin);

	virtual size_t GetSize(void);
	virtual size_t GetPosition(void);
	virtual bool IsActive(void) const;
};

//===========================================================================
class WADVECStream : public WAStream
{
protected:
	uint8_t *m_vec;
	size_t   m_idx;
public:
	WADVECStream();
	virtual ~WADVECStream();

	virtual size_t Write(const void *data, size_t size);
	virtual size_t Read(void *data, size_t size);
	virtual size_t Seek(size_t Pos, SeekReference Origin);

	virtual size_t GetPosition(void);
	virtual size_t GetSize(void);
	virtual bool SetSize(size_t Size);

	inline void * GetMemory(void) {return m_vec;}
	inline const void * GetMemory(void) const {return m_vec;}
};
#endif //#if defined(__cplusplus)

#endif
