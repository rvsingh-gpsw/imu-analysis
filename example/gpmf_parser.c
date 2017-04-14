//THIS CODE COMES FROM DAVID NEWMAN and Kevin Coopman
//simply modified it to write values directly to arrays
//I would not not fly the space shuttle with this code and Rolf Fischer
//is writing or owning a light weight parser now


// $Header$

/*! @file dump_gpmf.c

	@brief Code for testing metadata creation

	Copyright (c) 2016 GoPro Inc. All rights reserved.
	THIS SOURCE CODE IS THE PROPRIETARY INTELLECTUAL PROPERTY AND CONFIDENTIAL
	INFORMATION OF GOPRO, INC. AND IS PROTECTED UNDER U.S. AND INTERNATIONAL
	LAW. ANY USE OF THIS SOURCE CODE WITHOUT THE PRIOR WRITTEN AUTHORIZATION OF
	GOPRO IS STRICTLY PROHIBITED.
	s
	@version 1.8.0
*/

#define VER		"1.8.1"
char g_version[] = VER;

#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#endif

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "gpmf_parser.h"
#include "gpmf_metadata.h"
#ifdef WIN32
#define LONGSEEK  _fseeki64
#else
#define LONGSEEK  fseeko
#endif

#undef LOG_TAG
#define LOG_TAG "GPMFParser"

static int MetadataTypeSize(int type)
{
	int ssize = -1;

	switch ((int)type)
	{
	case METADATA_TYPE_STRING:				ssize = 1; break;
	case METADATA_TYPE_SIGNED_BYTE:			ssize = 1; break;
	case METADATA_TYPE_UNSIGNED_BYTE:		ssize = 1; break;
	case METADATA_TYPE_STRING_UTF8:			ssize = 1; break;

		// These datatype can always be stored in Big-Endian
	case METADATA_TYPE_SIGNED_SHORT:		ssize = 2; break;
	case METADATA_TYPE_UNSIGNED_SHORT:		ssize = 2; break;
	case METADATA_TYPE_FLOAT:				ssize = 4; break;
	case METADATA_TYPE_FOURCC:				ssize = 4; break;
	case METADATA_TYPE_HIDDEN:				ssize = 4; break;
	case METADATA_TYPE_UNSIGNED_LONG_HEX:	ssize = 4; break;
	case METADATA_TYPE_SIGNED_LONG:			ssize = 4; break;
	case METADATA_TYPE_UNSIGNED_LONG:		ssize = 4; break;
	case METADATA_TYPE_Q15_16_FIXED_POINT:  ssize = 4; break;
	case METADATA_TYPE_Q31_32_FIXED_POINT:  ssize = 8; break;
	case METADATA_TYPE_DOUBLE:				ssize = 8; break;
	case METADATA_TYPE_SIGNED_64BIT_INT:	ssize = 8; break;
	case METADATA_TYPE_UNSIGNED_64BIT_INT:  ssize = 8; break;

		//All unknown or largeer than 8-bytes store as is:

	case METADATA_TYPE_GUID:				ssize = 16; break;
	case METADATA_TYPE_UTC_DATE_TIME:		ssize = 16; break;

	case METADATA_TYPE_COMPLEX_SIZE_1:		ssize = 1; break;
	case METADATA_TYPE_COMPLEX_SIZE_2:		ssize = 2; break;
	case METADATA_TYPE_COMPLEX_SIZE_3:		ssize = 3; break;
	case METADATA_TYPE_COMPLEX_SIZE_4:		ssize = 4; break;
	case METADATA_TYPE_COMPLEX_SIZE_8:		ssize = 8; break;

	case METADATA_TYPE_XML:					ssize = -1; break; // unsupported for structsize type
	case METADATA_TYPE_COMPLEX:				ssize = -1; break; // unsupported for structsize type
	case METADATA_TYPE_NEST:				ssize = -1; break;  // unsupported for structsize type
	default:								ssize = -1;  // unsupported for structsize type
	}

	return ssize;
}



#if 0 //---------------------------------------------------------

static int MetadataEndianSize(int type)
{
	int ssize = -1;

	switch ((int)type)
	{
	case METADATA_TYPE_STRING:				ssize = 1; break;
	case METADATA_TYPE_SIGNED_BYTE:			ssize = 1; break;
	case METADATA_TYPE_UNSIGNED_BYTE:		ssize = 1; break;
	case METADATA_TYPE_STRING_UTF8:			ssize = 1; break;

		// These datatype can always be stored in Big-Endian
	case METADATA_TYPE_SIGNED_SHORT:		ssize = 2; break;
	case METADATA_TYPE_UNSIGNED_SHORT:		ssize = 2; break;
	case METADATA_TYPE_FLOAT:				ssize = 4; break;
	case METADATA_TYPE_FOURCC:				ssize = 4; break;
	case METADATA_TYPE_HIDDEN:				ssize = 4; break;
	case METADATA_TYPE_UNSIGNED_LONG_HEX:	ssize = 4; break;
	case METADATA_TYPE_SIGNED_LONG:			ssize = 4; break;
	case METADATA_TYPE_UNSIGNED_LONG:		ssize = 4; break;
	case METADATA_TYPE_Q15_16_FIXED_POINT:  ssize = 4; break;
	case METADATA_TYPE_Q31_32_FIXED_POINT:  ssize = 8; break;
	case METADATA_TYPE_DOUBLE:				ssize = 8; break;
	case METADATA_TYPE_SIGNED_64BIT_INT:	ssize = 8; break;
	case METADATA_TYPE_UNSIGNED_64BIT_INT:  ssize = 8; break;

		//All unknown,complex or larger than 8-bytes store as is:
	default:								ssize = -1;  // unsupported for structsize type
	}

	return ssize;
}
#endif //--------------------------------------------------------------

static uint32_t IsValidGPMF(uint32_t *buffer, uint32_t size, uint32_t recurse, uint32_t level) // test if the data is a completed GPMF structure starting with DEVC
{
	uint32_t pos = 0;
	size >>= 2; // long size
	while (pos + 1 < size)
	{
		uint32_t tag = buffer[pos];
		uint32_t tsize = buffer[pos + 1];
		uint32_t datasize = TAG_DATA_SIZE(tsize);
		if (level == 0 && tag != METADATA_TAG_DEVICE)
			return 0;
		else
		{
			if (0 == TAG_VALID_FOURCC(tag))
				return 0;

			pos += 2;
			if (recurse && TAG_SAMPLE_TYPE(tsize) == METADATA_TYPE_NEST)
			{
				int ret = IsValidGPMF(&buffer[pos], datasize, recurse, level + 1);
				if (ret == 0)
					return 0;
			}
			pos += (datasize >> 2);
		}
	}

	if (pos == size)
		return 1;
	else
		return 0;
}

static uint32_t MetadataIsValidGPMF(uint32_t *buffer, uint32_t size, uint32_t recurse) // test if the data is a completed GPMF structure starting with DEVC
{
	return IsValidGPMF(buffer, size, recurse, 0);
}


typedef struct media_header
{
	uint8_t version_flags[4];
	uint32_t creation_time;
	uint32_t modification_time;
	uint32_t time_scale;
	uint32_t duration;
	uint16_t language;
	uint16_t quality;
} media_header;


char *Type2String(int type)
{
	switch ((int)type)
	{
	case METADATA_TYPE_SIGNED_BYTE:			return (char *)"SIGNED_BYTE"; break;
	case METADATA_TYPE_UNSIGNED_BYTE:		return (char *)"UNSIGNED_BYTE"; break;
	case METADATA_TYPE_STRING_UTF8:			return (char *)"STRING_UTF8"; break;

	// These datatype can always be storreturn (char *)""; break;
	case METADATA_TYPE_SIGNED_SHORT:		return (char *)"SIGNED_SHORT"; break;
	case METADATA_TYPE_UNSIGNED_SHORT:		return (char *)"UNSIGNED_SHORT"; break;
	case METADATA_TYPE_FLOAT:				return (char *)"FLOAT"; break;
	case METADATA_TYPE_FOURCC:				return (char *)"FOURCC"; break;
	case METADATA_TYPE_HIDDEN:				return (char *)"HIDDEN"; break;
	case METADATA_TYPE_UNSIGNED_LONG_HEX:	return (char *)"UNSIGNED_LONG_HEX"; break;
	case METADATA_TYPE_SIGNED_LONG:			return (char *)"SIGNED_LONG"; break;
	case METADATA_TYPE_UNSIGNED_LONG:		return (char *)"UNSIGNED_LONG"; break;
	case METADATA_TYPE_Q15_16_FIXED_POINT:  return (char *)"Q15_16_FIXED_POINT"; break;
	case METADATA_TYPE_Q31_32_FIXED_POINT:  return (char *)"Q31_32_FIXED_POINT"; break;
	case METADATA_TYPE_DOUBLE:				return (char *)"DOUBLE"; break;
	case METADATA_TYPE_SIGNED_64BIT_INT:	return (char *)"SIGNED_64BIT_INT"; break;
	case METADATA_TYPE_UNSIGNED_64BIT_INT:  return (char *)"UNSIGNED_64BIT_INT"; break;

		//All unknown or largeer than 8-bytes store as is:

	case METADATA_TYPE_GUID:				return (char *)"GUID"; break;
	case METADATA_TYPE_UTC_DATE_TIME:		return (char *)"UTC_DATE_TIME"; break;

	case METADATA_TYPE_COMPLEX_SIZE_1:		return (char *)"COMPLEX_SIZE_1"; break;
	case METADATA_TYPE_COMPLEX_SIZE_2:		return (char *)"COMPLEX_SIZE_2"; break;
	case METADATA_TYPE_COMPLEX_SIZE_3:		return (char *)"COMPLEX_SIZE_3"; break;
	case METADATA_TYPE_COMPLEX_SIZE_4:		return (char *)"COMPLEX_SIZE_4"; break;
	case METADATA_TYPE_COMPLEX_SIZE_8:		return (char *)"COMPLEX_SIZE_8"; break;

	case METADATA_TYPE_XML:					return (char *)"XML"; break;
	case METADATA_TYPE_COMPLEX:				return (char *)"COMPLEX"; break;
	case METADATA_TYPE_NEST:				return (char *)"NEST"; break;
	}

	return (char *)"";
}


int testtagtsize(uint32_t tag, uint32_t tsize)
{
	int type,repeat,itemsize;

	if(tag != METADATA_TAG_DEVICE) return 0;

	if (!TAG_VALID_FOURCC(tag)) return 0;

	type = tsize&0xff;
	if(!((type>='A' && type<='Z') || (type>='a' && type<='z') || (type>='0' && type<='9') || type == 0))
		return 0;

	repeat = (tsize>>8)&0xff;
	itemsize = (((tsize>>16)&0xff)<<8) + ((tsize>>24)&0xff);

	if(repeat * itemsize > 4096)
		return 0;

	return 1;
}


int StructSize(char *typestring, int len, int items)
{
	int i,ssize = 0;

	for(i=0; i<len; i++)
	{
		int size = MetadataTypeSize((int)typestring[i]);
		if (size == -1) return -1;
		ssize += size;
	}

	return ssize;
}



#define STREAM_BUFFER_MAX	65536

int expectedelementcount = 0;
char currentunits[256];
int currentunitcharsize = 0;
char expectedstructure[256];
char expectedscalestructure[256];
int expectedstructsize = 0;
int expectedscalesize = 0;
char camera_firmware[256] = "";

#define DOSWAP	1

void swapmem64(void *X)
{
#if DOSWAP
	uint64_t x = *((uint64_t *)X);
	x = (x & 0x00000000FFFFFFFF) << 32 | (x & 0xFFFFFFFF00000000) >> 32;
	x = (x & 0x0000FFFF0000FFFF) << 16 | (x & 0xFFFF0000FFFF0000) >> 16;
	x = (x & 0x00FF00FF00FF00FF) << 8 | (x & 0xFF00FF00FF00FF00) >> 8;
	*((uint64_t *)X) = x;
#endif
}
void swapmem32(void *X)
{
#if DOSWAP
	uint32_t x = *((uint32_t *)X);
	x = (x & 0x0000FFFF) << 16 | (x & 0xFFFF0000) >> 16;
	x = (x & 0x00FF00FF) << 8  | (x & 0xFF00FF00) >> 8;
	*((uint32_t *)X) = x;
#endif
}
void swapmem16(void *X)
{
#if DOSWAP
	uint16_t x = *((uint16_t *)X);
	x = (x & 0x00FF) << 8 | (x & 0xFF00) >> 8;
	*((uint16_t *)X) = x;
#endif
}


#define BSWAP32(num) (((num>>24)&0xff) | ((num<<8)&0xff0000) | ((num>>8)&0xff00) | ((num<<24)&0xff000000))
#define BSWAP16(num) (((num<<8)&0xff00) | ((num>>8)&0xff))


uint32_t *metasizes = NULL;
uint32_t *metaoffsets = NULL;
uint32_t *metasampletime = NULL;
uint32_t indexcount = 0;
float videolength = 0.0;
float metadatalength = 0.0;
uint32_t subtitlingTrack = 0;
uint32_t clockdemon, clockcount;
uint32_t trak_clockdemon, trak_clockcount;
uint32_t meta_clockdemon, meta_clockcount;
uint32_t basemetadataduration = 0;


#define MAKETAG(d,c,b,a)		(((d&0xff)<<24)|((c&0xff)<<16)|((b&0xff)<<8)|(a&0xff))


void TestMP4Index(char *basename)
{
	int lastsize = 0, nest = 0, nestsize = 0;
	FILE *fp = fopen(basename, "rb");
	if (fp)
	{
		uint32_t tag, tsize, qttag, qtsize, len, skip, type = 0, num;

		len = fread(&tag, 1, 4, fp);

		if (tag == MAKEID('D', 'E', 'V', 'C') || tag == MAKEID('R', 'M', 'R', 'K')) // RAW GPMF data
		{
			int num;
			int ssize;
			int bytes;

			while (tag == MAKEID('D', 'E', 'V', 'C') || tag == MAKEID('R', 'M', 'R', 'K'))
			{
				if (tag == MAKEID('R', 'M', 'R', 'K')) videolength += 1.0, metadatalength += 1.0;
				len += fread(&tsize, 1, 4, fp);

				num = TAG_SAMPLES(tsize);
				ssize = TAG_SAMPLE_SIZE(tsize);
				bytes = num * ssize;

				len += (bytes + 3)&~3;

				LONGSEEK(fp, (bytes + 3)&~3, SEEK_CUR);

				tag = 0;
				len += fread(&tag, 1, 4, fp);
			}

			if (videolength)
			{
				int devices = 0,index = 0,payloadstart;
				indexcount = (int)metadatalength;
				LONGSEEK(fp, 0, SEEK_SET); // back to start

				metasizes = (uint32_t *)malloc(indexcount * 4 + 4);  memset(metasizes, 0, indexcount * 4 + 4);
				metaoffsets = (uint32_t *)malloc(indexcount * 4 + 4);  memset(metaoffsets, 0, indexcount * 4 + 4);

				len = fread(&tag, 1, 4, fp);
				while (tag == MAKEID('D', 'E', 'V', 'C') || tag == MAKEID('R', 'M', 'R', 'K'))
				{
					if (tag == MAKEID('R', 'M', 'R', 'K'))
					{
						if (devices> 0)
						{
							metasizes[index] = len - 4 - payloadstart;
							metaoffsets[index] = payloadstart;
							index++;
						}
						devices = 0;
					}
					if (tag == MAKEID('D', 'E', 'V', 'C'))
					{
						if (devices == 0)
						{
							payloadstart = len - 4;
						}
						devices++;
					}
					len += fread(&tsize, 1, 4, fp);

					num = TAG_SAMPLES(tsize);
					ssize = TAG_SAMPLE_SIZE(tsize);
					bytes = num * ssize;

					len += (bytes + 3)&~3;

					LONGSEEK(fp, (bytes + 3)&~3, SEEK_CUR);

					tag = 0;
					len += fread(&tag, 1, 4, fp);
				}
				if (devices> 0)
				{
					metasizes[index] = len - payloadstart;
					metaoffsets[index] = payloadstart;
					index++;
				}
			}

			return;  // not an MP4, RAW GPMF
		}
		LONGSEEK(fp, 0, SEEK_SET); // back to start

		do
		{

			len = fread(&qtsize, 1, 4, fp);
			len += fread(&qttag, 1, 4, fp);
			if (len == 8)
			{
				char spaces[40] = "                                      ";
				char atom[64];
				qtsize = BSWAP32(qtsize);

				if (!TAG_VALID_FOURCC(qttag))
				{
					LONGSEEK(fp, lastsize - 8 - 8, SEEK_CUR);
					nest--;
					continue;
				}
				else
					nest++;

				lastsize = qtsize;

				//		spaces[nest] = 0;
				spaces[0] = 0;
				sprintf(atom, "%s%c%c%c%c %d\n", spaces, qttag & 0xff, (qttag >> 8) & 0xff, (qttag >> 16) & 0xff, (qttag >> 24) & 0xff, qtsize);
#if DEBUG
				printf(atom);
#endif
				if (qttag != MAKEID('m', 'o', 'o', 'v') &&
					qttag != MAKEID('m', 'v', 'h', 'd') &&
					qttag != MAKEID('t', 'r', 'a', 'k') &&
					qttag != MAKEID('m', 'd', 'i', 'a') &&
					qttag != MAKEID('m', 'd', 'h', 'd') &&
					qttag != MAKEID('m', 'i', 'n', 'f') &&
					qttag != MAKEID('g', 'm', 'i', 'n') &&
					qttag != MAKEID('g', 'p', 'm', 'd') &&
					qttag != MAKEID('d', 'i', 'n', 'f') &&
					qttag != MAKEID('a', 'l', 'i', 's') &&
					qttag != MAKEID('s', 't', 's', 'd') &&
					qttag != MAKEID('s', 't', 's', 's') &&
					qttag != MAKEID('s', 't', 's', 'c') &&
					qttag != MAKEID('a', 'l', 'i', 's') &&
					qttag != MAKEID('a', 'l', 'i', 's') &&
					qttag != MAKEID('s', 't', 'b', 'l') &&
					qttag != MAKEID('s', 't', 't', 's') &&
					qttag != MAKEID('s', 't', 's', 'z') &&
					qttag != MAKEID('s', 't', 'c', 'o') &&
					qttag != MAKEID('u', 'd', 't', 'a') &&
					qttag != MAKEID('F', 'I', 'R', 'M') &&
					qttag != MAKEID('h', 'd', 'l', 'r'))
				{
					LONGSEEK(fp, qtsize - 8, SEEK_CUR);
					nest--;
				}
				else if (qttag == MAKEID('F', 'I', 'R', 'M'))
				{
					len = fread(camera_firmware, 1, (qtsize-8 < 255 ? qtsize-8: 255), fp);
					LONGSEEK(fp, qtsize - len - 8, SEEK_CUR); // skip over FIRM
					nest--;
				}
				else if (qttag == MAKEID('m', 'v', 'h', 'd')) //mvhd
				{

					len = fread(&skip, 1, 4, fp);
					len += fread(&skip, 1, 4, fp);
					len += fread(&skip, 1, 4, fp);
					len += fread(&clockdemon, 1, 4, fp); clockdemon = BSWAP32(clockdemon);
					len += fread(&clockcount, 1, 4, fp); clockcount = BSWAP32(clockcount);
					if (len == 20)
					{
						videolength = (float)((double)clockcount / (double)clockdemon);
					}
					LONGSEEK(fp, qtsize - 8 - len, SEEK_CUR); // skip over mvhd
					nest--;
				}
				else if (qttag == MAKEID('m', 'd', 'h', 'd')) //mdhd  media header
				{
					media_header md;
					len = fread(&md, 1, sizeof(md), fp);
					if (len == sizeof(md))
					{
						md.creation_time = BSWAP32(md.creation_time);
						md.modification_time = BSWAP32(md.modification_time);
						md.time_scale = BSWAP32(md.time_scale);
						md.duration = BSWAP32(md.duration);

						trak_clockdemon = md.time_scale;
						trak_clockcount = md.duration;

					}
					LONGSEEK(fp, qtsize - 8 - len, SEEK_CUR); // skip over mvhd
					nest--;
				}
				else if (qttag == MAKEID('h', 'd', 'l', 'r')) //hldr
				{
					len = fread(&skip, 1, 4, fp);
					len += fread(&skip, 1, 4, fp);
					len += fread(&type, 1, 4, fp);
					if (len == 12)
					{
#if DEBUG
						printf("type %c%c%c%c\n", type & 0xff, (type >> 8) & 0xff, (type >> 16) & 0xff, (type >> 24) & 0xff );
#endif

						if (type == MAKEID('m', 'e', 't', 'a') || type == MAKEID('g', 'p', 'm', 'f') || type == MAKEID('t', 'e', 'x', 't')) // meta || gpmf || text
						{
							int namelen = 0;
							LONGSEEK(fp, 12, SEEK_CUR), len += 12; // skip to "GoPro MET"
							fread(&namelen, 1, 1, fp), len++;
							if (namelen >= 9)
							{
								char name[10];
								fread(&name, 1, 9, fp), len += 9;

								if (0 == strncmp("GoPro TXT", name, 9))
								{
									subtitlingTrack = 1;
								}

								if (0 != strncmp("GoPro MET", name, 9) && 0 != strncmp("GoPro TXT", name, 9))
									type = 0; // not GoPro Metadata
							}

						}
					}
					LONGSEEK(fp, qtsize - 8 - len, SEEK_CUR); // skip over hldr
					nest--;
				}
				else if (qttag == MAKEID('s', 't', 's', 'z')) // metadata stsz - sizes
				{
					if (type == MAKEID('m', 'e', 't', 'a') || type == MAKEID('g', 'p', 'm', 'f') || type == MAKEID('t', 'e', 'x', 't')) // meta || gpmf || text
					{
						len = fread(&skip, 1, 4, fp);
						len += fread(&skip, 1, 4, fp);
						len += fread(&num, 1, 4, fp);
						num = BSWAP32(num);
						if ((metasizes = (uint32_t *)malloc(num * 4)))
						{
						  len += fread(metasizes, 1, num*4, fp);
						  do
						  {
							  num--;
							  metasizes[num] = BSWAP32(metasizes[num]);
						  } while (num > 0);
						}
						LONGSEEK(fp, qtsize - 8 - len, SEEK_CUR); // skip over stsz
					}
					else
						LONGSEEK(fp, qtsize - 8, SEEK_CUR);

					nest--;
				}
				else if (qttag == MAKEID('s', 't', 'c', 'o')) // metadata stco - offsets
				{
					if (type == MAKEID('m', 'e', 't', 'a') || type == MAKEID('g', 'p', 'm', 'f') || type == MAKEID('t', 'e', 'x', 't')) // meta || gpmf || text
					{
						len = fread(&skip, 1, 4, fp);
						len += fread(&num, 1, 4, fp);
						num = BSWAP32(num);
						indexcount = num;
						if ((metaoffsets = (uint32_t *)malloc(num * 4)))
						{
							len += fread(metaoffsets, 1, num * 4, fp);
							do
							{
								num--;
								metaoffsets[num] = BSWAP32(metaoffsets[num]);
							} while (num > 0);
						}
						LONGSEEK(fp, qtsize - 8 - len, SEEK_CUR); // skip over stco
					}
					else
						LONGSEEK(fp, qtsize - 8, SEEK_CUR);

					nest--;
				}
				else if (qttag == MAKEID('s', 't', 't', 's')) // time to samples
				{
					if (type == MAKEID('m', 'e', 't', 'a') || type == MAKEID('g', 'p', 'm', 'f') || type == MAKEID('t', 'e', 'x', 't')) // meta || gpmf || text
					{
						int entries = 0;
						len = fread(&skip, 1, 4, fp);
						len += fread(&num, 1, 4, fp);
						num = BSWAP32(num);
						entries = num;

						meta_clockdemon = trak_clockdemon;
						meta_clockcount = trak_clockdemon;

						while (entries > 0)
						{
							int samplecount;
							int duration;
							len += fread(&samplecount, 1, 4, fp);
							samplecount = BSWAP32(samplecount);
							len += fread(&duration, 1, 4, fp);
							duration = BSWAP32(duration);

							if (samplecount > 1)
								basemetadataduration = duration;
							entries--;

							metadatalength += (float)((double)samplecount * (double)duration / (double)meta_clockdemon);
						}
						LONGSEEK(fp, qtsize - 8 - len, SEEK_CUR); // skip over stco
					}
					else
						LONGSEEK(fp, qtsize - 8, SEEK_CUR);

					nest--;
				}
				else
				{
					nestsize = qtsize;
				}
			}
			else
				break;

		} while (1);

		fclose(fp);
	}
}


//
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
// Modified function of Dump2JSON(char *basename)
// so that it writes to arrays directly.
//List of HACKS KEVIN COOPMAN added
// 1)

#if 0
typedef struct accelerometer
{
  float sampling_rate;
  int num_samples;
  float * t;
  float * x;
  float * y;
  float * z;
  //for intialization
  float x_bias;
  float y_bias;
  float z_bias;
} Accelerometer_t;

typedef struct gyro
{
  float sampling_rate;
  int   num_samples;
  float * t;
  float * x;
  float * y;
  float * z;
  //for intialization
  float x_bias;
  float y_bias;
  float z_bias;
} Gyro_t;

#endif

//********************************************************************************************************
//********************************************************************************************************
//********************************************************************************************************
int ExtractIMU(char *basename, IMU_t * imu)
{

    // Initialize values
    metasizes = NULL;
    metaoffsets = NULL;
    metasampletime = NULL;
    indexcount = 0;
    videolength = 0.0;
    metadatalength = 0.0;
    subtitlingTrack = 0;
    basemetadataduration = 0;


	FILE *fp;//*fpout;
	//char filename[256];
	int accl_index = 0;
 	int gyro_index = 0;
 	float SCAL_factor = 1.0f;
 	float start_time;
 	float end_time;
  //float time_interval;
  int number_samples;

	uint32_t tag,tsize;
	char type;

	TestMP4Index(basename);

	if(indexcount == 0)
	{
		//sprintf(filename, "%s.json", basename);
		printf("no GPMF track in %s\n",basename);
        // SXLogW("No GPMF track in %s", basename);
		return 1;
	}
	fp = fopen(basename,"rb");
	if(fp)
	{
		int pos = 0;
		int comma = 0;
		int nestlevel = 0;
		int nestsize[16] = {0};
		uint32_t payload = 0;


		//sprintf(filename, "%s.json", basename);
		//fpout = fopen(filename,"w");
		//if(fpout)
		{

		 nestlevel++; expectedstructsize = 0; expectedscalesize = 0; expectedelementcount = 0;

			if (nestlevel == 1)
			{

				if (metasizes && metaoffsets && indexcount)
				{
					uint32_t i;

//					printf("%d seconds of metadata\n", indexcount);
          //OK, we have the seconds of metadata so lets allocate memory and start loading it up.
					//Assume sampling rate is 200Mhz and 400Mhz
					#define PAD_TIME 5
					imu->accl.sampling_rate = 200;
					imu->accl.num_samples   = imu->accl.sampling_rate*(indexcount);  //I added one second padding since above is not float
					imu->gyro.sampling_rate = 400;
					imu->gyro.num_samples   = imu->gyro.sampling_rate*(indexcount);

//                    printf("%d samples of accl\n", imu->accl.num_samples);
//                    printf("%d samples of gyro\n", imu->gyro.num_samples);

          //allocate memory now:
					imu->accl.t = NULL;
					imu->accl.x = NULL;
					imu->accl.y = NULL;
					imu->accl.z = NULL;
					imu->gyro.t = NULL;
					imu->gyro.x = NULL;
					imu->gyro.y = NULL;
					imu->gyro.z = NULL;
					imu->accl.t = (float *) malloc(4*(imu->accl.num_samples+imu->accl.sampling_rate*(indexcount+PAD_TIME)));    if( imu->accl.t == NULL) { printf( "could not allocate memory and will exit\n"); return 1;}
					imu->accl.x = (float *) malloc(4*(imu->accl.num_samples+imu->accl.sampling_rate*(indexcount+PAD_TIME)));     if( imu->accl.x == NULL) { printf( "could not allocate memory and will exit\n"); return 1;}
					imu->accl.y = (float *) malloc(4*(imu->accl.num_samples+imu->accl.sampling_rate*(indexcount+PAD_TIME)));     if( imu->accl.y == NULL) { printf( "could not allocate memory and will exit\n"); return 1;}
					imu->accl.z = (float *) malloc(4*(imu->accl.num_samples+imu->accl.sampling_rate*(indexcount+PAD_TIME)));     if( imu->accl.z == NULL) { printf( "could not allocate memory and will exit\n"); return 1;}
					imu->gyro.t = (float *) malloc(4*(imu->gyro.num_samples+imu->gyro.sampling_rate*(indexcount+PAD_TIME)));     if( imu->gyro.t == NULL) { printf( "could not allocate memory and will exit\n"); return 1;}
					imu->gyro.x = (float *) malloc(4*(imu->gyro.num_samples+imu->gyro.sampling_rate*(indexcount+PAD_TIME)));      if( imu->gyro.x == NULL) { printf( "could not allocate memory and will exit\n"); return 1;}
					imu->gyro.y = (float *) malloc(4*(imu->gyro.num_samples+imu->gyro.sampling_rate*(indexcount+PAD_TIME)));      if( imu->gyro.y == NULL) { printf( "could not allocate memory and will exit\n"); return 1;}
					imu->gyro.z = (float *) malloc(4*(imu->gyro.num_samples+imu->gyro.sampling_rate*(indexcount+PAD_TIME)));      if( imu->gyro.z == NULL) { printf( "could not allocate memory and will exit\n"); return 1;}


					for (i = 0; i < indexcount; i++)
					{
						uint32_t devc = 0;
						if (subtitlingTrack) metaoffsets[i] += 2;
						LONGSEEK(fp, metaoffsets[i], SEEK_SET);

						fread(&devc, 1, 4, fp);  devc = BSWAP32(devc);

						if (devc != 0x44455643)
						{
						//	fprintf(fpout, "  \"ERROR\": \"GPMF payload between %d and %d seconds is corrupt, offset: % 08x size: %d bytes\",\n", i, i + 1, metaoffsets[i], metasizes[i]);
#ifdef DEBUG
							printf("  \"ERROR\": \"GPMF payload between %d and %d seconds is corrupt, offset: % 08x size: %d bytes\",\n", i, i + 1, metaoffsets[i], metasizes[i]);
#endif
						}
						if (devc != 0x44455643) metaoffsets[i] = 0;
					}

					LONGSEEK(fp, 0, SEEK_SET);
				}
			}

#ifdef DEBUG
			printf("{");
#endif
			nestsize[nestlevel] = 0;

			do
			{
				int len;
				tag = tsize = 0;

				if (nestlevel == 1)
				{

					if (payload > 0 && payload < indexcount && nestsize[nestlevel & 15] <= 0)
					{
						//fprintf(fpout, ",");
#ifdef DEBUG
						printf(",");
#endif
					}
					while(metaoffsets[payload] == 0 && payload < indexcount)
						payload++;

					if(payload >= indexcount)
						break;

					if(payload < indexcount)
					{
						if (nestsize[nestlevel & 15] <= 0)
						{
							LONGSEEK(fp, metaoffsets[payload], SEEK_SET);

							nestsize[nestlevel & 15] = metasizes[payload] & ~0x3;
							if (metaoffsets[payload] == 0x15c76c09)
							{
								len = 0;
							}

							payload++;
						}
					}

					{
						uint32_t *buf = (uint32_t *)malloc(nestsize[nestlevel & 15]);
						uint32_t ret;

						fread(buf, 1, nestsize[nestlevel & 15], fp);
						LONGSEEK(fp, -nestsize[nestlevel & 15], SEEK_CUR);

						ret = MetadataIsValidGPMF(buf, nestsize[nestlevel & 15], 1);

						free(buf);
					}
				}

				len = fread(&tag, 1, 4, fp);  pos += len;
				len = fread(&tsize, 1, 4, fp);  pos += len;

				nestsize[nestlevel & 15] -= 8;
				type = TAG_TYPE(tsize);

				if (!TAG_VALID_FOURCC(tag))
				{
					int recovered = 0,cc = 0;
					int tabs = nestlevel;
#ifdef DEBUG
//					if(comma)
//						fprintf(fp,","), printf(",");
//					fprintf(fp,"\n"), printf("\n");
//					while(tabs-- > 0) fprintf(fpout,"  "), printf(" ");
#else
					//if (comma)
						//fprintf(fpout, ",");
					//fprintf(fpout, "\n");
					//while (tabs-- > 0) fprintf(fpout, "  ");
#endif
					//fprintf(fpout, "\"ERROR TAG(%08x)\": \"Invalid tag, %d bytes not processed\"", tag, nestsize[nestlevel & 15]);
#ifdef DEBUG
//					printf("ERROR TAG(%08x): \"Invalid tag, %d bytes not processed\"", tag, nestsize[nestlevel & 15]);
#endif

					while (nestsize[nestlevel & 15] > 0 && !recovered)
					{
						len = fread(&tag, 1, 4, fp);  pos += len;  cc += len;
						nestsize[nestlevel & 15] -= 4;

						if (TAG_VALID_FOURCC(tag))
						{
							len = fread(&tsize, 1, 4, fp);  pos += len;  cc += len;
							nestsize[nestlevel & 15] -= 4;

							if (TAG_DATA_SIZE(tsize) <= nestsize[nestlevel & 15])
							{
								tabs = nestlevel;
								type = TAG_TYPE(tsize);
								recovered = 1;

						//		if (comma)
							//		fprintf(fpout, ",");
							//	fprintf(fpout, "\n");
							//	while (tabs-- > 0) fprintf(fpout, "  ");

								//fprintf(fpout, "\"Recovered %c%c%c%c\": \"after %d bytes\"", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff, cc);
#ifdef DEBUG
//								printf("Recovered %c%c%c%c: \"after %d bytes\"", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff, cc);
#endif
							}
						}
					}

					if (!recovered)
					{
						LONGSEEK(fp, (nestsize[nestlevel & 15] + 3)&~3, SEEK_CUR);
						nestsize[nestlevel & 15] = 0, tag = 1; // to stay with the do-while
						goto resetnest;
					}
				}

				if(tag)
				{
					//int tabs = nestlevel;
					int num = TAG_SAMPLES(tsize);              //number of samples
					int ssize = TAG_SAMPLE_SIZE(tsize);
					int mtsize = MetadataTypeSize(type);
					number_samples  = num ; //KEVIN



#ifdef DEBUG
//					if(comma)
//						fprintf(fpout,","), printf(",");
//					fprintf(fpout,"\n"), printf("\n");
//					while(tabs-- > 0) fprintf(fpout,"  "), printf(" ");
#else
					//if(comma)
						//fprintf(fpout,",");
					//fprintf(fpout,"\n");
					//while(tabs-- > 0) fprintf(fpout,"  ");
#endif


					if (ssize <= 0 || num*ssize > nestsize[nestlevel & 15])
					{
						//fprintf(fpout, "\"ERROR %c%c%c%c\": \"Invalid tsize/repeat: %d samples of %d size offset %08x\",\n", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff, num, ssize, metaoffsets[payload - 1]);
#ifdef DEBUG
						printf("Invalid tsize/repeat %08x\n", metaoffsets[payload-1]);
						printf("ERROR %c%c%c%c: \"Invalid tsize/repeat: %d samples of %d size offset %08x\"\n", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff, num, ssize, metaoffsets[payload - 1]);
#endif
						nestsize[nestlevel & 15] = 0, tag = 1; // to stay with the do-while

						goto resetnest;
					}

					if (type > 0 && type != 'c' && type != '?' && ssize / mtsize > 1)
					{
						//int arraysize = ssize / mtsize;


             //printf("Number of samples is %i", number_samples);


						if (mtsize > 1)
						{
							//fprintf(fpout, "\"structure (%c%c%c%c)\": \"%d-axis %d samples\",\n", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff, arraysize, num);
							//tabs = nestlevel; while (tabs-- > 0) fprintf(fpout, "  ");
							//fprintf(fpout, "\"data type (%c%c%c%c)\": \"'%c' (%s)\",\n", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff, type, Type2String(type));
							//tabs = nestlevel; while (tabs-- > 0) fprintf(fpout, "  ");
						}
						else
						{
							//fprintf(fpout, "\"structure (%c%c%c%c)\": \"%d-bytes %d samples\",\n", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff, ssize, num);
							//tabs = nestlevel; while (tabs-- > 0) fprintf(fpout, "  ");
						}
					}

					//fprintf(fpout, "\"%c%c%c%c\": ", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff);
#ifdef DEBUG
//					printf("\"%c%c%c%c\": ", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff);
#endif

//  KEVIN IS ADDING THIS
        char four_cccc[4];
				four_cccc[0] =  (tag >> 24) & 0xff;
				four_cccc[1] =  (tag >> 16) & 0xff;
				four_cccc[2] =  (tag >> 8) & 0xff;
				four_cccc[3] =  (tag >> 0) & 0xff;

         //THIS IS THE FOURCC STUFF -------------------------------------------------------------------------------------
				 //KEVIN KEVIN KEVIN
				 //"KEVIN ACCL":
					//printf("\"KEVIN %c%c%c%c\": ", (tag >> 0) & 0xff, (tag >> 8) & 0xff, (tag >> 16) & 0xff, (tag >> 24) & 0xff);
					//num is the number of samples ------------------ NEED START TIME AND END TIME

					if (num == 0)
					{
						//fprintf(fpout, "\"No data\"");
#ifdef DEBUG
//						printf("\"No data\"");
#endif
						goto resetnest;
					}

					switch(type)
					{
					case 0:
						nestsize[nestlevel & 15] -= TAG_SAMPLES(tsize) * TAG_SAMPLE_SIZE(tsize);
						//fprintf(fpout, "{");
						 nestlevel++; expectedstructsize = 0; expectedscalesize = 0; expectedelementcount = 0;
#ifdef DEBUG
//						printf("{");
#endif

						if(tag == 0x43564544 ) //0x44455643)
						{

              start_time = (float)(payload - 1) *  (float)basemetadataduration / (float)meta_clockdemon;
							end_time   = (float)payload * (float)basemetadataduration / (float)meta_clockdemon;

              //printf("KEVIN  time interval  = %1.3f, %1.3f \n", start_time, end_time);// number_samples);
//printf("222Number of samples is %i\n", num);

							//fprintf(fpout, "\n    \"TIME\": [%1.3f, %1.3f],\n", (float)(payload - 1) *  (float)basemetadataduration / (float)meta_clockdemon, (float)payload * (float)basemetadataduration / (float)meta_clockdemon);
#ifdef DEBUG
//							printf("\n    \"TIME\": [%1.3f, %1.3f],", (float)(payload - 1) *  (float)basemetadataduration / (float)meta_clockdemon, (float)payload * (float)basemetadataduration / (float)meta_clockdemon);
#endif
						}


						nestsize[nestlevel & 15] = TAG_SAMPLES(tsize) * TAG_SAMPLE_SIZE(tsize);
						comma = 0;
						break;
					case 'j':
					case 'J':
					case 'l':
					case 'L':
					case 'H':
					case 'S':
					case 's':
					case 'b':
					case 'B':
						{
							int samplesize = MetadataTypeSize(type);
							int num = TAG_SAMPLES(tsize);
							int ssize = TAG_SAMPLE_SIZE(tsize);
							int bytes = num * ssize;
							int i, samples = bytes / samplesize;
							uint32_t val;
							int32_t *sval;



							if (((ssize / samplesize) * samplesize) != ssize)
							{
								//fprintf(fpout, "\"ERROR\": \"Type '%c' is wrong for the provided sample size of %d\"\n", type, ssize);
#ifdef DEBUG
//								printf("\nERROR\": \"Type '%c' is wrong for the provided sample size of %d\"\n", type, ssize);
#endif
								type = 'B';
								samplesize = 1;
								samples = bytes;
							}

							if (samples > 4096)
							{
								//fprintf(fpout, "\"ERROR\": \"likely data corruption\"\n");
#ifdef DEBUG
//								printf("\nERROR: likely data corruption\n");
#endif
								nestsize[nestlevel & 15] = 0;
								goto resetnest;
							}


//OK, we are here -----------------------------------------------------------------------
//Lets get the samples
							//if (samples > 1)  fprintf(fpout, "[");
#ifdef DEBUG
//							if (samples > 1)  printf("[");
#endif

							for (i = 0; i < samples; i++)
							{
								if (samplesize == 8)
								{
									uint64_t val64;
									len = fread(&val64, 1, samplesize, fp);    pos += len;
									swapmem64(&val64);
									val = (uint32_t)val64;
									//fprintf(fpout, "%d", val);
								}
								else
								{

									len = fread(&val, 1, samplesize, fp);
									pos += len;
#if DOSWAP
									val = BYTESWAP(val, samplesize);
#endif

									if (type == 'b' || type == 's' || type == 'l') // signed
									{
										sval = (int32_t *)&val;
										if (type == 'b')
										{
											*sval <<= 24;
											*sval >>= 24;
										}
										if (type == 's')
										{
											*sval <<= 16;
											*sval >>= 16;
										}

										//fprintf(fpout, "%d", *sval);

										//KEVIN ADDED THIS !!!!!!!!!!!!!!!!!!!!!! THIS IS HOW YOU CAN THE DIFFERENT TYPES
										//NEED TO ADD GPS I THINK
										if( (four_cccc[3] == 'S') && (four_cccc[2] == 'C') && (four_cccc[1] == 'A') && (four_cccc[0] == 'L') )
										{
											SCAL_factor = (float)(*sval);
										}//----------------------------------------------------------------------------------------------------------
										if( (four_cccc[3] == 'A') && (four_cccc[2] == 'C') && (four_cccc[1] == 'C') && (four_cccc[0] == 'L') )
										{

                      float interval = 3*(end_time - start_time)/(float)samples;
											imu->accl.t[accl_index] = start_time + (i/3)*interval;
											imu->accl.z[accl_index] = (float)*sval/SCAL_factor;
											//next sample
											len = fread(&val, 1, samplesize, fp);
											pos += len;
									#if DOSWAP
											val = BYTESWAP(val, samplesize);
									#endif
												sval = (int32_t *)&val;
												if (type == 'b')
												{
													*sval <<= 24;
													*sval >>= 24;
												}
												if (type == 's')
												{
													*sval <<= 16;
													*sval >>= 16;
												}
											imu->accl.x[accl_index] = (float)*sval/SCAL_factor;
											//next sample
											len = fread(&val, 1, samplesize, fp);
											pos += len;
									#if DOSWAP
											val = BYTESWAP(val, samplesize);
									#endif
												sval = (int32_t *)&val;
												if (type == 'b')
												{
													*sval <<= 24;
													*sval >>= 24;
												}
												if (type == 's')
												{
													*sval <<= 16;
													*sval >>= 16;
												}
											imu->accl.y[accl_index++] = (float)*sval/SCAL_factor;  //NOTE THE INCREMENT
                      i += 2;  //We did 2 extra samples
										} //-----------------------------------------------------------------------------------------------------------------
										if( (four_cccc[3] == 'G') && (four_cccc[2] == 'Y') && (four_cccc[1] == 'R') && (four_cccc[0] == 'O') )
										{
											float interval = 3*(end_time - start_time)/(float)samples;
											imu->gyro.t[gyro_index] = start_time + (i/3)*interval;
											imu->gyro.z[gyro_index] = (float)*sval/SCAL_factor;
											//next sample
											len = fread(&val, 1, samplesize, fp);
											pos += len;
										#if DOSWAP
											val = BYTESWAP(val, samplesize);
										#endif
												sval = (int32_t *)&val;
												if (type == 'b')
												{
													*sval <<= 24;
													*sval >>= 24;
												}
												if (type == 's')
												{
													*sval <<= 16;
													*sval >>= 16;
												}
											imu->gyro.x[gyro_index] = (float)*sval/SCAL_factor;
											//next sample
											len = fread(&val, 1, samplesize, fp);
											pos += len;
										#if DOSWAP
											val = BYTESWAP(val, samplesize);
										#endif
												sval = (int32_t *)&val;
												if (type == 'b')
												{
													*sval <<= 24;
													*sval >>= 24;
												}
												if (type == 's')
												{
													*sval <<= 16;
													*sval >>= 16;
												}
											imu->gyro.y[gyro_index++] = (float)*sval/SCAL_factor;  //NOTE THE INCREMENT
											i += 2;  //We did 2 extra samples
										}
										//if( (four_cccc[3] == 'G') && (four_cccc[2] == 'P') && (four_cccc[1] == 'S') && (four_cccc[0] == '5') )
										//{
										//}
										//if( gyro_index > 269200)
										//printf("gyro_index = %i", gyro_index);

									}
									else
									{
										if (type == 'H')
										{
											//fprintf(fpout, "0x%08X", val);
											//printf("KEVIN 0x%08X", val);
										}
										else
										{
											//fprintf(fpout, "%d", val);

											//printf("KEVIN %d \n", val);

										}
									}
								}

								if (i + 1 < samples)
								{
									//fprintf(fpout,", ");
#ifdef DEBUG
									printf(", ");
#endif
								}
							}
							//if (samples > 1) fprintf(fpout, "]");
#ifdef DEBUG
							if (samples > 1) printf("]");
#endif

							if (bytes & 3)
							{
								int buf;
								len = fread(&buf, 1, 4 - (bytes & 3), fp);  pos += len;
							}

							nestsize[nestlevel & 15] -= (bytes + 3)&~3;
							comma = 1;
						}
						break;
					case 'f':
						{
							int num = TAG_SAMPLES(tsize);
							int ssize = TAG_SAMPLE_SIZE(tsize);
							int bytes = num * ssize;
							int i,floats = bytes / 4;
							uint32_t val;
							float *fval;


							if (floats > 4096)
							{
							//	fprintf(fpout, "\"ERROR\": \"likely data corruption\"\n");
#ifdef DEBUG
								printf("\nERROR: likely data corruption\n");
#endif
								nestsize[nestlevel & 15] = 0;
								goto resetnest;
							}

							if(floats > 1)
							{
								//fprintf(fpout,"[");
#ifdef DEBUG
								printf("[");
#endif
							}

							for(i = 0; i < floats; i++)
							{
								len = fread(&val, 1, 4, fp);  pos += len;
#if DOSWAP
								val = BYTESWAP(val,4);
#endif
								fval = (float *)&val;

								//if(-0.0001 < *fval && *fval < 0.0001)
									//fprintf(fpout,"%1.7f", *fval);
								//else
									//fprintf(fpout, "%1.4f", *fval);

#ifdef DEBUG
								printf("%f", *fval);
#endif

								if(i+1 < floats)
								{
								//	fprintf(fpout,", ");
#ifdef DEBUG
									printf(", ");
#endif
								}
							}

							if(floats > 1)
							{
								//fprintf(fpout,"]");
#ifdef DEBUG
								printf("]");
#endif
							}
							nestsize[nestlevel & 15] -= (bytes + 3)&~3;
							comma = 1;
						}
						break;
					case 'c':
						{
							char stringbuf[STREAM_BUFFER_MAX];
							int i,j,num = TAG_SAMPLES(tsize);
							int ssize = TAG_SAMPLE_SIZE(tsize);
							int bytes = num * ssize;

							if (METADATA_TAG_UNITS == tag || METADATA_TAG_SI_UNITS == tag)
								i=0;

							if(MAKEID('F','I','L','E') == tag)
							{
								i =0;
							}

							if(ssize > 1 && num > 1) // it is an array of strings
							{
								//fprintf(fpout,"[");
#ifdef DEBUG
								printf("[");
#endif


								if (ssize * num > STREAM_BUFFER_MAX)
								{
									//fprintf(fpout, "\"ERROR\": \"buffer overflow, likely data corruption\"\n");
#ifdef DEBUG
									printf("\nERROR: buffer overflow, likely data corruption\n");
#endif
									nestsize[nestlevel & 15] = 0;
									goto resetnest;
								}

								len = fread(&stringbuf, 1, ssize * num, fp);  pos += len;
								for(i = 0; i < num; i++)
								{
									char tmp[1024];
									int outsize = 0;
									//len = fread(&stringbuf, 1, ssize, fp);  pos += len;
									for(j=0;j<ssize; j++)
									{
										tmp[outsize++] = stringbuf[i*ssize + j];
#if 0
 #ifndef WIN32  //Assuming the Linux hosted version
										if (tmp[outsize - 1] == '�') // power 2
										{
											tmp[outsize - 1] = 0;
											strcat(&tmp[outsize - 1], "^2"); outsize ++;
										}
										if (tmp[outsize - 1] == '�') // power 3
										{
											tmp[outsize - 1] = 0;
											strcat(&tmp[outsize - 1], "^3"); outsize ++;
 										}
 #endif
#endif
										if(stringbuf[i*ssize + j]=='\\')
											stringbuf[i*ssize + j] = '/'; //fix for file paths using a reserved character.
									}
									tmp[outsize] = 0;
									//fprintf(fpout,"\"%s\"", tmp);


#ifdef DEBUG
//									printf("\"%s\"", tmp); //deg""deg""m""m/s""m/s"
#endif

									if(i+1 < num)
									{
										//fprintf(fpout,", ");
#ifdef DEBUG
										printf(", ");
#endif
									}
								}
								stringbuf[num*ssize] = 0;

								if(bytes & 3)
								{
									if (bytes < STREAM_BUFFER_MAX)
									{
										len = fread(&stringbuf, 1, 4-(bytes&3), fp);  pos += len;
										for(i=0;i<bytes; i++)
										{
											if(stringbuf[i]=='\\')
												stringbuf[i]='/'; //fix for file paths using a reserved character.
										}
									}
									else
									{
										//fprintf(fpout, "\"ERROR\": \"buffer overflow, likely data corruption\"\n");
#ifdef DEBUG
//										printf("\nERROR: buffer overflow, likely data corruption\n");
#endif
										nestsize[nestlevel & 15] = 0;
										goto resetnest;
									}
								}

								nestsize[nestlevel & 15] -= (bytes+3)&~3;

						//		fprintf(fpout,"]");
#ifdef DEBUG
//								printf("]");   //KEVIN COMPLETED
#endif
								comma = 1;

							}
							else
							{
								if (bytes < STREAM_BUFFER_MAX)
								{
									len = fread(&stringbuf, 1, (bytes+3)&~3, fp), nestsize[nestlevel & 15] -= (bytes+3)&~3;  pos += len;
									for(i=0;i<bytes; i++)
									{
										if(stringbuf[i]=='\\')
											stringbuf[i]='/'; //fix for file paths using a reserved character.
									}
								}
								else
								{
							//		fprintf(fpout, "\"ERROR\": \"buffer overflow, likely data corruption\"\n");
#ifdef DEBUG
//									printf("\nERROR: buffer overflow, likely data corruption\n");
#endif
									nestsize[nestlevel & 15] = 0;
									goto resetnest;
								}

								stringbuf[bytes] = 0;
								//fprintf(fpout,"\"%s\"", stringbuf);


								//printf("KEVIN   \"%s\"", stringbuf);

								comma = 1;
							}


							// special cases
							if(METADATA_TAG_UNITS == tag || METADATA_TAG_SI_UNITS == tag)
							{
								expectedelementcount = num;
								currentunitcharsize = ssize;
								if(bytes < 256)
									memcpy(currentunits, stringbuf, bytes);
							}
							if(METADATA_TAG_TYPE == tag)
							{
								if (bytes != expectedelementcount && expectedelementcount > 0)
								{

				#ifdef DEBUG
//									if(comma)
//										fprintf(fpout,","), printf(",");
//									fprintf(fpout,"\n"), printf("\n");
//									while(tabs-- > 0) fprintf(fpout,"  "), printf(" ");
				#else
									//if(comma)
										//fprintf(fpout,",");
									//fprintf(fpout,"\n");
									//while(tabs-- > 0) fprintf(fpout,"  ");
				#endif

									//fprintf(fpout,"\"ERROR\": \"expected element count = %d (not %d)\"\n", expectedelementcount, bytes );
#ifdef DEBUG
//									printf("\nERROR: expected element count = %d (not %d)\n", expectedelementcount, bytes );
#endif
								}
								expectedelementcount = bytes;
								expectedstructsize = StructSize(stringbuf, bytes, expectedelementcount);
								if(bytes < 256)
									strncpy(expectedstructure, stringbuf, bytes);
							}
							if(METADATA_TAG_SCALE_TYPE == tag)
							{
								if (bytes != expectedelementcount && expectedelementcount > 0)
								{
				#ifdef DEBUG
//									if(comma)
//										fprintf(fpout,","), printf(",");
//									fprintf(fpout,"\n"), printf("\n");
//									while(tabs-- > 0) fprintf(fpout,"  "), printf(" ");
				#else
									//if(comma)
										//fprintf(fpout,",");
									//fprintf(fpout,"\n");
									//while(tabs-- > 0) fprintf(fpout,"  ");
				#endif
									//fprintf(fpout,"\"ERROR\": \"expected element count = %d (not %d)\"\n", expectedelementcount, bytes );
#ifdef DEBUG
									printf("\n\nERROR: expected element count = %d (not %d)\n", expectedelementcount, bytes );
#endif
								}
								expectedscalesize = StructSize(stringbuf, bytes, expectedelementcount);
								if(bytes < 256)
									strncpy(expectedscalestructure, stringbuf, bytes);
							}
						}
						break;

					case '?':
						{
							int i,num = TAG_SAMPLES(tsize);
							int ssize = TAG_SAMPLE_SIZE(tsize);
							int bytes = num * ssize;
							if(ssize > 1 && num >= 1) // it is an array of strings
							{
								unsigned char buf[4096];
								//fprintf(fpout,"[");
#ifdef DEBUG
								printf("[");
#endif
								for(i = 0; i < num; i++)
								{

									len = fread(&buf, 1, ssize, fp);  pos += len;
								//	DumpStructureValues(fpout, tag, buf, ssize);

									if(i+1 < num)
									{
									//	fprintf(fpout,", ");
#ifdef DEBUG
										printf(", ");
#endif
									}
								}

								if(bytes & 3)
								{
									if (bytes < STREAM_BUFFER_MAX)
									{
										len = fread(&buf, 1, 4-(bytes&3), fp);  pos += len;
									}
									else
									{

										nestsize[nestlevel & 15] = 0;
										goto resetnest;
									}
								}

								nestsize[nestlevel & 15] -= (bytes+3)&~3;

							//	fprintf(fpout,"]");
#ifdef DEBUG
								printf("]");
#endif
								comma = 1;
							}
							else
							{
								char stringbuf[STREAM_BUFFER_MAX];
								if (bytes < STREAM_BUFFER_MAX)
								{
									len = fread(&stringbuf, 1, (bytes+3)&~3, fp), nestsize[nestlevel & 15] -= (bytes+3)&~3;  pos += len;
									for(i=0;i<bytes; i++)
									{
										if(stringbuf[i]=='\\')
											stringbuf[i]='/'; //fix for file paths using a reserved character.
									}
								}
								else
								{
									//fprintf(fpout, "\"ERROR\": \"buffer overflow, likely data corruption\"\n");
#ifdef DEBUG
									printf("\nERROR: buffer overflow, likely data corruption\n");
#endif
									nestsize[nestlevel & 15] = 0;
									goto resetnest;
								}

								stringbuf[bytes] = 0;
					//			fprintf(fpout,"\"%s\"", stringbuf);
#ifdef DEBUG
								printf("\"%s\"", stringbuf);
#endif
								comma = 1;
							}


							// special cases
							if(METADATA_TAG_SCALE == tag)
							{

								if(expectedscalesize && ssize != expectedscalesize)
								{

								}
							}
							else
							{
								if(expectedstructsize && ssize != expectedstructsize)
								{

								}
							}
						}
						break;

					default: //TODO
						{
							int num = TAG_SAMPLES(tsize);
							int ssize = TAG_SAMPLE_SIZE(tsize);
							int bytes = num * ssize;
							char stringbuf[STREAM_BUFFER_MAX];
							if (bytes < STREAM_BUFFER_MAX)
							{
								len = fread(&stringbuf, 1, (bytes+3)&~3, fp), nestsize[nestlevel & 15] -= (bytes+3)&~3;   pos += len;
							}
							else
							{
						//		fprintf(fpout, "\"ERROR\": \"buffer overflow, likely data corruption\"\n");
#ifdef DEBUG
//								printf("\nERROR: buffer overflow, likely data corruption\n");
#endif
								nestsize[nestlevel & 15] = 0;
								goto resetnest;
							}

						//	fprintf(fpout,"\n\"ERROR\": \"unhandled data format\"\n");
#ifdef DEBUG
							printf("\"unhandled data format\"\n");
#endif
							comma = 1;
						}
						break;
					}
				}

			resetnest:
				while(nestsize[nestlevel & 15] <= 0 && nestlevel > 1)
				{
					int tabs;
					nestlevel--;
					tabs = nestlevel;

	//				fprintf(fpout,"\n");
#ifdef DEBUG
					printf("\n");
#endif
					while(tabs-- > 0)
					{
		//				fprintf(fpout,"  ");
#ifdef DEBUG
						printf("  ");
#endif
					}
			//		fprintf(fpout,"}");
#ifdef DEBUG
					printf("}");
#endif

					if(nestsize[nestlevel & 15] > 0 && nestlevel > 0)
					{
						comma = 1;
					}
					else
					{
						comma = 0;
					}
				}

			} while(tag);


			nestlevel--;
//			fprintf(fpout,"\n}\n");
#ifdef DEBUG
			printf("\n}\n");
#endif

			//fclose(fpout);
		}
		fclose(fp);
	}

	if (metasizes) free(metasizes);
	if (metaoffsets) free(metaoffsets);


//   printf("accl index = %i, gyro index = %i \n", accl_index, gyro_index);

   return 0;

}

//deallocate the memory
void DeAllocateImu(IMU_t * imu)
{

	//printf("deallocating memory Function\n");
	if( imu->accl.t) free(imu->accl.t);
	if( imu->accl.x) free(imu->accl.x);
	if( imu->accl.y) free(imu->accl.y);
	if( imu->accl.z) free(imu->accl.z);
	if( imu->gyro.t) free(imu->gyro.t);
	if( imu->gyro.x) free(imu->gyro.x);
	if( imu->gyro.y) free(imu->gyro.y);
	if( imu->gyro.z) free(imu->gyro.z);

}
