// $Header$

/*! @file gpmf2json_metadata.h

	@brief metadata control library include

	Copyright (c) 2016 GoPro Inc. All rights reserved.
	THIS SOURCE CODE IS THE PROPRIETARY INTELLECTUAL PROPERTY AND CONFIDENTIAL
	INFORMATION OF GOPRO, INC. AND IS PROTECTED UNDER U.S. AND INTERNATIONAL
	LAW. ANY USE OF THIS SOURCE CODE WITHOUT THE PRIOR WRITTEN AUTHORIZATION OF
	GOPRO IS STRICTLY PROHIBITED.

	@version 1.2.1
*/


#define MAKEID(a,b,c,d)		(((d&0xff)<<24)|((c&0xff)<<16)|((b&0xff)<<8)|(a&0xff))

#define BYTESWAP(a,s)		((s==4)?(((a&0xff)<<24)|((a&0xff00)<<8)|((a>>8)&0xff00)|((a>>24)&0xff)):((s==2)?(((a>>8)&0xff)|((a<<8)&0xff00)|((a>>8)&0xff0000)|((a<<8)&0xff000000)):(a)))

#define TAG_SAMPLES(a)		(((a>>24) & 0xff)|(((a>>16)&0xff)<<8))
#define TAG_SAMPLE_SIZE(a)	(((a)>>8)&0xff)
#define TAG_SAMPLE_TYPE(a)	(a&0xff)
#define MAKE_TYPE_SIZE_COUNT(t,s,c)		((t)&0xff)|(((s)&0xff)<<8)|(((c)&0xff)<<24)|(((c)&0xff00)<<8)
#define TAG_DATA_SIZE(a)	((TAG_SAMPLE_SIZE(a)*TAG_SAMPLES(a)+3)&~0x3)
#define TAG_DATA_PACKEDSIZE(a)	((TAG_SAMPLE_SIZE(a)*TAG_SAMPLES(a)))
#define TAG_VALID_FOURCC(a)	(((((a>>24)&0xff)>='a'&&((a>>24)&0xff)<='z') || (((a>>24)&0xff)>='A'&&((a>>24)&0xff)<='Z') || (((a>>24)&0xff)>='0'&&((a>>24)&0xff)<='9') || (((a>>24)&0xff)==' ') ) && \
							( (((a>>16)&0xff)>='a'&&((a>>24)&0xff)<='z') || (((a>>16)&0xff)>='A'&&((a>>16)&0xff)<='Z') || (((a>>16)&0xff)>='0'&&((a>>16)&0xff)<='9') || (((a>>16)&0xff)==' ') ) && \
							( (((a>>8)&0xff)>='a'&&((a>>24)&0xff)<='z') || (((a>>8)&0xff)>='A'&&((a>>8)&0xff)<='Z') || (((a>>8)&0xff)>='0'&&((a>>8)&0xff)<='9') || (((a>>8)&0xff)==' ') ) && \
							( (((a>>0)&0xff)>='a'&&((a>>24)&0xff)<='z') || (((a>>0)&0xff)>='A'&&((a>>0)&0xff)<='Z') || (((a>>0)&0xff)>='0'&&((a>>0)&0xff)<='9') || (((a>>0)&0xff)==' ') ))
#define TAG_TYPE(a)			(a&0xff)

#define TAGA(a) ((a>>24)&0xff)
#define TAGB(b) ((b>>16)&0xff)
#define TAGC(c) ((c>>8)&0xff)
#define TAGD(d) ((d)&0xff)



#define VALIDTAG(x)(((((x>>24)&0xff)>='A' && ((x>>24)&0xff)<='Z') || (((x>>24)&0xff)>='a' && ((x>>24)&0xff)<='z') || (((x>>24)&0xff)>='0' && ((x>>24)&0xff)<='9')) && \
					((((x>>16)&0xff)>='A' && ((x>>16)&0xff)<='Z') || (((x>>16)&0xff)>='a' && ((x>>16)&0xff)<='z') || (((x>>16)&0xff)>='0' && ((x>>16)&0xff)<='9')) && \
					((((x>>8)&0xff)>='A' && ((x>>8)&0xff)<='Z') || (((x>>8)&0xff)>='a' && ((x>>8)&0xff)<='z') || (((x>>8)&0xff)>='0' && ((x>>8)&0xff)<='9')) && \
					((((x>>0)&0xff)>='A' && ((x>>0)&0xff)<='Z') || (((x>>0)&0xff)>='a' && ((x>>0)&0xff)<='z') || (((x>>0)&0xff)>='0' && ((x>>0)&0xff)<='9')) )


typedef enum
{
	METADATA_TYPE_STRING = 'c', //single byte 'c' style character string
	METADATA_TYPE_STRING_ASCII = 'c', //single byte 'c' style character string
	METADATA_TYPE_SIGNED_BYTE = 'b',//single byte signed number
	METADATA_TYPE_UNSIGNED_BYTE = 'B', //single byte unsigned number
	METADATA_TYPE_DOUBLE = 'd', //64-bit double precision float (IEEE 754)
	METADATA_TYPE_SIGNED_64BIT_INT = 'j',//64-bit integer
	METADATA_TYPE_UNSIGNED_64BIT_INT = 'J', //64-bit integer
	METADATA_TYPE_FLOAT = 'f', //32-bit single precision float (IEEE 754)
	METADATA_TYPE_FOURCC = 'F', //32-bit four character tag
	METADATA_TYPE_GUID = 'G', //128-bit ID (like UUID)
	METADATA_TYPE_HIDDEN = 'h', //internal data not displayed (formatting not reported)
	METADATA_TYPE_UNSIGNED_LONG_HEX = 'H', //32-bit integer to be displayed 0xaabbccdd
	METADATA_TYPE_SIGNED_LONG = 'l',//32-bit integer
	METADATA_TYPE_UNSIGNED_LONG = 'L', //32-bit integer
	METADATA_TYPE_Q15_16_FIXED_POINT = 'q', // Q number Q15.16 - 16-bit signed integer (A) with 16-bit fixed point (B) for A.B value (range -32768.0 to 32767.99998).
	METADATA_TYPE_Q31_32_FIXED_POINT = 'Q', // Q number Q31.32 - 32-bit signed integer (A) with 32-bit fixed point (B) for A.B value.
	METADATA_TYPE_SIGNED_SHORT = 's',//16-bit integer
	METADATA_TYPE_UNSIGNED_SHORT = 'S',//16-bit integer
	METADATA_TYPE_STRING_UTF8 = 'u', //UTF-8 formatted text string.  As the character storage size varies, the size is in bytes, not UTF characters.
	METADATA_TYPE_UTC_DATE_TIME = 'U', //Date + UTC Time format yymmddhhmmss.sss - 16 bytes ASCII (years 20xx covered)
	METADATA_TYPE_XML = 'x', //XML, support other systems metadata
	METADATA_TYPE_COMPLEX= '?', //for sample with complex data structures, base size in bytes.  Data is either opaque, or the stream has a TYPE structure field for the sample.
	METADATA_TYPE_COMPLEX_SIZE_1 = '1', //1 byte of opaque data, no scale or unit definition -- avoid, please share metadata datatypes wherever possible.
	METADATA_TYPE_COMPLEX_SIZE_2 = '2', //2 bytes of opaque data, no scale or unit definition -- avoid, please share metadata datatypes wherever possible.
	METADATA_TYPE_COMPLEX_SIZE_3 = '3', //3 bytes of opaque data, no scale or unit definition -- avoid, please share metadata datatypes wherever possible.
	METADATA_TYPE_COMPLEX_SIZE_4 = '4', //4 bytes of opaque data, no scale or unit definition -- avoid, please share metadata datatypes wherever possible.
	METADATA_TYPE_COMPLEX_SIZE_8 = '8', //8 bytes of opaque data, no scale or unit definition -- avoid, please share metadata datatypes wherever possible.

	METADATA_TYPE_NEST = 0, // used to nest more GPMF formatted metadata

} MetadataType;


#define METADATA_STICKY_PAYLOAD_SIZE	256 // can be increased if need
#define METADATA_BUFFER_MINIMUM_SIZE	(sizeof(device_metadata)+2*METADATA_STICKY_PAYLOAD_SIZE)

#define METADATA_ERROR_OK				0
#define METADATA_ERROR_DEVICE			1
#define METADATA_ERROR_MEMORY			2
#define METADATA_ERROR_STICKY_MEMORY	3

#define METADATA_FLAGS_NONE				0
#define METADATA_FLAGS_STICKY			1
#define METADATA_FLAGS_BIG_ENDIAN		2  // Indiciate whether source data is formatted as big endian

//Kevin changed this
typedef enum MetadataTag_  // TAG in all caps are GoPro preserved (are defined by GoPro, but can be used by others.)
{
	// Internal Metadata structure and formatting tags
	METADATA_TAG_TIMING_OFFSET =	MAKEID('T','I','M','O'),//TIMO - Time offset of the metadata stream that follows (single 4 byte float)
	METADATA_TAG_DEVICE =			MAKEID('D','E','V','C'),//DEVC - nested device data to speed the parsing of multiple devices in post
	METADATA_TAG_STREAM =			MAKEID('S','T','R','M'),//STRM - nested channel/stream of telemetry data
	METADATA_TAG_DEVICE_ID =		MAKEID('D','V','I','D'),//DVID - unique id per stream for a metadata source (in camera or external input) (single 4 byte int)
	METADATA_TAG_DEVICE_NAME =		MAKEID('D','V','N','M'),//DVNM - human readable device type/name (char string)
	METADATA_TAG_SI_UNITS =			MAKEID('S','I','U','N'),//SIUN - Display string for metadata units where inputs are in SI units "uT","rad/s","km/s","m/s","mm/s" etc.
	METADATA_TAG_UNITS =			MAKEID('U','N','I','T'),//UNIT - Freedform display string for metadata units (char sting like "RPM", "MPH", "km/h", etc)
	METADATA_TAG_RANGE_MIN =		MAKEID('R','M','I','N'),//RMIN - Value range min and max can be used to predict Gauge rendering ranges.
	METADATA_TAG_RANGE_MAX =		MAKEID('R','M','A','X'),//RMAX -    "    "
	METADATA_TAG_SCALE =			MAKEID('S','C','A','L'),//SCAL - divisor for input data to scale to the correct units.
	METADATA_TAG_SCALE_NUMERATOR =	MAKEID('S','C','L','N'),//SCLN - same data sources may need additional scale precision.
	METADATA_TAG_TYPE =				MAKEID('T','Y','P','E'),//TYPE - Type define for complex data structures
	METADATA_TAG_SCALE_TYPE =		MAKEID('S','C','T','Y'),//SCTY - When using TYPE, is you what customized storage precision for parameter scaling
	METADATA_TAG_REMARK =			MAKEID('R','M','R','K'),//RMRK - addcing comments to the bitstream (debugging)

	//Burst time data is only used if an external device connection is too slow to transmit all the data that is triggered at a
	//particular time ('BTMB').  Example a Blutooth golf analysis puck that collects a lot of data only during a swing.
	//In this case the MP4 timing data would not present the data as it arrives slower than realtime.
	METADATA_TAG_BURST_TIME_BEGIN =	MAKEID('B','T','M','B'),//BTMB - Time in source floating point seconds for an extern device to a burst payload.
	METADATA_TAG_BURST_TIME_END =	MAKEID('B','T','M','E'),//BTME - Time in source floating point seconds for an extern device to a burst payload.

	// Common Telemetry sources
	METADATA_TAG_6AXIS_IMU =		MAKEID('I','M','U','6'),//ACCL accelerometer X,Y,Z, then Gyro X,Y,Z
	METADATA_TAG_9AXIS_IMU =		MAKEID('I','M','U','9'),//ACCL accelerometer X,Y,Z, then Gyro X,Y,Z, then Magn X,Y,Z
	METADATA_TAG_ACCEL_3AXIS =		MAKEID('A','C','C','L'),//ACCL accelerometer X,Y,Z values
	METADATA_TAG_GRYO_3AXIS =		MAKEID('G','Y','R','O'),//GYRO gyro X,Y,Z values
 	METADATA_TAG_MAGNITOMETER =		MAKEID('M','A','G','N'),//MAGN magnetometer X,Y,Z values
 	METADATA_TAG_HEART_RATE =		MAKEID('H','A','R','T'),//HART heart rate
	METADATA_TAG_GPS_3AXIS =		MAKEID('G','P','S','3'),//GPS3 GPS X,Y,Z coordinates
	METADATA_TAG_GPS_2AXIS =		MAKEID('G','P','S','2'),//GPS2 GPS X,Y coordinates

	// Common Camera quality
	METADATA_TAG_COLOR_MATRIX =		MAKEID('C','O','L','M'),//COLM f 48 1 floats (4x3 matrix)
	METADATA_TAG_SHUTTER_TIME =		MAKEID('S','H','U','T'),//SHUT f 4 1 float // shutter time in seconds
	METADATA_TAG_SENSOR_GAIN =		MAKEID('I','S','O','G'),//ISOG f 4 1 float // 1.0 to 64.0 (typical)

	// Camera Audio metadata
	//

	// add more
	//

	METADATA_TAG_FREESPACE =		MAKEID('F','R','E','E'),//FREE c 1 n bytes reserved for more metadata
	METADATA_TAG_END = 0//(null)
} MetadataTag;
