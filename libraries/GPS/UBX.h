/*
    UBX.h - Ublox GNSS receiver interface library
    Copyright (C) 2016  Hery A Mwenegoha. All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>
 */

/*
 *@ Run process stream at fast loop frequency 
 *@ Update lat lon etc at recommended 1Hz.
 */

#ifndef UBX_h
#define UBX_h
#include "AHRS.h"

#define UBX_BUFFER_SIZE 255  

// defining sentences
#define NAV_POSLLH    0		
#define NAV_STATUS    1
#define NAV_VELNED 	  2
#define NAV_SOL	 	  3
#define NAV_DOP		  4
#define CFG_PRT_UART  5
#define CFG_RATE_MEAS 6
#define CFG_MSG_RATE  7
#define OTHER_UBX    255

// message rates
class GPSUBX
{
	public:
	GPSUBX();
	bool initialise(HardwareSerial *Port);	
	bool initialise(HardwareSerial *Port, uint32_t baud);
	bool process_stream(void);
	struct _UBX{
		uint32_t fix_time_ms;
		bool commit;
		bool gpsFixOk;
		uint8_t fix_type;
		float lat;
		float lon;
		float speed;    // metres/second
		float gSpeed;   // metres/second ground speed
		float velN;     // metres/second North Velocity
		float velE;     // metres/second East Velocity
		float velD;     // metres/second Down Velocity
		float height;   // metres above ellipsoid
		float hMSL;     // metres above sea level
		float hBASE;    // metres baseline height 
		float hFELV;    // metres above field elevation
		float heading;  // deg
		uint8_t numSV;
		
		uint8_t hour;      // hour in UTC : EAST = UTC + 3
		uint8_t minutes;   // minutes
		uint8_t seconds;   // seconds	
	}UBX;
	
	/*
	struct _UTC{
		uint8_t hour;      // hour in UTC : EAST = UTC + 3
		uint8_t minutes;   // minutes
		uint8_t seconds;   // seconds		
	}UTC;
	*/
	
	struct _DOP{
		uint16_t hDOP;    // scale 0.01 horizontal DOP
		uint16_t vDOP;    // scale 0.01 vertical DOP
		uint16_t nDOP;    // scale 0.01 northing DOP
		uint16_t eDOP;    // scale 0.01 easting DOP
		uint16_t tDOP;    // scale 0.01 time DOP
	}DOP;
	
	uint32_t get_BaudRate();
	
	private:
	HardwareSerial *Port;
	uint32_t BaudRate;
	uint32_t update_Msec;
	
	void configure_measurementRate();
	void request_msgConfig(uint8_t msg_class, uint8_t msg_id);
	void request_measurementRate();
	
	void set_message_rate();
	void calculate_checksum(byte *buffer, const byte len);
	void UBX_decode(uint8_t read_byte, byte *buffer);
	void process_buffer(byte *buffer, uint8_t len);
	void populate_buffer(byte *buffer1, byte *buffer2, uint8_t len);
	uint8_t GET_LENGTH(uint8_t class_id, uint8_t msg_id);
	
	bool _byPass_Header;	
	int _count;
	uint8_t PAX_LENGTH;
	uint8_t FULL_LENGTH;
	uint8_t STOP_LENGTH;
	uint8_t SENTENCE_TYPE;

	// 0x06 0x01 UART Protocol
	struct config_prt_uart{
		uint8_t  portID;
		uint8_t  res0;
		uint16_t res1;
		uint32_t mode;
		uint32_t baudrate;
		uint16_t inProtoMask;
		uint16_t outProtoMask;
		uint16_t flags;
		uint16_t pad;
	};
	
	union _cfg_prt{
		struct config_prt_uart uart;
		byte buffer[sizeof(config_prt_uart)];
	};	
	
	// configure measurment rate
	struct config_rate_meas{
		uint16_t measRate; // ms measuremnet rate
		uint16_t navRate;  // cycles ublox 5 and 7 => 1e-2
		uint16_t timeRef;  // alignment to reference time 0 = UTC Time, 1 = GPS time.
	};
	
	union _cfg_meas{
		struct config_rate_meas packet;
		byte buffer[sizeof(config_rate_meas)];
	};
	
	// 0x01 0x02
	struct nav_posllh_t{
		uint32_t iTow;   // ms GPS millisecond Time of week.
		int32_t  lon;    // deg 1e-7
		int32_t  lat;    // deg 1e-7
		int32_t  height; // mm above ellipsoid
		int32_t  hmsl;   // mm above sea level
		uint32_t hAcc;   // mm horizontal accuracy estimate
		uint32_t vAcc;   // mm vertical accuracy estimate 
	};
	
	union _nav_pos{
		struct nav_posllh_t packet;
		byte buffer[sizeof(nav_posllh_t)]; // plus 2 checksum bytes
	};
	
	// 0x01 0x12
	struct nav_velned_t{
		uint32_t iTow;      // ms GPS millisecond Time of Week
		int32_t  velN;      // cm/s NED north velocity 
		int32_t  velE;      // cm/s NED east  velocity
		int32_t  velD;      // cm/s NED down  velocity
		uint32_t speed;     // cm/s Speed (3-D)
		uint32_t gSpeed;    // cm/s Ground Speed (2-D)
		int32_t  heading;   // deg scaling 1e-5 (2-D)
		uint32_t sAcc;      // cm/s Speed Accuracy Estimate
		uint32_t cAcc;      // Course/ Heading Accuracy Estimate scaling 1e-5
	};
	
	union _nav_velned{
		struct nav_velned_t packet;
		byte buffer[sizeof(nav_velned_t)]; 
	};
	
	// 0x01 0x03
	struct nav_status_t{
		uint32_t iTow;   // ms GPS millisecond Time of Week
		uint8_t  gpsFix; // 0x00 - no fix, 0x01 - DR, 0x02 - 2D, 0x03 - 3D, 0x04 GPS + DR, 0x05 - Time Only Fix
		uint8_t  flags;  // last bit resprents fix ok.
		uint8_t  diffStat;
		uint8_t  res;
		uint32_t ttff;
		uint32_t msss;
	};
	
	union _nav_status{
		struct nav_status_t packet;
		byte buffer[sizeof(nav_status_t)];
	};
	
	struct nav_sol_t{
		uint32_t iTow;   // ms GPS millisecond Time of week
		int32_t  fTow;   // ns nanoseconds remainder of rounded ms above, range - 500000 - 500000
		int16_t  week;   // GPS week (GPS time)
		uint8_t  gpsFix; // 0x00 - no fix, 0x01 - DR, 0x02 - 2d, 0x03 - 3d
		uint8_t	 flags;  // fix status flags
		int32_t  ecefX;  // cm ECEF X coordinate
		int32_t  ecefY;  // cm ECEF Y coordinate
		int32_t  ecefZ;  // cm ECEF Z coordinate
		uint32_t pAcc;   // cm 3D position accuracy
		int32_t  ecefVX; // cm/s ECEF X velocity
		int32_t  ecefVY; // cm/s ECEF Y velocity
		int32_t  ecefVZ; // cm/s ECEF Z velocity
		uint32_t sAcc;   // cm/s Speed Accuracy Estimate
		uint16_t pDOP;   // position DOP scaled to 0.01
		uint8_t  res1;   // reserved
		uint8_t  numSV;  // satellites used
		uint32_t res2;   // reserved.		
	};
	
	union _nav_sol {
		struct nav_sol_t packet;
		byte buffer[sizeof(nav_sol_t)];
	};
	
	typedef uint32_t u4;
	typedef uint16_t u2;
	typedef uint8_t  u1;
	typedef int32_t  i4;
	typedef int16_t  i2;
	typedef int8_t   i1;
	
	// NAV-DOP Dilution of Precision 0x01 0x04
	struct nav_dop_t{
		u4 iTow; // ms GPS millisecond Time of Week
		u2 gDOP; // Geometric DOP scale 0.01
		u2 pDOP; // Position DOP  scale 0.01
		u2 tDOP; // Time DOP scale 0.01
		u2 vDOP; // Vertical DOP scale 0.01
		u2 hDOP; // Horizontal DOP scale 0.01
		u2 nDOP; // Northing DOP scale 0.01
		u2 eDOP; // Easting DOP scale 0.01
	};
	
	union _nav_dop{
		struct nav_dop_t packet;
		byte buffer[sizeof(nav_dop_t)];
	};
	
	union _I16_un{
		int16_t value;
		byte buffer[sizeof(int16_t)];
	};
	
	void commit_velned(struct nav_velned_t &velned_t)
	{
		UBX.speed   = static_cast<float>(velned_t.speed   * 1e-2);
		UBX.gSpeed  = static_cast<float>(velned_t.gSpeed  * 1e-2);
		UBX.velN    = static_cast<float>(velned_t.velN    * 1e-2);
		UBX.velE    = static_cast<float>(velned_t.velE    * 1e-2);
		UBX.velD    = static_cast<float>(velned_t.velD    * 1e-2);
		UBX.heading = static_cast<float>(velned_t.heading * 1e-5);		
	}
	
	void commit_posllh(struct nav_posllh_t &posllh)
	{
	    UBX.lat    = static_cast<float>(posllh.lat    * 1e-7);
		UBX.lon    = static_cast<float>(posllh.lon    * 1e-7);
		UBX.height = static_cast<float>(posllh.height * 1e-3);
		UBX.hMSL   = static_cast<float>(posllh.hmsl   * 1e-3);
		UBX.hFELV  = UBX.hMSL - UBX.hBASE;		
	}
};
#endif
