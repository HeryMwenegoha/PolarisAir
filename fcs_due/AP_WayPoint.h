#pragma once
#include "Arduino.h"
#include <EEPROM.h>
#define MAX_WAYPOINTS             10
#define EEPROM_WAYPOINT_FIRSTPAGE 10
#define EEPROM_WAYPOINT_LASTPAGE  1700
#define MAX_STORED_WAYPOINTS      (EEPROM_WAYPOINT_LASTPAGE - EEPROM_WAYPOINT_FIRSTPAGE)/sizeof(Location)

#define EEPROM_WAYPOINT_VERSION        0x01
#define EEPROM_WAYPOINT_VERSION_PAGE   0x00
#define EEPROM_WAYPOINT_TOTALPAGE      0x02

class AP_WayPoint{
  public:
  AP_WayPoint(){
    count.total = 0;
    count.seq   = 0;
  };
  void initialise(); 
  
  struct _count{
    AP_u8  total;
    AP_u8  seq;        
    void   load(){  total = EEPROM.read(EEPROM_WAYPOINT_TOTALPAGE); }
    void   save(){  EEPROM.write(EEPROM_WAYPOINT_TOTALPAGE, total); }
  };
  _count count; 
  
  Location WayPoint[MAX_WAYPOINTS];
  Location homeWP;
  
  void write(float lat, float lon, AP_u16 alt, AP_u8 rad, AP_u8 seq, AP_u8 cmd);
  Location read(uint8_t _seq);
  void update(float lat, float lon, AP_u16 alt, AP_u8 rad, AP_u8 seq, AP_u8 cmd);
};

void AP_WayPoint::initialise()
{
  if(EEPROM.read(EEPROM_WAYPOINT_VERSION_PAGE) != EEPROM_WAYPOINT_VERSION){

    EEPROM.write(4000, 0); // Erase storage of old total waypoint

    // erase everthing else
    for(int se = 0; se < sizeof(Location); se++){
      for(int i = 0; i < 10; i++){
        if(EEPROM.read(i + (se * sizeof(Location))) != 0)
          EEPROM.write(i + (se * sizeof(Location)), 0); // old implementation
      }
    }

    EEPROM.write(EEPROM_WAYPOINT_VERSION_PAGE, EEPROM_WAYPOINT_VERSION); // NEW implementatio
  }
  
  count.load();

  if(count.total > MAX_WAYPOINTS){
    PRINTTAB(F("AP_WayPoint::TotalExceeds"));
    PRINTLN(count.total);
    for(int i = 0; i < MAX_WAYPOINTS; i++){  
      WayPoint[i] = read(i);
      WayPoint[i].print();
    }
  }else{
    PRINTTAB(F("AP_WayPoint::Total"));
    PRINTLN(count.total);
    for(int i = 0; i < count.total; i++){
      WayPoint[i] = read(i);
      WayPoint[i].print();
    }
  }

  homeWP = WayPoint[0];
  PRINTLN(F("AP_WayPoint:: Initialised"));
}

void AP_WayPoint::update(float lat, float lon, AP_u16 alt, AP_u8 rad, AP_u8 seq, AP_u8 cmd)
{
  if(seq < MAX_WAYPOINTS){
  WayPoint[seq].lat = lat;
  WayPoint[seq].lon = lon;
  WayPoint[seq].alt = alt;
  WayPoint[seq].rad = rad;
  WayPoint[seq].seq = seq;
  WayPoint[seq].cmd = cmd;
  }
  homeWP = WayPoint[0];
}

void AP_WayPoint::write(float lat, float lon, AP_u16 alt, AP_u8 rad, AP_u8 seq, AP_u8 cmd){
  union _wp_un{
    Location wp_struct;
    byte buffer[sizeof(Location)];
  };

  _wp_un wp_un;
  wp_un.wp_struct.lat  = lat;
  wp_un.wp_struct.lon  = lon;
  wp_un.wp_struct.alt  = alt;
  wp_un.wp_struct.rad  = rad;
  wp_un.wp_struct.seq  = seq;
  wp_un.wp_struct.cmd  = cmd;
  
  for(int i = 0; i < sizeof(Location); i++){
    if(EEPROM.read(EEPROM_WAYPOINT_FIRSTPAGE + (i + (seq * sizeof(Location))))  != wp_un.buffer[i])
       EEPROM.write(EEPROM_WAYPOINT_FIRSTPAGE + (i + (seq * sizeof(Location))) , wp_un.buffer[i]);  
  }
}

Location AP_WayPoint::read(uint8_t seq)
{
  union _wp_un{
    Location wp_struct;
    byte buffer[sizeof(Location)];
  };
  
  _wp_un wp_un; 
  
  for(int i = 0; i < sizeof(Location); i++){
    wp_un.buffer[i] = EEPROM.read(EEPROM_WAYPOINT_FIRSTPAGE + (i + (seq * sizeof(Location))));  
  } 

  Location _wp;
  _wp.zero();
  _wp = homeWP;
  _wp.seq = seq;

  // 2 attempts to read::
  if(wp_un.wp_struct.isvalid())
  {   
    _wp = wp_un.wp_struct;
  }
  else
  {
    for(int i = 0; i < sizeof(Location); i++){
     wp_un.buffer[i] = EEPROM.read(EEPROM_WAYPOINT_FIRSTPAGE + (i + (seq * sizeof(Location))));  
    }
      if(wp_un.wp_struct.isvalid())
      {   
        _wp = wp_un.wp_struct;
      } 
  }
  return _wp;
}

