  void setup(){
  if(!SD.begin(4)){
    Serial.println("SD Card Failed"); 
  }
  else{
    Serial.println("SD Initialised");  

    if(!SD.exists("Copter")){
      SD.mkdir("Copter");
    }
       
    for(int i = 1; i < 65535; i++){
      String ID     = String(i);     
      file          = "Copter/log" + ID + ".txt";     
      if(!SD.exists(file)){
         break;
      }
    }    
    File datalog = SD.open(file, FILE_WRITE);
    if(datalog){
      datalog.print(F("Millis(ms)"));
      datalog.print(F("\t"));
      datalog.print(F("UTC"));
      datalog.print(F("\t"));
      datalog.print(F("Latitude(deg)"));
      datalog.print(F("\t")); 
      datalog.print(F("Longitude(deg)"));
      datalog.print(F("\t"));
      datalog.print(F("raw_hamsl(m)"));
      datalog.print(F("\t"));
      datalog.print(F("raw_hafl(m)"));
      datalog.print(F("\t"));
      datalog.print(F("GroundSpeed(m/s)"));
      datalog.print(F("\t"));
      datalog.print(F("Airspeed(m/s)"));
      datalog.print(F("\t"));
      datalog.print(F("Mav_Link(%)"));
      datalog.print(F("\t"));
      datalog.print(F("Rssi_Link(%)"));
      datalog.print(F("\t"));
      datalog.print(F("Throttle"));
      datalog.print(F("\t"));
      datalog.print(F("Roll(deg)"));
      datalog.print(F("\t"));
      datalog.print(F("Pitch(deg)"));
      datalog.print(F("\t"));
      datalog.print(F("Yaw(deg)"));
      datalog.print(F("\t"));
      datalog.print(F("filt_height(m)"));
      datalog.print(F("\t"));
      datalog.println(F("filt_climbrate(m/s)"));

      datalog.close();
    }
    else{
      Serial.println("Failed to create File");
    }
  }
}

// loop function
void log_data(){
    File datalog = SD.open(file, FILE_WRITE);
    if(datalog){
    String time    = String(AP_gps.time().hour) + ":" + String(AP_gps.time().minutes) + ":" + String(AP_gps.time().seconds);
    Location loc;
    AP_ahrs.get_position(loc);
    datalog.print(millis());
    datalog.print(F("\t"));    
    datalog.print(time);
    datalog.print(F("\t"));
    datalog.print(loc.lat);
    datalog.print(F("\t")); 
    datalog.print(loc.lon);
    datalog.print(F("\t"));
    datalog.print(AP_gps.altitude());
    datalog.print(F("\t"));
    datalog.print(AP_ahrs.altitude_estimate());           // raw height from baro
    datalog.print(F("\t"));
    datalog.print(AP_ahrs.groundspeed_vector().length()); 
    datalog.print(F("\t"));
    datalog.print(AP_ahrs.airspeed_estimate());
    datalog.print(F("\t"));
    datalog.print(F("Mav_Link(%)"));
    datalog.print(F("\t"));
    datalog.print("Rssi_Link(%)");
    datalog.print(F("\t"));
    datalog.print(_servo_out.chan1);
    datalog.print(F("\t"));
    datalog.print(ToDeg(AP_ahrs.roll));
    datalog.print(F("\t"));
    datalog.print(ToDeg(AP_ahrs.pitch));
    datalog.print(F("\t"));
    datalog.print(ToDeg(AP_ahrs.yaw));
    datalog.print(F("\t"));
    datalog.print(AP_program._height);
    datalog.print(F("\t"));
    datalog.println(AP_program._climbrate);
    datalog.close();
  }   
}