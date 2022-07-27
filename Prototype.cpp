 // Antenna Tracking actuation
#include"header.h"
#define aLat -64.12
#define aLong -74.45
#define aElev 1.23

#define  aIYaw 14.42
#define  aIPitch 4.42

#define  rDLat -115.42
#define  rDLong 100.12
#define  rDElev 50000.21
bool status 1;

         
int main(){
  stepperMotor stepperMotor1;
  stepperMotor1.setup(aLat,aLong,aElev,aIYaw,aIPitch,rDLat, rDLong,rDElev);
      while(status==1){
        stepperMotor1.loop();
      }
      

  return 0;
}
