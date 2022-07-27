#include<math.h>
#include<iostream>
using namespace std;

// I/O Pins Data labels for Yaw Motor
    #define dirPinYaw 2  
    #define MS1PinYaw 3
    #define MS2PinYaw 4
    #define MS3PinYaw 7
    #define stepPinYaw 5

// T/O Pins Data Labels for Pitch Motor
    #define stepPinPitch 6
    #define dirPinPitch  8
    #define MS1PinPitch  9
    #define MS2PinPitch  10
    #define MS3PinPitch  11


// Class declaratons for drone antenna & Steppermotor
    class drone {
      public:
          float rDLat;
          float rDLong;
          float rDElev;  // stores real time co-ordinates of drones 
        
    };

    class antenna {
      public:
          drone drone1;
          float aLat;
          float aLong;
          float aElev;

          float aIYaw;
          float aIPitch;  // stores initial Pitch and Yaw Angle 

          bool  Flag;  // Two flags to keep track of rotation to be taken (1=> Yaw motion, 0=> Pitch Motion)

          float bAYaw;
          float bAPitch;  // stores required bearing angle( Yaw & Pitch ) each time drone is displaced

          void calBAYaw();
          void calBAPitch();
          void uaIYaw();
          void uaIPitch();    
    };

    class stepperMotor {
      private:
          int microSteps;
          int stepsTaken;
      public:
          antenna antenna1;
          void rotateA();
          void setup(float,float,float,float,float,float,float,float);
          void loop();     
          
    };

    void antenna :: calBAYaw(){
      //Calculation of final bearing RD Location and ALocation
            float x,y,aBA;
            x=cos((M_PI/180)*drone1.rDLat)*sin(drone1.rDLong - aLong);
            y=cos(aLat)*sin(drone1.rDLat)-sin(aLat)*cos(drone1.rDLat)*cos(drone1.rDLong-aLong);
            aBA= atan2(x,y)*(180/M_PI);
      //Calculation of required BAYaw
            bAYaw=aBA-aIYaw;
      //Printing status
            cout<<"Yaw Bearing angle: "<<bAYaw<<endl;

    } 

    void antenna::calBAPitch(){
      // determining distance between Antenna and Drone
           float x,y,p,q,aBA; // x=>Alat,y=>Along, p=>rDlat, q=>rDlong
            x=aLat*(M_PI/180);
            y=aLong*(M_PI/180);
            p=drone1.rDLat*(M_PI/180);
            q=drone1.rDLong*(M_PI/180);
            
           float d=3963*acos((sin(x)*sin(p))+cos(x)*cos(p)*cos(q-y));
            d=1609.344*d;
            aBA=atan((drone1.rDElev-aElev)/d)*(180/M_PI);
     //  Calculation of required BAYaw
            bAPitch=aBA-aIPitch;
     //  Printing Status
            cout<<"Pitch Bearing Angle: "<<bAPitch<<endl;
            
    }

    void antenna::uaIYaw(){
            aIYaw = aIYaw + bAYaw;
            cout<<"Updated aIyaw angle: "<<aIYaw<<endl;
    }

    void antenna::uaIPitch(){
            aIPitch = aIPitch + bAPitch;
            cout<<"Updated aIPitch angle: "<<aIPitch<<endl;
    }

    void stepperMotor::rotateA(){
      // To set direction Pin and calculate microsteps and no of steps required 
            int L;
            switch(antenna1.Flag){
              case 1: //for Yaw motion control 
                   if(antenna1.bAYaw>0 && (antenna1.bAYaw>antenna1.aIPitch && antenna1.bAYaw<=antenna1.aIPitch+180)){
                      digitalWrite(dirPinYaw,1);
                      cout<<"Yaw Direction Pin: 1"<<endl;
                      L=antenna1.bAYaw;
                   }
                   else if(antenna1.bAYaw>0 && antenna1.bAYaw>antenna1.aIPitch+180){
                      digitalWrite(dirPinYaw,0);
                      cout<<"Yaw direction Pin: 0"<<endl;
                      L=360+antenna1.aIYaw-antenna1.bAYaw;
                   }
                   else if(antenna1.bAYaw<0){
                      digitalWrite(dirPinYaw,0);
                      cout<<"Yaw direction Pin: 0"<<endl;
                      L=-antenna1.bAYaw;
                   }
                break;     
              case 0: //for Pitch motion
                    if(antenna1.bAPitch>0){
                        digitalWrite(dirPinPitch,0);
                        cout<<"Pitch direction Pin: 0"<<endl;
                        L=antenna1.bAPitch;
                    }
                    else{
                        digitalWrite(dirPinPitch,1);
                        cout<<"Pitch direction Pin: 1"<<endl;
                        L=-antenna1.bAPitch;
                    };
                 break;
            }
            float j=1; // for fraction
            float i    =antenna1.Flag==1?2:6;
            for(;i<=6;i++){
              float k=((pow(2.0,i)/360)*L);
              int m=floor(k);
              if(k-m<j){
                j=k-m;
                stepsTaken=m;
                microSteps=i-2;
             }
            }
            cout<<"Microsteps calculated: "<<microSteps<<endl;
       // To set microsteps resolution Pin;
             switch(microSteps){
            case 0:
              digitalWrite(antenna1.Flag==1?MS1PinYaw:MS1PinPitch,0);
              cout<<"MS1 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS2PinYaw:MS2PinPitch,0);
              cout<<"MS2 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS3PinYaw:MS3PinPitch,0);
              cout<<"MS3 Pin set"<<endl;
            break;
            case 1:
              digitalWrite(antenna1.Flag==1?MS1PinYaw:MS1PinPitch,1);
              cout<<"MS1 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS2PinYaw:MS2PinPitch,0);
              cout<<"MS2 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS3PinYaw:MS3PinPitch,0);
              cout<<"MS3 Pin set"<<endl;
            break;
            case 2:
              digitalWrite(antenna1.Flag==1?MS1PinYaw:MS1PinPitch,0);
              cout<<"MS1 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS2PinYaw:MS2PinPitch,1);
              cout<<"MS2 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS3PinYaw:MS3PinPitch,0);
              cout<<"MS3 Pin set"<<endl;
            break;
            case 3:
              digitalWrite(antenna1.Flag==1?MS1PinYaw:MS1PinPitch,1);
              cout<<"MS1 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS2PinYaw:MS2PinPitch,1);
              cout<<"MS2 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS3PinYaw:MS3PinPitch,0);
              cout<<"MS3 Pin set"<<endl;
            break;
            case 4:
              digitalWrite(antenna1.Flag==1?MS1PinYaw:MS1PinPitch,1);
              cout<<"MS1 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS2PinYaw:MS2PinPitch,1);
              cout<<"MS2 Pin set"<<endl;
              digitalWrite(antenna1.Flag==1?MS3PinYaw:MS3PinPitch,1);
              cout<<"MS3 Pin set"<<endl;
          }
    // To set step pin
       antenna1.Flag==1?j=stepPinYaw:j=stepPinPitch;
       cout<<"Steps Counting: ";
       for(i=1;i<=stepsTaken;i++){
        digitalWrite(j,0);
        delayMicroseconds(500);
        digitalWrite(j,1);
        delayMicroseconds(500);
        cout<<i<<endl;
       }
       digitalWrite(j,0);
    
    };
void stepperMotor::setup(float aLat,float aLong,float aElev,float aIyaw,float aIPitch,float rDLat,float rDLong,float rDElev) {
  // mode declarations of Pins
    /*pinMode(dirPinYaw,OUTPUT);
    pinMode(stepPinYaw,OUTPUT);
    pinMode(MS1PinYaw,OUTPUT);
    pinMode(MS2PinYaw,OUTPUT);
    pinMode(MS3PinYaw,OUTPUT);

    pinMode(dirPinPitch,OUTPUT);
    pinMode(stepPinPitch,OUTPUT);
    pinMode(MS1PinPitch,OUTPUT);
    pinMode(MS2PinPitch,OUTPUT);
    pinMode(MS3PinPitch,OUTPUT);*/
    antenna1.aLat = aLat;
    antenna1.aLong = aLong;
    antenna1.aElev = aElev;
    antenna1.aIYaw = aIyaw;
    antenna1.aIPitch = aIPitch;
        

    antenna1.drone1.rDLat = rDLat;
    antenna1.drone1.rDLong = rDLong;
    antenna1.drone1.rDElev = rDElev;
};
void stepperMotor::loop() {
      
  // calculate BAYaw
      antenna1.calBAYaw();
  // check if bAYaw == 0 , if not then set flag to high,rotate BAYaw angle & update BAYaw 
      if(antenna1.bAYaw != 0){
          antenna1.Flag = 1;
          rotateA();
         antenna1.uaIYaw();
      }
  // calculate BAPitch
      antenna1.calBAPitch();
  // check if BAPitch == 0, if not then set flag to high,rotate BAPitch angle and update BAPitch
      if(antenna1.bAPitch != 0){
          antenna1.Flag = 0;
          rotateA();
          antenna1.uaIPitch();
          
      }
}