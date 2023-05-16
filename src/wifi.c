//Add on 2022/10/22
//Connect optitrack(NatNet) by wifi 
//Using NodeMCU esp8266 v0.9 for testing
//Using mini naze32 channel 3(Tx) 4(Rx) for communication

#include "board.h"
#include "mw.h"
#define lenght_of_wifidata 6   //9999cm ~ -999cm range from optitrack data

#ifdef WIFI

int16_t wifidata_x,wifidata_y,wifidata_z;
char wifidatachar;
char wifidatastr[lenght_of_wifidata] = {'\0'};
int16_t tmp_px = 0, tmp_py = 0,  tmp_pz = 0;
uint8_t wificount = 0;
int16_t wifiloopcount = 1;
bool wifidataready = 0;

void wifiInit(void){  
    portMode_t mode = MODE_RXTX;
    core.wifiport = uartOpen(USART2, NULL, 115200, mode); //set port for wifi
    serialSetBaudRate(core.wifiport, 115200);   //set baudrate for wifi
    //featureSet(FEATURE_GPS) and mcfg.spektrum_sat_on_flexport = 1 are need to declear in config.c to get the right reciever channel signal
}

void getwifidata(void){
    if(wifidataready == 0){     //read wifidata from uart2 if update_wifidata have update wifidata,if it hasn't update yet,dont read...
        while(serialTotalBytesWaiting(core.wifiport)){            
            wifidatachar = serialRead(core.wifiport);        
            wifidatastr[wificount] = wifidatachar;

            if(wifidatachar == 'x' ){
                wifidata_x = atoi(wifidatastr);
                wificount = 0;
                wifidatastr[0] = 0;     //clear str buffer
                break;      
            }
            if(wifidatachar == 'y' ){
                wifidata_y = atoi(wifidatastr);
                wificount = 0;
                wifidatastr[0] = 0;     //clear str buffer
                break;      
            }
            if(wifidatachar == 'z' ){
                wifidata_z = atoi(wifidatastr);
                wificount = 0;
                wifidatastr[0] = 0;     //clear str buffer
                wifidataready = 1;      //wait for update_wifi function
                break;      
            }
            if(wificount > lenght_of_wifidata){
                wificount = 0;
                wifidatastr[0] = 0;     //clear str buffer
                break;
            }
            wificount++;
        }
    }
}

void update_wifidata(void){         //update wifidata in 40hz 
    if(wifidataready == 1){
        wifiloopcount++;
        tmp_px = wifidata_x;
        tmp_py = wifidata_y;
        tmp_pz = wifidata_z;
        wifidataready = 0;          //set wifidataready = 0 for getwifidata function to read new data
    }
}

void wifi_ledblink(void){           //blink red led for checking whether data pass through or not, DONT USE delay for blinking LED !!
    if(wifiloopcount % 20 == 0){
        LED1_ON;
    }
    else{
        LED1_OFF;
    }
}

#endif
