#include <stdio.h>
#include <stdint.h>
#include "bno055.h"
#include <wiringPi.h>

BNO055 bno = BNO055();

//compile with g++ -o senstool test.c bno055.cpp -lwiringPi

int main(){
        if(!bno.begin()){
                printf("error");
                return 1;
        }
        while(1){
                double dat[3];
                bno.readEuler(dat);
                printf("x: %f y: %f z: %f\n",dat[0],dat[1],dat[2]);
                system("@cls||clear");
                delay(100);
        }
        return 0;
}
