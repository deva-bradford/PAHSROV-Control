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
                printf("x: %f \ny: %f \nz: %f\n",dat[0],dat[1],dat[2]);

                printf("\033[2J");
		printf("\033[%d;%dH", 0, 0);
                
                delay(100);
        }
        return 0;
}
