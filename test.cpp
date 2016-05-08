#include <iostream>
#include "CRC16.h"
#include <stdio.h>
#include <string.h>
int main(){
    unsigned char text[]={0x02,0x03,0x00,0x10,0x00,0x0b,0xa0,0xa0};
    unsigned short parity;
    parity  =  CRC16(text,6);
    text[6]=parity;
    text[7]=parity>>8;
    printf("%#x and %#x\n",text[6],text[7]);

    unsigned char a=255+1024;
    printf("%x\n",a);
    float b=0.02;
    //unsigned char b1[]=[b>>24,b>>16,b>>8,b];
    unsigned char *b1 =  (unsigned char*)&b;
    printf("%#2x %#2x %#2x %#2x\n",b1[0],b1[1],b1[2],b1[3]);

    short c=257;
    unsigned char c1[2]={c>>8,c};
    printf("%x %x\n",c1[0],c1[1]);

    char d1[]={0x01,0x01};
    short d=d1[0]<<8|d1[1];
    std::cout<<"d="<<d<<std::endl;
}
