#include <iostream>
#include "CRC16.h"
#include <stdio.h>
int main(){
    unsigned char text[]={0x02,0x03,0x00,0x10,0x00,0x0b,0xa0,0xa0};
    unsigned short parity = CRC16(text,6);
    text[6]=parity;
    text[7]=parity>>8;
    printf("%x and %x",text[6],text[7]);
}
