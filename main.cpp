#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <ctime>

#include "AQMD3620NS-A"
#include "serialport.h"

#define portname  "/dev/ttyUSB0"
int main()
{
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("error opening the device\n");
    }

    if(set_interface_attribs(fd, B9600, 0,2)!=0)//此处有修改，stopbit is 2bit
    {            printf("hello");
        printf("error set interface\n");
    }
    else
    if(set_blocking(fd, 0)!=0)
    {
        printf("error set blocking\n");
    }
    else
        printf("done");

    if( set_interface_attribs (fd, B9600, 0,2) )
    { // then set_interface_attribs failed
        return -1;
    }
    // implied else set_interface_attribs successful
    if( set_blocking (fd, 0) )                // set no blocking
    { // then set_blocking failed
        return -1;   // might need to also restore oldtty attributes
    }
    // implied else, set_blocking successful



    char receivebuffer [20];


}
