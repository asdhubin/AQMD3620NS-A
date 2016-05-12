#ifndef SERIALPORT_H_INCLUDED
#define SERIALPORT_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>


static struct termios oldtty;
int set_interface_attribs (int fd, int speed, int parity,int stop);
int set_blocking (int fd, int should_block);
/**int set_interface_attribs (int fd, int speed, int parity,int stop=1)
*�봮�ж˿ڽ�������
*ע�⣺Ĭ��ÿ�ֽ�1bit��ʼλ+8bit����λ+1bitֹͣλ����У��λ
*����Ŀ��ʹ�õ�ֱ�����������485ͨѶװ���У���2bitֹͣλ��û��У��λ
*/
int set_interface_attribs (int fd, int speed, int parity,int stop)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);

        if (tcgetattr (fd, &oldtty) != 0)
        {
                // error_message ("error %d from tcgetattr", errno);
                printf("error opening the device");
                return -1;
        }

        memcpy( &tty,  &oldtty, sizeof (struct termios) );

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        if(stop==1){tty.c_cflag &= ~CSTOPB;}
        else if(stop==2){tty.c_cflag |= CSTOPB;}
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
            // error_message ("error %d from tcsetattr", errno);
            printf("error opening the device");
            return -1;
        }

        return 0;
} // end function: set_interface_attribs

/**int set_blocking (int fd, int should_block)
*�趨����ͨ������ģʽʱ����������ģʽ�����ڻ��������ʱ�Ĵ���
һ�㶼�趨Ϊ set_blocking(fd,0) ��//Ĭ�ϲ�����
*/
int set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
            //error_message ("error %d from tggetattr", errno);
            printf("error opening the device");
            return -1;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
          //  error_message ("error %d setting term attributes", errno);
            printf("error opening the device");
    return 0;
} // end function: set_blocking


#endif // SERIALPORT_H_INCLUDED
