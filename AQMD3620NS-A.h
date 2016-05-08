#ifndef AQMD3620NS-A_H_INCLUDED
#define AQMD3620NS-A_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <ctime>
#include "serialport.h"

#define ADR 0x02
/**define wait_answer_duration 1000
*Ĭ�Ϸ�������󵽶�ȡ������Ϣ�м䣬�ȴ�1ms
*/
#define wait_answer_duration 1000
int fd;//�豸��

/**\brief ϵͳ�����趨*/
/**int set_stopbit(int stop=2)
*�趨ֹͣλΪ2bit��û�п���У��λ��һ��ҪУ��λ�Ļ����޸�Դ��"serialport.h"
*/
int set_stopbit(int stop=2){
    unsigned char order[]={ADR,0x06,0x00,0x8c,0x00,0x00,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ //��վ���մ���ʱ�����ص���ϢΪ����ֽ�
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set stop bits success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set srop bits error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}
/**int set_breaktime(unsigned char bkt=32)
*�趨ͨѶ�ж�ͣ��ʱ��
*0~255
*��ֵ����0.1ΪͨѶ�ж�ͣ��ʱ�䣬��λΪ�룻
���ϴ�ͨѶ�󾭹����趨ʱ��������һ��ͨѶ����ô���Ե�������ƶ�(ɲ��)��
*Ĭ��3.2��û���յ�485ͨѶ��Ϣ�ͽ��н���ͣ��
*/
int set_breaktime(unsigned char bkt=32){//Ĭ��3.2��û���յ�485ͨѶ��Ϣ�ͽ��н���ͣ��
    unsigned char order[]={ADR,0x06,0x00,0x8e,0x00,bkt,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set break time limit success"<<std::endl;}
    else if(result[1]=0x86){printf("set break time limit error! Code %x \n",result[2]);
            return -1;
    }
    return 0;
}
/**int set_current_fluctuation_threshold(unsigned char cft=0)
*�趨�Բ����Զ������������ֵ
*0~255��Ĭ��Ϊ0
*��ֵ����0��ԽС����Ƶ�ʼ��Խ����������Ч��Խ�ã�
�����������������������ܵ��¶�ת���ʧЧ��
����ֵΪ0ʱ��ʹ���Զ������������ֵ�����û���Ĭ��ֵ��
*/
int set_current_fluctuation_threshold(unsigned char cft=0){
    unsigned char order[]={ADR,0x06,0x00,0x72,0x00,cft,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set current fluctuation threshold success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set current fluctuation threshold error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}
/**int set_speed_governing(unsigned char sg=2)
*�趨���ٷ�ʽ
*0��PWM��ʽ
*1��ת�ط�ʽ
*2���Բ��ٱջ�����
*3����Ӳ��ٷ�����ջ�����
*Ĭ��ȡ2���Բ��ٱջ�����
*��ʹ��0x06�����룬��ʹ��0x10�������ҼĴ�������С��15ʱ�����ò���ֻ���ݴ棬��δ��Ч��
��0x10������ļĴ�������Ϊ15��ʹ��0x06��������0x0180�Ĵ���д1�����ò����Ż���Ч��
*/
int set_speed_governing(unsigned char sg=2){
    unsigned char order[]={ADR,0x06,0x00,0x80,0x00,sg,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set speed governing method success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set speed governing method error! Code %x\n",result[2]);
            return -1;
    }
    unsigned char order2[]={ADR,0x06,0x01,0x80,0x00,0x01,0x00,0x00};
    parity  =CRC16(order2,6);//����CRC16������У��
    order2[6]=parity;
    order2[7]=parity>>8;
    write(fd,order2,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"save speed governing method success"<<std::endl;}
    else if(result[1]=0x86){
            printf("save speed governing method error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}
/**int set_block_stop(unsigned char bs=16)
*�趨��תֹͣʱ��
*0~255
*Ĭ�϶�ת1.6s�����ͣ������ֹ����ջ�
*��ֵ����0.1Ϊ��תֹͣʱ�䣬��λΪ�룻
��ֵΪ0ʱ�������ж�תֹͣ������ֵ��0ʱ�����ڶ�ת��Ӧ����ʱ��������λ�ƶ�(ɲ��)��
*/
int set_block_stop(unsigned char bs=16){
    unsigned char order[]={ADR,0x06,0x00,0x81,0x00,bs,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set block stop time success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set block stop time error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}
/**int set_accelarate_time(unsigned char sat=16)
*�趨���ٻ���ʱ��
*0~255
*Ĭ�ϼ��ٻ���ʱ��Ϊ1.6��
*��ֵ����0.1ΪPWM��0���ӵ����ֵ�Ļ���ʱ��
*/
int set_accelarate_time(unsigned char sat=16){
    unsigned char order[]={ADR,0x06,0x00,0x85,0x00,sat,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set accelerate time success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set accelerate time error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}
/**set_current_consumption(unsigned short current=300)
*��������
*Ĭ��3A,Ϊ�˲��԰�ȫ
*0~2000
*��ֵ����0.01Ϊ����ֵ����λΪA��
*
*/
int set_current_consumption(unsigned short current=300){
    unsigned char order[]={ADR,0x06,0x00,0x86,current>>8,current,0x00,0x00};//ʵ��λ�������
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set current consumption success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set current consumption error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}
/**set_current_max(unsigned short current=300)
*�趨����������/���ص���
*Ĭ��3A
*0~2000
*��ֵ����0.01Ϊ����ֵ����λΪA��
*/
int set_current_max(unsigned short current=300){
    unsigned char order[]={ADR,0x06,0x00,0x87,current>>8,current,0x00,0x00};//ʵ��λ�������
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set max current success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set max current error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}
/**int set_brake_current(unsigned short current=200)
*����ƶ�(ɲ��)����
*Ĭ��2A
*��ֵ����0.01Ϊ����ֵ����λΪA��
*/
int set_brake_current(unsigned short current=200){
    unsigned char order[]={ADR,0x06,0x00,0x88,current>>8,current,0x00,0x00};//ʵ��λ�������
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set brake current success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set brake current error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}
/**int set_brake_time(unsigned char time=10)
*�ƶ�����ʱ��
*Ĭ��1s
*��ֵ����0.1Ϊɲ��PWM��0���ӵ����ֵ�Ļ���ʱ�䡣
*/
int set_brake_time(unsigned char time=10){
    unsigned char order[]={ADR,0x06,0x00,0x89,0x00,time,0x00,0x00};//ʵ��λ�������
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set brake time success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set brake time error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}

/**int set_decelerate_time(unsigned char time=10)
*0~255
*���ٻ���ʱ��,Ĭ��1s
*��ֵ����0.1ΪPWM�����ֵ��С��0�Ļ���ʱ�䡣
*/
int set_decelerate_time(unsigned char time=10){
    unsigned char order[]={ADR,0x06,0x00,0x8F,0x00,time,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set decelerate time success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set decelerate time error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}

/**int set_serial_configuration(unsigned char sc=0)
*����ͨѶ���Ʒ�ʽʱ�Ƿ��ֹ����
*0������ֹ��Ĭ�ϣ�
*1����ֹ
*/
int set_serial_configuration(unsigned char sc=0){
    unsigned char order[]={ADR,0x06,0x00,0x8D,0x00,sc,0x00,0x00};//ʵ��λ�������
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set serial configuration enable success"<<std::endl;}
    else if(result[1]=0x86){
            printf("set serial configuration enable error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}


/**int set_baud_rate(int32_t baud=9600)
*�����ʣ�1200~115200
*Ĭ��9600
*��λΪbps
*/
int set_baud_rate(int32_t baud=9600){
    unsigned char order[]={ADR,0x10,0x00,0x8A,0x00,0x02,0x04,baud>>24,baud>>16,baud>>8,baud,0x00,0x00};//�ܹ�13�ֽ�
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,11);//����CRC16������У��ǰ11���ֽ�
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x10){std::cout<<"set baud rate "<<baud<< " success"<<std::endl;}
    else if(result[1]=0x90){
            printf("set baud rate error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}

/**PIDϵ������*/
/**int set_pid_p(float p=0.02)
*Pϵ��
*����0.001~1,Ĭ��0.02
*/
int set_pid_p(float p=0.02){
    unsigned char *b1 =  (unsigned char*)&p;
    unsigned char order[]={ADR,0x10,0x00,0x90,0x00,0x02,0x04,b1[0],b1[1],b1[2],b1[3],0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,11);//����CRC16������У��ǰ11���ֽ�
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x10){std::cout<<"set PID parameter p success"<<std::endl;}
    else if(result[1]=0x90){
            printf("set PID parameter p error! Code %x\n",result[2]);
            return -1;
    }

    //����Pϵ��������
    unsigned char order2[]={ADR,0x06,0x01,0x90,0x00,0x01,0x00,0x00};
    parity  =CRC16(order2,6);//����CRC16������У��
    order2[6]=parity;
    order2[7]=parity>>8;
    write(fd,order2,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"save PID parameter success"<<std::endl;}
    else if(result[1]=0x86){
            printf("save PID parameter error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}

/**int set_pid_i(float i=0.02)
*iϵ��
*����0.001~1,Ĭ��0.02
*/
int set_pid_i(float i=0.02){
    unsigned char *b1 =  (unsigned char*)&i;
    unsigned char order[]={ADR,0x10,0x00,0x92,0x00,0x02,0x04,b1[0],b1[1],b1[2],b1[3],0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,11);//����CRC16������У��ǰ11���ֽ�
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x10){std::cout<<"set PID parameter i success"<<std::endl;}
    else if(result[1]=0x90){
            printf("set PID parameter i error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}
/**int set_pid_d(float d=0.02)
*dϵ��
*����0.001~1,Ĭ��0.02
*/
int set_pid_d(float d=0.02){
    unsigned char *b1 =  (unsigned char*)&d;
    unsigned char order[]={ADR,0x10,0x00,0x94,0x00,0x02,0x04,b1[0],b1[1],b1[2],b1[3],0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,11);//����CRC16������У��ǰ11���ֽ�
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x10){std::cout<<"set PID parameter d success"<<std::endl;}
    else if(result[1]=0x90){
            printf("set PID parameter d error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}

/**int set_pid_td(short td=1)
*D���ڽ�������
*1~1000,Ĭ��1
*D���ڽ��ģ���λΪms��PI���ڵĽ��Ĺ̶�Լ1ms��
*/
int set_pid_td(unsigned short td=1){
    unsigned char order[]={ADR,0x06,0x00,0x97,td>>8,td,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set Td success"<<endl;}
    else if(result[1]=0x86){
            printf("set Td error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}

/**int set_speed(short sp=0)
*�����ٶȣ���ֵ����0��ת��С��0��ת������0ɲ��
*Ĭ���Բ��ٱջ����ٷ�ʽ��û����ӱ�Ĺ��ܣ�-1000��1000
*���û���Ƶ�ʣ���λΪ��/�롣
*/
int set_speed(short sp=0){
    if(sp>1000 or sp<-1000){
        std::cout<<"speed is limited in -1000~1000"<<std::endl;
        if(sp>1000){sp=2000;std::cout<<"speed is set to 1000"<<std::endl;}
        else{sp=-1000;std::cout<<"speed is set to -1000"<<std::endl;}
    }
    unsigned char order[]={ADR,0x06,0x00,0x40,sp>>8,sp,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//����CRC16������У��
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//ֹͣ1ms���ȴ���������
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x06){std::cout<<"set speed success"<<endl;}
    else if(result[1]=0x86){
            printf("set speed error! Code %x\n",result[2]);
            return -1;
    }
    return 0;
}

/**int state_now()
*��ȡ��ǰ�������״̬
*��ʵ�������õ�����ֻ��
*ʵʱPWM
*ʵʱ����
*����Ƶ�ʣ���ӵõ����ת�٣�������Ҫ֪����������ԣ�תһȦ���򼸴Σ�
*/
int state_now(){
    unsigned char order[]={ADR,0x03,0x00,0x10,0x00,0x0B,0x05,0xFB};
    unsigned char result[27];
    if(order[0]!=0x02){
        unsigned short parity;
        parity  =CRC16(order,6);//����CRC16������У��
        order[6]=parity;
        order[7]=parity>>8;
    }
    write(fd,order,8);
    usleep(3*wait_answer_duration);//�������Ƚϴ󣬶��һ��
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            std::cout<<"read failed"<<std::endl;
            return -1;
            break;
    }
    if(result[1]=0x03){
            short state[11];
            for(int i=0,i<11,i++){
                state[i]=result[2*i+4]<<8|result[2*i+5];
            }
            std::cout<<"ʵʱPWM  "<<state[0]<<std::endl;
            std::cout<<"ʵʱ���� "<<state[1]<<std::endl;
            std::cout<<"����Ƶ�� "<<state[2]<<std::endl;
            std::cout<<"AI1��ѹ  "<<state[4]<<std::endl;
            std::cout<<"AI2��ѹ  "<<state[5]<<std::endl;
            std::cout<<"AI1��AI2���ֵ�ѹ "<<state[6]<<std::endl;
            std::cout<<"SQ1��ƽ "<<state[8]<<std::endl;
            std::cout<<"SQ2��ƽ "<<state[9]<<std::endl;
            std::cout<<"DE��ƽ "<<state[10]<<std::endl;
    }
    else if(result[1]=0x90){
            printf("get state error! Code %x\n",result[2]);
            return -1;
    }
    return 0;

}
#endif // AQMD3620NS-A_H_INCLUDED
