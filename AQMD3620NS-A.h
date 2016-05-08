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
*默认发送命令后到读取返回信息中间，等待1ms
*/
#define wait_answer_duration 1000
int fd;//设备号

/**\brief 系统参数设定*/
/**int set_stopbit(int stop=2)
*设定停止位为2bit，没有考虑校验位，一定要校验位的话请修改源码"serialport.h"
*/
int set_stopbit(int stop=2){
    unsigned char order[]={ADR,0x06,0x00,0x8c,0x00,0x00,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
    if( 5> read (fd, result, sizeof result) ){ //从站接收错误时，返回的信息为五个字节
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
*设定通讯中断停机时间
*0~255
*数值乘以0.1为通讯中断停机时间，单位为秒；
当上次通讯后经过此设定时间尚无下一次通讯，那么将对电机进行制动(刹车)。
*默认3.2秒没有收到485通讯信息就进行紧急停机
*/
int set_breaktime(unsigned char bkt=32){//默认3.2秒没有收到485通讯信息就进行紧急停机
    unsigned char order[]={ADR,0x06,0x00,0x8e,0x00,bkt,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*设定自测速自定义电流波动阈值
*0~255，默认为0
*数值大于0且越小换向频率检测越灵敏，稳速效果越好，
但电流波动检测过于灵敏可能导致堵转检测失效。
当数值为0时不使用自定义电流波动阈值，采用机器默认值。
*/
int set_current_fluctuation_threshold(unsigned char cft=0){
    unsigned char order[]={ADR,0x06,0x00,0x72,0x00,cft,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*设定调速方式
*0：PWM方式
*1：转矩方式
*2：自测速闭环调速
*3：外接测速发电机闭环调速
*默认取2，自测速闭环调速
*当使用0x06功能码，或使用0x10功能码且寄存器数量小于15时，配置操作只是暂存，并未生效；
当0x10功能码的寄存器数量为15或使用0x06功能码向0x0180寄存器写1，配置操作才会生效。
*/
int set_speed_governing(unsigned char sg=2){
    unsigned char order[]={ADR,0x06,0x00,0x80,0x00,sg,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
    parity  =CRC16(order2,6);//计算CRC16，用于校验
    order2[6]=parity;
    order2[7]=parity>>8;
    write(fd,order2,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*设定堵转停止时间
*0~255
*默认堵转1.6s后紧急停机，防止电机烧坏
*数值乘以0.1为堵转停止时间，单位为秒；
数值为0时，不进行堵转停止，当数值非0时，将在堵转相应秒数时间后进行限位制动(刹车)。
*/
int set_block_stop(unsigned char bs=16){
    unsigned char order[]={ADR,0x06,0x00,0x81,0x00,bs,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*设定加速缓冲时间
*0~255
*默认加速缓冲时间为1.6秒
*数值乘以0.1为PWM由0增加到最大值的缓冲时间
*/
int set_accelarate_time(unsigned char sat=16){
    unsigned char order[]={ADR,0x06,0x00,0x85,0x00,sat,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*电机额定电流
*默认3A,为了测试安全
*0~2000
*数值乘以0.01为电流值，单位为A。
*
*/
int set_current_consumption(unsigned short current=300){
    unsigned char order[]={ADR,0x06,0x00,0x86,current>>8,current,0x00,0x00};//实测位运算可行
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*设定电机最大启动/负载电流
*默认3A
*0~2000
*数值乘以0.01为电流值，单位为A。
*/
int set_current_max(unsigned short current=300){
    unsigned char order[]={ADR,0x06,0x00,0x87,current>>8,current,0x00,0x00};//实测位运算可行
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*电机制动(刹车)电流
*默认2A
*数值乘以0.01为电流值，单位为A。
*/
int set_brake_current(unsigned short current=200){
    unsigned char order[]={ADR,0x06,0x00,0x88,current>>8,current,0x00,0x00};//实测位运算可行
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*制动缓冲时间
*默认1s
*数值乘以0.1为刹车PWM由0增加到最大值的缓冲时间。
*/
int set_brake_time(unsigned char time=10){
    unsigned char order[]={ADR,0x06,0x00,0x89,0x00,time,0x00,0x00};//实测位运算可行
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*减速缓冲时间,默认1s
*数值乘以0.1为PWM由最大值减小到0的缓冲时间。
*/
int set_decelerate_time(unsigned char time=10){
    unsigned char order[]={ADR,0x06,0x00,0x8F,0x00,time,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*串口通讯控制方式时是否禁止配置
*0：不禁止（默认）
*1：禁止
*/
int set_serial_configuration(unsigned char sc=0){
    unsigned char order[]={ADR,0x06,0x00,0x8D,0x00,sc,0x00,0x00};//实测位运算可行
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*波特率：1200~115200
*默认9600
*单位为bps
*/
int set_baud_rate(int32_t baud=9600){
    unsigned char order[]={ADR,0x10,0x00,0x8A,0x00,0x02,0x04,baud>>24,baud>>16,baud>>8,baud,0x00,0x00};//总共13字节
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,11);//计算CRC16，用于校验前11个字节
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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

/**PID系数设置*/
/**int set_pid_p(float p=0.02)
*P系数
*建议0.001~1,默认0.02
*/
int set_pid_p(float p=0.02){
    unsigned char *b1 =  (unsigned char*)&p;
    unsigned char order[]={ADR,0x10,0x00,0x90,0x00,0x02,0x04,b1[0],b1[1],b1[2],b1[3],0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,11);//计算CRC16，用于校验前11个字节
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
    if( 5> read (fd, result, sizeof result) ){ // then read failed
            return -1;
            break;
    }
    if(result[1]=0x10){std::cout<<"set PID parameter p success"<<std::endl;}
    else if(result[1]=0x90){
            printf("set PID parameter p error! Code %x\n",result[2]);
            return -1;
    }

    //保存P系数的配置
    unsigned char order2[]={ADR,0x06,0x01,0x90,0x00,0x01,0x00,0x00};
    parity  =CRC16(order2,6);//计算CRC16，用于校验
    order2[6]=parity;
    order2[7]=parity>>8;
    write(fd,order2,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*i系数
*建议0.001~1,默认0.02
*/
int set_pid_i(float i=0.02){
    unsigned char *b1 =  (unsigned char*)&i;
    unsigned char order[]={ADR,0x10,0x00,0x92,0x00,0x02,0x04,b1[0],b1[1],b1[2],b1[3],0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,11);//计算CRC16，用于校验前11个字节
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*d系数
*建议0.001~1,默认0.02
*/
int set_pid_d(float d=0.02){
    unsigned char *b1 =  (unsigned char*)&d;
    unsigned char order[]={ADR,0x10,0x00,0x94,0x00,0x02,0x04,b1[0],b1[1],b1[2],b1[3],0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,11);//计算CRC16，用于校验前11个字节
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*D调节节拍周期
*1~1000,默认1
*D调节节拍，单位为ms；PI调节的节拍固定约1ms。
*/
int set_pid_td(unsigned short td=1){
    unsigned char order[]={ADR,0x06,0x00,0x97,td>>8,td,0x00,0x00};
    unsigned char result[8];
    unsigned short parity;
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*设置速度；数值大于0正转，小于0反转，等于0刹车
*默认自测速闭环调速方式，没有添加别的功能：-1000～1000
*设置换向频率，单位为次/秒。
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
    parity  =CRC16(order,6);//计算CRC16，用于校验
    order[6]=parity;
    order[7]=parity>>8;
    write(fd,order,8);
    usleep(wait_answer_duration);//停止1ms，等待返回数据
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
*读取当前电机运作状态
*其实现在有用的数据只有
*实时PWM
*实时电流
*换向频率（间接得到电机转速，但是你要知道电机的特性，转一圈换向几次）
*/
int state_now(){
    unsigned char order[]={ADR,0x03,0x00,0x10,0x00,0x0B,0x05,0xFB};
    unsigned char result[27];
    if(order[0]!=0x02){
        unsigned short parity;
        parity  =CRC16(order,6);//计算CRC16，用于校验
        order[6]=parity;
        order[7]=parity>>8;
    }
    write(fd,order,8);
    usleep(3*wait_answer_duration);//数据量比较大，多等一会
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
            std::cout<<"实时PWM  "<<state[0]<<std::endl;
            std::cout<<"实时电流 "<<state[1]<<std::endl;
            std::cout<<"换向频率 "<<state[2]<<std::endl;
            std::cout<<"AI1电压  "<<state[4]<<std::endl;
            std::cout<<"AI2电压  "<<state[5]<<std::endl;
            std::cout<<"AI1、AI2间差分电压 "<<state[6]<<std::endl;
            std::cout<<"SQ1电平 "<<state[8]<<std::endl;
            std::cout<<"SQ2电平 "<<state[9]<<std::endl;
            std::cout<<"DE电平 "<<state[10]<<std::endl;
    }
    else if(result[1]=0x90){
            printf("get state error! Code %x\n",result[2]);
            return -1;
    }
    return 0;

}
#endif // AQMD3620NS-A_H_INCLUDED
