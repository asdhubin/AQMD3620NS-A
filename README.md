# AQMD3620NS-A
AQMD3620NS-A直流电机驱动器驱动程序，用于电机调速以及状态读取

/**define wait_answer_duration 1000
*默认发送命令后到读取返回信息中间，等待1ms
*/

/**\brief 系统参数设定*/
/**int set_stopbit(int stop=2)
*设定停止位为2bit，没有考虑校验位，一定要校验位的话请修改源码"serialport.h"
*/

/**int set_breaktime(unsigned char bkt=32)
*设定通讯中断停机时间
*0~255
*数值乘以0.1为通讯中断停机时间，单位为秒；
当上次通讯后经过此设定时间尚无下一次通讯，那么将对电机进行制动(刹车)。
*默认3.2秒没有收到485通讯信息就进行紧急停机
*/

/**int set_current_fluctuation_threshold(unsigned char cft=0)
*设定自测速自定义电流波动阈值
*0~255，默认为0
*数值大于0且越小换向频率检测越灵敏，稳速效果越好，
但电流波动检测过于灵敏可能导致堵转检测失效。
当数值为0时不使用自定义电流波动阈值，采用机器默认值。
*/

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

/**int set_block_stop(unsigned char bs=16)
*设定堵转停止时间
*0~255
*默认堵转1.6s后紧急停机，防止电机烧坏
*数值乘以0.1为堵转停止时间，单位为秒；
数值为0时，不进行堵转停止，当数值非0时，将在堵转相应秒数时间后进行限位制动(刹车)。
*/

/**int set_accelarate_time(unsigned char sat=16)
*设定加速缓冲时间
*0~255
*默认加速缓冲时间为1.6秒
*数值乘以0.1为PWM由0增加到最大值的缓冲时间
*/

/**set_current_consumption(unsigned short current=300)
*电机额定电流
*默认3A,为了测试安全
*0~2000
*数值乘以0.01为电流值，单位为A。
*
*/

/**set_current_max(unsigned short current=300)
*设定电机最大启动/负载电流
*默认3A
*0~2000
*数值乘以0.01为电流值，单位为A。
*/

/**int set_brake_current(unsigned short current=200)
*电机制动(刹车)电流
*默认2A
*数值乘以0.01为电流值，单位为A。
*/

/**int set_brake_time(unsigned char time=10)
*制动缓冲时间
*默认1s
*数值乘以0.1为刹车PWM由0增加到最大值的缓冲时间。
*/

/**int set_decelerate_time(unsigned char time=10)
*0~255
*减速缓冲时间,默认1s
*数值乘以0.1为PWM由最大值减小到0的缓冲时间。
*/

/**int set_serial_configuration(unsigned char sc=0)
*串口通讯控制方式时是否禁止配置
*0：不禁止（默认）
*1：禁止
*/

/**int set_baud_rate(int32_t baud=9600)
*波特率：1200~115200
*默认9600
*单位为bps
*/

/**\brief PID系数设置*/
/**int set_pid_p(float p=0.02)
*P系数
*建议0.001~1,默认0.02
*/

/**int set_pid_i(float i=0.02)
*i系数
*建议0.001~1,默认0.02
*/

/**int set_pid_d(float d=0.02)
*d系数
*建议0.001~1,默认0.02
*/

/**int set_pid_td(short td=1)
*D调节节拍周期
*1~1000,默认1
*D调节节拍，单位为ms；PI调节的节拍固定约1ms。
*/

/**int set_speed(short sp=0)
*设置速度；数值大于0正转，小于0反转，等于0刹车
*默认自测速闭环调速方式，没有添加别的功能：-1000～1000
*设置换向频率，单位为次/秒。
*这个是唯一可玩的命令，可以让小车跑来跑去，前面这么多废话都是参数设定。
*/

/**int state_now()
*读取当前电机运作状态
*其实现在有用的数据只有
*实时PWM
*实时电流
*换向频率（间接得到电机转速，但是你要知道电机的特性，转一圈换向几次）
*/