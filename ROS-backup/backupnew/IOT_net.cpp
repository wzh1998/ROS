/******************************************************************
功能如下：

串口通信说明：
1.写入串口
                               
（1）内容：         地址  读/写命令  灯光   窗帘   电源   风扇 
（2）格式：  oxA5    1     1/2     1/2    1/2   1/2   1/2   0x0d 0x0a
2.读取串口                        光照度   温度   湿度   CO2
（1）内容                         煤气     漏水
（2）格式：
*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <iot_modules/IOTnet.h>
#include <std_msgs/String.h>
#include <sstream>
#include <std_msgs/Int32.h>
// Remote control
#include <ros_arduino_msgs/Analog.h>
//以下为串口通讯需要的头文件
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
#include <ctime>

#define OFFSIG 2
#define ONSIG 1
#define MOTIONLEN 43
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
bool lightOnFlag = false;
bool lightOffFlag = false;
bool sensorFlag = false;
//************************************************************************
int module_adderss_S=0;  //发送的模块地址，
int module_adderss_R=0;  //接收到的模块地址
int sim_;
//************************通讯参数配置*******************************************************
unsigned char data_terminal0=0x0d;  
unsigned char data_terminal1=0x0a;  
unsigned char module_command[21]={0};   //要发给串口的数据
string rec_buffer;  //串口数据接收变量

float data_status_LED, data_status_CURTAIN, data_status_FAN, data_warn_WATER, data_warn_GAS, data_warn_DOOR;

// prevent if can't read the real data
float data_tem = 25;
float data_wet = 40;
float data_co2 = 534;
float data_light = 150;




ros::Subscriber sub_cmd;
ros::Subscriber sub_nlu;

ros::Publisher pub_words;
ros::Publisher pub_dobot;
ros::Publisher pub_dobot_motion;
ros::Publisher pub_debug;
/*
string port("/dev/IOT_netstm");
unsigned long baud = 9600;
serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
*/

unsigned char mod[] = {1,2,4,5,9};

union floatData {
    float d ;
    unsigned char data[4];
}module_data1,module_data2,module_data3,module_data4,command_data1,command_data2,command_data3,command_data4;


//********************************************************************************************/

//***********************************格式转换**************************************************************

template <typename T>
std::string ToString(T val) {
    std::stringstream stream;
    stream << val;
    return stream.str();
}
/*
const std::string getCurrentSystemTime(){
        long long tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        struct tm* ptm = localtime(&tt);
        char date[60] = { 0 };
        sprintf(date, "%d-%02d-%02d      %02d:%02d:%02d",
                (int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday,
                (int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
        return std::string(date);
}
*/

//***************************************************************************************88

void   Delay(int   time) {
clock_t   now   =   clock(); 

while(   clock()   -   now   <   time   ); 
} 




//***********************串口函数**********************************************************
void serial_write(int address,int command,float data1,float data2,float data3,float data4) {
    string port("/dev/IOT_netstm");
    unsigned long baud = 9600;    
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000)); 
   
    command_data1.d=data1;
    command_data2.d=data2;
    command_data3.d=data3;
    command_data4.d=data4;

    module_command[0] = 0xa5;
    module_command[1] = address;  //模块地址
    module_command[2] = command; //写入下位机命令指令， 

    for(int i=0;i<4;i++)    //发送要读取模块的指令
    {
 	 module_command[i+3]=command_data1.data[i];
         module_command[i+7]=command_data2.data[i];
         module_command[i+11]=command_data3.data[i];
         module_command[i+15]=command_data4.data[i];
 //        ROS_INFO("command_data1: %x ",command_data1.data[i]); 
    }
 
    module_command[19]=data_terminal0;
    module_command[20]=data_terminal1;
    
    my_serial.write(module_command,21);
    ROS_INFO("already send data ");
}

//****************************************************************************************

//*************************回调函数******************************************************
void callback(const iot_modules::IOTnet & cmd_input) {
    ROS_INFO("IOT program is running in callback!!!");  
    int position;     
    position = cmd_input.netcmd ;//获取命令
   
    ROS_INFO("position:%d",position);
  
    if(position == 1) {
        // 检测光强
        std_msgs::String words;
        words.data = "当前室内光照强度为 " + ToString(data_light) + " 勒克思。";
       	ROS_INFO("now at position 1"); 
        float ACTION_LIGHT,ACTION_CURTAIN;
        // 决定开灯或者关灯
        if(data_status_LED == OFFSIG) {
            words.data += "光照充足，已经为您开启一二号电灯";
            ACTION_LIGHT = ONSIG;
        }
        else { // if(data_status_LED == OFFSIG) {
            words.data += "光照不足，已经为您关闭一二号电灯";
            ACTION_LIGHT = OFFSIG;
        }
        
        if(data_status_CURTAIN == ONSIG) {
            words.data += "并关闭窗帘";
            ACTION_CURTAIN = OFFSIG;
        }
        else { // if(data_status_CURTAIN == OFFSIG) {
            words.data += "并开启窗帘";
            ACTION_CURTAIN = ONSIG;
        }
        pub_words.publish(words);
        for(int i = 0;i < 5;i++) {
            serial_write(1,2,0,ACTION_LIGHT,0,ACTION_LIGHT);
            serial_write(9,2,ACTION_CURTAIN,ACTION_CURTAIN,ACTION_CURTAIN,ACTION_CURTAIN);
        }
    }
    
    else if(position == 2) {
        // 取水
	std_msgs::String words;
        words.data = "正在为您拿取水杯";
	pub_dobot.publish(1);
    }
    
    else if(position == 3) {
        std_msgs::String words;
	time_t tt = time(NULL);
	tm* t = localtime(&tt);

	words.data = "现在时间是" + ToString(t->tm_mon + 1) + "月" + ToString(t->tm_mday) + "日。"+ ToString(t->tm_hour) + "点" + ToString(t->tm_min) + "分" +  "。您该吃药了";
	pub_words.publish(words);


	
	words.data = "当前室内温度为 " + ToString(data_tem) + " 摄氏度。相对湿度为百分之 " + ToString(data_wet) + "。二氧化碳浓度为 " + ToString(data_co2) + "屁屁m。";
        float ACTION_FAN;
        if(data_status_FAN == ONSIG) {
            words.data += "温度适宜，已经为您关闭一二号风扇";
            ACTION_FAN = OFFSIG;
        }
        else { // if(data_status_FAN == OFFSIG) {
            words.data += "温度过高，已经为您开启一二号风扇";
            ACTION_FAN = ONSIG;
        }
        
        pub_words.publish(words);
        for(int i = 0;i < 5;i++) serial_write(1,2,ACTION_FAN,0,ACTION_FAN,0);

	Delay(5);   
 
//	words.data = "您该吃药了";
        //pub_words.publish(words);
	pub_dobot.publish(2);
	
       //***new 娱乐
words.data ="我会陪小人跳舞";

    pub_words.publish(words);

    serial_write(6,1,0,0,0,0);
    serial_write(6,1,0,0,0,0);
    serial_write(6,1,0,0,0,0);
    serial_write(6,1,0,0,0,0);
    serial_write(6,1,0,0,0,0);
    //pub_dobot.publish(3);
    pub_dobot.publish(99);

	words.data ="我还会讲笑话。一个人在输液时突然开始大笑，。";
	pub_words.publish(words);
	words.data ="医生问他怎么了。病人回答，他笑点滴。";
	pub_words.publish(words);
       
	    
    }
    else if(position == 4) { // sensorFlag debug test
        std_msgs::String words;
        words.data = "下面为您播报安全检测情况。";
//        pub_words.publish(words); // debug test 
        if(data_warn_GAS == OFFSIG) {
            words.data += "煤气情况异常，请您注意家中煤气泄漏，如需帮助您可以通过机器人语音模式进行求助。";
        }
        else {// if(data_warn_GAS == OFFSIG) {
            words.data += "煤气检测正常，无煤气泄漏。";
        }
        
        if(data_warn_WATER == OFFSIG) {
            words.data += "漏水情况异常，家中有漏水情况发生，请您小心滑倒，及时关闭漏水阀门。";
        }
        else { // if(data_warn_WATER == OFFSIG) {
            words.data += "漏水检测正常，无漏水情况发生。";
        }
	words.data += "门锁检测正常，门窗均已锁好";
        pub_words.publish(words);
        
        string commentst, comment1, comment2, comment3, commented, commentfn;
        
        commentst = "室内情况初步评估完成，下面为您播报居室评价和建议。";
        
        if(data_tem > 30) {
            comment1 = "当前您的室内温度过高，建议您及时开启空调，以防中暑等意外情况的发生。不过";
        }
        else if(data_tem >= 24 && data_tem <= 30) {
            comment1 = "当前您的室内温度比较适宜居住。但是";
        }
        else {
            comment1 = "当前您的室内温度略低，建议您调高空调温度至25度，以防着凉感冒。不过";
        }
        
        
        
        if(data_wet > 60 && data_tem >= 24 && data_tem <= 30) {
            comment2 = "室内过于潮湿，容易滋生病毒和细菌，建议您勤开窗通风或启动除湿器。此外，过湿的环境还有可能影响您的身体健康，希望您多加注意。";
        }
        else {
            comment2 = "室内湿度很适宜居住，希望您继续保持。";
        }
        
        
        if(data_co2 > 900) {
            comment3 = "二氧化碳含量超标，建议您通风换气或者开启空气清新器，良好的室内空气将有助于您的呼吸健康。";
        }
        else {
            comment3 = "您的室内空气较为清新，良好的室内空气将有助于您的呼吸健康。";
        }

	        
        commented = "以上是您的室内评估报告，健康生活和室内环境密切相关，从身边点点滴滴做起，改善室内环境，打造美好新生活。";
        commentfn = commentst + comment1 + comment2 + comment3 + commented;
        pub_words.publish(commentfn);
    }

	
    
}

void subcallback_nlu(const std_msgs::Int32::ConstPtr& msg) {
    std_msgs::String words; 
    if(msg->data == 10){
	serial_write(6,1,0,0,0,0);
	sensorFlag = true;
    }
    if(msg->data == 11){
	words.data = "已经为您开启电灯";
        pub_words.publish(words);
        serial_write(1,2,0,ONSIG,0,ONSIG);
    }

    else if(msg->data == 12){
	words.data = "已经为您关闭电灯";
        pub_words.publish(words);
        serial_write(1,2,0,OFFSIG,0,OFFSIG);
    }
 
    else if(msg->data == 13){
        words.data = "已经为您开启风扇";
        pub_words.publish(words);
        serial_write(1,2,ONSIG,0,ONSIG,0);
    }

    else if(msg->data == 14){
	words.data = "已经为您关闭风扇";
	pub_words.publish(words);
	serial_write(1,2,OFFSIG,0,OFFSIG,0);
    }

    else if(msg->data == 15){
        words.data = "已经为您开启窗帘";
        pub_words.publish(words);
        serial_write(9,2,ONSIG,ONSIG,ONSIG,ONSIG);
    }

    else if(msg->data == 16){
        words.data = "已经为您关闭窗帘";
	pub_words.publish(words);
        serial_write(9,2,OFFSIG,OFFSIG,OFFSIG,OFFSIG);
    }

}

void catchData() {
  
    string port("/dev/IOT_netstm");
    unsigned long baud = 9600;    
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));    
   
    rec_buffer =my_serial.readline(50,"\n");
    const char *receive_data=rec_buffer.data();
    ROS_INFO("receive_data[0]:%x",receive_data[0]);
    ROS_INFO("read data len: %d ",rec_buffer.length());  //显示接收数据的长度
    
    if(rec_buffer.length()==21) {
        if(receive_data[0]==0xffffffA5) {
            int receive_address = receive_data[1];
            
            for(int i=0;i<4;i++) {
                module_data1.data[i]=receive_data[i+3];
                module_data2.data[i]=receive_data[i+7];
                module_data3.data[i]=receive_data[i+11];
                module_data4.data[i]=receive_data[i+15];
            }
            ROS_INFO("module_data:%f %f %f %f",module_data1.d,module_data2.d,module_data3.d,module_data4.d);
            if(receive_address == 1) {
                data_status_LED = module_data1.d;
                data_status_FAN = module_data3.d;
            }
            else if(receive_address == 2 && module_data1.d!=0.0 &&  module_data2.d!=0.0 && module_data3.d!=0.0 && module_data4.d!=0.0) {
                data_light = module_data1.d;
                data_tem = module_data2.d;
                data_wet = module_data3.d;
                data_co2 = module_data4.d;
            }
            else if(receive_address == 4) {
                data_warn_GAS = module_data2.d;
                data_warn_WATER = module_data1.d;
            }
            else if(receive_address == 9) {
                data_status_CURTAIN = module_data1.d;
            }

	    // 门锁情况
	    else if(receive_address == 5) {
                data_warn_DOOR = module_data1.d;
            }
        }
    }
    else if(rec_buffer.length() == MOTIONLEN) {
        if(receive_data[0]==0xffffffA5 && receive_data[41]==0x0000005A) {
//	    string tmp;
	    string motionString;
//	    ROS_INFO("motion:");   //debug
//	    for(int i = 0;i < 42;i++) ROS_INFO("%x",receive_data[i]);  //debug
  	    motionString = receive_data;

	    const std_msgs::String rosStr = receive_data;
// 	    for(int i = 0;i < 42;i++) tmp[i] = receive_data[i];
//	    pilot[42] = '\0';
//	    motionString.data = tmp;
// ROS_INFO("tmp:");   //debug
 //       for(int i = 0;i < 42;i++) ROS_INFO("%x",tmp[i]);  //debug
	    for(int i = 0;i < MOTIONLEN - 1;i++) motionString[i] = receive_data[i];
	    for(int i = 0;i < MOTIONLEN - 1;i++) ROS_INFO("RD:%x",receive_data[i]);
  	    for(int i = 0;i < MOTIONLEN - 1;i++) ROS_INFO("string:%x",motionString[i]); // debug
 	    for(int i = 0;i < MOTIONLEN - 1;i++) ROS_INFO("rosStr:%x",rosStr.data[i]);	    
//	    pub_dobot_motion.publish(motionString);

	    
            serial::Serial motion_serial;
	     try{
	        motion_serial.setPort("/dev/Dobot0");
		motion_serial.setBaudrate(9600);

        	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
       	        motion_serial.setTimeout(to);
     	        serial::stopbits_t sb = serial::stopbits_one;
       		motion_serial.setStopbits(sb);
        	motion_serial.open();
    	    }
    	    catch(serial::IOException& e){
        	ROS_ERROR("Unable to open port");
    	    }

    	    if(my_serial.isOpen()){
        	ROS_INFO("Serial Port initialized");
    	    }

   	    motion_serial.write(rosStr);    
		    
//	    pub_dobot_motion.publish(tmp);
	}
    }		
}

//****************************主函数***********************************************************/
int main(int argc, char **argv) {
    
    ros::init(argc, argv, "IOT_net");
    ros::NodeHandle n;  
    string port("/dev/IOT_netstm");
    unsigned long baud = 9600;
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
    
    sub_cmd = n.subscribe("/IOT_cmd",20, callback);
    pub_debug = n.advertise<iot_modules::IOTnet>("/IOT_cmd",200);
    // 2018.7.28
    sub_nlu = n.subscribe("/voice_system/room_topic",20,subcallback_nlu);
    pub_words= n.advertise<std_msgs::String>("/voice_system/tts_topic",200);
    
    pub_dobot = n.advertise<std_msgs::Int32>("/dobot_cmd",200);
    pub_dobot_motion = n.advertise<std_msgs::String>("/dobot_cmd_motion",20);
    ros::Rate loop_rate(10);

   
    int i = 0; 
    while(ros::ok()) {
	ROS_INFO("in loop!");
        serial_write(mod[i++],1,0,0,0,0);
        if(i >= 5) i = 0;
	  /*iot_modules::IOTnet db;
	    db.netcmd = 2;
	    pub_debug.publish(db);
	   */
//       	pub_words.publish("1");
	catchData();
        ros::spinOnce();
    }
    
    return 0;
}
