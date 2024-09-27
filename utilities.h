#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <string>
#include <list>
#include <ctime>


//-------------------------------- Define --------------------------------//
#define RAD_2_DEGREE(_x)		(180*(_x)/3.1415926)
#define DEGREE_2_RAD(_x)		(3.1415926*(_x)/180)

//------------------------------- Typedef---------------------------------//
class Utilities
{
public:
    static bool checkCRC16(unsigned char *buf, int len);		//校验buf[0]——buf[len-3]的CRC，是不是最后两位
public:
    static bool getCRC16(unsigned char *buf, int len);			//计算buf[0]——buf[len-3]的CRC，填充到最后两位
    static bool cheCRC16(unsigned char *buf, int len);		//校验buf[0]——buf[len-3]的CRC，是不是最后两位

private:

};

#endif // UTILITIES_H
