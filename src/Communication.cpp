// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
//#include <string>
#include <cmath>
#include <stdlib.h>
//#include <pthread.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function0
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <Communication.h>
#include <sys/select.h>

#define PI 3.141592

typedef unsigned char 	BYTE;
typedef unsigned int 	WORD;

struct timeval tv;
fd_set readfds;
int ret;

typedef struct {
	BYTE byLow;
	BYTE byHigh;
} IByte;

short Byte2Int(BYTE byLow, BYTE byHigh)
{
return (byLow | (short)byHigh<<8);
}

IByte Int2Byte(int nIn)
{
	IByte Ret;

	Ret.byLow = nIn & 0xff;
	Ret.byHigh = nIn>>8 & 0xff;
	return Ret;
}

long Byte2Long(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4)
{
	return((long)byData1 | (long)byData2<<8 | (long)byData3<<16 | 
		(long)byData4<<24);
}

short IsChkSumOK(BYTE *byArray, short nPacketSize)
{
short i;
BYTE cbySum;
cbySum = 0;
for(i=0; i<nPacketSize; i++) {
cbySum += *(byArray + i);
}
if(cbySum==0) return 1;
else return 0;
}

int getVel()
{
	int vellocity;
	std::cout << "\n vellocity : ";
	std::cin >> vellocity;
	return vellocity;
}

CCommunication::CCommunication()
{
  _cnt = 0;
  _REQ_DATA[0] = 183;
  _REQ_DATA[1] = 184;
  _REQ_DATA[2] = 1;
  _REQ_DATA[3] = 4;
  _REQ_DATA[4] = 1;
  _REQ_DATA[5] = 210;
  _REQ_DATA[6] = GetCheckSum(sizeof(_REQ_DATA)-1,_REQ_DATA);
  
  _vellocity[0] = 183;
  _vellocity[1] = 184;
  _vellocity[2] = 1;
  _vellocity[3] = 207;
  _vellocity[4] = 7;
  _vellocity[5] = 1;
  _vellocity[6] = 0x00;
  _vellocity[7] = 0x00;
  _vellocity[8] = 1;
  _vellocity[9] = 0x00;
  _vellocity[10] = 0x00;
  _vellocity[11] = 2;
 

  _torque[0] = 184;
  _torque[1] = 183;
  _torque[2] = 1;
  _torque[3] = 140;
  _torque[4] = 2;
  _torque[5] = 0x00;
  _torque[6] = 0x00;

  _TQ_off[0] = 183;
  _TQ_off[1] = 184;
  _TQ_off[2] = 1;
  _TQ_off[3] = 5;
  _TQ_off[4] = 1;
  _TQ_off[5] = 0;
  _TQ_off[6] = GetCheckSum(sizeof(_TQ_off)-1,_TQ_off);

}
CCommunication::~CCommunication()
{

}

void CCommunication::Open_port()
{
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  _port = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY);

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  
  bzero(&tty, sizeof(tty)); 
  // Read in existing settings, and handle any error
  if(tcgetattr(_port, &tty) != 0) 
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  //tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  //tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  //tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  //tty.c_cflag |= CS8; // 8 bits per byte (most common)
  //tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  //tty.c_cflag = CREAD; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  //tty.c_cflag =  IGNPAR | ICRNL;
  //tty.c_ lflag &= ~ICANON;
  //tty.c_lflag &= ECHO; // Disable echo
  //tty.c_lflag &= ~ECHOE; // Disable erasure
  //tty.c_lflag &= ~ECHONL; // Disable new-line echo
  //tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  //tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  //tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  //tty.c_iflag = INPCK;
  //tty.c_oflag = 0;
  //tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  //tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  //tty.c_cc[VTIME] = 0.01;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  //tty.c_cc[VMIN] = 1;
  //tty.c_lflag = ICANON;
  
  // Set in/out baud rate to be 9600
  //cfsetispeed(&tty, B9600);

  // Set in/out baud rate to be 115200
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_lflag = 0;
  bzero(tty.c_cc, NCCS);
  tty.c_cc[VTIME] = 0; 
  tty.c_cc[VMIN] = 1;  

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CS8;
  tty.c_lflag &= ~ECHO; 
  //tty.c_lflag &= ~ECHO; // Disable echo
  //tty.c_lflag &= ~ECHOE; // Disable erasure
  //tty.c_lflag &= ~ECHONL; // Disable new-line echo
  //tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  //tty.c_cflag &= CREAD;
  //tty.c_iflag = ICRNL | INPCK | IGNPAR;
  //tty.c_oflag = 0;
  //tty.c_lflag  =  ICANON;

  
  tcsetattr(_port,  TCSANOW,  &tty);
  tcflush(_port,  TCIOFLUSH);//
  // Save tty settings, also checking for error
  if (tcsetattr(_port, TCSANOW, &tty) != 0) 
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  //write(_port, _REQ_DATA, sizeof(_REQ_DATA)); 
  //read(_port, _buf, sizeof(_buf));
  //std::cout<<Byte2Long(_buf[10],_buf[11],_buf[12],_buf[13])<<std::endl;
}

void CCommunication::Read_encoder()
{ 
  tcflush(_port,  TCIOFLUSH);
  IByte iData;
  iData = Int2Byte(_target_vel);

  for(int i = 0 ; i < 255 ; i++)
  {
    _buf[i] = 0;
  }

  if(_target_vel == 10)
  {
    iData = Int2Byte(-10);
    _vellocity[6] = ~iData.byLow;
    _vellocity[7] = ~iData.byHigh;
  }
  else
  {
    _vellocity[6] = iData.byLow;
    _vellocity[7] = iData.byHigh;
  }
  _vellocity[12] = GetCheckSum(sizeof(_vellocity)-1,_vellocity);
  write(_port, _vellocity, sizeof(_vellocity));
  usleep(1000);

  FD_ZERO(&readfds);
  FD_SET(_port, &readfds);
  tv.tv_sec = 0;
  tv.tv_usec = 16500; // 0.02s
  ret = select(_port + 1, &readfds, NULL, NULL, &tv);
  if (FD_ISSET(_port, &readfds)) 
  {
  read(_port, _buf, sizeof(_buf));
  }
  if(IsChkSumOK(_buf,24))
  {
    std::cout<<_cnt<<std::endl;
    _encoder_vel = Byte2Int(_buf[5],_buf[6]);
    _encoder_pos = Byte2Long(_buf[10],_buf[11],_buf[12],_buf[13]);
    _vello_MS = _encoder_vel * 2 * PI * 0.206 / 60; 
    std::cout<<"RPM : "<<_encoder_vel <<" (Rotate Per Minute)" <<std::endl;
    std::cout<<"Vellocity : "<<_vello_MS<< " (m/s)"<<std::endl;
    std::cout<<"Position : " << _encoder_pos << " (CNT)" << std::endl;
    std::cout<<"Status : " << (int) _buf[9] << std::endl;
    // std::cout<<"good__buf[0]" << (int) _buf[0] << std::endl;
    // std::cout<<"_buf[1]" << (int) _buf[1] << std::endl;
    // std::cout<<"_buf[2]" << (int) _buf[2] << std::endl;
    // std::cout<<"_buf[3]" << (int) _buf[3] << std::endl;
    // std::cout<<"_buf[4]" << (int) _buf[4] << std::endl;
    // std::cout<<"_buf[5]" << (int) _buf[5] << std::endl;
    // std::cout<<"_buf[6]" << (int) _buf[6] << std::endl;
    // std::cout<<"_buf[7]" << (int) _buf[7] << std::endl;
    // std::cout<<"_buf[8]" << (int) _buf[8] << std::endl;
    // std::cout<<"_buf[9]" << (int) _buf[9] << std::endl;
    // std::cout<<"_buf[10]" << (int) _buf[10] << std::endl;
    // std::cout<<"_buf[11]" << (int) _buf[11] << std::endl;
    // std::cout<<"_buf[12]" << (int) _buf[12] << std::endl;
    // std::cout<<"_buf[13]" << (int) _buf[13] << std::endl;
    // std::cout<<"_buf[14]" << (int) _buf[14] << std::endl;
    // std::cout<<"_buf[15]" << (int) _buf[15] << std::endl;
    // std::cout<<"_buf[16]" << (int) _buf[16] << std::endl;
    // std::cout<<"_buf[17]" << (int) _buf[17] << std::endl;
    // std::cout<<"_buf[18]" << (int) _buf[18] << std::endl;
    // std::cout<<"_buf[19]" << (int) _buf[19] << std::endl;
    // std::cout<<"_buf[20]" << (int) _buf[20] << std::endl;
    // std::cout<<"_buf[21]" << (int) _buf[21] << std::endl;
    // std::cout<<"_buf[22]" << (int) _buf[22] << std::endl;
    // std::cout<<"good__buf[23]" << (int) _buf[23] << std::endl;
    //std::cout<<"good_buf[23]" << (int) _buf[23] << std::endl;
    _cnt++;
  }
  // else
  // {
  //   std::cout<<"_buf[0]" << (int) _buf[0] << std::endl;
  //   std::cout<<"_buf[1]" << (int) _buf[1] << std::endl;
  //   std::cout<<"_buf[2]" << (int) _buf[2] << std::endl;
  //   std::cout<<"_buf[3]" << (int) _buf[3] << std::endl;
  //   std::cout<<"_buf[4]" << (int) _buf[4] << std::endl;
  //   std::cout<<"_buf[5]" << (int) _buf[5] << std::endl;
  //   std::cout<<"_buf[6]" << (int) _buf[6] << std::endl;
  //   std::cout<<"_buf[7]" << (int) _buf[7] << std::endl;
  //   std::cout<<"_buf[8]" << (int) _buf[8] << std::endl;
  //   std::cout<<"_buf[9]" << (int) _buf[9] << std::endl;
  //   std::cout<<"_buf[10]" << (int) _buf[10] << std::endl;
  //   std::cout<<"_buf[11]" << (int) _buf[11] << std::endl;
  //   std::cout<<"_buf[12]" << (int) _buf[12] << std::endl;
  //   std::cout<<"_buf[13]" << (int) _buf[13] << std::endl;
  //   std::cout<<"_buf[14]" << (int) _buf[14] << std::endl;
  //   std::cout<<"_buf[15]" << (int) _buf[15] << std::endl;
  //   std::cout<<"_buf[16]" << (int) _buf[16] << std::endl;
  //   std::cout<<"_buf[17]" << (int) _buf[17] << std::endl;
  //   std::cout<<"_buf[18]" << (int) _buf[18] << std::endl;
  //   std::cout<<"_buf[19]" << (int) _buf[19] << std::endl;
  //   std::cout<<"_buf[20]" << (int) _buf[20] << std::endl;
  //   std::cout<<"_buf[21]" << (int) _buf[21] << std::endl;
  //   std::cout<<"_buf[22]" << (int) _buf[22] << std::endl;
  //   std::cout<<"_buf[23]" << (int) _buf[23] << std::endl;
  // }
}

void CCommunication::Select_Speed()
{
  _target_vel = getVel();
  _cnt = 0; 
}


unsigned char CCommunication::GetCheckSum(char nPacketSize, unsigned char *byArray)
{
    char byTmp=0;
    char i;
    for(i=0; i<nPacketSize; i++) 
    {   
        byTmp += *(byArray+i);
    }
    return (~byTmp + 1); 
}


