#pragma once // ?


class CCommunication
{

public:
	
    CCommunication();
	virtual ~CCommunication(); //	

	void Open_port();
    void Read_encoder();
    void Select_Speed();
    unsigned char GetCheckSum(char nPacketSize, unsigned char *byArray);
    
private:
    unsigned char _REQ_DATA[7];
    unsigned char _vellocity[13];
    unsigned char _torque[8];
    unsigned char _TQ_off[7];
    int _target_vel;
    int _port;
    unsigned char _buf[255];
    long _encoder_vel;
    long _encoder_pos;
    double _vello_MS;
    int _cnt;
};