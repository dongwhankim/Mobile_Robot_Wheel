#include "Communication.h"
#include <pthread.h>
#include <iostream>

#define PERIOD_NS 20000000  // 0.03s
#define SEC_IN_NSEC 1000000000

CCommunication communication;
using namespace std;

int main()
{ 
    double now_time , now_time2;
    struct timespec ts;
    struct timespec ts2;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    clock_gettime(CLOCK_MONOTONIC, &ts2);
    int start_time = ts2.tv_sec;

    int cnt = 0;

    communication.Open_port();
    communication.Select_Speed();

    while(1)
    {
         while(ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }

        while(ts2.tv_nsec >= SEC_IN_NSEC)
        {
            ts2.tv_sec++;
            ts2.tv_nsec -= SEC_IN_NSEC;
        }
        
        //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ts.tv_nsec +=PERIOD_NS;
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        now_time = ts2.tv_sec - start_time;
        now_time2 = now_time + ts2.tv_nsec / 1000000000.0;
        cout<<"Time : "<<now_time2<<endl;

        if(cnt == 200)
        {
            communication.Select_Speed();
            cnt = 0;
        }
        communication.Read_encoder();
        cnt++;
    }
    return (0);
}