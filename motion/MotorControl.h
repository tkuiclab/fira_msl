//-------cssl include-------//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

#include "cssl/cssl.h"
#include "cssl/port.h"
#include "cssl/cssl.c"
//#include "cssl/info.h"
#define VERSION "by Lain-Jinn Hwang\n"
#include "math.h"

double pre_v_l=0;
double pre_v_r=0;

//-------cssl variable-------//
char *echo ="\r";
cssl_t *serial=0;

//====================//
//   cssl callback    //
//====================//
static void mcssl_callback(int id, uint8_t *buf, int length)
{
}

//====================//
//   cssl init        //
//====================//
int mcssl_init()
{
    char *devs="/dev/ttyUSB0";

     cssl_start();

    // modify 19200 to desire value
    serial=cssl_open(devs, mcssl_callback, 0, 115200, 8, 0, 1);

    if (!serial){
        printf("%s\n",cssl_geterrormsg());
        puts("\n--->RS232 OPEN FAIL (cssl_open error) <---");
        fflush(stdout);
        return -1;
    }
    cssl_setflowcontrol(serial, 0, 0);


    return 1;
}

//====================//
//   cssl finish      //
//====================//
void mcssl_finish(){

    cssl_close(serial);
    cssl_stop();

}

void mcssl_send2motor(double w1, double w2,double w3,double en1,double en2,double en3){
    //w3=103;
    char w1_dir = (w1 > 0)  ? 0x80 : 0;
    char w2_dir = (w2 > 0)  ? 0x80 : 0;
    char w3_dir = (w3 > 0)  ? 0x80 : 0;

    if(abs(w1)<=25)en1=0;
    if(abs(w2)<=25)en2=0;
    if(abs(w3)<=25)en3=0;

    if(abs(w1)>103)en1=0;
    if(abs(w2)>103)en2=0;
    if(abs(w3)>103)en3=0;

    char w1_byte = (char) abs(w1);
    char w2_byte = (char) abs(w2);
    char w3_byte = (char) abs(w3);

    char en_byte = (char) abs(en1*128+en2*64+en3*32);

    ROS_INFO("w1:%0.2f w2:%0.2f w3:%0.2f en1:%f en2:%f en3:%f\n",w1,w2,w3,en1,en2,en3);

    cssl_putchar(serial,0xff);
    cssl_putchar(serial,0xfa);

    cssl_putchar(serial,w1_byte + w1_dir);
    cssl_putchar(serial,w2_byte + w2_dir);
    cssl_putchar(serial,w3_byte + w3_dir);
    cssl_putchar(serial,en_byte);
    //cssl_putchar(serial,0xe0);

    
    cssl_putchar(serial,0x00);
}
