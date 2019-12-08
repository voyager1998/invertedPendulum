/*******************************************************************************
* optitrack_client.c
*
* This program will get ground truth data from optitrack
* over XBee and publish it on an LCM Channel
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <lcm/lcm.h>
#include "../common/mb_defs.h"
#include "../lcmtypes/pose_xyt_t.h"
#include "../optitrack/common/serial.h"

FILE* f1;
lcm_t * lcm = NULL;
const int baudRate = 57600;
const char port[] = "/dev/ttyO5";
const char headByte = 0x1B;
const char tailByte = 0xFF;
int num_gates = 4;
int fd;
uint64_t last_utime = 0;
int bytes_avail = 0;
int err_counter = 0;

void getData(pose_xyt_t *Pose);
void printData(pose_xyt_t Pose);

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
    lcm = lcm_create(LCM_ADDRESS);
    //open serial port non-blocking
    fd = serial_open(port,baudRate,0);

    if(fd == -1){
        printf("Failed to open Serial Port: %s", port);
        return -1;
    }

    //construct message for storage
    pose_xyt_t Pose;

    int packetLength = pose_xyt_t_encoded_size(&Pose)+2;
    printf("packetLength: %d\n", packetLength);
    
    // Print Header
    printf("\n");
    printf("   Time   |");
    printf("Buf|");
    printf("Err|");
    printf("  Rate  |");
    printf("    X    |");
    printf("    Y    |");
    printf("    θ    |");
    printf("\n");

    while(1)
    {
        //check bytes in serial buffer
        ioctl(fd, FIONREAD, &bytes_avail);
        //printf("bytes: %d\n",bytes_avail);
        if(bytes_avail >= packetLength){
            getData(&Pose);
            printData(Pose);
        }
        usleep(100);
    }
    serial_close(fd);
	return 0;
}

void getData(pose_xyt_t* Pose){
    char *ptr;
    int packetLength = pose_xyt_t_encoded_size(Pose)+2;
    char *dataPacket = (char*) malloc (packetLength);
    ptr = dataPacket;
    while(read(fd, ptr, 1) > 0){
        // if the first Byte is wrong keep looking
        if((ptr == dataPacket)&&(*ptr != headByte)){
            continue;
        }
        ptr++;
        // Once we have all of the Bytes check to make sure first and last are good
        if((ptr-dataPacket) == packetLength){
            if((dataPacket[0] != headByte) || (dataPacket[packetLength-1] != tailByte)){
                err_counter += 1;
            } 
            else{
                //packet is good, decode it into Pose
                int status = pose_xyt_t_decode(dataPacket, 1, packetLength-2, Pose);
                if (status < 0) {
                    fprintf (stderr, "error %d decoding balancebot_msg_t!!!\n", status);;
                }
                // if we have less than a full message in the serial buffer
                // we are done, we'll get the next one next time
                ioctl(fd, FIONREAD, &bytes_avail);
                if(bytes_avail < packetLength){
                    break;
                }
            }
            //keep reading until buffer is almost empty
            ptr = dataPacket;
        }
    }
}

void printData(pose_xyt_t Pose){
    pose_xyt_t_publish(lcm, OPTITRACK_CHANNEL ,&Pose);
    if      (Pose.utime < 1000000)   printf("    %"PRId64"|",Pose.utime);
    else if (Pose.utime < 10000000)  printf("   %"PRId64"|",Pose.utime);
    else if (Pose.utime < 100000000) printf("  %"PRId64"|",Pose.utime);
    else if (Pose.utime < 1000000000)printf(" %"PRId64"|",Pose.utime);
    else                              printf("%"PRId64"|",Pose.utime);   

    if (bytes_avail < 10)       printf("  %d|",bytes_avail);
    else if (bytes_avail < 100) printf(" %d|",bytes_avail);
    else                        printf("%d|",bytes_avail);

    if (err_counter < 10)       printf("  %d|",err_counter);
    else if (err_counter < 100) printf(" %d|",err_counter);
    else                        printf("%d|",err_counter);

    printf("  %3.2f |", 1.0E6/(Pose.utime - last_utime));
    printf("%+7.6f|"  ,Pose.x);
    printf("%+7.6f|"  ,Pose.y);
    printf("%+7.6f|"  ,Pose.theta);
    printf("         \r");
    fflush(stdout);

    last_utime = Pose.utime;
}