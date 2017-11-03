#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <lirc/include/media/lirc.h>
#include <unistd.h>
#include <error.h>

#define BURST_LENGTH 360
#define MARK_LENGTH 1800
#define SPACE_LENGTH 720
#define START_BURST 5000
#define START_SPACE 2100
#define PACKET_SPACE 29400
#define FIRST_PACKET_LEN (56*2)
#define SECOND_PACKET_LEN (104*2)
#define START_BIT_LEN 2
#define END_BIT_LEN 1
#define PACKET_SPACE_LEN 1
#define DAIK_PACKET_SIZE (START_BIT_LEN+FIRST_PACKET_LEN+END_BIT_LEN+PACKET_SPACE_LEN+START_BIT_LEN+SECOND_PACKET_LEN+END_BIT_LEN)
//      <delimiter name="startbit">#5000 _2125</delimiter>
//      <data name="unk" length="40" bitorder="LSB" />
//      <data name="ON/OFF" length="1" bitorder="LSB" />
//      <data name="unk1" length="3" bitorder="LSB" />
//      <data name="Mode" length="3" bitorder="LSB" />
//      <data name="unk2" length="1" bitorder="LSB" />
//      <data name="swing" length="1" bitorder="LSB" />
//      <data name="unk3" length="2" bitorder="LSB" />
//      <data name="POWERFUL" length="1" bitorder="LSB" />
//      <data name="unk4" length="21" bitorder="LSB" />
//      <data name="temp" length="5" bitorder="LSB" />
//      <data name="unk5" length="2" bitorder="LSB" />
//      <data name="fanspeed" length="3" bitorder="LSB" />
//      <data name="fanauto" length="1" bitorder="LSB" />
//      <data name="unk6" length="12" bitorder="LSB" />
//      <data name="checksum" length="8" bitorder="LSB" />
//      <delimiter name="stopbit">#350</delimiter>

typedef union {
    struct  __attribute__((packed)){
        unsigned unknown1 : 32;
        unsigned unknownb : 8;
        unsigned on_off : 1;
        unsigned unknown2 : 3;
        unsigned mode : 3;
        unsigned unknown3 : 1;
        unsigned swing : 1;
        unsigned unknown4 : 2;
        unsigned powerful : 1;
        unsigned unknown5 : 21;
        unsigned temperature : 5;
        unsigned unknown6 : 2;
        unsigned fanspeed : 3;
        unsigned fanauto : 1;
        unsigned unknown7 : 12;
        unsigned checksum : 8;
    }bf;
    unsigned char packet[13];
}daikin_command_t;


int charbuf2array(const unsigned char *src, size_t srclen, int *dest, int dest_from, int dest_max) {
    int i, j;
    int destindex = dest_from;
    unsigned char c;
    for (i = 0; i < srclen && destindex < dest_max; i++) {
        c = src[i];
        for (j = 0; j < 8 && destindex < dest_max; j++) {
            dest[destindex++] = BURST_LENGTH;
            if (c & 0x01) {
                dest[destindex++] = MARK_LENGTH;
            } else {
                dest[destindex++] = SPACE_LENGTH;
            }
            c >>= 1;
        }
    }
    printf("destindex %d \r\n", destindex);
    return destindex;

}

int main() {
    int fd;
    int freq = 33333;
    unsigned char first_packet[7] = {17, 218, 39, 240, 13, 0, 15};
    unsigned char second_packet[13] = {17, 218, 39, 0, 211, 65, 0, 0, 0, 28, 3, 8, 77};
    int packet_buffer[DAIK_PACKET_SIZE];
    int packet_index = 0;
    daikin_command_t cmd;
    cmd.bf.on_off=1;
    cmd.bf.fanspeed=3;
    cmd.bf.temperature=10;
    cmd.bf.swing=1;
    
    packet_buffer[packet_index++] = START_BURST;
    packet_buffer[packet_index++] = START_SPACE;
    packet_index = charbuf2array(first_packet, sizeof (first_packet), packet_buffer, packet_index, DAIK_PACKET_SIZE);
    packet_buffer[packet_index++] = BURST_LENGTH; //stop bit
    packet_buffer[packet_index++] = PACKET_SPACE; //stop bit
    packet_buffer[packet_index++] = START_BURST;
    packet_buffer[packet_index++] = START_SPACE;
    packet_index = charbuf2array(second_packet, sizeof (second_packet), packet_buffer, packet_index, DAIK_PACKET_SIZE);
    packet_buffer[packet_index++] = BURST_LENGTH; //stop bit

    printf("Total packet size %d %d\r\n", sizeof (daikin_command_t), packet_index);
    fd = open("/dev/lirc0", O_RDWR);
    if (ioctl(fd, LIRC_SET_SEND_CARRIER, &freq) == -1)
        printf("Set frequency failed\rºn");
    else {
        printf("Command sent\r\n");
        printf("wrote %d %d\r\n", packet_index, write(fd, packet_buffer, sizeof (int)*packet_index));

    }
    printf("%d\r\n", errno);
    close(fd);
}

