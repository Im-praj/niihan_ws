#include "niihan_core.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t sent[8];static bool inhibited=true;
static bool send(uint16_t id,const uint8_t data[8]){assert(id==0x601);memcpy(sent,data,8);return true;}
static void inhibit(bool state){inhibited=state;}
int main(void){
 niihan_core s;niihan_init(&s,1,1,1,send,inhibit);assert(inhibited);
 s.fault=false;s.rearm_required=false;
 uint8_t data[8];niihan_sdo(data,0x23,0x60ff,1,-100);uint8_t expected[]={0x23,0xff,0x60,1,0x9c,0xff,0xff,0xff};assert(memcmp(data,expected,8)==0);
 const char *payload="N1,1,1.000000,-1.000000,1";char frame[100];int n=snprintf(frame,sizeof(frame),"%s*%04X\n",payload,niihan_crc16((const uint8_t*)payload,strlen(payload)));
 assert(niihan_command(&s,frame,(size_t)n,100));assert(!niihan_command(&s,frame,(size_t)n,101));frame[4]='2';assert(!niihan_command(&s,frame,(size_t)n,102));
 s.enabled=true;s.fault=false;s.last_response=100;s.phase=12;
 niihan_tick(&s,400,false);assert(inhibited);assert(!s.enabled);assert(sent[0]==0x40||sent[1]==0x40);
 s.enabled=true;s.last_response=401;s.last_command=401;niihan_tick(&s,402,true);assert(inhibited);assert(!s.enabled);
 char output[200];assert(niihan_telemetry(&s,output,sizeof(output),402)>0);assert(strncmp(output,"T1,",3)==0);
 /* A CAN abort latches a re-arm requirement and asserts inhibit. */
 niihan_core failed;niihan_init(&failed,1,1,1,send,inhibit);
 niihan_tick(&failed,10,false);assert(failed.pending&&failed.pending_index==0x6041);
 uint8_t abort_frame[8]={0x80,0x41,0x60,0,0,0,0,0};
 niihan_can_receive(&failed,0x581,abort_frame,11);
 assert(failed.fault&&failed.rearm_required&&inhibited&&!failed.pending);
 /* Missing acknowledgement times out; INT_MIN signed encoder reversal wraps safely. */
 niihan_tick(&failed,12,false);niihan_tick(&failed,113,false);
 assert(failed.fault&&failed.rearm_required&&inhibited);
 failed.right_sign=-1;failed.pending=true;failed.pending_index=0x6064;
 failed.pending_sub=2;failed.pending_command=0x40;failed.pending_advances=false;
 uint8_t encoder[8]={0x43,0x64,0x60,2,0,0,0,0x80};
 niihan_can_receive(&failed,0x581,encoder,114);assert(failed.right_ticks==INT32_MIN);
 puts("MCU core packet, signed CANopen target, timeout and E-stop tests passed");return 0;
}
