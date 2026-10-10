/* ZLAC8015D V4, CANopen routine 1.02 (2026-04-21), pp.19, 42-45.
 * Portable logic only. Physical E-stop and power/brake safety remain independent. */
#include "niihan_core.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
uint16_t niihan_crc16(const uint8_t *data,size_t size){
 uint16_t c=0xffff;for(size_t i=0;i<size;i++){c^=(uint16_t)data[i]<<8;for(int j=0;j<8;j++)c=(uint16_t)((c&0x8000)?(c<<1)^0x1021:c<<1);}return c;
}
void niihan_sdo(uint8_t out[8],uint8_t command,uint16_t index,uint8_t sub,int32_t value){
 out[0]=command;out[1]=(uint8_t)index;out[2]=(uint8_t)(index>>8);out[3]=sub;
 uint32_t v=(uint32_t)value;for(int i=0;i<4;i++)out[4+i]=(uint8_t)(v>>(8*i));
}
void niihan_init(niihan_core *s,uint8_t node,int left_sign,int right_sign,niihan_can_send tx,niihan_inhibit inhibit){
 memset(s,0,sizeof(*s));s->node_id=node;s->left_sign=left_sign;s->right_sign=right_sign;s->max_rpm=30;s->send=tx;s->inhibit=inhibit;
 s->fault=true;s->rearm_required=true;s->inhibit(true);
}
bool niihan_command(niihan_core *s,const char *line,size_t length,uint32_t now){
 if(length<10||length>255)return false;
 char text[256];memcpy(text,line,length);text[length]=0;char *star=strchr(text,'*');if(!star)return false;
 char *end;unsigned long checksum=strtoul(star+1,&end,16);
 if(end!=star+5||(*end!='\n'&&*end!='\r'&&*end!=0)||checksum>0xffff)return false;
 if(niihan_crc16((const uint8_t*)text,(size_t)(star-text))!=(uint16_t)checksum)return false;
 *star=0;unsigned long seq;float left,right;int enabled;int consumed=0;
 if(sscanf(text,"N1,%lu,%f,%f,%d%n",&seq,&left,&right,&enabled,&consumed)!=4||text[consumed]!=0)return false;
 if(seq>0xffffffffUL||!isfinite(left)||!isfinite(right)||(enabled!=0&&enabled!=1))return false;
 uint32_t delta=(uint32_t)seq-s->last_seq;
 if(s->seen_command&&(delta==0||delta>=0x80000000U))return false;
 if(enabled&&s->rearm_required)return false;
 if(!enabled&&!s->estop&&!s->fault)s->rearm_required=false;
 s->last_seq=(uint32_t)seq;s->seen_command=true;s->last_command=now;s->left_rad_s=left;s->right_rad_s=right;
 s->requested_enable=enabled!=0;
 return true;
}
static bool request(niihan_core *s,uint8_t command,uint16_t index,uint8_t sub,int32_t value){
 uint8_t data[8];niihan_sdo(data,command,index,sub,value);
 if(!s->send((uint16_t)(0x600+s->node_id),data)){s->fault=true;s->rearm_required=true;s->inhibit(true);return false;}
 s->pending=true;s->request_time=s->now;s->pending_index=index;s->pending_sub=sub;s->pending_command=command;return true;
}
static int32_t rpm(niihan_core *s,float value,int sign){
 float v=value*9.5492965855f*sign;
 if(v>s->max_rpm)v=(float)s->max_rpm;
 if(v< -s->max_rpm)v=(float)-s->max_rpm;
 return (int32_t)lroundf(v);
}
void niihan_tick(niihan_core *s,uint32_t now,bool estop){
 s->now=now;s->estop=estop;
 if(estop)s->rearm_required=true;
 bool fresh=s->seen_command&&(uint32_t)(now-s->last_command)<=250;
 bool feedback=(uint32_t)(now-s->last_response)<=500&&s->last_response!=0;
 bool allowed=fresh&&feedback&&!estop&&!s->fault&&!s->rearm_required&&s->requested_enable;
 if(!allowed){
  s->inhibit(true);
  if(s->enabled||s->phase>=10){uint8_t stop[8];niihan_sdo(stop,0x2b,0x6040,0,6);s->send((uint16_t)(0x600+s->node_id),stop);s->enabled=false;s->phase=0;s->pending=false;}
 }
 if(s->pending){if((uint32_t)(now-s->request_time)>100){s->pending=false;s->fault=true;s->rearm_required=true;s->phase=0;s->inhibit(true);}return;}
 if(s->node_id<1||s->node_id>127||abs(s->left_sign)!=1||abs(s->right_sign)!=1)return;
 /* Read status and both encoder counters before enabling. No persistent EEPROM writes. */
 s->pending_advances=allowed&&s->phase<=11;
 if(!allowed){
  switch(s->poll++%3){case 0:request(s,0x40,0x6041,0,0);break;case 1:request(s,0x40,0x6064,1,0);break;default:request(s,0x40,0x6064,2,0);break;}return;
 }
 switch(s->phase){
 case 0:request(s,0x2b,0x200f,0,0);break;
 case 1:request(s,0x2f,0x6060,0,3);break;
 case 2:request(s,0x23,0x60ff,1,0);break;
 case 3:request(s,0x23,0x60ff,2,0);break;
 case 4:request(s,0x23,0x6083,1,500);break;
 case 5:request(s,0x23,0x6083,2,500);break;
 case 6:request(s,0x23,0x6084,1,500);break;
 case 7:request(s,0x23,0x6084,2,500);break;
 case 8:request(s,0x2b,0x6040,0,6);break;
 case 9:request(s,0x2b,0x6040,0,7);break;
 case 10:request(s,0x2b,0x6040,0,15);break;
 case 11:request(s,0x40,0x6041,0,0);break;
 default:
  if((s->statusword&0x6f)!=0x27){s->fault=true;s->rearm_required=true;s->inhibit(true);return;}
  s->enabled=true;s->inhibit(false);
  switch(s->poll++%5){
   case 0:request(s,0x23,0x60ff,1,rpm(s,s->left_rad_s,s->left_sign));break;
   case 1:request(s,0x23,0x60ff,2,rpm(s,s->right_rad_s,s->right_sign));break;
   case 2:request(s,0x40,0x6064,1,0);break;
   case 3:request(s,0x40,0x6064,2,0);break;
   default:request(s,0x40,0x6041,0,0);break;
  }
 }
}
void niihan_can_receive(niihan_core *s,uint16_t id,const uint8_t data[8],uint32_t now){
 if(id!=(uint16_t)(0x580+s->node_id)||!s->pending)return;
 uint16_t index=(uint16_t)(data[1]|data[2]<<8);
 if(index!=s->pending_index||data[3]!=s->pending_sub)return;
 if(data[0]==0x80){s->pending=false;s->fault=true;s->rearm_required=true;s->phase=0;s->inhibit(true);return;}
 bool read=s->pending_command==0x40;
 if((read&&data[0]!=0x43&&data[0]!=0x4b)||(!read&&data[0]!=0x60))return;
 s->pending=false;s->last_response=now;
 uint32_t value=(uint32_t)data[4]|(uint32_t)data[5]<<8|(uint32_t)data[6]<<16|(uint32_t)data[7]<<24;
 if(read){
  if(index==0x6041){s->statusword=(uint16_t)value;s->fault=(value&8)!=0||(value&0x8000)!=0;if(s->fault){s->rearm_required=true;s->inhibit(true);}}
  if(index==0x6064&&data[3]==1)s->left_ticks=(int32_t)(s->left_sign<0?0U-value:value);
  if(index==0x6064&&data[3]==2)s->right_ticks=(int32_t)(s->right_sign<0?0U-value:value);
 }
 if(s->pending_advances&&!s->estop&&!s->fault)s->phase++;
}
size_t niihan_telemetry(niihan_core *s,char *out,size_t capacity,uint32_t now){
 char payload[180];int n=snprintf(payload,sizeof(payload),"T1,%lu,%lu,%ld,%ld,%d,%d",(unsigned long)s->telemetry_seq++,(unsigned long)now,(long)s->left_ticks,(long)s->right_ticks,s->estop?1:0,(s->fault||(uint32_t)(now-s->last_response)>500)?1:0);
 if(n<0||(size_t)n>=sizeof(payload))return 0;
 int total=snprintf(out,capacity,"%s*%04X\n",payload,niihan_crc16((const uint8_t*)payload,(size_t)n));
 return total>0&&(size_t)total<capacity?(size_t)total:0;
}
