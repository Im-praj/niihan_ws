#ifndef NIIHAN_CORE_H
#define NIIHAN_CORE_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef bool (*niihan_can_send)(uint16_t id,const uint8_t data[8]);
typedef void (*niihan_inhibit)(bool inhibit);
typedef struct {
  uint8_t node_id; int left_sign,right_sign; int32_t max_rpm;
  niihan_can_send send; niihan_inhibit inhibit;
  uint32_t now,last_command,last_response,request_time,last_seq;
  bool rearm_required,pending_advances,seen_command,pending,estop,fault,enabled,requested_enable;
  float left_rad_s,right_rad_s;
  int32_t left_ticks,right_ticks; uint16_t statusword;
  uint16_t pending_index; uint8_t pending_sub,pending_command;
  int phase,poll; uint32_t telemetry_seq;
} niihan_core;
void niihan_init(niihan_core *s,uint8_t node,int left_sign,int right_sign,niihan_can_send tx,niihan_inhibit inhibit);
bool niihan_command(niihan_core *s,const char *line,size_t length,uint32_t now);
void niihan_tick(niihan_core *s,uint32_t now,bool estop);
void niihan_can_receive(niihan_core *s,uint16_t id,const uint8_t data[8],uint32_t now);
size_t niihan_telemetry(niihan_core *s,char *output,size_t capacity,uint32_t now);
uint16_t niihan_crc16(const uint8_t *data,size_t size);
void niihan_sdo(uint8_t out[8],uint8_t command,uint16_t index,uint8_t sub,int32_t value);
#endif
