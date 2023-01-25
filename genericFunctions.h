#include <DW1000.h>
#include "ProtocolConsts.h"

void setCurrentConfigs()
{
    DW1000.setDefaults();
}

void any_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
	int i;
	for (i = 0; i < ANY_MSG_TS_LEN; i++) {
		ts_field[i] = (uint8_t) ts;
		ts >>= 8;
	}
}

void any_msg_get_ts(const uint8_t *ts_field, uint64_t *ts) {
	int i;
	*ts = 0;
	for (i = 0; i < ANY_MSG_TS_LEN; i++) {
		*ts += ((uint64_t)ts_field[i]) << (i * 8);
	}
}

void short_msg_set(uint8_t *ts_field, uint16_t ts) {
	int i;
	for (i = 0; i < SHORT_MSG_LEN; i++) {
		ts_field[i] = (uint8_t) ts;
		ts >>= 8;
	}
}

void short_msg_get(const uint8_t *ts_field, uint16_t *ts) {
	int i;
	*ts = 0;
	for (i = 0; i < SHORT_MSG_LEN; i++) {
		*ts += ((uint64_t)ts_field[i]) << (i * 8);
	}
}

double get_time_us(){
    DW1000Time currentDWTime;
    DW1000.getSystemTimestamp(currentDWTime);
    uint64_t tmpTime_u64 = currentDWTime.getTimestamp();
    return tmpTime_u64 * TIME_UNIT * 1e6;
}

double get_time_ms(){
    DW1000Time currentDWTime;
    DW1000.getSystemTimestamp(currentDWTime);
    uint64_t tmpTime_u64 = currentDWTime.getTimestamp();
    return tmpTime_u64 * TIME_UNIT * 1e3;
}

uint16_t get_time_ms_uint16(){
    DW1000Time currentDWTime;
    DW1000.getSystemTimestamp(currentDWTime);
    uint64_t tmpTime_u64 = currentDWTime.getTimestamp();
    return (uint16_t)(tmpTime_u64 * TIME_UNIT * 1e3);
}

uint64_t get_time_u64(){
    DW1000Time currentDWTime;
    DW1000.getSystemTimestamp(currentDWTime);
    return currentDWTime.getTimestamp();
}

double get_elapsed_time_us(double startTime, double currentTime){
    return (startTime < currentTime) ? currentTime - startTime : currentTime + 17207401.0256 - startTime;
}

double get_elapsed_time_ms(double startTime, double currentTime){
    return (startTime < currentTime) ? currentTime - startTime : currentTime + 17207.4010256 - startTime;
}

uint16_t getNonnegRxTimeout(double timeout){
	if(timeout > TYPICAL_TIMEOUT){
		return TYPICAL_TIMEOUT;
	}else if(timeout > 0 && timeout <= TYPICAL_TIMEOUT){
		return (uint16_t)(timeout);
	}else{
		return 1;
	}
}

void print_uint64(uint64_t a){
	int r;
	int out[20]; int j = 0;
	while(a > 0){
		r = a % 10;
		a = (a - r) / 10;
		out[j++] = r;
	}
	for(int i = j - 1; i>= 0; i--){
		Serial.print(out[i]);
	}
}

void generic_send(uint8_t *buffer_to_send, int buffer_size) {
    
    
    uint32_t txDelay;
    
    DW1000.setData(buffer_to_send, buffer_size);
    
    DW1000.startTransmit();
}

void embedDistance(uint8_t *msg, uint16_t *distVec, int idx, int myid){
	int j = idx;
	for(int i = 1; i <= TRX_NUM; i++){
		if( i!= myid){
			memcpy(&msg[j], &distVec[i], DIST_LEN);
			j += DIST_LEN;
		}
	}
}

void embedImus(uint8_t* msg, uint8_t* imuBuffer, int idx, int imu_buf_len){
	int j = idx;
	memcpy(&msg[j], imuBuffer, imu_buf_len);
}

void embedPowers(uint8_t* msg, uint8_t* powerBuffer, int idx, int myid){
	int j = idx;
	for(int i = 1; i <= TRX_NUM; i++){
		if( i!= myid){
			msg[j] = powerBuffer[i];
			j += 1;
		}
	}
}

void uwb_delay(int milliseconds){
	double start = get_time_ms();
	double current = get_time_ms();
	Serial.print("start time: ");
	Serial.print(start);
	Serial.print("current time: ");
	Serial.print(current);
	Serial.println();

	while (current - start < milliseconds){
		current =  get_time_ms();
		if (current < start) {
			current += 172074.01;
		}
		Serial.print("current time: ");
		Serial.print(current);
		Serial.print("current - start: ");
		Serial.print(current - start);
		Serial.println();
	}
}