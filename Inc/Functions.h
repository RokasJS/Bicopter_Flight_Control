// Type definitions
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned long long uint64_t;
typedef signed long long int64_t;



void NSS_RX(uint8_t Number);
void RFM96WRX_Config(void);
void reciever_settings(uint8_t adress, uint8_t data, uint8_t size);
void RFM96WRX_recieve(uint8_t *data);
