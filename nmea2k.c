#include <string.h>
#include <nmea2k.h>
#include <errno.h>

uint8_t _N2kMaxCanBusAddress = 251;
uint32_t _N2kPGNIsoAddressClaim = 60928L;

struct {
    uint8_t source_id;
}nmea2k_t;

int nmea_init(uint8_t source){
    if (source >= _N2kMaxCanBusAddress){
        return EINVAL;
    }
    nmea2k_t.source_id = source;
    return 0;
}


void SetN2kPGN127488(tN2kMsg_t *N2kMsg, uint8_t EngineInstance, uint16_t rpm, uint32_t boost, int8_t trim) {
    N2kMsg->PGN =127488L;

    N2kMsg->Data[0] = EngineInstance;

    N2kMsg->Data[1] = rpm;
    N2kMsg->Data[2] = rpm >> 8;

    boost = boost;
    N2kMsg->Data[3] = boost;
    N2kMsg->Data[4] = boost >> 8;

    N2kMsg->Data[5] = trim;

    N2kMsg->Data[6] = 0xff;
    N2kMsg->Data[7] = 0xff;

    N2kMsg->Priority=2;
}

int ParseN2kPGN127488(tN2kMsg_t *N2kMsg, uint8_t *EngineInstance, uint16_t *rpm, uint32_t *boost, int8_t *trim) {
    if (N2kMsg->PGN != 127488L){
        return EINVAL;
    }

    *EngineInstance=N2kMsg->Data[0];
    *rpm = (N2kMsg->Data[1] | (uint16_t) N2kMsg->Data[2] << 8);
    *boost = (N2kMsg->Data[3] | (uint16_t) N2kMsg->Data[4] << 8);

    *trim = N2kMsg->Data[5];

    return 0;
}


void SetN2kPGN128259(tN2kMsg_t *N2kMsg, uint8_t SID, uint16_t WaterRef, uint16_t GroundRef, tN2kSpeedWaterReferenceType SWRT) {
    N2kMsg->PGN =128259L;

    N2kMsg->Data[0] = SID;

     
    N2kMsg->Data[1] = WaterRef; // 0.01f
    N2kMsg->Data[2] = WaterRef >> 8;

    N2kMsg->Data[3] = GroundRef; // 0.01f
    N2kMsg->Data[4] = GroundRef >> 8;

    N2kMsg->Data[5] = SWRT; // 0.01f

    N2kMsg->Data[6] = 0xff;
    N2kMsg->Data[7] = 0xff;

    N2kMsg->Priority=2;
}

int ParseN2kPGN128259(tN2kMsg_t *N2kMsg, uint8_t *SID, uint16_t * WaterRef, uint16_t *GroundRef, tN2kSpeedWaterReferenceType *SWRT) {
    if (N2kMsg->PGN != 128259L){
        return EINVAL;
    }
    *SID = N2kMsg->Data[0];

    *WaterRef = (N2kMsg->Data[1] | (uint16_t) N2kMsg->Data[2] << 8);
    *GroundRef = (N2kMsg->Data[3] | (uint16_t) N2kMsg->Data[4] << 8);

    *SWRT = N2kMsg->Data[5] & 0x0F;

    return 0;
}

void SetN2kPGN127508(tN2kMsg_t *N2kMsg, uint8_t BatInst, uint16_t BatVolt, uint16_t BatCur, uint16_t BatTemp, uint8_t SID) {
    N2kMsg->PGN =127508L;

    N2kMsg->Data[0] = BatInst;

    N2kMsg->Data[1] = BatVolt;
    N2kMsg->Data[2] = BatVolt >> 8;

    N2kMsg->Data[3] = BatCur;
    N2kMsg->Data[4] = BatCur >> 8;

    N2kMsg->Data[5] = BatTemp;
    N2kMsg->Data[6] = BatTemp >> 8;

    N2kMsg->Data[7] = SID;

    N2kMsg->Priority=6;    
}


void SetN2kPGN127501(tN2kMsg_t *N2kMsg, uint32_t bank, tN2kOnOff *sw, const uint8_t sw_num) {
    uint64_t BankStatus = 0xffffffffffffffff;
    for (uint8_t i=0; i<sw_num; i++){
        BankStatus = (BankStatus << 2) | sw[i];
    }
    BankStatus = (BankStatus << 8) | bank;
    memcpy(N2kMsg->Data, &BankStatus, sizeof(BankStatus));
    N2kMsg->PGN = 127501L;
    N2kMsg->Priority=3;
}

void SetN2kPGN127502(tN2kMsg_t *N2kMsg, uint32_t bank, tN2kOnOff *sw, const uint8_t sw_num) {
    uint64_t BankStatus = 0xffffffffffffffff;
    for (uint8_t i=0; i<sw_num; i++){
        BankStatus = (BankStatus << 2) | sw[i];
    }
    BankStatus = (BankStatus << 8) | bank;
    memcpy(N2kMsg->Data, &BankStatus, sizeof(BankStatus));
    N2kMsg->PGN = 127502L;
    N2kMsg->Priority=3;
}

void SetN2kPGN127505(tN2kMsg_t *N2kMsg, uint8_t Instance, tN2kFluidType FluidType, uint16_t Level, uint32_t Capacity) {
    N2kMsg->PGN = 127505L;

    N2kMsg->Priority=6;

    N2kMsg->Data[0] = (Instance & 0x0f) | ((FluidType & 0x0f)<<4);

    //0.004
    N2kMsg->Data[1] = Level;
    N2kMsg->Data[2] = Level >> 8;

    //0.1
    N2kMsg->Data[3] = Capacity;
    N2kMsg->Data[4] = Capacity >> 8;
    N2kMsg->Data[5] = Capacity >> 16;
    N2kMsg->Data[6] = Capacity >> 24;

    N2kMsg->Data[7] = 0xff;
}

int ParseN2kPGN127505(tN2kMsg_t *N2kMsg, uint8_t *Instance, tN2kFluidType *FluidType, uint16_t *Level, uint32_t *Capacity) {
    if (N2kMsg->PGN != 127505L){
        return EINVAL;
    }

    uint8_t IFt=N2kMsg->Data[0];

    *Instance=IFt&0x0f;
    *FluidType=(tN2kFluidType)((IFt>>4)&0x0f);
    *Level = (N2kMsg->Data[1] | (uint16_t) N2kMsg->Data[2] << 8);
    *Capacity = (N2kMsg->Data[3] | ((uint32_t) N2kMsg->Data[4] << 8) 
                | ((uint32_t) N2kMsg->Data[5] << 16) | ((uint32_t) N2kMsg->Data[6] << 24));

    return 0;
}

int ParseN2kPGN127508(tN2kMsg_t *N2kMsg, uint8_t* BatInst, uint16_t *BatVolt, uint16_t *BatCur, uint16_t *BatTemp, uint8_t *SID) {
    if (N2kMsg->PGN != 127508L){
        return EINVAL;
    }
    *BatInst = N2kMsg->Data[0];

    *BatVolt = (N2kMsg->Data[1] | (uint16_t) N2kMsg->Data[2] << 8);
    *BatCur = (N2kMsg->Data[3] | (uint16_t) N2kMsg->Data[4] << 8);
    *BatTemp = (N2kMsg->Data[5] | (uint16_t) N2kMsg->Data[6] << 8);
    *SID = N2kMsg->Data[7];

    return 0;
}

void SetN2kPGN127751(tN2kMsg_t *N2kMsg, uint8_t Instance, int16_t Voltage, int32_t Current, uint8_t SID) {
    N2kMsg->PGN = 127751L;
    N2kMsg->Data[0] = SID;
    N2kMsg->Data[1] = Instance;

    N2kMsg->Data[2] = Voltage;
    N2kMsg->Data[3] = Voltage >> 8;

    N2kMsg->Data[4] = Current;
    N2kMsg->Data[5] = Current >> 8;
    N2kMsg->Data[6] = Current >> 16;

    N2kMsg->Data[7] = 0xff;
}

int ParseN2kPGN127501(tN2kMsg_t *N2kMsg, tN2kOnOff *sw, uint8_t *bank) {
    if (N2kMsg->PGN != 127501L){
        return EINVAL;
    }
    uint8_t sw_num = 0;
    *bank = N2kMsg->Data[0];
    for (uint8_t i=1; i<8; i++){
        sw[sw_num++] = (N2kMsg->Data[i] >> 0) & 0x3;
        sw[sw_num++] = (N2kMsg->Data[i] >> 2) & 0x3;
        sw[sw_num++] = (N2kMsg->Data[i] >> 4) & 0x3;
        sw[sw_num++] = (N2kMsg->Data[i] >> 6) & 0x3;
    }

    return 0;
}

int ParseN2kPGN127502(tN2kMsg_t *N2kMsg, tN2kOnOff *sw, uint8_t *bank) {
    if (N2kMsg->PGN != 127502L){
        return EINVAL;
    }
    uint8_t sw_num = 0;
    *bank = N2kMsg->Data[0];
    for (uint8_t i=7; i>=1; i--){
        sw[sw_num++] = (N2kMsg->Data[i] >> 6) & 0x3;
        sw[sw_num++] = (N2kMsg->Data[i] >> 4) & 0x3;
        sw[sw_num++] = (N2kMsg->Data[i] >> 2) & 0x3;
        sw[sw_num++] = (N2kMsg->Data[i] >> 0) & 0x3;
    }

    return 0;
}

int ParseN2kPGN127751(tN2kMsg_t *N2kMsg, uint8_t* instance, int16_t *voltage, int32_t *current, uint8_t *SID) {
    if (N2kMsg->PGN != 127751L){
        return EINVAL;
    }

    *SID = N2kMsg->Data[0];
    *instance = N2kMsg->Data[1];
    *voltage = (N2kMsg->Data[2] | (uint16_t) N2kMsg->Data[3] << 8);
    *current = (N2kMsg->Data[4] | ((uint32_t) N2kMsg->Data[5] << 8) | ((uint32_t) N2kMsg->Data[6] << 16));

    return 0;
}

int packN2k(tN2kMsg_t *N2kMsg, can_fifo_t *fifo){
    if (N2kMsg->PGN & 0xff){
        N2kMsg->Destination = 0xff;
    }
    N2kMsg->Source = nmea2k_t.source_id;
    // CAN bus address range is 0-251. Anyway allow ISO address claim mgs.
    if (N2kMsg->Source > _N2kMaxCanBusAddress && N2kMsg->PGN != _N2kPGNIsoAddressClaim) {
        return EFAULT;
    }

    uint32_t id = ( ((unsigned long)(N2kMsg->Priority & 0x7))<<26 | N2kMsg->PGN<<8 | (unsigned long)N2kMsg->Source);
    if ((uint8_t)(N2kMsg->PGN >> 8) < 240) {  // PDU1 format
        if (N2kMsg->PGN & 0xff) {
            return EFAULT;  // for PDU1 format PGN lowest byte has to be 0 for the destination.
        }
        id = ( ((unsigned long)(N2kMsg->Priority & 0x7))<<26 | N2kMsg->PGN<<8 | ((unsigned long)N2kMsg->Destination)<<8 | (unsigned long)N2kMsg->Source);
    }

    fifo->flg.dlc = 8;
    fifo->flg.rtr = 0;
    fifo->flg.ide = 1;
    memcpy(fifo->data8, N2kMsg->Data, 8);
    fifo->id = id;
    return 0;
}

void CanIdToN2k(uint32_t id, tN2kMsg_t *msg) {
    uint8_t CanIdPF = (uint8_t ) (id >> 16);
    uint8_t CanIdPS = (uint8_t) (id >> 8);
    uint8_t CanIdDP = (uint8_t) (id >> 24) & 1;

    msg->Source = (uint8_t) id >> 0;
    msg->Priority = (uint8_t) ((id >> 26) & 0x7);

    if (CanIdPF < 240) {
      /* PDU1 format, the PS contains the destination address */
        msg->Destination= CanIdPS;
        msg->PGN = (((unsigned long)CanIdDP) << 16) | (((unsigned long)CanIdPF) << 8);
    } else {
      /* PDU2 format, the destination is implied global and the PGN is extended */
        msg->Destination = 0xff;
        msg->PGN = (((unsigned long)CanIdDP) << 16) | (((unsigned long)CanIdPF) << 8) | (unsigned long)CanIdPS;
    }
}