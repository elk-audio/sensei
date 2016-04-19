#ifndef MANAGEDATAPACKET_H
#define MANAGEDATAPACKET_H

#include "common.h"

typedef union uDataPacket
{
	sSenseiDataPacket data;
	uint8_t vectorData[SENSEI_LENGTH_DATA_PACKET];
} uDataPacket;

class ManageDataPacket {
public:
	ManageDataPacket();
	~ManageDataPacket();
	void receivePacket();
	void preparePacket(uint8_t cmd, uint8_t sub_cmd, uint16_t nPacketsMissing, uint8_t *pPayload, uint16_t nBytePayload);
	void prepareACK(int32_t status, uint32_t timestamp, uint8_t cmd, uint8_t sub_cmd);
	void getPacketID(uint8_t& cmd, uint8_t& sub_cmd, uint32_t& timestamp);
	uint8_t getCmd();
	uint8_t getSubCmd();
	uint32_t getTimestamp();
	int32_t checkPacket();
    uint32_t getNpacketReceived();
	void setPayloadToVariable(void* dataOutput, uint16_t nByte);
	void send();

private:
	void clearDataPacket();
	uint16_t calcCRC();
	bool checkCRC();
	uint32_t _nPacketReceived;
	uDataPacket _dataPacket;
	sSenseiACKPacket* _ACKPacket;
};

#endif // MANAGEDATAPACKET_H
