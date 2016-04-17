#ifndef MANAGEDATAPACKET_H
#define MANAGEDATAPACKET_H

#include "common.h"

typedef union uDataPacket
{
	sSenseiDataPacket sData;
	uint8_t vData[SENSEI_LENGTH_DATA_PACKET];
} uDataPacket;

class ManageDataPacket {
public:
	ManageDataPacket();
	~ManageDataPacket();
	void receivePacket();
	void preparePacket(uint8_t cmd, uint8_t sub_cmd, uint16_t nPacketsMissing, uint8_t *pPayload, uint16_t nBytePayload);
	void prepareACK(int32_t status, uint32_t timestamp, uint8_t cmd, uint8_t sub_cmd);
	int32_t checkPacket();

    uint32_t getNpacketReceived();
	uDataPacket dataPacket;

private:
	uint32_t nPacketReceived;

	uint16_t calcCRC();
	bool checkCRC();
	sSenseiACKPacket* ACKPacket;
};

#endif // MANAGEDATAPACKET_H
