#include "manage_data_packet.h"

ManageDataPacket::ManageDataPacket()
{
	nPacketReceived = 0;
}

ManageDataPacket::~ManageDataPacket()
{

}

void ManageDataPacket::receivePacket()
{
	Serial.readBytes((char*)dataPacket.vData, SENSEI_LENGTH_DATA_PACKET);

	if (DEBUG)
	{
		for(int i=0;i<SENSEI_LENGTH_DATA_PACKET;i++)
		{
			SerialDebug.print("[" + String(dataPacket.vData[i]) +"] ");
		}
		SerialDebug.println("---------------");
	}
	
	nPacketReceived++;
}

uint32_t ManageDataPacket::getNpacketReceived()
{
	return nPacketReceived;
}

uint16_t ManageDataPacket::calcCRC()
{
	uint16_t crc = 0;

	for (uint16_t idx = SENSEI_START_IDX_CRC; idx < (SENSEI_STOP_IDX_CRC + 1); idx++)
	{
		crc += dataPacket.vData[idx];
	}

	return crc;
}

bool ManageDataPacket::checkCRC()
{
	uint16_t crc = calcCRC();

	if (dataPacket.sData.crc == crc)
	return true;
	else
	return false;
}

void ManageDataPacket::preparePacket(uint8_t cmd, uint8_t sub_cmd, uint16_t nPacketsMissing,uint8_t *pPayload,uint16_t nBytePayload)
{
	//Clear packet
	memset(&dataPacket, 0x00, SENSEI_LENGTH_DATA_PACKET);

	dataPacket.sData.start_header = START_HEADER;
	dataPacket.sData.cmd = cmd;
	dataPacket.sData.sub_cmd = sub_cmd;

	dataPacket.sData.continuation = nPacketsMissing;
	dataPacket.sData.timestamp = micros();
	dataPacket.sData.stop_header = STOP_HEADER;

	memcpy(&dataPacket.sData.payload[0], &(*pPayload), nBytePayload);

	dataPacket.sData.crc = calcCRC();
}

void ManageDataPacket::prepareACK(int32_t status,uint32_t timestamp, uint8_t cmd, uint8_t sub_cmd)
{
	//Clear packet
	memset(&dataPacket, 0x00, SENSEI_LENGTH_DATA_PACKET);

	dataPacket.sData.start_header = START_HEADER;
	dataPacket.sData.cmd = SENSEI_CMD::ACK;
	dataPacket.sData.sub_cmd = 0x00;

	ACKPacket = (sSenseiACKPacket*)&dataPacket.sData.payload[0];
	ACKPacket->status = status;
	ACKPacket->timestamp = timestamp;
	ACKPacket->cmd = cmd;
	ACKPacket->sub_cmd = sub_cmd;

	dataPacket.sData.continuation = 0;
	dataPacket.sData.timestamp = micros();
	dataPacket.sData.stop_header = STOP_HEADER;

	dataPacket.sData.crc = calcCRC();

}

int32_t ManageDataPacket::checkPacket()
{

	if ((dataPacket.sData.start_header.vByte[0] == START_HEADER.vByte[0]) &&
	(dataPacket.sData.start_header.vByte[1] == START_HEADER.vByte[1]) &&
	(dataPacket.sData.start_header.vByte[2] == START_HEADER.vByte[2]))
	{
		if ((dataPacket.sData.stop_header.vByte[0] == STOP_HEADER.vByte[0]) &&
		(dataPacket.sData.stop_header.vByte[1] == STOP_HEADER.vByte[1]) &&
		(dataPacket.sData.stop_header.vByte[2] == STOP_HEADER.vByte[2]))
		{
			if (checkCRC() == true)
			{
				return SENSEI_ERROR_CODE::OK;
			}
			else
			{
				return SENSEI_ERROR_CODE::CRC_NOT_CORRECT;
			}
		}
		else
		{
			return SENSEI_ERROR_CODE::STOP_HEADER_NOT_PRESENT;;
		}
	}
	else
	{
		return SENSEI_ERROR_CODE::START_HEADER_NOT_PRESENT;;
	}
}
