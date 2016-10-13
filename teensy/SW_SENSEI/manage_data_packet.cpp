#include "manage_data_packet.h"

using namespace sensei;

ManageDataPacket::ManageDataPacket()
{
	_nPacketReceived = 0;
}

ManageDataPacket::~ManageDataPacket()
{

}

void ManageDataPacket::setPayloadToVariable(void* dataOutput,uint16_t nByte)
{
	memcpy(dataOutput,&_dataPacket.data.payload[0],nByte);
}

void ManageDataPacket::getPacketID(uint8_t& cmd, uint8_t& sub_cmd, uint32_t& timestamp)
{
	cmd = _dataPacket.data.cmd;
	sub_cmd = _dataPacket.data.sub_cmd;
	timestamp = _dataPacket.data.timestamp;
}

uint8_t ManageDataPacket::getCmd()
{
	return _dataPacket.data.cmd;
}

uint8_t ManageDataPacket::getSubCmd()
{
	return _dataPacket.data.sub_cmd;
}

uint32_t ManageDataPacket::getTimestamp()
{
	return _dataPacket.data.timestamp;
}

void ManageDataPacket::receivePacket()
{
	Serial.readBytes((char*)_dataPacket.vectorData, SENSEI_LENGTH_DATA_PACKET);

	if (DEBUG)
	{
		for(int i=0;i<SENSEI_LENGTH_DATA_PACKET;i++)
		{
			SerialDebug.print("[" + String(_dataPacket.vectorData[i]) +"] ");
		}
		SerialDebug.println("---------------");
	}

	_nPacketReceived++;
}

uint32_t ManageDataPacket::getNpacketReceived()
{
	return _nPacketReceived;
}

uint16_t ManageDataPacket::calcCRC()
{
	uint16_t crc = 0;

	for (uint16_t idx = SENSEI_START_IDX_CRC; idx < (SENSEI_STOP_IDX_CRC + 1); idx++)
	{
		crc += _dataPacket.vectorData[idx];
	}

	return crc;
}

void ManageDataPacket::clearDataPacket()
{
	memset(&_dataPacket, 0x00, SENSEI_LENGTH_DATA_PACKET);
}

bool ManageDataPacket::checkCRC()
{
	uint16_t crc = calcCRC();

	if (_dataPacket.data.crc == crc)
	return true;
	else
	return false;
}

void ManageDataPacket::preparePacket(uint8_t cmd, uint8_t sub_cmd, uint16_t nPacketsMissing,uint8_t *pPayload,uint16_t nBytePayload)
{
	clearDataPacket();

	_dataPacket.data.start_header = START_HEADER;
	_dataPacket.data.cmd = cmd;
	_dataPacket.data.sub_cmd = sub_cmd;

	_dataPacket.data.continuation = nPacketsMissing;
	_dataPacket.data.timestamp = micros();
	_dataPacket.data.stop_header = STOP_HEADER;

	memcpy(&_dataPacket.data.payload[0], &(*pPayload), nBytePayload);

	_dataPacket.data.crc = calcCRC();
}

void ManageDataPacket::prepareACK(int32_t status,uint32_t timestamp, uint8_t cmd, uint8_t sub_cmd)
{
	clearDataPacket();

	_dataPacket.data.start_header = START_HEADER;
	_dataPacket.data.cmd = SENSEI_CMD::ACK;
	_dataPacket.data.sub_cmd = SENSEI_SUB_CMD::EMPTY;

	_ACKPacket = (sSenseiACKPacket*)&_dataPacket.data.payload[0];
	_ACKPacket->status = status;
	_ACKPacket->timestamp = timestamp;
	_ACKPacket->cmd = cmd;
	_ACKPacket->sub_cmd = sub_cmd;

	_dataPacket.data.continuation = 0;
	_dataPacket.data.timestamp = timestamp;
	_dataPacket.data.stop_header = STOP_HEADER;

	_dataPacket.data.crc = calcCRC();

}

void ManageDataPacket::send()
{
	Serial.write(_dataPacket.vectorData, SENSEI_LENGTH_DATA_PACKET);
	Serial.send_now();
}

int32_t ManageDataPacket::checkPacket()
{
	if (!((_dataPacket.data.start_header.vByte[0] == START_HEADER.vByte[0]) &&
	   (_dataPacket.data.start_header.vByte[1] == START_HEADER.vByte[1])  &&
	   (_dataPacket.data.start_header.vByte[2] == START_HEADER.vByte[2])))
	   return SENSEI_ERROR_CODE::START_HEADER_NOT_PRESENT;

	if (!((_dataPacket.data.stop_header.vByte[0] == STOP_HEADER.vByte[0]) &&
	   (_dataPacket.data.stop_header.vByte[1] == STOP_HEADER.vByte[1])  &&
	   (_dataPacket.data.stop_header.vByte[2] == STOP_HEADER.vByte[2])))
	   return SENSEI_ERROR_CODE::STOP_HEADER_NOT_PRESENT;;

	if (checkCRC() == true)
		return SENSEI_ERROR_CODE::OK;
	else
		return SENSEI_ERROR_CODE::CRC_NOT_CORRECT;
}
