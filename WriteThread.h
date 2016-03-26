#ifndef WRITE_THREAD_H
#define WRITE_THREAD_H
#include "thread.h"
#include "SerialPort.h"

class WriteThread : public Thread
{
private:
	SerialPort* _pSerialPort;
public:
	WriteThread(SerialPort* pSerialPort) : _pSerialPort(pSerialPort) {}
protected:
	void run() override
	{//������ ������ � ����
		cout << "can write"<<endl;
		//������������� ��������� ������
		Generator generator;
		size_t dataSize = 50;
		//�������� ������ ��������� ������
		vector<byte> sendingData;
		sendingData.reserve(dataSize);
		
		while (true)
		{
			//��������� ������� �� ������� �� �������
			_getch();
			sendingData.clear();
			sendingData = generator.generateVector<byte>(50);
			//���������� ��������������� ������ � �������� ����
			displayContainer(sendingData.begin(), sendingData.end(), std::cout);
			//������� ������,���������� � �����
			_pSerialPort->sendFrame(sendingData);
		}
		
	}
};


#endif WRITE_THREAD_H