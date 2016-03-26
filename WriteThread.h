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
	{//запись данных в порт
		cout << "can write"<<endl;
		//сгенерировать случайные данные
		Generator generator;
		size_t dataSize = 50;
		//получить массив случайных данных
		vector<byte> sendingData;
		sendingData.reserve(dataSize);
		
		while (true)
		{
			//генерация вектора по нажатию на клавиши
			_getch();
			sendingData.clear();
			sendingData = generator.generateVector<byte>(50);
			//отобразить сгенерированные данные в бинарном виде
			displayContainer(sendingData.begin(), sendingData.end(), std::cout);
			//послать данные,помещённые в фрейм
			_pSerialPort->sendFrame(sendingData);
		}
		
	}
};


#endif WRITE_THREAD_H