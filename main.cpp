#include "SerialPort.h"
#include "WriteThread.h"
//��������� �� com-port


void readRoutine(SerialPort* pSerialPort)
{//������ ����� ��������
	vector<byte> acquiredInfo;
	while (true)
	{
		acquiredInfo = pSerialPort->getFrame();
		//���������� �������� ������ � �������� ����
		//displayContainerBinary(acquiredInfo.begin(), acquiredInfo.end());
		displayContainer(acquiredInfo.begin(), acquiredInfo.end(),std::cout);
	}
}


int main(int argc,char* argv[])
{	
	vector<bit> bvect = {0,1,1,1,1,1,1, 0,1,1,1,1,1,1,1,0,1,1,0,0, 0,1,1,1,1,1,1,1,1,1,0,1, 0,1,1,1,1,1,1,1,0,};
	//vector<bit> bvect = {0,1,1,1,1,1,1,0,1,1,1,1,1,1};
	//BitStuffing bs;
	//cout << bs.bitStuffing(bvect, BitStuffing::ExecutedBy::Sender, 0, bvect.size()) << endl;
	//cout << bs.bitStuffing(bvect, BitStuffing::ExecutedBy::Recepient, 0, bvect.size()) << endl;

	if (argc != 2)
	{
		cout << "unknown port" << endl;
		return -1;
	}
	try
	{
		SerialPort serialPort(argv[1]);
		//��������� ����� ������ 
		WriteThread writeThread(&serialPort);
		writeThread.start();
		//���� ������
		readRoutine(&serialPort);
	}
	catch (exception e){ 
		cout << e.what() << endl;
		_getch();
	}

	//system("pause");
	return 0;
}