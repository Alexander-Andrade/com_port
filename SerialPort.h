#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H
#include "Includes.h"
#include "BitStuffing.h"
#include "Generator.h"

class SerialPort
{
private:
	//��� �����
	string _name;
	//���������� �����
	HANDLE _hPort;
	//�������� �������� � �����
	DWORD _baudRate;
	//Contains information used in asynchronous (or overlapped) input and output (I/O).
	OVERLAPPED _syncRead;
	//���������� ��� ������� ������
	OVERLAPPED _syncWrite;
	//������� �������� � ��������� ��������
	//For Ethernet - based communications, a recommended buffer size might be 1600 bytes,
	//which is slightly larger than a single Ethernet frame.
	static const int _defaultBufSize = 1600;
	//����� ��� ������/������
	byte _buffer[_defaultBufSize];
	//����� ��� ����������� �������� ������,������
	const int _rwTime = 100;
	static const int _timeout = 1000;

	//��� �������� �������
	BitStuffing _bitStaffing;

	// copy and assignment not allowed
	SerialPort(const SerialPort& o);
	const SerialPort& operator=(const SerialPort& o);
public:
	
	SerialPort(const string& name,DWORD baudRate) 
	{
		init(name, baudRate);
	}
	SerialPort(const string& name)
	{
		init(name, CBR_9600);
	}

	~SerialPort()
	{ 
		close(); 
	}
	
	bool sendFrame(const vector<byte>& data)
	{//������� �����
		//calculate bit vector capacity for hold data after bit-stuffing
		size_t bitPacketCapacity = Conversion::reserveBitVectorCapacity(data.size(), _bitStaffing.senderPatternLength());
		//convert packet to bit representation
		vector<bit> bitPacket = Conversion::convertByteToBitVector(data, bitPacketCapacity);
		//bit-stuffing
		_bitStaffing.bitStuffing(bitPacket, BitStuffing::ExecutedBy::Sender,0, bitPacket.size());
		//convert packet to byte representation for sending it
		//new size = rounded up to paste bit vector inside + 2 flags
		size_t bytePacketSize = Conversion::reserveByteVectorCapacity(bitPacket.size()) + 2;
		//����� ��� ��������, ����� flags + header + data
		vector<byte> bytePacket(bytePacketSize);
		//frame delimiter
		byte FD = _bitStaffing.getFrameDelimiter();
		//front flag to packet
		bytePacket.front() = FD;
		Conversion::toByteVector(bytePacket.begin() + 1, bytePacket.end() - 1, bitPacket.begin(), bitPacket.end());
		//back flag to packet
		bytePacket.back() = FD;
		//send packet
		return write(bytePacket);
	}

	vector<byte> getFrame()
	{//����������� �������, �������� ������ � ������� receiveFrame �������
		return  waitingForThePortReading(std::bind(&SerialPort::receiveFrame,this));
	}

private:
	void init(const string& name, DWORD baudRate)
	{
		//�������� ������������ �����
		//checkName(name);
		_name = name;
		_baudRate = baudRate;
		_hPort = INVALID_HANDLE_VALUE;
		memset(&_syncWrite, 0, sizeof(_syncWrite));
		//Fills a block of memory with zeros.
		memset(&_syncRead, 0, sizeof(_syncRead));

		//�������� �����
		open();
		//��������� com-�����
		configurePort(_baudRate);

	}

	byte readByte()
	{//����������� ������ �����

	 //out ��������� ������
		DWORD bytesRead = 0;
		ReadFile(_hPort,			//A handle to the device
									//A pointer to the buffer that receives the data read from a file or device.
			_buffer,
			1,	//The maximum number of bytes to be read.
			&bytesRead,			//A pointer to the variable that receives the number of bytes read when using a synchronous hFile parameter.
								/*
								A pointer to an OVERLAPPED structure is required if the hFile parameter was opened with
								FILE_FLAG_OVERLAPPED, otherwise it can be NULL.
								*/
			&_syncRead
			);
		//��� ���������� �������� ������
		DWORD retVal = WaitForSingleObject(_syncRead.hEvent, INFINITE);
		//���� ��� ������� ���������, ������ ����� ����� ������ ��������
		if (retVal == WAIT_OBJECT_0)
			//���������� ������
			//data.append(_buffer,bytesRead);
			return _buffer[0];

		return 0;
	}
	bool write(const vector<byte>& data)
	{
		if (_hPort == INVALID_HANDLE_VALUE)
			errorOccured("failed to write throw the com-port");
		DWORD bytesWritten = 0;
		BOOL writeResult = WriteFile(_hPort,	//���������� com-�����
			data.data(),	//������ ��� ������
			(DWORD)data.size(), //������ ������
			&bytesWritten,	//out, ���� ��������
			&_syncWrite	//�������� �����������
			);

		//��� ����� �������� ������
		DWORD retVal = WaitForSingleObject(_syncWrite.hEvent, _rwTime);
		//��������� ������� � ������������ ���������
		//Sets the specified event object to the nonsignaled state.
		BOOL reseted = ResetEvent(_syncWrite.hEvent);
		if (reseted == 0)
			errorOccured("failed to reset event func:write");
		//�������� ������������ ������ ������
		if (retVal == WAIT_OBJECT_0 && bytesWritten == data.size())
			//������ ��������
			return true;

		//�� ��������
		return false;
	}
	vector<byte> waitingForThePortReading(std::function<vector<byte>(void)>readingProcedure)
	{
		DWORD eventMask = 0;
		//��������� ���� � ������ �������������
		/*
		Waits for an event to occur for a specified communications device.
		The set of events that are monitored by this function is contained in the event mask
		associated with the device handle.
		*/

		// Make a call to WaitCommEvent().  This call will return immediatly
		// because our port was created as an async port (FILE_FLAG_OVERLAPPED
		// and an _syncRead structure specified).  This call will cause the 
		// _syncRead element _syncRead.hEvent to 
		// be placed in a non-signaled state if there are no bytes available to be read,
		// or to a signaled state if there are bytes available.  If this event handle 
		// is set to the non-signaled state, it will be set to signaled when a 
		// character arrives at the port.
		// we do this for each port!

		BOOL flCheck = WaitCommEvent(_hPort,
			//A pointer to a variable that receives a mask indicating the type of event that occurred.
			//If an error occurs, the value is zero; otherwise, it is one of the following values.
			&eventMask,
			&_syncRead);
		/*if (flCheck == 0)
		errorOccured("failed to set waitCommEvent");
		*/
		//�������� ������
		DWORD retVal = WaitForSingleObject(_syncRead.hEvent,
			INFINITE);	//INFINITE, the function will return only when the object is signaled.
		if (retVal == WAIT_OBJECT_0)
			//The state of the specified object is signaled.
			//������ ������
				return readingProcedure();
		
		return vector<byte>();
	}

	vector<byte> receiveFrame()
	{
		//packet that we want to get
		vector<byte> byteData;
		//frame delimiter
		byte FD = _bitStaffing.getFrameDelimiter();
		//�������� �� ���� - ������ ������
		byte received = readByte();
		/*if (received != FD)
		{
			//���������� �� ������ ������������ ������
			while ((received = readByte()) != FD);
			return vector<byte>();
		}
		*/
		while (true)
		{
			received = readByte();
			//delete last flag from packet
			if (received == FD) break;

			//read each byte
			byteData.push_back(received);
		}
		
		size_t bitDataSize = byteData.size() * Conversion::byteSize;
		vector<bit> bitData = Conversion::convertByteToBitVector(byteData, bitDataSize);
		//reverse bit-stuffing
		size_t bitDeleted = _bitStaffing.bitStuffing(bitData, BitStuffing::ExecutedBy::Recepient, 0, bitData.size());
		//�������������� ����� , ������� �� �������� ��� �������� ������
		size_t extraBytes = ceil((double)bitDeleted / Conversion::byteSize);
		//convert to byte data
		byteData.clear();
		byteData = Conversion::convertBitToByteVector(bitData);
		//delete extra bytes after transfering data
		byteData.erase(byteData.end() - extraBytes, byteData.end());

		//return clear dATA
		return byteData;
	}

	void open()
	{//�������� �����
		_hPort = CreateFileA(_name.c_str(),//��� �����
			GENERIC_READ | GENERIC_WRITE,//������,������
			0,//��� ����������� �������
			NULL,
			OPEN_EXISTING,
			FILE_FLAG_OVERLAPPED,	//����������� ������ � ������
			NULL
			);
		if (_hPort == INVALID_HANDLE_VALUE)
			throw runtime_error("failed to open the port: " + GetLastError());
	}
	void configurePort(int baudRate)
	{
		//������������� ������� �������� � ��������� ��������
		BOOL flCheck = SetupComm(_hPort,_defaultBufSize,_defaultBufSize);
		if (flCheck == 0)
			errorOccured("failed to do func:setupComm");
		//��������� �������� ��� ����������� ��������
		setTimeOuts();
		//����� ��������� com-�����
		setDCB();
		//��������� ������� ������
		setReadEventReaction();
		//�������� ������� ��� ������
		_syncWrite.hEvent = CreateEventA(NULL,FALSE,FALSE,NULL);
	}

	void setTimeOuts()
	{
		//��������� ��������� ����������
		COMMTIMEOUTS commTimeOuts;
		//����� � �������������, �������� ������������ �����, ��� ��������� ����� ������������ ���� �������� �� ����� �����.
		commTimeOuts.ReadIntervalTimeout = 0xFFFFFFFF;
		//���������, ������������, ����� ��������� ������ ������ ������� ������� ��� �������� ������, � �������������.
		//��� ������ �������� ������, ��� �������� ���������� �� ������������� ����� ������, ������� ��������.
		commTimeOuts.ReadTotalTimeoutMultiplier = 0;
		//���������, ������������, ����� ��������� ������ ������ ������� ������� ��� �������� ������, � �������������. 
		commTimeOuts.ReadTotalTimeoutConstant = 0;	//0 - ��� ������������ ������ ��������
		//(������������ �������� ��� ������ � ������ = _timeout)
		int retValue = SetCommTimeouts(_hPort, &commTimeOuts);
		// If the function succeeds, the return value is nonzero.
		if (retValue == 0)
			errorOccured("failed to set timeouts: ");
	}

	void setDCB()
	{	//�������� ��������� ����������������� �����
		DCB comDCB;

		memset(&comDCB, 0, sizeof(comDCB));
		//������ �����, � ������, ���������
		comDCB.DCBlength = sizeof(comDCB);
		//Retrieves the current control settings for a specified communications device.
		int retValue = GetCommState(_hPort,&comDCB);
		//�������� �������� ������(� �����)
		comDCB.BaudRate = _baudRate;
		//���������� ����� �������������� ��� � ������������ � ����������� ������.
		//����� �������������� ��� ����� ���� � ��������� �� 4 �� 8.
		comDCB.ByteSize = 8;
		/*
		���������� ����� ����� �������� ��������. ������ ���� ������ ��������� ���� �� ��������� ��������:
		EVENPARITY     ���������� �� ��������
		MARKPARITY     ��� �������� ������ 1
		NOPARITY         ��� �������� �����������
		ODDPARITY       ���������� �� ����������
		SPACEPARITY   ��� �������� ������ 0
		*/
		comDCB.Parity = NOPARITY;
		/*
		������ ���������� �������� ���. ���� ����� ��������� ��������� ��������:
		ONESTOPBIT     ���� �������� ���
		ONE5STOPBIT   ������� �������� ����
		TWOSTOPBIT     ��� �������� ����
		*/
		comDCB.StopBits = ONESTOPBIT;
		/*������ ������������� ���� �������� ������/������ ��� ������������� ������. ���� ��� ���� ����� TRUE,
		������� ���������� ��� �������� ������/������ ��� ����� ��� ������������� ������.
		���������� �������� � ������ ����� ����� ������ ����� ���������� ������� ������ � ������ ������� ClearCommError.*/
		comDCB.fAbortOnError = TRUE;
		/*
		������ ����� ���������� ������� ��� ������� [[|DTR]]. ���� ����� ��������� ��������� ��������:
		DTR_CONTROL_DISABLE         ������ DTR ��������� ��� �������� �����. � ��������� ����� ����� ���� ������ �������� EscapeCommFunction.
		DTR_CONTROL_ENABLE          ������ DTR ��������������� ��� �������� �����. � ��������� ����� ����� ���� ������ �������� EscapeCommFunction.
		DTR_CONTROL_HANDSHAKE   ������ DTR ������������� ���������������/��������� � ���� ������ � ������. �� ����� ���� ������ �������� EscapeCommFunction.
		*/
		comDCB.fDtrControl = DTR_CONTROL_DISABLE;
		/*
		����� ����� ���������� ������� ��� ������� RTS. ���� ����� ��������� ��������� ��������:
		RTS_CONTROL_DISABLE          ������ RTS ��������� ��� �������� �����. � ��������� ����� ����� ���� ������ �������� EscapeCommFunction.
		RTS_CONTROL_ENABLE           ������ RTS ��������������� ��� �������� �����. � ��������� ����� ����� ���� ������ �������� EscapeCommFunction.
		RTS_CONTROL_HANDSHAKE   ������ RTS ������������� ���������������/��������� � ���� ������ � ������. �� ����� ���� ������ �������� EscapeCommFunction. ������ RTS ���������������, ����� �������� ����� �������� �����, ��� �� ��������, � ���������, ����� ����� ����������� ����� ��� �� ��� ��������.
		RTS_CONTROL_TOGGLE          �����, ��� ������ RTS ����������, ����� ���� ������ ��� ��������. ����� ��� ������� �� ����������� ������ ��������, ������ ���������.
		*/
		comDCB.fRtsControl = RTS_CONTROL_TOGGLE;
		//�������� �������� ����� ������.
		comDCB.fBinary = TRUE;
		//�������� ����� �������� ��������. (����)
		comDCB.fParity = FALSE;
		/*
		������ ������������� XON/XOFF ���������� ������� ��� ������. ���� ��� ���� ����� TRUE,
		�� ������� �������� ������ XoffChar, ����� � �������� ������ ��������� ����� XoffLim,
		� XonChar, ����� � �������� ������ �������� ����� XonLim ��������.
		*/
		comDCB.fInX = FALSE;
		//������ ������������� XON/XOFF ���������� ������� ��� ��������
		comDCB.fOutX = FALSE;
		//������ ������ XON ������������ ��� ��� ������, ��� � ��� ��������. ������ 0x11
		comDCB.XonChar = 0;
		//������ ������ XOFF ������������ ��� ��� ������, ��� � ��� ��������. ������ 0x13
		comDCB.XoffChar = (unsigned char)(0xff);
		/*
		��������� �� ������������� ������ �������� � ������� �������� �� ������ ���������� �����
		ErrorChar. ���� ��� ���� ����� TRUE, � ���� fParity ����� TRUE, �� ����������� ������.
		*/
		comDCB.fErrorChar = FALSE;
		//���������� �������� ����������� ��� ������ �������� �����. ���� ��� ���� TRUE, �� ������� ����� ������������� ��� ��������.
		comDCB.fNull = FALSE;
		/*
		�������� ����� �������� �� �������� [[|CTS]].
		���� ��� ���� ����� [[|TRUE]] � ������ [[|CTS]] �������, �������� ������ ������������������
		�� ��������� ������� CTS. ��� ��������� ������������ � ���������� ������� ������������� �����
		������������ � ���� ����������, ���� �� �� �������� �� ������������.
		*/
		comDCB.fOutxCtsFlow = FALSE;
		/*
		�������� ����� �������� �� �������� [[|DSR]]. ���� ��� ���� ����� TRUE � ������ DSR �������,
		�������� ������ ������������ �� ��������� ������� DSR.
		*/
		comDCB.fOutxDsrFlow = FALSE;
		//������ ����������� ����� �������� � �������� ������ ����� �������� ������� XON.
		comDCB.XonLim = 128;
		/*
		���������� ������������ ���������� ���� � �������� ������ ����� �������� ������� XOFF.
		����������� ���������� ���������� ���� � ������ ����������� ���������� ������� �������� ��
		������� ��������� ������ � ������.
		*/
		comDCB.XoffLim = 128;


		//��������� ���������� ��� ����������
		/*
		Configures a communications device according to the specifications in
		a device-control block (a DCB structure). The function reinitializes all
		hardware and control settings, but it does not empty output or input queues.
		*/

		int retVal = SetCommState(_hPort, &comDCB);
		if (retVal == 0)
			errorOccured("failured to set com state");
	}

	void setReadEventReaction()
	{//��������� ������� (������� ������)
		//������ ������ �������������
		_syncRead.hEvent = CreateEventA(NULL,//A pointer to a SECURITY_ATTRIBUTES structure.
										/*If this parameter is TRUE, the function creates a manual-reset event object,
										which requires the use of the ResetEvent function to set the event state to nonsignaled.
										If this parameter is FALSE, the function creates an auto-reset event object,
										and system automatically resets the event state to nonsignaled after a single waiting
										thread has been released.*/
									TRUE,
									//If this parameter is TRUE, the initial state of the event object is signaled; otherwise, it is nonsignaled.
									FALSE,
									//The name of the event object.
									NULL);
		// ������������� ����� �� ������� �����
		//Specifies a set of events to be monitored for a communications device.
		int retVal = SetCommMask(_hPort,
											//The events to be enabled. A value of zero disables all events.	
								EV_RXCHAR	//A character was received and placed in the input buffer.
								);
		if (retVal == 0)
			errorOccured("failed to set event reaction");
		
	}
	
	DWORD resultOfOverlappedOperation(DWORD& bytesRead)
	{
		/*
		Retrieves the results of an overlapped operation on the specified file,
		named pipe, or communications device.
		*/
		DWORD retVal = GetOverlappedResult(_hPort,	//A handle to the file, named pipe, or communications device.
									&_syncRead,	//A pointer to an OVERLAPPED structure that was specified when the overlapped operation was started.
									&bytesRead, //A pointer to a variable that receives the number of bytes that were actually transferred by a read or write operation.
									/*
									If this parameter is FALSE and the operation is still pending, the function returns
									FALSE and the GetLastError function returns ERROR_IO_INCOMPLETE.
									*/
									FALSE
									);
		if (retVal == WAIT_OBJECT_0)
			return bytesRead;
		return 0;
	}
/*
	void checkName(const string& name)
	{//�������� ������������ ����� �����
		//���� �� ��������� -> ����������
		std::regex regExp("COM[1-9]");
		if (!std::regex_match(name, regExp))
			throw runtime_error("incorrect serial port name");
	}
	*/
	void errorOccured(const char* message)
	{//��������� ������ �� ������ ������
		close();
		throw runtime_error(message + to_string(GetLastError()));
	}

	void close()
	{	//������� COM-����
		if (_hPort != INVALID_HANDLE_VALUE)
			CloseHandle(_hPort);
		if (_syncRead.hEvent != INVALID_HANDLE_VALUE)
			//������� ������ �������������
			CloseHandle(_syncRead.hEvent);
	}
	
};




#endif SERIAL_PORT_H