#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H
#include "Includes.h"
#include "BitStuffing.h"
#include "Generator.h"

class SerialPort
{
private:
	//имя порта
	string _name;
	//дескриптор порта
	HANDLE _hPort;
	//скорость передачи в бодах
	DWORD _baudRate;
	//Contains information used in asynchronous (or overlapped) input and output (I/O).
	OVERLAPPED _syncRead;
	//дескриптор для события записи
	OVERLAPPED _syncWrite;
	//размыры входного и выходного буфферов
	//For Ethernet - based communications, a recommended buffer size might be 1600 bytes,
	//which is slightly larger than a single Ethernet frame.
	static const int _defaultBufSize = 1600;
	//буфер для чтения/записи
	byte _buffer[_defaultBufSize];
	//время для асинхронной операции чтения,записи
	const int _rwTime = 100;
	static const int _timeout = 1000;

	//для передачи пакетов
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
	{//посылка кадра
		//calculate bit vector capacity for hold data after bit-stuffing
		size_t bitPacketCapacity = Conversion::reserveBitVectorCapacity(data.size(), _bitStaffing.senderPatternLength());
		//convert packet to bit representation
		vector<bit> bitPacket = Conversion::convertByteToBitVector(data, bitPacketCapacity);
		//bit-stuffing
		_bitStaffing.bitStuffing(bitPacket, BitStuffing::ExecutedBy::Sender,0, bitPacket.size());
		//convert packet to byte representation for sending it
		//new size = rounded up to paste bit vector inside + 2 flags
		size_t bytePacketSize = Conversion::reserveByteVectorCapacity(bitPacket.size()) + 2;
		//пакет для передачи, имеет flags + header + data
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
	{//блокирующая функция, получает данные с помощью receiveFrame функции
		return  waitingForThePortReading(std::bind(&SerialPort::receiveFrame,this));
	}

private:
	void init(const string& name, DWORD baudRate)
	{
		//проверка правильности имени
		//checkName(name);
		_name = name;
		_baudRate = baudRate;
		_hPort = INVALID_HANDLE_VALUE;
		memset(&_syncWrite, 0, sizeof(_syncWrite));
		//Fills a block of memory with zeros.
		memset(&_syncRead, 0, sizeof(_syncRead));

		//открытие порта
		open();
		//настройка com-порта
		configurePort(_baudRate);

	}

	byte readByte()
	{//асинхронное чтение порта

	 //out прочитано байтов
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
		//ждём завершения операции чтения
		DWORD retVal = WaitForSingleObject(_syncRead.hEvent, INFINITE);
		//Если все успешно завершено, узнаем какой объем данных прочитан
		if (retVal == WAIT_OBJECT_0)
			//записываем данные
			//data.append(_buffer,bytesRead);
			return _buffer[0];

		return 0;
	}
	bool write(const vector<byte>& data)
	{
		if (_hPort == INVALID_HANDLE_VALUE)
			errorOccured("failed to write throw the com-port");
		DWORD bytesWritten = 0;
		BOOL writeResult = WriteFile(_hPort,	//дескриптор com-порта
			data.data(),	//данные для записи
			(DWORD)data.size(), //длинна данных
			&bytesWritten,	//out, байт записано
			&_syncWrite	//операция асинхронная
			);

		//ждём конец операции записи
		DWORD retVal = WaitForSingleObject(_syncWrite.hEvent, _rwTime);
		//скидываем событие в несигнальное состояние
		//Sets the specified event object to the nonsignaled state.
		BOOL reseted = ResetEvent(_syncWrite.hEvent);
		if (reseted == 0)
			errorOccured("failed to reset event func:write");
		//проверка корректности записи данных
		if (retVal == WAIT_OBJECT_0 && bytesWritten == data.size())
			//данные записаны
			return true;

		//не записаны
		return false;
	}
	vector<byte> waitingForThePortReading(std::function<vector<byte>(void)>readingProcedure)
	{
		DWORD eventMask = 0;
		//Связываем порт и объект синхронизации
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
		//ожидание данных
		DWORD retVal = WaitForSingleObject(_syncRead.hEvent,
			INFINITE);	//INFINITE, the function will return only when the object is signaled.
		if (retVal == WAIT_OBJECT_0)
			//The state of the specified object is signaled.
			//чтение данных
				return readingProcedure();
		
		return vector<byte>();
	}

	vector<byte> receiveFrame()
	{
		//packet that we want to get
		vector<byte> byteData;
		//frame delimiter
		byte FD = _bitStaffing.getFrameDelimiter();
		//проверка на флаг - начало фрейма
		byte received = readByte();
		/*if (received != FD)
		{
			//вычитываем из канала неправильные данные
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
		//дополнительные байты , которые мы добавили для передачи данных
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
	{//открытие порта
		_hPort = CreateFileA(_name.c_str(),//имя порта
			GENERIC_READ | GENERIC_WRITE,//писать,читать
			0,//нет совместного доступа
			NULL,
			OPEN_EXISTING,
			FILE_FLAG_OVERLAPPED,	//асинхронная работа с портом
			NULL
			);
		if (_hPort == INVALID_HANDLE_VALUE)
			throw runtime_error("failed to open the port: " + GetLastError());
	}
	void configurePort(int baudRate)
	{
		//устанавливаем размеры входного и выходного буфферов
		BOOL flCheck = SetupComm(_hPort,_defaultBufSize,_defaultBufSize);
		if (flCheck == 0)
			errorOccured("failed to do func:setupComm");
		//настроить таймауты для асинхронной передачи
		setTimeOuts();
		//общие настройки com-порта
		setDCB();
		//настройка событий чтения
		setReadEventReaction();
		//создание события для записи
		_syncWrite.hEvent = CreateEventA(NULL,FALSE,FALSE,NULL);
	}

	void setTimeOuts()
	{
		//настройка таймаутов устройства
		COMMTIMEOUTS commTimeOuts;
		//время в миллисекундах, задающее максимальное время, для интервала между поступлением двух символов по линии связи.
		commTimeOuts.ReadIntervalTimeout = 0xFFFFFFFF;
		//Множитель, используемый, чтобы вычислить полный период времени простоя для операций чтения, в миллисекундах.
		//Для каждой операции чтения, это значение умножается на затребованное число байтов, которые читаются.
		commTimeOuts.ReadTotalTimeoutMultiplier = 0;
		//Константа, используемая, чтобы вычислить полный период времени простоя для операций записи, в миллисекундах. 
		commTimeOuts.ReadTotalTimeoutConstant = 0;	//0 - для асинхронного режима передачи
		//(максимальная задержка при чтении и записи = _timeout)
		int retValue = SetCommTimeouts(_hPort, &commTimeOuts);
		// If the function succeeds, the return value is nonzero.
		if (retValue == 0)
			errorOccured("failed to set timeouts: ");
	}

	void setDCB()
	{	//Основные параметры последовательного порта
		DCB comDCB;

		memset(&comDCB, 0, sizeof(comDCB));
		//Задает длину, в байтах, структуры
		comDCB.DCBlength = sizeof(comDCB);
		//Retrieves the current control settings for a specified communications device.
		int retValue = GetCommState(_hPort,&comDCB);
		//Скорость передачи данных(в бодах)
		comDCB.BaudRate = _baudRate;
		//Определяет число информационных бит в передаваемых и принимаемых байтах.
		//Число информационных бит может быть в диапазоне от 4 до 8.
		comDCB.ByteSize = 8;
		/*
		Определяет выбор схемы контроля четности. Данное поле должно содержать одно из следующих значений:
		EVENPARITY     Дополнение до четности
		MARKPARITY     Бит четности всегда 1
		NOPARITY         Бит четности отсутствует
		ODDPARITY       Дополнение до нечетности
		SPACEPARITY   Бит четности всегда 0
		*/
		comDCB.Parity = NOPARITY;
		/*
		Задает количество стоповых бит. Поле может принимать следующие значения:
		ONESTOPBIT     Один стоповый бит
		ONE5STOPBIT   Полтора стоповых бита
		TWOSTOPBIT     Два стоповых бита
		*/
		comDCB.StopBits = ONESTOPBIT;
		/*Задает игнорирование всех операций чтения/записи при возникновении ошибки. Если это поле равно TRUE,
		драйвер прекращает все операции чтения/записи для порта при возникновении ошибки.
		Продолжать работать с портом можно будет только после устранения причины ошибки и вызова функции ClearCommError.*/
		comDCB.fAbortOnError = TRUE;
		/*
		Задает режим управления обменом для сигнала [[|DTR]]. Поле может принимать следующие значения:
		DTR_CONTROL_DISABLE         Сигнал DTR снимается при открытии порта. У открытого порта может быть изменён функцией EscapeCommFunction.
		DTR_CONTROL_ENABLE          Сигнал DTR устанавливается при открытии порта. У открытого порта может быть изменён функцией EscapeCommFunction.
		DTR_CONTROL_HANDSHAKE   Сигнал DTR автоматически устанавливается/снимается в ходе работы с портом. Не может быть изменён функцией EscapeCommFunction.
		*/
		comDCB.fDtrControl = DTR_CONTROL_DISABLE;
		/*
		адает режим управления потоком для сигнала RTS. Поле может принимать следующие значения:
		RTS_CONTROL_DISABLE          Сигнал RTS снимается при открытии порта. У открытого порта может быть изменён функцией EscapeCommFunction.
		RTS_CONTROL_ENABLE           Сигнал RTS устанавливается при открытии порта. У открытого порта может быть изменён функцией EscapeCommFunction.
		RTS_CONTROL_HANDSHAKE   Сигнал RTS автоматически устанавливается/снимается в ходе работы с портом. Не может быть изменён функцией EscapeCommFunction. Сигнал RTS устанавливается, когда приемный буфер заполнен менее, чем на половину, и снимается, когда буфер заполняется более чем на три четверти.
		RTS_CONTROL_TOGGLE          Задаёт, что сигнал RTS установлен, когда есть данные для передачи. Когда все символы из передающего буфера переданы, сигнал снимается.
		*/
		comDCB.fRtsControl = RTS_CONTROL_TOGGLE;
		//Включает двоичный режим обмена.
		comDCB.fBinary = TRUE;
		//Включает режим контроля четности. (нету)
		comDCB.fParity = FALSE;
		/*
		Задает использование XON/XOFF управления потоком при приеме. Если это поле равно TRUE,
		то драйвер передает символ XoffChar, когда в приемном буфере находится более XoffLim,
		и XonChar, когда в приемном буфере остается менее XonLim символов.
		*/
		comDCB.fInX = FALSE;
		//Задает использование XON/XOFF управления потоком при передаче
		comDCB.fOutX = FALSE;
		//Задает символ XON используемый как для приема, так и для передачи. Обычно 0x11
		comDCB.XonChar = 0;
		//Задает символ XOFF используемый как для приема, так и для передачи. Обычно 0x13
		comDCB.XoffChar = (unsigned char)(0xff);
		/*
		Указывает на необходимость замены символов с ошибкой четности на символ задаваемый полем
		ErrorChar. Если это поле равно TRUE, и поле fParity равно TRUE, то выполняется замена.
		*/
		comDCB.fErrorChar = FALSE;
		//Определяет действие выполняемое при приеме нулевого байта. Если это поле TRUE, то нулевые байты отбрасываются при передаче.
		comDCB.fNull = FALSE;
		/*
		Включает режим слежения за сигналом [[|CTS]].
		Если это поле равно [[|TRUE]] и сигнал [[|CTS]] сброшен, передача данных приостанавливается
		до установки сигнала CTS. Это позволяет подключеному к компьютеру прибору приостановить поток
		передаваемой в него информации, если он не успевает ее обрабатывать.
		*/
		comDCB.fOutxCtsFlow = FALSE;
		/*
		Включает режим слежения за сигналом [[|DSR]]. Если это поле равно TRUE и сигнал DSR сброшен,
		передача данных прекращается до установки сигнала DSR.
		*/
		comDCB.fOutxDsrFlow = FALSE;
		//Задает минимальное число символов в приемном буфере перед посылкой символа XON.
		comDCB.XonLim = 128;
		/*
		Определяет максимальное количество байт в приемном буфере перед посылкой символа XOFF.
		Максимально допустимое количество байт в буфере вычисляется вычитанием данного значения из
		размера приемного буфера в байтах.
		*/
		comDCB.XoffLim = 128;


		//установка параметров для устройства
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
	{//настройка событий (прихода данных)
		//создаём объект синхронизации
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
		// Устанавливаем маску на события порта
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
	{//проверка правильности имени порта
		//если не правильно -> исключение
		std::regex regExp("COM[1-9]");
		if (!std::regex_match(name, regExp))
			throw runtime_error("incorrect serial port name");
	}
	*/
	void errorOccured(const char* message)
	{//обработка ошибки на уровне класса
		close();
		throw runtime_error(message + to_string(GetLastError()));
	}

	void close()
	{	//закрыть COM-порт
		if (_hPort != INVALID_HANDLE_VALUE)
			CloseHandle(_hPort);
		if (_syncRead.hEvent != INVALID_HANDLE_VALUE)
			//закрыть объект синхронизации
			CloseHandle(_syncRead.hEvent);
	}
	
};




#endif SERIAL_PORT_H