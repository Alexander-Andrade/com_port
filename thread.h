#ifndef THREAD_H
#define THREAD_H
#include "Includes.h"

class Thread 
{
private:
	DWORD _id;
	HANDLE _handle;
	
	// copy and assignment not allowed
	Thread(const Thread& o);
	const Thread& operator=(const Thread& o);

	static DWORD WINAPI start_routine(LPVOID lpParam)
	{
		((Thread*)lpParam)->run();
		return 0;
	}
	
public:
	Thread(){};
	virtual ~Thread(){};
	
	void start()
	{
		_handle = CreateThread(NULL,			// default security attributes
			0,				// use default stack size  
			start_routine,   // thread function name
			(void*)this,		// argument to thread function 
			0,				// use default creation flags
			&_id// returns the thread identifier 
			);
		if (_handle == NULL)
			cout << GetLastError() << endl;
	}
	void wait()
	{//ожидание завершения потока,вызывается из родидельского
		WaitForSingleObject(_handle, INFINITE);
	}
protected:
	//наследоваться и перегружать
	virtual void run() = 0;
};

#endif