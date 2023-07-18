#ifndef simpleTimer_h
#define simpleTimer_h
#include "Arduino.h"

class simpleTimer{
	public:
		simpleTimer(unsigned long interval);
		void initialize();
		bool triggered();
		bool triggeredNoReset();
		void reset();
		void setInterval(unsigned long interval);
		unsigned long getInterval();
		
	private:
		unsigned long _interval;
		unsigned long _millisNow;
		unsigned long _millisThen;
};

#endif