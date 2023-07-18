#include "Arduino.h"
#include "simpleTimer.h"

simpleTimer::simpleTimer(unsigned long interval){
	_interval = interval;
}

void simpleTimer::initialize(){
	_millisNow = millis();
	_millisThen = 0;
}

bool simpleTimer::triggered(){
	bool triggered = false;
	_millisNow = millis();
	if(_millisNow - _millisThen >= _interval){
		triggered = true;
		simpleTimer::reset();
	}
	return triggered;
}

bool simpleTimer::triggeredNoReset(){
	bool triggered = false;
	_millisNow = millis();
	if(_millisNow - _millisThen >= _interval){
		triggered = true;
	}
	return triggered;
}

void simpleTimer::reset(){
	_millisNow = millis();
	_millisThen = _millisNow;
}	

void simpleTimer::setInterval(unsigned long interval){
	_interval = interval;
}

unsigned long simpleTimer::getInterval(){
	return _interval;
}
