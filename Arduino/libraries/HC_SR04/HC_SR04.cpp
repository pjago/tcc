#include "HC_SR04.h"

//HC_SR04 *HC_SR04::_instance=NULL;
HC_SR04 *HC_SR04::_instance(NULL);

HC_SR04::HC_SR04(int trigger, int echo, int interrupt, int max_dist) //todo: find out why max is here
    : _trigger(trigger), _echo(echo), _int(interrupt), _max(max_dist), _finished(false), _last(0)
{
  if(_instance==0) _instance=this;    
}

void HC_SR04::begin(){
  pinMode(_trigger, OUTPUT);
  digitalWrite(_trigger, LOW);
  pinMode(_echo, INPUT);  
  attachInterrupt(_int, _echo_isr, CHANGE);
}

void HC_SR04::start(){
  _finished=false;
  digitalWrite(_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger, LOW);
}

// there are two cases where it goes wrong: 
// 1. object is too close, and signal doesn't echoes
// 2. object is too far, and signal doesn't echoes
unsigned int HC_SR04::getRange(bool units){
  unsigned int next = (_end-_start)/((units)?58:148);
  _last = next <= MAX ? next : _last;
  return _last;
}

void HC_SR04::_echo_isr(){
  HC_SR04* _this=HC_SR04::instance();
  
  switch(digitalRead(_this->_echo)){
    case HIGH:
      _this->_start=micros();
      break;
    case LOW:
      _this->_end=micros();
      _this->_finished=true;
      break;
  }   
}
