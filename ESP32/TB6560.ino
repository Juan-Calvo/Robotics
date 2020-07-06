/***********************************************************************
 *   DRIVERS CONFIGURATION                                             *
 ***********************************************************************/
#include "TB6560.h"
TB6560::TB6560(uint16_t pinstep,uint16_t pindir, uint16_t pinenable){
  _stepPin       = pinstep;
  _dirPin        = pindir;
  _enablePin     = pinenable;
}
void TB6560::inicio(){
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_enablePin, OUTPUT);
}
void TB6560::EnableMotor(){
  digitalWrite(_enablePin, HIGH);
}
void TB6560::DisableMotor(){
  digitalWrite(_enablePin, LOW);
}

void TB6560::Step(){
  digitalWrite(_stepPin, HIGH);
}

void TB6560::RStep(){
  digitalWrite(_stepPin, LOW);
}

void TB6560::Sdir(){
digitalWrite(_dirPin, HIGH);
}

void TB6560::Rdir(){
  digitalWrite(_dirPin, LOW);
}
