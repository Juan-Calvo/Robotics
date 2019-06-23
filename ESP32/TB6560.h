#ifndef TB6560_H
#define TB6560_H
  class TB6560{
    private:
      uint16_t _stepPin;
      uint16_t _dirPin;
      uint16_t _enablePin;
  
    public:
      TB6560(uint16_t pinstep,uint16_t pindir, uint16_t pinenable);
      void inicio();
      void EnableMotor();
      void DisableMotor();
      void Step();
      void RStep();
      void Sdir();
      void Rdir();
  };
#endif
