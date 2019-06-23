/***********************************************************************
 *   STEPS                                                             *
 ***********************************************************************/
#include "POINTS.h"
POINTS::POINTS(void) {
  _x=0;
  _y=0;
}
void POINTS::setx( int xcc ) {
   _x= xcc;
}
void POINTS::sety( int ycc ) {
   _y= ycc;
}
void POINTS::setz( int zcc ) {
   _z= zcc;
}
void POINTS::setw( int wcc ) {
   _w= wcc;
}
void POINTS::setu( int ucc ) {
   _u= ucc;
}
void POINTS::sett( int tcc ) {
   _t= tcc;
}
int POINTS::getx( void ) {
   return _x;
}
int POINTS::gety( void ) {
   return _y;
}
int POINTS::getz( void ) {
   return _z;
}
int POINTS::getw( void ) {
   return _w;
}
int POINTS::getu( void ) {
   return _u;
}
int POINTS::gett( void ) {
   return _t;
}
