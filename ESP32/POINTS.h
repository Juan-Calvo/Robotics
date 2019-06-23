#ifndef POINTS_H
#define POINTS_H
      class POINTS{
         public:
            void setx( int xcc );
            void sety( int ycc );
            void setz( int zcc );
            void setw( int wcc );
            void setu( int ucc );
            void sett( int tcc );
            int getx( void );
            int gety( void );
            int getz( void );
            int getw( void );
            int getu( void );
            int gett( void );
            POINTS();
         private:
            int _x;
            int _y;
            int _z;
            int _w;
            int _u;
            int _t;
      };
#endif
