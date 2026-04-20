#pragma once

#include "vex.h"

namespace bot {
    class piston {
        public:
        	piston(vex::digital_out& piston1, vex::digital_out& piston2) : _piston1(piston1), _piston2(piston2) {}
            inline bool set(bool value) {
                if (_piston2.value() && value) {return false;}
                else _piston1.set(value);
                return true;
            }

            inline void toggle() {
                this->set(!_piston1.value());
            }   
        private:
        	vex::digital_out& _piston1;
        	vex::digital_out& _piston2;

    };

    class piston_group {
        public:
            piston_group( vex::digital_out &p1, vex::digital_out &p2 ) : p1(p1), p2(p2) {};
            inline void set( bool value ) {
                p1.set(value);
                p2.set(value);
            };
            inline void set( int value ) {
                this->set( value != 0 );
            };
            inline bool value() {return p1.value() && p2.value();};
            inline void toggle() {
                this->set( !this->value() );
            };
        private:
            vex::digital_out &p1;
            vex::digital_out &p2;
    };
}