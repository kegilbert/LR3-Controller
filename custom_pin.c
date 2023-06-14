/*
 * File:   custom_pin.c
 * Author: kgilbert
 *
 * Created on January 16, 2022, 9:36 PM
 */

#include <xc.h>
#include "custom_pin.h"

void write_reg(tPin pin, uint8_t data) {
    *(pin.port) &= ~pin.mask;
    *(pin.port) |= (data << pin.pos & pin.mask);
}