#include "SpotMemory.h"
#include <EEPROM.h>

long Memory::getMaxSize() {
    return EEPROM.length();
}