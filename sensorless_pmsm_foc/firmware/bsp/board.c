#include "board.h"

/* Board facts are declarative (see board.h): this translation unit carries only the
   identity string and any pure lookup tables. Nothing here performs I/O — the code that
   actually drives a pin lives in platforms/<vendor>/. */

const char *board_name(void)
{
    return BOARD_NAME;
}
