#include "telem.h"

void telem_init(){
#if TELEM_TYPE==TTL_xG
	TTL_xG_init();
//#elif
#endif
}


