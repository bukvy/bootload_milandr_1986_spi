#ifndef BLOCK_H_
#define BLOCK_H_

#include "config.h"

extern void block_state_init(void);
extern void block_state_deinit(void);

extern void block_state_int_handle();

extern void block_state_set_int(uint8_t block);
extern uint8_t block_state_get_int(void);
extern uint8_t block_state_get_ext(void);

#endif /* BLOCK_H_ */
