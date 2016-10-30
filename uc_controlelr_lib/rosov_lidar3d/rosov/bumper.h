#ifndef __ROSOV_BUMPER_H__
#define __ROSOV_BUMPER_H__

#include <stdbool.h>
#include <stdint.h>

void bumper_init(void);
uint16_t bumper_read(void);

#endif /* __ROSOV_BUMPER_H__ */
