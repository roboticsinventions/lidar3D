#ifndef __ROSOV_COMM_H__
#define __ROSOV_COMM_H__

#include <stdbool.h>
#include <stdint.h>

void comm_init(void);

uint8_t comm_get(void);
void comm_parse_vel(uint8_t *msg);

#endif /* __ROSOV_COMM_H__ */
