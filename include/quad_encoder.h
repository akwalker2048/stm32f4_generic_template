#ifndef QUAD_ENCODER_H
#define QUAD_ENCODER_H

void quad_encoder_init(void);

void quad_encoder_read_position(uint32_t *position_counts);
void quad_encoder_set_position(uint32_t position_counts);

#endif
