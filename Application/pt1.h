/**AI genrated just for testing */
#ifndef PT1_H
#define PT1_H

#include <stdint.h>

typedef struct PT1_Handle{
    float a;
    float b;
    float y_prev;
} PT1_Handle_t;

void PT1_Init(PT1_Handle_t *h, float T1, float K, float Ts);
float PT1_Update(PT1_Handle_t *h, float u);

#endif //PT1_H