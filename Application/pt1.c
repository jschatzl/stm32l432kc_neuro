/**AI genrated just for testing */
#include "pt1.h"

void PT1_Init(PT1_Handle_t *h, float T1, float K, float Ts) {
    float denom = T1 + Ts;
    h->a = T1 / denom;
    h->b = (K * Ts) / denom;
    h->y_prev = 0.0f;
}

float PT1_Update(PT1_Handle_t *h, float u) {
    float y = h->a * h->y_prev + h->b * u;
    h->y_prev = y;
    return y;
}