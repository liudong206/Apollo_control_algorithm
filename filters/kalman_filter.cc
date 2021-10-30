#include "kalman_filter.h"


void kalman_init(kalman_state *state, double init_x, double init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->C = 1;
    state->q = 0.01;
    state->r = 0.05;
}

double kalman_filter(kalman_state *state, double z_measure)
{
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;

    /* Measurement */
    state->gain = state->p * state->C / (state->p * state->C * state->C + state->r);
    state->x = state->x + state->gain * (z_measure - state->C * state->x);
    state->p = (1 - state->gain * state->C) * state->p;

    return state->x;
}


