#pragma once

#include <stdbool.h>

typedef enum {
    KIN_OK = 0,
    KIN_ERR_NO_SOLUTION,
} kin_status_t;

/* Cinemática directa */
float kin_solve_theta(float A);
float kin_solve_phi(float A);

/* Cinemática inversa — bisección */
kin_status_t kin_inverse(float angle_deg, bool use_phi, float *A_out);
