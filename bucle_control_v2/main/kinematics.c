#include "kinematics.h"

#include <math.h>
#include <stddef.h>

#define PI_F        3.14159265f
#define DEG_PER_RAD (180.0f / PI_F)
#define RAD_PER_DEG (PI_F / 180.0f)

#define A_MIN_MM     0.0f
#define A_MAX_MM   300.0f
#define BISECT_ITERS  50
#define SCAN_STEPS    64

float kin_solve_theta(float A)
{
    const float rho1 = 316.57f;
    const float rho2 = 120.0f;
    const float rho3 = 200.0f;
    const float rho4 = 400.0f;
    const float rho5 = 221.08f;

    float theta_A = atanf(rho5 / (rho4 - A));
    float theta_B = atanf(rho2 / rho3);

    float a = sqrtf(rho5 * rho5 + (rho4 - A) * (rho4 - A));
    float b = sqrtf(rho2 * rho2 + rho3 * rho3);

    float cos_C = (a * a + b * b - rho1 * rho1) / (2.0f * a * b);
    float theta_C = acosf(cos_C);

    float theta_rad = PI_F - theta_A - theta_B - theta_C;
    return theta_rad * DEG_PER_RAD;
}

float kin_solve_phi(float A)
{
    const float rho3 = 262.5719f;
    const float rho5 = 120.0f * 1.73205081f;
    const float rho6 = 120.0f;
    const float OB   = 165.0094f;
    const float OC   = 240.0f;
    const float OQ   = 120.0f;

    float OA = 422.4261f - A;

    float cos_OB = (OA * OA + OB * OB - rho3 * rho3) / (2.0f * OA * OB);
    float thetaOB = PI_F - acosf(cos_OB);
    float thetaOC = thetaOB - 47.7f * RAD_PER_DEG;

    float Cx = OC * cosf(thetaOC);
    float Cy = OC * sinf(thetaOC);

    float QCx   = Cx - OQ;
    float QCy   = Cy;
    float distQC = sqrtf(QCx * QCx + QCy * QCy);
    float ang_QC = atan2f(QCy, QCx);

    float cos_alpha = (rho6 * rho6 + distQC * distQC - rho5 * rho5)
                      / (2.0f * rho6 * distQC);
    if (cos_alpha < -1.0f) cos_alpha = -1.0f;
    if (cos_alpha >  1.0f) cos_alpha =  1.0f;
    float alpha = acosf(cos_alpha);

    float phi_rad = ang_QC - alpha;
    return phi_rad * DEG_PER_RAD;
}

static float residual(float A, float angle_deg, bool use_phi)
{
    float val = use_phi ? kin_solve_phi(A) : kin_solve_theta(A);
    return val - angle_deg;
}

kin_status_t kin_inverse(float angle_deg, bool use_phi, float *A_out)
{
    float lo = A_MIN_MM, hi = A_MAX_MM;
    float flo = residual(lo, angle_deg, use_phi);
    float fhi = residual(hi, angle_deg, use_phi);

    if (flo * fhi > 0.0f) {
        const float step = (A_MAX_MM - A_MIN_MM) / SCAN_STEPS;
        float prev  = lo;
        float fprev = flo;
        bool  found = false;

        for (int i = 1; i <= SCAN_STEPS; i++) {
            float cur  = A_MIN_MM + (float)i * step;
            float fcur = residual(cur, angle_deg, use_phi);

            if (fprev * fcur <= 0.0f) {
                lo = prev;  flo = fprev;
                hi = cur;
                found = true;
                break;
            }
            prev = cur;  fprev = fcur;
        }

        if (!found) return KIN_ERR_NO_SOLUTION;
    }

    for (int i = 0; i < BISECT_ITERS; i++) {
        float mid  = 0.5f * (lo + hi);
        float fmid = residual(mid, angle_deg, use_phi);

        if (flo * fmid <= 0.0f) {
            hi = mid;
        } else {
            lo  = mid;
            flo = fmid;
        }
    }

    *A_out = 0.5f * (lo + hi);
    return KIN_OK;
}
