/**
 * @file test.c
 * @brief FOC my_* 单元测试（对照 math.h）
 * @see FOC/test.md
 */

#include "test.h"
#include "foc_alg.h"
#include "SEGGER_RTT.h"

#include <math.h>
#include <stdint.h>

#if defined(USE_HAL_DRIVER)
#include "main.h"
#endif

extern float my_sat(float e, float r);
extern int32_t my_fast_round(float x);

#define EPS      1e-5f
#define EPS_TRIG 5e-3f
#define EPS_POW  1e-4f
#define REL_POW  1e-3f

static int g_pass;
static int g_fail;

static void test_delay(void)
{
#if defined(USE_HAL_DRIVER)
    HAL_Delay(TEST_LOG_DELAY_MS);
#else
    volatile uint32_t c = (uint32_t)TEST_LOG_DELAY_MS * 8000u;
    while (c-- != 0u)
    {
    }
#endif
}

#define LOG(...)                    \
    do                              \
    {                               \
        SEGGER_RTT_printf(0, __VA_ARGS__); \
        test_delay();               \
    } while (0)

#define SEC(title)  LOG("\n--- %s ---\n", title)

/* SEGGER_RTT_printf 仅可靠支持 %s / %f，数值统一用 double + %f */
static void check_f(const char *id, float actual, float reference, float tolerance)
{
    const float absolute_error = fabsf(actual - reference);

    if (absolute_error <= tolerance)
    {
        g_pass++;
        LOG("PASSED | test case %s | actual=%f | reference=%f | "
            "absolute_error=%f | tolerance=%f\n",
            id, (double)actual, (double)reference, (double)absolute_error,
            (double)tolerance);
    }
    else
    {
        g_fail++;
        LOG("FAILED | test case %s | actual=%f | reference=%f | "
            "absolute_error=%f | tolerance=%f\n",
            id, (double)actual, (double)reference, (double)absolute_error,
            (double)tolerance);
    }
}

static void check_i(const char *id, int32_t actual, int32_t reference)
{
    if (actual == reference)
    {
        g_pass++;
        LOG("PASSED | test case %s | actual_integer=%f | reference_integer=%f\n",
            id, (double)actual, (double)reference);
    }
    else
    {
        g_fail++;
        LOG("FAILED | test case %s | actual_integer=%f | reference_integer=%f\n",
            id, (double)actual, (double)reference);
    }
}

#define CF(id, got_expr, ref_expr, tol)          \
    do                                           \
    {                                            \
        const float _g = (got_expr);             \
        const float _r = (ref_expr);             \
        check_f(id, _g, _r, tol);                \
    } while (0)

#define CI(id, got_expr, ref_expr)               \
    check_i(id, (int32_t)(got_expr), (int32_t)(ref_expr))

static void check_in_range(const char *id, float value, float low, float high)
{
    if (value >= low && value < high)
    {
        g_pass++;
        LOG("PASSED | test case %s | value=%f in range [%f, %f)\n",
            id, (double)value, (double)low, (double)high);
    }
    else
    {
        g_fail++;
        LOG("FAILED | test case %s | value=%f not in range [%f, %f)\n",
            id, (double)value, (double)low, (double)high);
    }
}

static float tol_pow(float ref)
{
    const float t = EPS_POW;
    const float r = REL_POW * fabsf(ref);
    return (r > t) ? r : t;
}

int test_run_all_math(void)
{
    g_pass = 0;
    g_fail = 0;

    LOG("======== FOC math unit test (versus standard math.h library) ========\n");

    SEC("absolute value: my_abs");
    CF("ABS-01", my_abs(0.0f), 0.0f, EPS);
    CF("ABS-02", my_abs(3.5f), fabsf(3.5f), EPS);
    CF("ABS-03", my_abs(-3.5f), fabsf(-3.5f), EPS);
    CF("ABS-04", my_abs(1e-30f), fabsf(1e-30f), EPS);
    CF("ABS-05", my_abs(-1e30f), fabsf(-1e30f), EPS);

    SEC("sign function: my_sgn");
    CF("SGN-01", my_sgn(1.0f), 1.0f, EPS);
    CF("SGN-02", my_sgn(-1.0f), -1.0f, EPS);
    CF("SGN-03", my_sgn(0.0f), 0.0f, EPS);

    SEC("saturation: my_sat");
    CF("SAT-01", my_sat(0.5f, 1.0f), 0.5f, EPS);
    CF("SAT-02", my_sat(2.0f, 1.0f), 1.0f, EPS);
    CF("SAT-03", my_sat(-2.0f, 1.0f), -1.0f, EPS);

    SEC("limiter: my_Limit");
    CF("LIM-01", my_Limit(0.5f, 1.0f, 0.0f), 0.5f, EPS);
    CF("LIM-02", my_Limit(2.0f, 1.0f, 0.0f), 1.0f, EPS);
    CF("LIM-03", my_Limit(-1.0f, 1.0f, 0.0f), 0.0f, EPS);

    SEC("linear map: my_map");
    CF("MAP-01", my_map(0.0f, -1.0f, 1.0f, 0.0f, 1.0f), 0.5f, EPS);
    CF("MAP-02", my_map(1.0f, -1.0f, 1.0f, 0.0f, 1.0f), 1.0f, EPS);
    CF("MAP-03", my_map(0.5f, 0.0f, 10.0f, 0.0f, 100.0f), 5.0f, EPS);

    SEC("round: my_round / my_fast_round");
    CF("RND-01", my_round(2.3f), roundf(2.3f), EPS);
    CF("RND-02", my_round(2.5f), roundf(2.5f), EPS);
    CF("RND-03", my_round(-2.5f), roundf(-2.5f), EPS);
    CI("RND-04", my_fast_round(2.49f), 2);
    CI("RND-05", my_fast_round(-2.51f), -3);
    CF("RND-06", my_round(-2.3f), roundf(-2.3f), EPS);
    CF("RND-07", my_round(0.0f), roundf(0.0f), EPS);
    CF("RND-08", my_round(1.4999999e6f), roundf(1.4999999e6f), EPS);
    CI("RND-09", my_fast_round(3.14f), (int32_t)my_round(3.14f));
    CI("RND-10", my_fast_round(-7.6f), (int32_t)my_round(-7.6f));
    CI("RND-11", my_fast_round(0.0f), (int32_t)my_round(0.0f));

    SEC("floor: my_floor");
    CF("FLR-01", my_floor(2.9f), floorf(2.9f), EPS);
    CF("FLR-02", my_floor(-2.1f), floorf(-2.1f), EPS);
    CF("FLR-03", my_floor(3.0f), floorf(3.0f), EPS);
    CF("FLR-04", my_floor(-3.0f), floorf(-3.0f), EPS);
    CF("FLR-05", my_floor(0.0f), floorf(0.0f), EPS);

    SEC("floating remainder: my_fmodf");
    CF("FMOD-01", my_fmodf(5.5f, 2.0f), fmodf(5.5f, 2.0f), EPS);
    CF("FMOD-02", my_fmodf(-5.5f, 2.0f), fmodf(-5.5f, 2.0f), EPS);
    CF("FMOD-03", my_fmodf(3.0f, 0.0f), 0.0f, EPS);
    {
        const float x = 0.8f;
        const float y = 2.0f * PI;
        const float got = my_fmodf(x, y);

        CF("FMOD-04", got, fmodf(x, y), EPS);
        check_in_range("FMOD-04-RANGE", got, 0.0f, y);
    }

    SEC("power: my_pow");
    CF("POW-01", my_pow(10.0f, 3.0f), powf(10.0f, 3.0f), EPS);
    CF("POW-02", my_pow(2.0f, 10.0f), powf(2.0f, 10.0f), EPS);
    CF("POW-03", my_pow(5.0f, 0.0f), 1.0f, EPS);
    CF("POW-04", my_pow(2.0f, -2.0f), 0.25f, EPS);
    {
        const float ref = powf(10.0f, 2.5f);
        CF("POW-05", my_pow(10.0f, 2.5f), ref, tol_pow(ref));
    }
    CF("POW-06", my_pow(-2.0f, 0.5f), 0.0f, EPS);
    CF("POW-07", my_pow(0.0f, -1.0f), 1.0f, EPS);
    {
        const float ref08 = powf(0.5f, 3.0f);
        CF("POW-08", my_pow(0.5f, 3.0f), ref08, tol_pow(ref08));
    }
    {
        const float r = 0.2f;
        const float n = 5.0f;
        const float ref09 = powf(1.0f - r, n);
        CF("POW-09", my_pow(1.0f - r, n), ref09, tol_pow(ref09));
    }

    SEC("sine and cosine: my_sin / my_cos");
    CF("SIN-01", my_sin(0.0f), sinf(0.0f), EPS);
    CF("SIN-02", my_sin(PI_2), sinf(PI_2), EPS_TRIG);
    CF("SIN-03", my_sin(PI), sinf(PI), EPS_TRIG);
    CF("COS-01", my_cos(0.0f), cosf(0.0f), EPS_TRIG);
    CF("COS-02", my_cos(PI_2), cosf(PI_2), EPS_TRIG);
    CF("COS-03", my_cos(PI), cosf(PI), EPS_TRIG);

    {
        const float spot[] = {0.1f, 1.2f, 3.5f};
        float max_e = 0.0f;
        float sum_e = 0.0f;
        uint32_t n = 0u;
        uint32_t i;

        for (i = 0; i < 3u; i++)
        {
            const float x = Limit_angle_el(spot[i]);
            const float es = fabsf(my_sin(x) - sinf(x));
            const float ec = fabsf(my_cos(x) - cosf(x));

            LOG("  sample_point index=%f angle=%f sine_error=%f cosine_error=%f\n",
                (double)i, (double)x, (double)es, (double)ec);

            if (es > max_e)
            {
                max_e = es;
            }
            if (ec > max_e)
            {
                max_e = ec;
            }
            sum_e += es + ec;
            n += 2u;
        }
        for (i = 0; i < 360u; i++)
        {
            const float x = Limit_angle_el((float)i * _2PI / 360.0f);
            const float es = fabsf(my_sin(x) - sinf(x));
            const float ec = fabsf(my_cos(x) - cosf(x));

            if (es > max_e)
            {
                max_e = es;
            }
            if (ec > max_e)
            {
                max_e = ec;
            }
            sum_e += es + ec;
            n += 2u;
        }
        LOG("  sweep_statistics sample_count=%f maximum_error=%f "
            "mean_error=%f tolerance=%f\n",
            (double)n, (double)max_e, (double)(sum_e / (float)n), (double)EPS_TRIG);
        CF("TRIG-SWEEP", max_e, 0.0f, EPS_TRIG);
    }

    SEC("decimal round: my_round_to_decimal");
    CF("DEC-01", my_round_to_decimal(3.14159f, 2), 3.14f, EPS);
    CF("DEC-02", my_round_to_decimal(3.145f, 2), 3.15f, EPS);
    CF("DEC-03", my_round_to_decimal(1.23f, -1), 1.23f, EPS);
    CF("DEC-04", my_round_to_decimal(0.0f, 3), 0.0f, EPS);

    LOG("\n======== summary: passed %f, failed %f, total %f ========\n",
        (double)g_pass, (double)g_fail, (double)(g_pass + g_fail));
    return g_fail;
}


