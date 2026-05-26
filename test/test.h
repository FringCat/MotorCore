#ifndef TEST_H_
#define TEST_H_

#ifndef TEST_LOG_DELAY_MS
#define TEST_LOG_DELAY_MS  1u
#endif

#define USE_HAL_DRIVER 1
/** 运行 my_* 数学单元测试，返回失败条数 */
int test_run_all_math(void);

#endif
