// #define HLOG_LOG_PATH "."
#define HLOG_IMPLEMENTATION
#define HLOG_LOG_TIME
#define HLOG_LOG_FILENAME
#define HLOG_MIN_LOG_LEVEL HLOG_INFO
#include "hlog.h"

int main()
{
    hlog(HLOG_INFO, "12345678910");
    hlog(HLOG_INFO, "11121314151617181920");
    hlog_info("20212223242526272829");
    hlog_trace("-------------------------");
    hlog_flush_all();
    return 0;
}
