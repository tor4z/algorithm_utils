#include <assert.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdio.h>
#include <time.h>

enum hlog_LogLevel
{
    HLOG_TRACE = 0,
    HLOG_INFO,
    HLOG_WARN,
    HLOG_ERROR,
    HLOG_FATAL,
}; // enum hlog_LogLevel

#ifndef HLOG_BUFF_SIZE
#   define HLOG_BUFF_SIZE 16
#endif // HLOG_BUFF_SIZE

struct hlog_LogBuff {
    int idx;
    enum hlog_LogLevel log_level;
    char s[HLOG_BUFF_SIZE];
};

#define HLOG_LOG_PATH "."

#define hlog(...)                \
    hlog_append(__FILE__, __LINE__, __VA_ARGS__)

#define hlog_trace(...)                \
    hlog_append(__FILE__, __LINE__, HLOG_TRACE, __VA_ARGS__)

#define hlog_info(...)                \
    hlog_append(__FILE__, __LINE__, HLOG_INFO, __VA_ARGS__)

#define hlog_warn(...)                \
    hlog_append(__FILE__, __LINE__, HLOG_WARN, __VA_ARGS__)

#define hlog_error(...)                \
    hlog_append(__FILE__, __LINE__, HLOG_ERROR, __VA_ARGS__)

#define hlog_fatal(...)                \
    hlog_append(__FILE__, __LINE__, HLOG_FATAL, __VA_ARGS__)

#define HLOG_UNKNOWN_LOG_LEVEL() assert(0 && "Unknown log level")


struct hlog_LogBuff* hlog_get_log_buff(enum hlog_LogLevel log_level)
{
    static struct hlog_LogBuff trace_buff = {.idx=0, .log_level=HLOG_TRACE};
    static struct hlog_LogBuff info_buff = {.idx=0, .log_level=HLOG_INFO};
    static struct hlog_LogBuff warn_buff = {.idx=0, .log_level=HLOG_WARN};
    static struct hlog_LogBuff error_buff = {.idx=0, .log_level=HLOG_ERROR};
    static struct hlog_LogBuff fatal_buff = {.idx=0, .log_level=HLOG_FATAL};

    switch (log_level) {
    case HLOG_TRACE: return &trace_buff;
    case HLOG_INFO: return &info_buff;
    case HLOG_WARN: return &warn_buff;
    case HLOG_ERROR: return &error_buff;
    case HLOG_FATAL: return &fatal_buff;
    default: HLOG_UNKNOWN_LOG_LEVEL(); return NULL;
    }
}

const char* hlog_log_level_name(enum hlog_LogLevel log_level)
{
    static const char* trace = "TRACE";
    static const char* info = "INFO";
    static const char* warn = "WARN";
    static const char* error = "ERROR";
    static const char* fatal = "FATAL";

    switch (log_level) {
    case HLOG_TRACE: return trace;
    case HLOG_INFO: return info;
    case HLOG_WARN: return warn;
    case HLOG_ERROR: return error;
    case HLOG_FATAL: return fatal;
    default: HLOG_UNKNOWN_LOG_LEVEL(); return NULL;
    }
}

const char* hlog_file_name_from_path(const char* path)
{
    int file_name_start = 0;
    for (int i = 0;; ++i) {
        char c = path[i];
        if (c == '\0') break;
        if (c == '/') file_name_start = i;
    }

    return path + file_name_start + 1;
}

FILE* hlog_stream(enum hlog_LogLevel log_level)
{
    switch (log_level) {
    case HLOG_TRACE: return stdout;
    case HLOG_INFO: return stdout;
    case HLOG_WARN: return stdout;
    case HLOG_ERROR: return stderr;
    case HLOG_FATAL: return stderr;
    default: HLOG_UNKNOWN_LOG_LEVEL(); return NULL;
    }
}

void hlog_flush(struct hlog_LogBuff* log_buff)
{
#ifdef HLOG_LOG_PATH
#define HLOG_LOG_PATH_LEN 256
    char file_path[HLOG_LOG_PATH_LEN];
    snprintf(file_path, HLOG_LOG_PATH_LEN, "%s/%s-%d.log", HLOG_LOG_PATH, hlog_log_level_name(log_buff->log_level), getpid());
    FILE* fh = fopen(file_path, "a+");
    if (fh == NULL) {
        fprintf(stderr, "Log file: %s\n", file_path);
        assert(0 && "Open log file failed");
        return;
    }
    printf("flush\n");
    printf("%s", log_buff->s);
    // fprintf(fh, "%s", log_buff->s);
    fclose(fh);
    log_buff->idx = 0;
#endif // HLOG_LOG_PATH
}

void hlog_append(const char* file_path, int line, enum hlog_LogLevel log_level, const char* format, ...)
{
    struct hlog_LogBuff* log_buff = hlog_get_log_buff(log_level);
    int n;
    int should_re_append;

#define HLOG_SNPRINTF(...)                                                  \
    do {                                                                    \
        int max_len = HLOG_BUFF_SIZE - log_buff->idx;                       \
        n = snprintf(log_buff->s + log_buff->idx, max_len, __VA_ARGS__);    \
        if (n >= max_len) {                                                 \
            hlog_flush(log_buff);                                           \
            should_re_append = 1;                                           \
        } else {                                                            \
            log_buff->idx += n;                                             \
            should_re_append = 0;                                           \
        }                                                                   \
    } while(should_re_append)

    HLOG_SNPRINTF("[%s", hlog_log_level_name(log_level));

#ifdef HLOG_LOG_TIME
    time_t raw_time;
    time(&raw_time);
    char str_time[64]
    n = strftime(str_time, 64, " %y%m%d %H:%M:%S", localtime(&raw_time));
    HLOG_SNPRINTF("%s", str_time);
#endif // HLOG_LOG_TIME

#ifdef HLOG_LOG_FILENAME
    HLOG_SNPRINTF(" %s:%d", hlog_file_name_from_path(file_path), line);
#endif // HLOG_LOG_FILENAME

    HLOG_SNPRINTF("] ");

    va_list args;
    do {
        va_start(args, format);
        int max_len = HLOG_BUFF_SIZE - log_buff->idx;
        n = vsnprintf(log_buff->s + log_buff->idx, max_len, format, args);
        printf("idx: %d, n: %d, maxlen: %d\n", log_buff->idx, n, max_len);
        if (n >= max_len) {
            hlog_flush(log_buff);
            should_re_append = 1;
        } else {
            log_buff->idx += n;
            should_re_append = 0;
        }
        va_end(args);
    } while(should_re_append);

    HLOG_SNPRINTF("\n");

#ifndef HLOG_LOG_PATH
    fprintf(hlog_stream(log_level), "%s", log_buff->s);
    log_buff->idx = 0;
#endif // HLOG_LOG_PATH
}


int main()
{
    hlog(HLOG_INFO, "hello, hlog");
    hlog(HLOG_INFO, "hello, %s", "hlog");
    hlog_info("hello, %s", "hlog");
    return 0;
}
