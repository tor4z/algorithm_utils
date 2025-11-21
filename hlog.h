#if 0 // examples

#define HLOG_IMPLEMENTATION
#define HLOG_LOG_PATH "."                // optional settings
#define HLOG_WITH_TIME                   // optional settings
#define HLOG_WITH_FILENAME               // optional settings
#define HLOG_MIN_LOG_LEVEL HLOG_INFO     // optional settings
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

#endif // examples

#ifdef __cplusplus
extern "C" {
#endif

#ifndef H_LOG_H_
#define H_LOG_H_

enum hlog_LogLevel
{
    HLOG_TRACE = 0,
    HLOG_INFO,
    HLOG_WARN,
    HLOG_ERROR,
    HLOG_FATAL,
}; // enum hlog_LogLevel

#define hlog(...)                                               \
    hlog_append(__FILE__, __LINE__, __VA_ARGS__)

#define hlog_trace(...)                                         \
    hlog_append(__FILE__, __LINE__, HLOG_TRACE, __VA_ARGS__)

#define hlog_info(...)                                          \
    hlog_append(__FILE__, __LINE__, HLOG_INFO, __VA_ARGS__)

#define hlog_warn(...)                                          \
    hlog_append(__FILE__, __LINE__, HLOG_WARN, __VA_ARGS__)

#define hlog_error(...)                                         \
    hlog_append(__FILE__, __LINE__, HLOG_ERROR, __VA_ARGS__)

#define hlog_fatal(...)                                         \
    hlog_append(__FILE__, __LINE__, HLOG_FATAL, __VA_ARGS__)

void hlog_append(const char* file_path, int line, enum hlog_LogLevel log_level, const char* format, ...);
void hlog_flush_all();

#endif // H_LOG_H_

// #define HLOG_IMPLEMENTATION // delete me

#ifdef HLOG_IMPLEMENTATION
#ifndef HLOG_C_
#define HLOG_C_

#include <assert.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

#ifndef HLOG_BUFF_SIZE
#   define HLOG_BUFF_SIZE 2048
#endif // HLOG_BUFF_SIZE

#ifndef HLOG_LOG_PATH_LEN
#   define HLOG_LOG_PATH_LEN 256
#endif // HLOG_LOG_PATH_LEN

#ifndef HLOG_TIME_STR_LEN
#   define HLOG_TIME_STR_LEN 32
#endif // HLOG_TIME_STR_LEN

#ifndef LOG_MIN_LOG_LEVEL
#   define LOG_MIN_LOG_LEVEL 0
#endif // LOG_MIN_LOG_LEVEL

struct hlog_LogBuff {
    pthread_mutex_t mutex;
    int idx;
    enum hlog_LogLevel log_level;
    char file[HLOG_LOG_PATH_LEN];
    char s[HLOG_BUFF_SIZE];
};

#define HLOG_UNKNOWN_LOG_LEVEL() assert(0 && "Unknown log level")

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

struct hlog_LogBuff* hlog_get_log_buff(enum hlog_LogLevel log_level)
{
    static struct hlog_LogBuff trace_buff = {.idx=0, .log_level=HLOG_TRACE};
    static struct hlog_LogBuff info_buff = {.idx=0, .log_level=HLOG_INFO};
    static struct hlog_LogBuff warn_buff = {.idx=0, .log_level=HLOG_WARN};
    static struct hlog_LogBuff error_buff = {.idx=0, .log_level=HLOG_ERROR};
    static struct hlog_LogBuff fatal_buff = {.idx=0, .log_level=HLOG_FATAL};

#ifdef HLOG_LOG_PATH
    char str_time[HLOG_TIME_STR_LEN];
    time_t raw_time;

#define HLOG_SET_LOG_FILE(log_buff)                                                         \
    if (log_buff.file[0] == '\0') {                                                         \
        time(&raw_time);                                                                    \
        strftime(str_time, HLOG_TIME_STR_LEN, "%Y%m%d_%H%M%S", localtime(&raw_time));       \
        snprintf(log_buff.file, HLOG_LOG_PATH_LEN, "%s/%s-%s-%d.log",                       \
            HLOG_LOG_PATH, hlog_log_level_name(log_buff.log_level), str_time, getpid());    \
    }

    HLOG_SET_LOG_FILE(trace_buff)
    HLOG_SET_LOG_FILE(info_buff)
    HLOG_SET_LOG_FILE(warn_buff)
    HLOG_SET_LOG_FILE(error_buff)
    HLOG_SET_LOG_FILE(fatal_buff)

#undef HLOG_SET_LOG_FILE
#endif // HLOG_LOG_PATH

    switch (log_level) {
    case HLOG_TRACE: return &trace_buff;
    case HLOG_INFO: return &info_buff;
    case HLOG_WARN: return &warn_buff;
    case HLOG_ERROR: return &error_buff;
    case HLOG_FATAL: return &fatal_buff;
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
    if (log_buff->idx <= 0) return;

    FILE* fh = fopen(log_buff->file, "a+");
    if (fh == NULL) {
        assert(0 && "Open log file failed");
        return;
    }

    log_buff->s[log_buff->idx] = '\0';
    fprintf(fh, "%s", log_buff->s);
    fclose(fh);
    log_buff->idx = 0;
#endif // HLOG_LOG_PATH
}

void hlog_flush_all()
{
    hlog_flush(hlog_get_log_buff(HLOG_TRACE));
    hlog_flush(hlog_get_log_buff(HLOG_INFO));
    hlog_flush(hlog_get_log_buff(HLOG_WARN));
    hlog_flush(hlog_get_log_buff(HLOG_ERROR));
    hlog_flush(hlog_get_log_buff(HLOG_FATAL));
}

void hlog_append(const char* file_path, int line, enum hlog_LogLevel log_level, const char* format, ...)
{
    struct hlog_LogBuff* log_buff = hlog_get_log_buff(log_level);
    int n;
    int should_re_append;

    if (log_level < LOG_MIN_LOG_LEVEL && log_level != HLOG_FATAL) {
        return;
    }

    pthread_mutex_lock(&log_buff->mutex);
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

    HLOG_SNPRINTF("[%-5s", hlog_log_level_name(log_level));

#ifdef HLOG_WITH_TIME
    time_t raw_time;
    time(&raw_time);
    char time_str[HLOG_TIME_STR_LEN];
    n = strftime(time_str, HLOG_TIME_STR_LEN, " %y%m%d %H:%M:%S", localtime(&raw_time));
    HLOG_SNPRINTF("%s", time_str);
#endif // HLOG_WITH_TIME

#ifdef HLOG_WITH_FILENAME
    HLOG_SNPRINTF(" %s:%d", hlog_file_name_from_path(file_path), line);
#endif // HLOG_WITH_FILENAME

    HLOG_SNPRINTF("] ");

    va_list args;
    do {
        va_start(args, format);
        int max_len = HLOG_BUFF_SIZE - log_buff->idx;
        n = vsnprintf(log_buff->s + log_buff->idx, max_len, format, args);
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

    if (log_buff->log_level == HLOG_FATAL) {
        hlog_flush_all();
        exit(1);
    }
#undef HLOG_SNPRINTF
    pthread_mutex_unlock(&log_buff->mutex);
}

#endif // HLOG_C_
#endif // HLOG_IMPLEMENTATION

#ifdef __cplusplus
}
#endif
