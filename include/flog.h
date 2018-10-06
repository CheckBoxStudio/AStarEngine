/*******************************************************************************
* flog.h
* A simple logger.
*
* History
* ----------------
* Created. 05 Oct. 2018. WeiQM<weiquanmao@hotmail.com>
*
* <Encodeing in UTF-8>
*******************************************************************************/
#ifndef LOG_UTILITY_H
#define LOG_UTILITY_H

#include <iostream>
#include <fstream>

#include <stdarg.h>
#include <stdio.h>

#define _USE_FLOG_NAMESPACE_

#define _PRINT_TO_CONSOLE_

#ifdef _USE_FLOG_NAMESPACE_
namespace FLOG {
#endif

#define FLOG_DEFAULT_LOG_FILE "../flog.txt"

    class FLog
    {
    public:
        FLog()
            : m_auto_indent(true)
            , m_tabn(2)
            , m_max_stage(10)
            , m_cur_stage(0)
            , m_log_file_path_default(FLOG_DEFAULT_LOG_FILE)
            , m_log_mode("w")
            , m_is_log_file_opened(false)
            , m_log_file(0)
        {
            openLogFile(m_log_file_path_default);
        }
        ~FLog()
        {
            closeLogFile(true);
        }

        void setAutoIndent(const bool bOn) { m_auto_indent = bOn; }
        bool isAutoIndent() { return m_auto_indent; }
        int setTabSize(const int tabn) { return (m_tabn = (tabn >= 0) ? tabn : m_tabn); }
        int getTabSize() { return m_tabn; }
        int setMaxIndentStage(const int maxn) { return (m_max_stage = (maxn >= 0) ? maxn : m_max_stage); }
        int getMaxIndentStage() { return m_max_stage; }
        int getCurrentIndentStage() { return m_cur_stage; }
        int pushIndent() { return (m_cur_stage = (m_cur_stage<m_max_stage) ? ++m_cur_stage : m_max_stage); }
        int popIndent() { return (m_cur_stage = (m_cur_stage>0) ? --m_cur_stage : 0); }

        bool isLogFileOpened() { return m_is_log_file_opened; }
        bool setDefaultLogFile() { return openLogFile(m_log_file_path_default); }
        bool setLogFile(const char *logFile) { return openLogFile(logFile); }
        const char* getLogFile() { return m_log_file_path; }
        void setAppendMode() { sprintf(m_log_mode, "a"); }
        void setWriteMode() { sprintf(m_log_mode, "w"); }

        FLog& operator++ () { pushIndent(); return *this; }
        FLog& operator-- () { popIndent(); return *this; }

        int operator() (const char *format, ...)
        {
            int n1 = 0;

            if (m_auto_indent)
                printIndent();

#ifdef _PRINT_TO_CONSOLE_
            va_list ap1;
            va_start(ap1, format);
            vprintf(format, ap1);
            va_end(ap1);
#endif

            if (!m_is_log_file_opened)
                openLogFile(m_log_file_path_default);

            if (m_is_log_file_opened) {
                va_list ap2;
                va_start(ap2, format);
                n1 = vfprintf(m_log_file, format, ap2);
                va_end(ap2);
                fflush(m_log_file);
            }
            return n1;
        }
        template<typename T> FLog& operator << (const T &a)
        {
            if (isAutoIndent())
                printIndent();
#ifdef _PRINT_TO_CONSOLE_
            std::cout << a;
#endif
            if (m_log_file_c.is_open()) {
                m_log_file_c << a;
                m_log_file_c.flush();
            }
            return *this;
        }
    private:
        void printIndent() {
            if (m_auto_indent && m_is_log_file_opened) {
                char buff[2048];
                memset(buff, 0, 2048);
                for (int i = 0; i < m_tabn*m_cur_stage; ++i) {
                    buff[i] = ' ';
                }
#ifdef _PRINT_TO_CONSOLE_
                printf("%s", buff);
#endif
                fprintf(m_log_file, "%s", buff);
            }
        }
        bool closeLogFile(bool cleanPath)
        {
            bool bRet = true;
            if (m_is_log_file_opened) {
                if (fclose(m_log_file) == 0) {
                    m_log_file = 0;
                    m_log_file_c = std::ofstream(m_log_file);
                    m_is_log_file_opened = false;
                    if (cleanPath)
                        sprintf(m_log_file_path, "");
                }
                else {
                    printf("[FLOG ERROR]: Failed to close log file %s.", m_log_file_path);
                    bRet = false;
                }
            }
            return bRet;
        }
        bool openLogFile(const char *logFile)
        {
            bool bRet = true;
            if (logFile == 0) {
                bRet = closeLogFile(true);
            }
            else {
                if (m_is_log_file_opened && !closeLogFile(false))
                    return false;

                FILE *log_file_temp = fopen(logFile, m_log_mode);
                if (log_file_temp != 0) {
                    m_log_file = log_file_temp;
                    m_log_file_c = std::ofstream(m_log_file);
                    sprintf(m_log_file_path, logFile);
                    m_is_log_file_opened = true;
                    bRet = true;
                }
                else {
                    printf("[FLOG ERROR]: Failed to open log file %s.", m_log_file_path);
                    bRet = false;
                }
            }

            return bRet;
        }
    private:
        bool m_auto_indent;
        int m_tabn;
        int m_max_stage;
        int m_cur_stage;

        const char *m_log_file_path_default;
        char m_log_file_path[1024];
        char m_log_mode[4];
        bool m_is_log_file_opened;
        FILE *m_log_file;
        std::ofstream m_log_file_c;
    };
    static FLog flog;

#ifdef _USE_FLOG_NAMESPACE_
}
#endif

#endif // !LOG_UTILITY_H