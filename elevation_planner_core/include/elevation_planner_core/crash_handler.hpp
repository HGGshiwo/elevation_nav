#pragma once


#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>
#include <execinfo.h>
#include <fcntl.h>
#include <unistd.h>
#include <dlfcn.h>

#include <map>
#include <string>

namespace elevation_planner
{

/**
 * @brief CrashHandler 捕获致命信号（SIGBUS, SIGABRT, SIGSEGV, SIGFPE, SIGILL）以及未捕获的 C++ 异常，
 *        将调用栈和故障原因输出到 stderr，并以覆写（修改非追加）模式写入指定的日志文件。
 *
 * 符号化说明: 本项目的代码几乎都在共享库内 (libelevation_planner_core.so 等),
 * 因此按模块分组收集每帧偏移, 逐模块调用 addr2line —— 共享库帧同样能解出
 * 函数名与文件:行号 (库需带调试信息, 如 RelWithDebInfo/-g 构建)。
 */
class CrashHandler {
   public:
    /**
     * @brief 安装全局崩溃捕获器
     * @param log_path 崩溃日志文件路径，默认为 /tmp/elevation_nav_crash.log。
     *                 采用 O_TRUNC 覆写模式，每次崩溃只保留最新一次记录。
     */
    static void install(const std::string &log_path = "/tmp/elevation_nav_crash.log");

   private:
    static void signalHandler(int sig);
    static void terminateHandler();
    static void writeCallstack(int fd, int sig, const char *extra_msg = nullptr);

    static char log_path_[256];
};



char CrashHandler::log_path_[256] = "/tmp/elevation_nav_crash.log";

void CrashHandler::install(const std::string &log_path) {
    if (!log_path.empty()) {
        std::strncpy(log_path_, log_path.c_str(), sizeof(log_path_) - 1);
        log_path_[sizeof(log_path_) - 1] = '\0';
    }

    // 注册致命系统信号处理器
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = CrashHandler::signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESETHAND; // 触发一次后恢复默认，防止信号处理中死循环

    sigaction(SIGBUS, &sa, nullptr);   // -7: Bus Error (总线错误, 对齐/mmap截断)
    sigaction(SIGABRT, &sa, nullptr);  // -6: Abort (断言失败、堆损坏、std::terminate)
    sigaction(SIGSEGV, &sa, nullptr);  // -11: Segmentation Fault (段错误)
    sigaction(SIGFPE, &sa, nullptr);   // -8: Floating Point Exception (浮点错误)
    sigaction(SIGILL, &sa, nullptr);   // -4: Illegal Instruction (非法指令)

    // 注册 C++ 未捕获异常拦截器 (捕获逃逸异常详细信息，防止直接无声 abort)
    std::set_terminate(CrashHandler::terminateHandler);
}

void CrashHandler::writeCallstack(int fd, int sig, const char *extra_msg) {
    if (fd < 0) return;

    auto safe_write = [fd](const char *str) {
        if (!str) return;
        size_t len = 0;
        while (str[len] != '\0') ++len;
        ssize_t ret = write(fd, str, len);
        (void)ret;
    };

    safe_write("\n=======================================================\n");
    safe_write("[CrashHandler] Process Crashed! Details:\n");

    // 获取并写入时间戳
    time_t now = time(nullptr);
    char time_buf[64];
    struct tm tm_info;
    if (localtime_r(&now, &tm_info)) {
        if (asctime_r(&tm_info, time_buf)) {
            safe_write("Timestamp: ");
            safe_write(time_buf);
        }
    }

    // 写入信号描述
    switch (sig) {
        case SIGBUS:
            safe_write("Signal: SIGBUS (exit code -7, Bus Error / Unaligned Memory / mmap failed)\n");
            break;
        case SIGABRT:
            safe_write("Signal: SIGABRT (exit code -6, Abort / Assert / Heap corruption)\n");
            break;
        case SIGSEGV:
            safe_write("Signal: SIGSEGV (exit code -11, Segmentation Fault / Null pointer)\n");
            break;
        case SIGFPE:
            safe_write("Signal: SIGFPE (exit code -8, Floating Point Exception)\n");
            break;
        case SIGILL:
            safe_write("Signal: SIGILL (exit code -4, Illegal Instruction)\n");
            break;
        default:
            safe_write("Signal: Unknown fatal signal\n");
            break;
    }

    if (extra_msg && extra_msg[0] != '\0') {
        safe_write("Exception Info: ");
        safe_write(extra_msg);
        safe_write("\n");
    }

    void *callstack[128];
    int frames = backtrace(callstack, 128);

    // 先输出原始帧 (binary+offset 形式, 离线也可用 addr2line 还原)
    safe_write("\nBacktrace / Callstack:\n");
    backtrace_symbols_fd(callstack, frames, fd);

    // 分模块收集每帧偏移, 逐模块 addr2line —— 本项目代码在共享库内, 必须按模块
    // 符号化才能覆盖 libelevation_planner_core.so 等关键帧
    std::map<std::string, std::string> module_args;
    Dl_info info;
    for (int i = 0; i < frames; ++i) {
        if (dladdr(callstack[i], &info) && info.dli_fbase && info.dli_fname) {
            char buf[80];
            std::snprintf(buf, sizeof(buf), " 0x%lx",
                          (unsigned long)callstack[i] - (unsigned long)info.dli_fbase);
            module_args[info.dli_fname] += buf;
        }
    }

    for (const auto & m : module_args) {
        std::string cmd = "addr2line -e " + m.first + " -f -C -p" + m.second + " 2>/dev/null";
        FILE *ap = popen(cmd.c_str(), "r");
        if (ap == nullptr) continue;
        safe_write("\nSymbols (addr2line, module: ");
        safe_write(m.first.c_str());
        safe_write("):\n");
        char line[1024];
        while (std::fgets(line, sizeof(line), ap) != nullptr) {
            safe_write(line);
        }
        pclose(ap);
    }
    safe_write("=======================================================\n\n");
}

void CrashHandler::signalHandler(int sig) {
    // 1. 输出到终端标准错误 STDERR
    writeCallstack(STDERR_FILENO, sig);

    // 2. 以覆写方式 (O_TRUNC) 写入专用崩溃日志文件，保留最新一次崩溃信息
    int fd = open(log_path_, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) {
        writeCallstack(fd, sig);
        close(fd);
    }

    // 退出进程，退出码设置为 128 + 信号编号 (POSIX 习惯)
    _exit(128 + sig);
}

void CrashHandler::terminateHandler() {
    char ex_buf[512] = {0};
    try {
        std::exception_ptr e = std::current_exception();
        if (e) {
            std::rethrow_exception(e);
        } else {
            std::strncpy(ex_buf, "std::terminate called without active exception", sizeof(ex_buf) - 1);
        }
    } catch (const std::exception &ex) {
        std::strncpy(ex_buf, ex.what(), sizeof(ex_buf) - 1);
    } catch (...) {
        std::strncpy(ex_buf, "Unknown non-standard C++ exception caught in std::terminate", sizeof(ex_buf) - 1);
    }

    // 输出到 stderr
    writeCallstack(STDERR_FILENO, SIGABRT, ex_buf);

    // 覆写至崩溃日志文件
    int fd = open(log_path_, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) {
        writeCallstack(fd, SIGABRT, ex_buf);
        close(fd);
    }

    _exit(128 + SIGABRT);
}

} // namespace elevation_planner
