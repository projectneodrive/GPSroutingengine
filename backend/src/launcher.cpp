// Véli launcher: one double-clickable executable that starts everything.
//
// Resolves its own directory first, so it works wherever it is launched from
// (Explorer double-click, a shortcut, another drive). Builds the routing
// backend if it is missing and a compiler is available, preloads the bundled
// map extract when there is one, picks a free port, then starts the server as
// a child process inside a job object so the server can never outlive this
// window. Once the port answers, the default browser is opened on the app.

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <shellapi.h>

#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

const int kPortFirst = 8080;
const int kPortLast = 8099;
const int kStartupTimeoutSec = 180;

std::string exe_dir() {
    char buf[MAX_PATH];
    DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    std::string path(buf, n);
    size_t slash = path.find_last_of("\\/");
    return slash == std::string::npos ? std::string(".")
                                     : path.substr(0, slash);
}

bool file_exists(const std::string& path) {
    DWORD a = GetFileAttributesA(path.c_str());
    return a != INVALID_FILE_ATTRIBUTES && !(a & FILE_ATTRIBUTE_DIRECTORY);
}

bool dir_exists(const std::string& path) {
    DWORD a = GetFileAttributesA(path.c_str());
    return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY);
}

// True when nothing is listening on the loopback port yet.
bool port_free(int port) {
    SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == INVALID_SOCKET)
        return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((unsigned short)port);
    bool ok = bind(s, (sockaddr*)&addr, sizeof addr) == 0;
    closesocket(s);
    return ok;
}

bool port_answers(int port) {
    SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == INVALID_SOCKET)
        return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((unsigned short)port);
    bool ok = connect(s, (sockaddr*)&addr, sizeof addr) == 0;
    closesocket(s);
    return ok;
}

bool have_compiler() {
    return std::system("g++ --version >nul 2>&1") == 0;
}

bool build_backend() {
    printf("  backend/veli.exe not found - compiling it now ...\n");
    fflush(stdout);
    if (!have_compiler()) {
        printf("\n  No C++ compiler found on PATH.\n"
               "  Install MinGW-w64 (or Strawberry Perl, which bundles g++)\n"
               "  and run this launcher again.\n");
        return false;
    }
    int rc = std::system("g++ -O2 -std=c++17 -o backend\\veli.exe "
                         "backend\\src\\main.cpp -lws2_32 -static");
    if (rc != 0 || !file_exists("backend\\veli.exe")) {
        printf("\n  Compilation failed (exit %d).\n", rc);
        return false;
    }
    printf("  compiled backend/veli.exe\n");
    return true;
}

HANDLE g_job = nullptr;

BOOL WINAPI ctrl_handler(DWORD type) {
    if (type == CTRL_C_EVENT || type == CTRL_CLOSE_EVENT ||
        type == CTRL_BREAK_EVENT || type == CTRL_LOGOFF_EVENT ||
        type == CTRL_SHUTDOWN_EVENT) {
        printf("\nStopping Veli ...\n");
        fflush(stdout);
        if (g_job)
            CloseHandle(g_job); // kills the server with the job
        return TRUE;
    }
    return FALSE;
}

}

int main() {
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleTitleA("Veli - routing server");

    std::string dir = exe_dir();
    if (!SetCurrentDirectoryA(dir.c_str())) {
        printf("Cannot enter %s\n", dir.c_str());
        printf("\nPress Enter to close ...");
        getchar();
        return 1;
    }

    printf("Veli - GPS for intermediate human-electric vehicles\n");
    printf("%s\n\n", dir.c_str());

    // The frontend has to be there; the backend can be rebuilt.
    if (!dir_exists("frontend") || !file_exists("frontend\\index.html")) {
        printf("  frontend\\index.html is missing.\n"
               "  Run this launcher from the project folder.\n");
        printf("\nPress Enter to close ...");
        getchar();
        return 1;
    }
    if (!file_exists("backend\\veli.exe") && !build_backend()) {
        printf("\nPress Enter to close ...");
        getchar();
        return 1;
    }

    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        printf("  Windows sockets failed to start.\n");
        printf("\nPress Enter to close ...");
        getchar();
        return 1;
    }

    int port = 0;
    for (int p = kPortFirst; p <= kPortLast; ++p) {
        if (port_free(p)) {
            port = p;
            break;
        }
    }
    if (port == 0) {
        printf("  Ports %d-%d are all busy. Close whatever uses them.\n",
               kPortFirst, kPortLast);
        WSACleanup();
        printf("\nPress Enter to close ...");
        getchar();
        return 1;
    }

    // Preload the bundled extract when present, otherwise the server fetches
    // each area on demand.
    std::string args = "backend\\veli.exe serve";
    bool preloaded = file_exists("data\\map.osm");
    if (preloaded) {
        args += " --map data\\map.osm";
        if (file_exists("data\\elevations.csv"))
            args += " --ele data\\elevations.csv";
    }
    args += " --web frontend --port " + std::to_string(port);

    printf("  map data : %s\n", preloaded
                                    ? "data\\map.osm (preloaded)"
                                    : "downloaded on demand (first route is "
                                      "slower)");
    printf("  port     : %d\n\n", port);
    fflush(stdout);

    // A job object with kill-on-close guarantees the server exits with this
    // window instead of lingering and holding the port.
    g_job = CreateJobObjectA(nullptr, nullptr);
    if (g_job) {
        JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
        limits.BasicLimitInformation.LimitFlags =
            JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
        SetInformationJobObject(g_job, JobObjectExtendedLimitInformation,
                                &limits, sizeof limits);
    }
    SetConsoleCtrlHandler(ctrl_handler, TRUE);

    STARTUPINFOA si{};
    si.cb = sizeof si;
    PROCESS_INFORMATION pi{};
    std::string cmd = args;
    if (!CreateProcessA(nullptr, &cmd[0], nullptr, nullptr, TRUE,
                        CREATE_BREAKAWAY_FROM_JOB, nullptr, nullptr, &si,
                        &pi) &&
        !CreateProcessA(nullptr, &cmd[0], nullptr, nullptr, TRUE, 0, nullptr,
                        nullptr, &si, &pi)) {
        printf("  Could not start backend\\veli.exe (error %lu).\n",
               GetLastError());
        WSACleanup();
        printf("\nPress Enter to close ...");
        getchar();
        return 1;
    }
    if (g_job)
        AssignProcessToJobObject(g_job, pi.hProcess);

    // Wait for the graph to load and the socket to open.
    std::string url = "http://localhost:" + std::to_string(port) + "/";
    bool up = false;
    for (int i = 0; i < kStartupTimeoutSec * 4 && !up; ++i) {
        if (WaitForSingleObject(pi.hProcess, 0) == WAIT_OBJECT_0)
            break; // server gave up before it opened the port
        up = port_answers(port);
        if (!up)
            Sleep(250);
    }

    if (up) {
        printf("\n  Ready. Opening %s\n", url.c_str());
        printf("  Close this window (or press Ctrl-C) to stop Veli.\n\n");
        fflush(stdout);
        ShellExecuteA(nullptr, "open", url.c_str(), nullptr, nullptr,
                      SW_SHOWNORMAL);
    } else {
        printf("\n  The server did not open port %d.\n", port);
        printf("  Its messages are above.\n");
    }

    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD code = 0;
    GetExitCodeProcess(pi.hProcess, &code);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    WSACleanup();

    printf("\nVeli stopped (exit %lu).\n", code);
    printf("Press Enter to close ...");
    getchar();
    return (int)code;
}
