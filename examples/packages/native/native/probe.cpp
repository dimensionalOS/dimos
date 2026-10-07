// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
// Minimal real native process: no SDK/toolchain downloads or robot dependencies.
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

static volatile std::sig_atomic_t stopping = 0;
static void stop(int) { stopping = 1; }

int main(int argc, char** argv) {
    std::string resource;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (std::string(argv[i]) == "--message_file") resource = argv[i + 1];
        else return 2;
    }
    std::ifstream input(resource);
    std::string message;
    if (!std::getline(input, message)) return 3;
    const char* report = std::getenv("DIMOS_PACKAGE_REPORT");
    if (!report) return 4;
    std::signal(SIGTERM, stop);
    std::signal(SIGINT, stop);
    {
        std::ofstream output(report);
        output << "ready " << getpid() << " " << message << std::endl;
        if (!output) return 5;
    }
    std::cout << message << std::endl;
    while (!stopping) usleep(10000);
    std::ofstream(report, std::ios::app) << "stopped" << std::endl;
    return 0;
}
