// Bench test: lists interfaces through getifaddrs, then frees the list many times (run with and without the preload).
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void) {
    struct ifaddrs *list;
    for (int round = 0; round < (getenv("ROUNDS") ? atoi(getenv("ROUNDS")) : 100000); round++) {
        if (getifaddrs(&list) != 0) {
            perror("getifaddrs");
            return 1;
        }
        if (round == 0) {
            for (struct ifaddrs *node = list; node; node = node->ifa_next) {
                if (node->ifa_addr && node->ifa_addr->sa_family == AF_INET) {
                    char text[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &((struct sockaddr_in *)node->ifa_addr)->sin_addr, text, sizeof text);
                    printf("%s %s\n", node->ifa_name, text);
                }
            }
        }
        freeifaddrs(list);
    }
    printf("ok\n");
    return 0;
}
