// LD_PRELOAD shim: getifaddrs() leaves out the interface named in HIDE_IFACE (default tailscale0).
// ctrlmcu_node exits at start if any non-lo IPv4 address ends in .104/.106, and AOS's tailscale0 is 100.106.x.106.
// Build: gcc -O2 -shared -fPIC -o hide_tailscale.so hide_tailscale.c -ldl
#define _GNU_SOURCE
#include <dlfcn.h>
#include <ifaddrs.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

enum { MAX_LISTS = 64 };

// glibc frees a getifaddrs list as one block from its first node, so remember each list's real head.
static struct {
    struct ifaddrs *shown;
    struct ifaddrs *real;
} lists[MAX_LISTS];
static pthread_mutex_t lists_lock = PTHREAD_MUTEX_INITIALIZER;

static const char *hidden_name(void) {
    const char *name = getenv("HIDE_IFACE");
    return name && *name ? name : "tailscale0";
}

int getifaddrs(struct ifaddrs **out) {
    static int (*real_getifaddrs)(struct ifaddrs **);
    if (!real_getifaddrs) {
        real_getifaddrs = (int (*)(struct ifaddrs **))dlsym(RTLD_NEXT, "getifaddrs");
    }
    struct ifaddrs *real = NULL;
    int result = real_getifaddrs(&real);
    if (result != 0) {
        return result;
    }
    const char *hide = hidden_name();
    // Unlink hidden nodes; the first kept node becomes the head the caller sees.
    struct ifaddrs *shown = NULL, **tail = &shown;
    for (struct ifaddrs *node = real; node; node = node->ifa_next) {
        if (node->ifa_name && strcmp(node->ifa_name, hide) == 0) {
            continue;
        }
        *tail = node;
        tail = &node->ifa_next;
    }
    *tail = NULL;
    if (shown != real) {
        int saved = 0;
        pthread_mutex_lock(&lists_lock);
        for (int i = 0; i < MAX_LISTS && !saved; i++) {
            if (!lists[i].real) {
                lists[i].shown = shown;
                lists[i].real = real;
                saved = 1;
            }
        }
        pthread_mutex_unlock(&lists_lock);
        // No slot to remember the real head: keep it in the list so freeifaddrs still gets the block start.
        if (!saved) {
            real->ifa_next = shown;
            shown = real;
        }
    }
    *out = shown;
    return 0;
}

void freeifaddrs(struct ifaddrs *list) {
    static void (*real_freeifaddrs)(struct ifaddrs *);
    if (!real_freeifaddrs) {
        real_freeifaddrs = (void (*)(struct ifaddrs *))dlsym(RTLD_NEXT, "freeifaddrs");
    }
    struct ifaddrs *real = list;
    pthread_mutex_lock(&lists_lock);
    for (int i = 0; i < MAX_LISTS; i++) {
        if (lists[i].real && lists[i].shown == list) {
            real = lists[i].real;
            lists[i].real = NULL;
            lists[i].shown = NULL;
            break;
        }
    }
    pthread_mutex_unlock(&lists_lock);
    real_freeifaddrs(real);
}
