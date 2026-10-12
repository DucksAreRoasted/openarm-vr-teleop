#pragma once

// Shared IPv4 endpoint for Quest-bridge UDP into the control program.
// OPENARM_QUEST_UDP_HOST defaults to loopback. Set 0.0.0.0 to listen on every
// interface, or a specific address to bind that interface. The bridge sends to
// the same value (--udp-host / the same variable).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <string>

inline std::string quest_udp_host() {
    const char* env = std::getenv("OPENARM_QUEST_UDP_HOST");
    if (env == nullptr) {
        return "127.0.0.1";
    }
    std::string host(env);
    const auto begin = host.find_first_not_of(" \t");
    if (begin == std::string::npos) {
        return "127.0.0.1";
    }
    const auto end = host.find_last_not_of(" \t");
    return host.substr(begin, end - begin + 1);
}

// Binds fd to quest_udp_host():port. On success writes the host that was bound.
inline bool bind_quest_udp(int fd, uint16_t port, std::string* bound_host) {
    const std::string host = quest_udp_host();
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        errno = EINVAL;
        return false;
    }
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        return false;
    }
    if (bound_host != nullptr) {
        *bound_host = host;
    }
    return true;
}
