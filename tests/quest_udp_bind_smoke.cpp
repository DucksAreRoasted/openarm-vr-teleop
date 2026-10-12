// Binds one Quest UDP socket the same way the control receivers do, then prints
// the address from getsockname. Optional --recv waits for one datagram.
#include <io/quest_udp_endpoint.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    uint16_t port = 0;
    bool wait_recv = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--recv") {
            wait_recv = true;
        } else {
            port = static_cast<uint16_t>(std::stoi(arg));
        }
    }

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "socket failed: " << std::strerror(errno) << std::endl;
        return 1;
    }
    std::string configured;
    if (!bind_quest_udp(fd, port, &configured)) {
        std::cerr << "bind failed: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return 1;
    }

    sockaddr_in actual {};
    socklen_t len = sizeof(actual);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&actual), &len) != 0) {
        std::cerr << "getsockname failed: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return 1;
    }
    char ip[INET_ADDRSTRLEN] = {};
    if (::inet_ntop(AF_INET, &actual.sin_addr, ip, sizeof(ip)) == nullptr) {
        ::close(fd);
        return 1;
    }
    if (configured != ip) {
        std::cerr << "configured " << configured << " != bound " << ip << std::endl;
        ::close(fd);
        return 1;
    }
    std::cout << "bound " << ip << ":" << ntohs(actual.sin_port) << std::endl;
    std::cout.flush();

    if (wait_recv) {
        timeval tv {};
        tv.tv_sec = 2;
        if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
            std::cerr << "setsockopt failed: " << std::strerror(errno) << std::endl;
            ::close(fd);
            return 1;
        }
        char buffer[512];
        const ssize_t nbytes = ::recv(fd, buffer, sizeof(buffer) - 1, 0);
        if (nbytes < 0) {
            std::cerr << "recv failed: " << std::strerror(errno) << std::endl;
            ::close(fd);
            return 1;
        }
        buffer[nbytes] = '\0';
        std::cout << "recv " << buffer << std::endl;
    }
    ::close(fd);
    return 0;
}
