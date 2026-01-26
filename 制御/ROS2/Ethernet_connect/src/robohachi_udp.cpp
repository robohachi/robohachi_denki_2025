#include <cstdint>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <fcntl.h>
#include "robohachi_udp.hpp"

Ros2UDP::Ros2UDP(const std::string& address, int port){
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return;
    }
    
    int yes = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
        perror("setsockopt(SO_REUSEADDR) failed");
    }
    
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    
    int bufsize = 104;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
    
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    
    if (address == "0.0.0.0") {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        addr.sin_addr.s_addr = inet_addr(address.c_str());
    }
    
    addr.sin_port = htons(port);
}

void Ros2UDP::udp_bind(){
    if (bind(sock, (const struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
    } else {
        printf("Successfully bound to %s:%d\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    }
}

void Ros2UDP::send_packet(uint8_t *packet, uint8_t size){
    if (sendto(sock, packet, size, 0, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Sendto failed");
    }
}

ssize_t Ros2UDP::udp_recv(uint8_t *buf, uint8_t size){
    memset(buf, 0, size);
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    
    ssize_t len = recvfrom(sock, buf, size, 0, (struct sockaddr *)&sender_addr, &sender_len);
    return len;
}
