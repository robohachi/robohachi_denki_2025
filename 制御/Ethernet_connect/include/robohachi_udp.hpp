#ifndef ROBOHACHI_UDP_HPP
#define ROBOHACHI_UDP_HPP

#include <cstdint>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>

class Ros2UDP {
public:
    // コンストラクタ
    Ros2UDP(const std::string& address, int port);
    
    // ソケットにアドレスをバインド
    void udp_bind();
    
    // パケットを送信
    void send_packet(uint8_t *packet, uint8_t size);
    
    // パケットを受信
    ssize_t udp_recv(uint8_t *buf, uint8_t size);
    
private:
    int sock;                  // ソケットディスクリプタ
    struct sockaddr_in addr;   // アドレス情報
};

#endif // ROBOHACHI_UDP_HPP

