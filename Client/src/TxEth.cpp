#include "TxRx.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cerrno>

//#define SERVER_IP "192.168.0.105" // IP адрес дом
//#define SERVER_IP "10.10.1.211"  // IP адрес работа

void TxEth(unsigned char * buffer) { // TODO Change int to void, or return error codes instead of exit
    int sockfd;
    struct sockaddr_in server_addr;

    printf("TX_ETH EXECS NOW\n");

    // Создание сокета
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        exit(EXIT_FAILURE);
    }

    // Настройка структуры адреса сервера
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(7777);

    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        perror("Invalid address/ Address not supported");
        exit(EXIT_FAILURE);
    }

    // Подключение к серверу
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Connection failed");
        exit(EXIT_FAILURE);
    }
    
    /*for (int i = 0; i < BUFFER_SIZE; ++i) 
        printf("Transmited data: %02X (Char: `%c`, Int: `%d`)\n", buffer[i], buffer[i], buffer[i]);
*/
    // Отправка сообщения на сервер
    send(sockfd, buffer, BUFFER_SIZE, 0);
    close(sockfd);
}


static bool send_all(int fd, const void* p, size_t n) {
    const unsigned char* b = (const unsigned char*)p;
    while (n) {
        ssize_t w = ::send(fd, b, n, 0);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        b += (size_t)w;
        n -= (size_t)w;
    }
    return true;
}

static bool recv_ack(int fd, uint8_t* ack_out) {
    while (true) {
        ssize_t r = ::recv(fd, ack_out, 1, 0);
        if (r == 1) return true;
        if (r == 0) return false;
        if (errno == EINTR) continue;
        return false;
    }
}

bool TxEthN(const unsigned char* data, size_t len, int port) {
    constexpr uint8_t kAckOk = 0x01;
    constexpr int kAckTimeoutMs = 1300; // must cover UART bus wait on receiver

    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) { perror("socket"); return false; }

    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    if (inet_pton(AF_INET, SERVER_IP, &a.sin_addr) != 1) { ::close(s); return false; }

    timeval rcv_to{};
    rcv_to.tv_sec = kAckTimeoutMs / 1000;
    rcv_to.tv_usec = (kAckTimeoutMs % 1000) * 1000;
    (void)::setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &rcv_to, sizeof(rcv_to));

    if (::connect(s, (sockaddr*)&a, sizeof(a)) != 0) { perror("connect"); ::close(s); return false; }

    bool ok = send_all(s, data, len);
    if (!ok) {
        ::close(s);
        return false;
    }

    (void)::shutdown(s, SHUT_WR);

    uint8_t ack = 0;
    ok = recv_ack(s, &ack) && (ack == kAckOk);
    ::close(s);
    return ok;
}
