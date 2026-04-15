#include "TxRx.h"

int RxEth(unsigned char *buffer) {
    static int sockfd = -1;
    static bool initialized = false;
    //printf("init: %d\n", initialized);
    if (!initialized) {
        struct sockaddr_in serv_addr;

        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            perror("Error opening socket");
            return 0;
        }

        int optval = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

        memset((char *) &serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(7777);

        if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
            perror("Error on binding");
            close(sockfd);
            sockfd = -1;
            return 0;
        }

        listen(sockfd, 5);
        initialized = true;
    }
    
    if (sockfd < 0) return 0;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sockfd, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int activity = select(sockfd + 1, &readfds, NULL, NULL, &timeout);

    if (activity <= 0) { 
	    return 0;
    }

    struct sockaddr_in cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    int newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0) {
        perror("Error on accept");
        return 0;
    }
    int n = recv(newsockfd, buffer, BUFFER_SIZE, 0);
    if (n < 0) {
        perror("Error reading from socket");
        close(newsockfd);
        return 0;
    }
    
    /*for (int i = 0; i < BUFFER_SIZE; i++) {
	printf("%02x", buffer[i]);
    }
    printf("\n");*/
    initialized = false;
    close(newsockfd);
    close(sockfd);
    return n;
}
