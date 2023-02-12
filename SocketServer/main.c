#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include "libSocketFunctions.h"

#define MAX_CONNECTIONS 2

//TODO: make connections not block

int main()
{
    struct sockaddr_in clientAddress;  //to hold the address of incoming clients
    socklen_t clientAddressLen = sizeof(clientAddress);
    int servSocket = 0;
    int newCon = 0;
    int conCount = 0;
    ssize_t bytesReceived = 0;
    char buff[200];
    int recvData = 0;
    int sendData = 0;
    int yes = 1;

    // Start off with room for 5 connections
    // (We'll realloc as necessary)
    //int fd_count = 0;
    //int fd_size = 5;
    struct pollfd *pfds = malloc(sizeof *pfds * MAX_CONNECTIONS);

    servSocket = createSocket();
    setsockopt(servSocket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    bindSocket(servSocket);
    listen(servSocket, MAX_CONNECTIONS);
    setbuf(stdout, NULL); //disable printf buffering
    printf("server on-line and listening...\n");

    // Add the listener to first location in set
    pfds[0].fd = servSocket;
    pfds[0].events = POLLIN; // Report ready to read on incoming connection
    conCount++; //we treat the server listener as the first connection

    while(1) //infinite loop
    {
        int poll_count = poll(pfds, conCount, -1); //block here, and wait indefinitely until a new connection arrives

        if (poll_count == -1) {
            handle_error("poll");
        }

        //loop through all connections to see who is ready to read. handle each one

        //TODO: check if our connection limit is reached
        for(int i = 0; i < conCount; i++)
        {
            if(pfds[i].revents & POLLIN) //check if return value from poll is POLLIN. We got a new connection!!
            {
                //if it's the listener, then we have a new connection arrived
                if (pfds[i].fd == servSocket)
                {
                    if(conCount <= MAX_CONNECTIONS)
                    {
                        //handle new connection by adding it to our pfds array
                        newCon = accept(servSocket, (struct sockaddr*)&clientAddress, &clientAddressLen);
                        if(newCon == -1)
                            perror("accept");
                        else {
                            addToPfds(pfds, newCon, &conCount);
                            printf("New connection %d\nConnection count:%d\n", newCon, conCount);
                        }
                    } else
                    {
                        printf("unable to accept new connections\n");
                    }

                } else {
                    //handle existing connection...
                    memset(buff, '\0', sizeof buff); //clear buffer
                    bytesReceived = recvSocket(pfds[i].fd, buff, sizeof(buff));

                    if(bytesReceived <= 0) { //we must close connection
                        closeConnection(pfds[i].fd);  //close
                        remFromPfds(pfds, i, &conCount); //remove connection from array
                    } else {//END close connection
                        //operate on the data
                        if((recvData = atoi(buff)) == 0) { //a non number was sent
                            printf("Error: Messaged received is invalid. must be a number\n");
                            memset(buff, '\0', sizeof buff); //clear buffer
                            snprintf(buff, sizeof(buff),"%s", "Error: Messaged received is invalid. must be a number\n");
                        } else {  //we have valid data
                            //based on the received data, we set the reply
                            switch(recvData)
                            {
                                case 1:
                                    sendData = 100;
                                    break;
                                case 2:
                                    sendData = 200;
                                    break;
                                case 3:
                                    sendData = 300;
                                    break;
                                default:
                                    sendData = 0;
                                    break;
                            }
                            memset(buff, '\0', sizeof buff); //clear buffer
                            snprintf(buff, sizeof(buff),"%d", sendData);
                        }//END set valid reply

                            sendSocket(pfds[i].fd, buff, sizeof(buff));

                    } //END operate on received data
                } //END handle data from existing connection
            } //END check if any connections have waiting data
        } //END loop each connection
    } //END infinite loop

    return 0;
}
