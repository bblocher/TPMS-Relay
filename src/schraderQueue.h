// SchraderQueue.h
#ifndef SCHRADER_QUEUE_H
#define SCHRADER_QUEUE_H

#include "schrader_entry.h"
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include "AM_ESP32Ble.h"

class SchraderQueue {
private:
    static const int MAX_QUEUE_SIZE = 12;
    SchraderEntry receiveQueue[MAX_QUEUE_SIZE];
    int queueSize;
    int maxRetransmissions;
    time_t retransmissionInterval;

public:
    SchraderQueue(int maxRetrans, time_t retransInterval);
    bool addOrUpdateEntry(const uint8_t* rawData);
    bool getNextEntryToRetransmit(uint8_t* byteBuffer, time_t now);
    std::string formatEntry(int index) const;

    int getQueueSize() const;
};

#endif // SCHRADER_QUEUE_H
