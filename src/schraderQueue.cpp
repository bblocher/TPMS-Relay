#include "schraderQueue.h"

// Constructor
SchraderQueue::SchraderQueue(int maxRetrans, time_t retransInterval)
    : maxRetransmissions(maxRetrans),
      retransmissionInterval(retransInterval),
      queueSize(0) {
    memset(receiveQueue, 0, sizeof(receiveQueue));
}

// Add or update an entry in the queue
bool SchraderQueue::addOrUpdateEntry(const uint8_t* rawData) {
    time_t now = time(NULL);

    // Decode the fields from the raw data
    uint32_t id = decode_id(&rawData[4]);
    uint16_t pressure = decode_pressure(rawData[7]);
    int8_t temperature = decode_temperature(rawData[8]);
    uint32_t flags = (rawData[0] << 24) | (rawData[1] << 16) | (rawData[2] << 8) | rawData[3];

    // Search for an existing entry with the same ID
    for (int i = 0; i < queueSize; i++) {
        if (decode_id(receiveQueue[i].id) == id) {
            memcpy(receiveQueue[i].id, &rawData[4], 3);
            memcpy(receiveQueue[i].flags, rawData, 4);
            receiveQueue[i].pressure_raw = rawData[7];
            receiveQueue[i].temperature_raw = rawData[8];
            receiveQueue[i].mic = rawData[9];
            receiveQueue[i].timestamp = now + retransmissionInterval;
            receiveQueue[i].retransmit_count = 0;
            return true;
        }
    }

    // Add a new entry if there's space
    if (queueSize < MAX_QUEUE_SIZE) {
        memcpy(receiveQueue[queueSize].id, &rawData[4], 3);
        memcpy(receiveQueue[queueSize].flags, rawData, 4);
        receiveQueue[queueSize].pressure_raw = rawData[7];
        receiveQueue[queueSize].temperature_raw = rawData[8];
        receiveQueue[queueSize].mic = rawData[9];
        receiveQueue[queueSize].timestamp = now + retransmissionInterval;
        receiveQueue[queueSize].retransmit_count = 0;
        queueSize++;
        return true;
    }

    return false; // Queue is full
}

// Get the next entry to retransmit
bool SchraderQueue::getNextEntryToRetransmit(uint8_t* byteBuffer, time_t now) {
    uint8_t syncWord[] = {0x0, 0x0, 0x0, 0x0, 0x0};

    for (int i = 0; i < queueSize; i++) {
        if (receiveQueue[i].timestamp < now) {
            memcpy(byteBuffer, syncWord, 5);
            memcpy(byteBuffer + 5, receiveQueue[i].flags, 4);
            memcpy(byteBuffer + 9, receiveQueue[i].id, 3);
            byteBuffer[12] = receiveQueue[i].pressure_raw;
            byteBuffer[13] = receiveQueue[i].temperature_raw;
            byteBuffer[14] = receiveQueue[i].mic;

            receiveQueue[i].retransmit_count++;
            receiveQueue[i].timestamp = now + retransmissionInterval;

            if (receiveQueue[i].retransmit_count > maxRetransmissions) {
                for (int j = i; j < queueSize - 1; j++) {
                    receiveQueue[j] = receiveQueue[j + 1];
                }
                queueSize--;
            }

            return true;
        }
    }

    return false;
}

// Print the queue contents (for debugging)
std::string SchraderQueue::formatEntry(int index) const {
    if (index < 0 || index >= queueSize) {
        return "Invalid index";
    }
    char buffer[256];
    snprintf(buffer, sizeof(buffer),
             "%06X | %ld | %d",
             decode_id(receiveQueue[index].id),
             receiveQueue[index].timestamp - millis(),
             receiveQueue[index].retransmit_count);
    return std::string(buffer);
}

// Get the current queue size
int SchraderQueue::getQueueSize() const {
    return queueSize;
}