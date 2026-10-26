#include "can_transport.h"

uint8_t CanTransport::interrupts() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    const uint8_t flags = controller_.getInterrupts();
    xSemaphoreGive(mutex_);
    return flags;
}

bool CanTransport::receive(MCP2515::RXBn buffer, can_frame& frame) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    // readMessage copies the frame before releasing this RX buffer.
    const bool received = controller_.readMessage(buffer, &frame) == MCP2515::ERROR_OK;
    xSemaphoreGive(mutex_);
    return received;
}

bool CanTransport::send(const can_frame& frame) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    const bool sent = controller_.sendMessage(&frame) == MCP2515::ERROR_OK;
    xSemaphoreGive(mutex_);
    return sent;
}
