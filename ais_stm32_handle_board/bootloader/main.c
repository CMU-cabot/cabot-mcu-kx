#include "stm32f303x8.h"
#include "protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FLASH_BASE_ADDRESS 0x08000000UL
#define APP_ADDRESS 0x08002000UL
#define APP_MAX_SIZE 0x0000D800UL
#define METADATA_ADDRESS 0x0800F800UL
#define FLASH_PAGE_SIZE 2048U
#define APP_PAGE_COUNT 27U
#define SRAM_START 0x20000000UL
#define SRAM_END 0x20003000UL
#define BOOT_MAGIC 0x314C4243UL
#define METADATA_FORMAT 1U

#define CPU_HZ 8000000UL
#define MS_CYCLES (CPU_HZ / 1000UL)
#define BOOT_WINDOW_CYCLES (500UL * MS_CYCLES)
#define UPDATE_TIMEOUT_CYCLES (5000UL * MS_CYCLES)

#define MCP_RESET 0xC0U
#define MCP_READ 0x03U
#define MCP_WRITE 0x02U
#define MCP_BIT_MODIFY 0x05U
#define MCP_RTS_TX0 0x81U
#define MCP_CANSTAT 0x0EU
#define MCP_CANCTRL 0x0FU
#define MCP_CNF3 0x28U
#define MCP_CNF2 0x29U
#define MCP_CNF1 0x2AU
#define MCP_CANINTE 0x2BU
#define MCP_CANINTF 0x2CU
#define MCP_TXB0CTRL 0x30U
#define MCP_TXB0SIDH 0x31U
#define MCP_RXB0CTRL 0x60U
#define MCP_RXB0SIDH 0x61U
#define MCP_RXB1CTRL 0x70U
#define MCP_RXB1SIDH 0x71U

typedef struct {
  uint32_t magic;
  uint16_t format;
  uint16_t reserved;
  uint32_t image_size;
  uint32_t image_crc32;
  uint32_t metadata_crc32;
} image_metadata_t;

_Static_assert(sizeof(image_metadata_t) == 20U, "metadata wire layout changed");

typedef struct {
  uint16_t id;
  uint8_t dlc;
  uint8_t data[8];
} can_frame_t;

static uint8_t page_buffer[FLASH_PAGE_SIZE];
static uint32_t image_size;
static uint32_t image_crc32;
static uint32_t pending_size;
static uint32_t page_crc32;
static uint32_t last_activity;
static uint16_t expected_offset;
static uint16_t page_length;
static uint8_t page_index;
static uint8_t next_page;
static uint8_t state;
static bool pending_info;
static bool stay_in_bootloader;

static uint32_t cycles_now(void) {
  return DWT->CYCCNT;
}

static bool elapsed(uint32_t start, uint32_t duration) {
  return (uint32_t)(cycles_now() - start) >= duration;
}

static void delay_cycles(uint32_t duration) {
  uint32_t start = cycles_now();
  while (!elapsed(start, duration)) {
  }
}

static uint32_t read_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t crc32_buffer(const uint8_t *data, uint32_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  while (length-- != 0U) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

static void spi_select(bool selected) {
  if (selected) {
    GPIOA->BRR = GPIO_BRR_BR_15;
  } else {
    GPIOA->BSRR = GPIO_BSRR_BS_15;
  }
}

static uint8_t spi_transfer(uint8_t value) {
  while ((SPI1->SR & SPI_SR_TXE) == 0U) {
  }
  *(__IO uint8_t *)&SPI1->DR = value;
  while ((SPI1->SR & SPI_SR_RXNE) == 0U) {
  }
  return *(__IO uint8_t *)&SPI1->DR;
}

static uint8_t mcp_read(uint8_t address) {
  uint8_t value;
  spi_select(true);
  spi_transfer(MCP_READ);
  spi_transfer(address);
  value = spi_transfer(0xFFU);
  spi_select(false);
  return value;
}

static void mcp_read_many(uint8_t address, uint8_t *data, uint8_t length) {
  spi_select(true);
  spi_transfer(MCP_READ);
  spi_transfer(address);
  while (length-- != 0U) {
    *data++ = spi_transfer(0xFFU);
  }
  spi_select(false);
}

static void mcp_write(uint8_t address, uint8_t value) {
  spi_select(true);
  spi_transfer(MCP_WRITE);
  spi_transfer(address);
  spi_transfer(value);
  spi_select(false);
}

static void mcp_write_many(uint8_t address, const uint8_t *data, uint8_t length) {
  spi_select(true);
  spi_transfer(MCP_WRITE);
  spi_transfer(address);
  while (length-- != 0U) {
    spi_transfer(*data++);
  }
  spi_select(false);
}

static void mcp_modify(uint8_t address, uint8_t mask, uint8_t value) {
  spi_select(true);
  spi_transfer(MCP_BIT_MODIFY);
  spi_transfer(address);
  spi_transfer(mask);
  spi_transfer(value);
  spi_select(false);
}

static void encode_standard_id(uint16_t id, uint8_t encoded[4]) {
  encoded[0] = (uint8_t)(id >> 3);
  encoded[1] = (uint8_t)(id << 5);
  encoded[2] = 0U;
  encoded[3] = 0U;
}

static void mcp_set_filter(uint8_t address, uint16_t id) {
  uint8_t encoded[4];
  encode_standard_id(id, encoded);
  mcp_write_many(address, encoded, 4U);
}

static bool mcp_init(void) {
  static const uint8_t filter_addresses[6] = {0x00U, 0x04U, 0x08U,
                                               0x10U, 0x14U, 0x18U};
  spi_select(true);
  spi_transfer(MCP_RESET);
  spi_select(false);
  delay_cycles(10U * MS_CYCLES);

  if ((mcp_read(MCP_CANSTAT) & 0xE0U) != 0x80U) {
    return false;
  }

  mcp_write(MCP_CNF1, 0x00U);
  mcp_write(MCP_CNF2, 0xD9U);
  mcp_write(MCP_CNF3, 0x82U);
  mcp_set_filter(0x20U, 0x7FFU);
  mcp_set_filter(0x24U, 0x7FFU);
  for (uint8_t i = 0; i < 6U; ++i) {
    mcp_set_filter(filter_addresses[i], (i & 1U) == 0U ? CAN_ID_CONTROL : CAN_ID_DATA);
  }
  mcp_write(MCP_RXB0CTRL, 0x04U);
  mcp_write(MCP_RXB1CTRL, 0x00U);
  mcp_write(MCP_CANINTE, 0x00U);
  mcp_write(MCP_CANINTF, 0x00U);
  mcp_modify(MCP_CANCTRL, 0xE0U, 0x00U);

  uint32_t start = cycles_now();
  while ((mcp_read(MCP_CANSTAT) & 0xE0U) != 0U) {
    if (elapsed(start, 20U * MS_CYCLES)) {
      return false;
    }
  }
  return true;
}

static bool mcp_receive(can_frame_t *frame) {
  uint8_t flags = mcp_read(MCP_CANINTF);
  uint8_t address;
  uint8_t clear_mask;
  uint8_t raw[13];
  if ((flags & 0x01U) != 0U) {
    address = MCP_RXB0SIDH;
    clear_mask = 0x01U;
  } else if ((flags & 0x02U) != 0U) {
    address = MCP_RXB1SIDH;
    clear_mask = 0x02U;
  } else {
    return false;
  }
  mcp_read_many(address, raw, sizeof(raw));
  mcp_modify(MCP_CANINTF, clear_mask, 0U);
  frame->id = ((uint16_t)raw[0] << 3) | (raw[1] >> 5);
  frame->dlc = raw[4] & 0x0FU;
  if (frame->dlc > 8U) {
    frame->dlc = 8U;
  }
  for (uint8_t i = 0; i < frame->dlc; ++i) {
    frame->data[i] = raw[5U + i];
  }
  return true;
}

static bool mcp_send(uint16_t id, const uint8_t data[8]) {
  uint32_t start = cycles_now();
  while ((mcp_read(MCP_TXB0CTRL) & 0x08U) != 0U) {
    if (elapsed(start, 20U * MS_CYCLES)) {
      return false;
    }
  }
  uint8_t raw[13];
  encode_standard_id(id, raw);
  raw[4] = 8U;
  for (uint8_t i = 0; i < 8U; ++i) {
    raw[5U + i] = data[i];
  }
  mcp_write_many(MCP_TXB0SIDH, raw, sizeof(raw));
  spi_select(true);
  spi_transfer(MCP_RTS_TX0);
  spi_select(false);
  return true;
}

static void send_response(uint8_t operation, uint8_t status, uint16_t detail) {
  uint8_t response[8];
  response[0] = operation | 0x80U;
  response[1] = status;
  response[2] = state == STATE_PAGE ? page_index : next_page;
  response[3] = state;
  response[4] = (uint8_t)expected_offset;
  response[5] = (uint8_t)(expected_offset >> 8);
  response[6] = (uint8_t)detail;
  response[7] = (uint8_t)(detail >> 8);
  (void)mcp_send(CAN_ID_RESPONSE, response);
}

static bool flash_wait(void) {
  uint32_t start = cycles_now();
  while ((FLASH->SR & FLASH_SR_BSY) != 0U) {
    if (elapsed(start, 100U * MS_CYCLES)) {
      return false;
    }
  }
  return true;
}

static void flash_unlock(void) {
  if ((FLASH->CR & FLASH_CR_LOCK) != 0U) {
    FLASH->KEYR = 0x45670123UL;
    FLASH->KEYR = 0xCDEF89ABUL;
  }
}

static void flash_lock(void) {
  FLASH->CR |= FLASH_CR_LOCK;
}

static bool flash_erase_page(uint32_t address) {
  flash_unlock();
  if (!flash_wait()) {
    flash_lock();
    return false;
  }
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = address;
  FLASH->CR |= FLASH_CR_STRT;
  bool ok = flash_wait();
  FLASH->CR &= ~FLASH_CR_PER;
  ok = ok && ((FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPERR)) == 0U);
  flash_lock();
  return ok;
}

static bool flash_program_halfword(uint32_t address, uint16_t value) {
  flash_unlock();
  if (!flash_wait()) {
    flash_lock();
    return false;
  }
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;
  FLASH->CR |= FLASH_CR_PG;
  *(__IO uint16_t *)address = value;
  bool ok = flash_wait();
  FLASH->CR &= ~FLASH_CR_PG;
  ok = ok && ((FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPERR)) == 0U) &&
       (*(__IO uint16_t *)address == value);
  flash_lock();
  return ok;
}

static bool flash_program_page(uint32_t address, const uint8_t *data) {
  if (!flash_erase_page(address)) {
    return false;
  }
  for (uint32_t offset = 0; offset < FLASH_PAGE_SIZE; offset += 2U) {
    uint16_t value = (uint16_t)data[offset] | ((uint16_t)data[offset + 1U] << 8);
    if (value != 0xFFFFU && !flash_program_halfword(address + offset, value)) {
      return false;
    }
  }
  for (uint32_t offset = 0; offset < FLASH_PAGE_SIZE; ++offset) {
    if (*(__IO uint8_t *)(address + offset) != data[offset]) {
      return false;
    }
  }
  return true;
}

static bool vector_valid(uint32_t size) {
  if (size < 8U || size > APP_MAX_SIZE) {
    return false;
  }
  uint32_t stack = *(__IO uint32_t *)APP_ADDRESS;
  uint32_t reset = *(__IO uint32_t *)(APP_ADDRESS + 4U);
  uint32_t reset_address = reset & ~1UL;
  return stack >= SRAM_START && stack <= SRAM_END && (reset & 1U) != 0U &&
         reset_address >= APP_ADDRESS && reset_address < APP_ADDRESS + size;
}

static bool metadata_valid(void) {
  const image_metadata_t *metadata = (const image_metadata_t *)METADATA_ADDRESS;
  if (metadata->magic != BOOT_MAGIC || metadata->format != METADATA_FORMAT ||
      metadata->reserved != 0U || !vector_valid(metadata->image_size)) {
    return false;
  }
  uint32_t header_crc = crc32_buffer((const uint8_t *)&metadata->format, 12U);
  if (header_crc != metadata->metadata_crc32) {
    return false;
  }
  return crc32_buffer((const uint8_t *)APP_ADDRESS, metadata->image_size) ==
         metadata->image_crc32;
}

static bool metadata_commit(void) {
  image_metadata_t metadata;
  metadata.magic = BOOT_MAGIC;
  metadata.format = METADATA_FORMAT;
  metadata.reserved = 0U;
  metadata.image_size = image_size;
  metadata.image_crc32 = image_crc32;
  metadata.metadata_crc32 = crc32_buffer((const uint8_t *)&metadata.format, 12U);

  const uint8_t *bytes = (const uint8_t *)&metadata;
  for (uint32_t offset = 4U; offset < sizeof(metadata); offset += 2U) {
    uint16_t value = (uint16_t)bytes[offset] | ((uint16_t)bytes[offset + 1U] << 8);
    if (!flash_program_halfword(METADATA_ADDRESS + offset, value)) {
      return false;
    }
  }
  if (!flash_program_halfword(METADATA_ADDRESS, (uint16_t)metadata.magic) ||
      !flash_program_halfword(METADATA_ADDRESS + 2U, (uint16_t)(metadata.magic >> 16))) {
    return false;
  }
  return metadata_valid();
}

static uint16_t expected_page_length(uint8_t index) {
  uint32_t offset = (uint32_t)index * FLASH_PAGE_SIZE;
  uint32_t remaining = image_size - offset;
  return remaining > FLASH_PAGE_SIZE ? FLASH_PAGE_SIZE : (uint16_t)remaining;
}

static void reset_transfer(void) {
  state = STATE_WAIT;
  pending_info = false;
  expected_offset = 0U;
  next_page = 0U;
}

static void handle_control(const can_frame_t *frame) {
  if (frame->dlc != 8U) {
    send_response(frame->dlc == 0U ? 0U : frame->data[0], STATUS_BAD_ARGUMENT, 0U);
    return;
  }
  uint8_t operation = frame->data[0];
  last_activity = cycles_now();
  switch (operation) {
    case OP_ENTER:
      if (frame->data[1] != 'C' || frame->data[2] != 'B' ||
          frame->data[3] != 'L' || frame->data[4] != '1') {
        send_response(operation, STATUS_BAD_ARGUMENT, 0U);
        return;
      }
      stay_in_bootloader = true;
      send_response(operation, STATUS_OK, BOOT_VERSION);
      break;
    case OP_QUERY:
      send_response(operation, STATUS_OK, BOOT_VERSION);
      break;
    case OP_BEGIN_INFO: {
      uint32_t size = read_u32(&frame->data[2]);
      if (frame->data[1] != BOOT_PROTOCOL_VERSION || size < 8U || size > APP_MAX_SIZE) {
        send_response(operation, STATUS_BAD_ARGUMENT, 0U);
        break;
      }
      pending_size = size;
      pending_info = true;
      send_response(operation, STATUS_OK, BOOT_PROTOCOL_VERSION);
      break;
    }
    case OP_BEGIN_CRC:
      if (!pending_info) {
        if ((state == STATE_UPDATE || state == STATE_PAGE || state == STATE_READY) &&
            read_u32(&frame->data[1]) == image_crc32) {
          send_response(operation, STATUS_OK, 0U);
          break;
        }
        send_response(operation, STATUS_BAD_STATE, 0U);
        break;
      }
      if (!flash_erase_page(METADATA_ADDRESS)) {
        send_response(operation, STATUS_FLASH, 0U);
        break;
      }
      image_size = pending_size;
      image_crc32 = read_u32(&frame->data[1]);
      next_page = 0U;
      expected_offset = 0U;
      pending_info = false;
      state = STATE_UPDATE;
      stay_in_bootloader = true;
      send_response(operation, STATUS_OK, 0U);
      break;
    case OP_PAGE_BEGIN: {
      uint8_t requested_page = frame->data[1];
      uint16_t requested_length = (uint16_t)frame->data[2] |
                                  ((uint16_t)frame->data[3] << 8);
      bool new_page = state == STATE_UPDATE && requested_page == next_page;
      bool repeated_page = state == STATE_PAGE && requested_page == page_index;
      if ((!new_page && !repeated_page) || requested_page >= APP_PAGE_COUNT ||
          requested_length != expected_page_length(requested_page)) {
        send_response(operation, STATUS_BAD_STATE, next_page);
        break;
      }
      page_index = requested_page;
      page_length = requested_length;
      page_crc32 = read_u32(&frame->data[4]);
      expected_offset = 0U;
      for (uint32_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
        page_buffer[i] = 0xFFU;
      }
      state = STATE_PAGE;
      send_response(operation, STATUS_OK, 0U);
      break;
    }
    case OP_PAGE_COMMIT:
      if ((state == STATE_UPDATE || state == STATE_READY) && next_page > 0U &&
          frame->data[1] == (uint8_t)(next_page - 1U)) {
        send_response(operation, STATUS_OK, 0U);
        break;
      }
      if (state != STATE_PAGE || frame->data[1] != page_index ||
          expected_offset != page_length) {
        send_response(operation, STATUS_BAD_STATE, expected_offset);
        break;
      }
      if (crc32_buffer(page_buffer, page_length) != page_crc32) {
        send_response(operation, STATUS_PAGE_CRC, expected_offset);
        break;
      }
      if (!flash_program_page(APP_ADDRESS + (uint32_t)page_index * FLASH_PAGE_SIZE,
                              page_buffer)) {
        send_response(operation, STATUS_FLASH, page_index);
        break;
      }
      ++next_page;
      expected_offset = 0U;
      state = ((uint32_t)next_page * FLASH_PAGE_SIZE >= image_size) ? STATE_READY
                                                                    : STATE_UPDATE;
      send_response(operation, STATUS_OK, 0U);
      break;
    case OP_FINISH:
      if (state == STATE_VALID_APP && metadata_valid()) {
        send_response(operation, STATUS_OK, 0U);
        break;
      }
      if (state != STATE_READY) {
        send_response(operation, STATUS_BAD_STATE, 0U);
        break;
      }
      if (crc32_buffer((const uint8_t *)APP_ADDRESS, image_size) != image_crc32) {
        send_response(operation, STATUS_IMAGE_CRC, 0U);
        break;
      }
      if (!vector_valid(image_size)) {
        send_response(operation, STATUS_BAD_VECTOR, 0U);
        break;
      }
      if (!metadata_commit()) {
        send_response(operation, STATUS_FLASH, 0U);
        break;
      }
      state = STATE_VALID_APP;
      send_response(operation, STATUS_OK, 0U);
      delay_cycles(20U * MS_CYCLES);
      NVIC_SystemReset();
      break;
    case OP_ABORT:
      reset_transfer();
      stay_in_bootloader = true;
      send_response(operation, STATUS_OK, 0U);
      break;
    default:
      send_response(operation, STATUS_UNSUPPORTED, 0U);
      break;
  }
}

static void handle_data(const can_frame_t *frame) {
  last_activity = cycles_now();
  if (state != STATE_PAGE || frame->dlc != 8U) {
    send_response(OP_DATA, STATUS_BAD_STATE, expected_offset);
    return;
  }
  uint16_t offset = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8);
  if (offset != expected_offset || offset >= page_length) {
    send_response(OP_DATA, STATUS_OFFSET, expected_offset);
    return;
  }
  uint16_t count = (uint16_t)(page_length - offset);
  if (count > 6U) {
    count = 6U;
  }
  for (uint16_t i = 0; i < count; ++i) {
    page_buffer[offset + i] = frame->data[2U + i];
  }
  expected_offset += count;
}

static bool consume_boot_magic(void) {
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_DBP;
  while ((PWR->CR & PWR_CR_DBP) == 0U) {
  }
  bool requested = RTC->BKP0R == BOOT_MAGIC;
  if (requested) {
    RTC->BKP0R = 0U;
  }
  return requested;
}

__attribute__((noreturn)) static void jump_to_application(void) {
  uint32_t stack = *(__IO uint32_t *)APP_ADDRESS;
  uint32_t reset = *(__IO uint32_t *)(APP_ADDRESS + 4U);
  typedef void (*entry_t)(void);
  entry_t entry = (entry_t)reset;

  __disable_irq();
  SysTick->CTRL = 0U;
  spi_select(true);
  spi_transfer(MCP_RESET);
  spi_select(false);
  SPI1->CR1 &= ~SPI_CR1_SPE;
  for (uint32_t i = 0; i < 8U; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }
  SCB->VTOR = APP_ADDRESS;
  __set_CONTROL(0U);
  __set_MSP(stack);
  __DSB();
  __ISB();
  __enable_irq();
  entry();
  for (;;) {
  }
}

static void hardware_init(void) {
  RCC->CR |= RCC_CR_HSION;
  while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
  }
  RCC->CFGR = 0U;
  while ((RCC->CFGR & RCC_CFGR_SWS) != 0U) {
  }
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  GPIOA->MODER = (GPIOA->MODER & ~((3UL << (14U * 2U)) | (3UL << (15U * 2U)))) |
                 (1UL << (14U * 2U)) | (1UL << (15U * 2U));
  GPIOA->OTYPER &= ~((1UL << 14U) | (1UL << 15U));
  GPIOA->BSRR = GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;

  GPIOB->MODER = (GPIOB->MODER & ~((3UL << 6U) | (3UL << 8U) | (3UL << 10U))) |
                 (2UL << 6U) | (2UL << 8U) | (2UL << 10U);
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~((0xFUL << 12U) | (0xFUL << 16U) |
                                     (0xFUL << 20U))) |
                  (5UL << 12U) | (5UL << 16U) | (5UL << 20U);
  GPIOB->OSPEEDR |= (3UL << 6U) | (3UL << 8U) | (3UL << 10U);

  GPIOA->BRR = GPIO_BRR_BR_14;
  delay_cycles(10U * MS_CYCLES);
  GPIOA->BSRR = GPIO_BSRR_BS_14;
  delay_cycles(10U * MS_CYCLES);

  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
  SPI1->CR2 = (7UL << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
  SPI1->CR1 |= SPI_CR1_SPE;
}

int main(void) {
  hardware_init();
  bool boot_requested = consume_boot_magic();
  bool valid_application = metadata_valid();
  state = valid_application ? STATE_VALID_APP : STATE_WAIT;
  stay_in_bootloader = boot_requested || !valid_application;
  last_activity = cycles_now();
  bool can_ready = mcp_init();

  uint32_t window_start = cycles_now();
  while (can_ready && !stay_in_bootloader && !elapsed(window_start, BOOT_WINDOW_CYCLES)) {
    can_frame_t frame;
    if (mcp_receive(&frame)) {
      if (frame.id == CAN_ID_CONTROL) {
        handle_control(&frame);
      }
    }
  }
  if (!stay_in_bootloader && valid_application) {
    jump_to_application();
  }

  if (state == STATE_VALID_APP) {
    state = STATE_WAIT;
  }
  for (;;) {
    can_frame_t frame;
    if (can_ready && mcp_receive(&frame)) {
      if (frame.id == CAN_ID_CONTROL) {
        handle_control(&frame);
      } else if (frame.id == CAN_ID_DATA) {
        handle_data(&frame);
      }
    }
    if ((state == STATE_UPDATE || state == STATE_PAGE || state == STATE_READY) &&
        elapsed(last_activity, UPDATE_TIMEOUT_CYCLES)) {
      reset_transfer();
    }
  }
}
