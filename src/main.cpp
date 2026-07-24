/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdio.h>
#include <string.h>
#include <string_view>
#include <sys/param.h> // MIN and MAX

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "i2s_audio.h"
#include "board_config.h"
#include "board.h"

#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pico/bootrom.h"
#include "hardware/structs/rosc.h"
#include "hardware/watchdog.h"
#include "pico/timeout_helper.h"

// Approximate exponential volume ramp - (n / 64) ^ 4
// Tested with pure square for perceptual loudness.
const uint8_t volume_ramp[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
  2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4,
  5, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9,
  9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15,
  16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 22, 22, 23, 24, 24,
  25, 26, 27, 27, 28, 29, 30, 30, 31, 32, 33, 34, 35, 36, 37, 38,
  39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 52, 53, 54, 55,
  57, 58, 59, 61, 62, 63, 65, 66, 68, 69, 71, 72, 74, 76, 77, 79,
  81, 82, 84, 86, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105, 107, 109,
  111, 113, 115, 118, 120, 122, 125, 127, 129, 132, 134, 137, 139, 142, 144, 147,
  150, 152, 155, 158, 161, 163, 166, 169, 172, 175, 178, 181, 184, 188, 191, 194,
  197, 201, 204, 207, 211, 214, 218, 221, 225, 229, 232, 236, 240, 244, 248, 255
};

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+

// List of supported sample rates
const uint32_t sample_rates[] = {48000};
uint32_t current_sample_rate  = 48000;

#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

/* Blink pattern
 * - 25 ms   : streaming data
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
  BLINK_STREAMING = 25,
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

int system_volume = 255;
int volume_speed = 10;

uint8_t led_red = 0;
uint8_t led_green = 0;
uint8_t led_blue = 0;

// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];       // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];    // +1 for master channel 0

// Buffer for speaker data
int32_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];
// Speaker data size received in the last frame
int spk_data_size;
// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX,
                                                                        CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;


const size_t MAX_UART_PACKET = 64;

const size_t COMMAND_LEN = 4;
uint8_t command_buffer[COMMAND_LEN];
std::string_view command((const char *)command_buffer, COMMAND_LEN);


void led_task(void);
void audio_task(void);
void usb_serial_init(void);
uint cdc_task(uint8_t *buf, size_t buf_len);

// TinyUSB expects the application to supply a millisecond timebase when no RTOS is used
extern "C" uint32_t tusb_time_millis_api(void) {
  return to_ms_since_boot(get_absolute_time());
}

uint cdc_task(uint8_t *buf, size_t buf_len) {

    if (tud_cdc_connected()) {
        if (tud_cdc_available()) {
            return tud_cdc_read(buf, buf_len);
        }
    }

    return 0;
}

bool cdc_wait_for(std::string_view data, uint timeout_ms=50) {
    timeout_state ts;
    absolute_time_t until = delayed_by_ms(get_absolute_time(), timeout_ms);
    check_timeout_fn check_timeout = init_single_timeout_until(&ts, until);

    for(auto expected_char : data) {
        char got_char;
        while(1){
            tud_task();
            if (cdc_task((uint8_t *)&got_char, 1) == 1) break;
            if(check_timeout(&ts, false)) return false;
        }
        if (got_char != expected_char) return false;
    }
    return true;
}

size_t cdc_get_bytes(const uint8_t *buffer, const size_t len, const uint timeout_ms=1000) {
    memset((void *)buffer, len, 0);

    uint8_t *p = (uint8_t *)buffer;

    timeout_state ts;
    absolute_time_t until = delayed_by_ms(get_absolute_time(), timeout_ms);
    check_timeout_fn check_timeout = init_single_timeout_until(&ts, until);

    size_t bytes_remaining = len;
    while (bytes_remaining && !check_timeout(&ts, false)) {
        tud_task(); // tinyusb device task
        size_t bytes_read = cdc_task(p, std::min(bytes_remaining, MAX_UART_PACKET));
        bytes_remaining -= bytes_read;
        p += bytes_read;
    }
    return len - bytes_remaining;
}

void serial_task(void) {
  if (tud_cdc_connected()) {
      if (tud_cdc_available()) {
        if(!cdc_wait_for("multiverse:")) {
            return; // Couldn't get 16 bytes of command
        }

        if(cdc_get_bytes(command_buffer, COMMAND_LEN) != COMMAND_LEN) {
            //display::info("cto");
            return;
        }

        if(command == "_rst") {
            sleep_ms(500);
            save_and_disable_interrupts();
            rosc_hw->ctrl = ROSC_CTRL_ENABLE_VALUE_ENABLE << ROSC_CTRL_ENABLE_LSB;
            watchdog_reboot(0, 0, 0);
            return;
        }

        if(command == "_usb") {
            sleep_ms(500);
            save_and_disable_interrupts();
            rosc_hw->ctrl = ROSC_CTRL_ENABLE_VALUE_ENABLE << ROSC_CTRL_ENABLE_LSB;
            reset_usb_boot(0, 0);
            return;
        }
      }
    }
}

/*------------- MAIN -------------*/
void __isr __time_critical_func(audio_i2s_get_data_handler)() {
  spk_data_size = tud_audio_read(spk_buf, sizeof(spk_buf));

  if (spk_data_size)
  {
    // "Hardware" volume is 0 - 100 in steps of 256, with a maximum value of 25600
    int current_volume = volume_ramp[system_volume];

    if (mute[0]) {
      current_volume = 0;
    }

    i2s_audio_give_buffer(spk_buf, (size_t)spk_data_size, current_resolution, current_volume);
    spk_data_size = 0;
  }
}

int main(void)
{

  system_init();

  board_init();

  // Fetch the Pico serial (actually the flash chip ID) into `usb_serial`
  // This has nothing to do with CDC serial!
  usb_serial_init();

  // init device stack on configured roothub port
  const tusb_rhport_init_t device_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &device_init);

  i2s_audio_init();
  i2s_audio_start();

  TU_LOG1("Picade Max Audio Running\r\n");

  irq_add_shared_handler(DMA_IRQ_0 + PICO_AUDIO_I2S_DMA_IRQ, audio_i2s_get_data_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

  while (1)
  {
    tud_task();
    audio_task();
    serial_task();
    led_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Helper for clock get requests
static bool audio20_clock_get_request(uint8_t rhport, tusb_control_request_t const *p_request)
{
  uint8_t const ctrl_sel = TU_U16_HIGH(p_request->wValue);

  TU_ASSERT(TU_U16_HIGH(p_request->wIndex) == UAC2_ENTITY_CLOCK);

  if (ctrl_sel == AUDIO20_CS_CTRL_SAM_FREQ)
  {
    if (p_request->bRequest == AUDIO20_CS_REQ_CUR)
    {
      TU_LOG1("Clock get current freq %" PRIu32 "\r\n", current_sample_rate);

      audio20_control_cur_4_t curf = { (int32_t) tu_htole32(current_sample_rate) };
      return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &curf, sizeof(curf));
    }
    else if (p_request->bRequest == AUDIO20_CS_REQ_RANGE)
    {
      audio20_control_range_4_n_t(N_SAMPLE_RATES) rangef =
      {
        .wNumSubRanges = tu_htole16(N_SAMPLE_RATES)
      };
      TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
      for(uint8_t i = 0; i < N_SAMPLE_RATES; i++)
      {
        rangef.subrange[i].bMin = (int32_t) sample_rates[i];
        rangef.subrange[i].bMax = (int32_t) sample_rates[i];
        rangef.subrange[i].bRes = 0;
        TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
      }

      return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &rangef, sizeof(rangef));
    }
  }
  else if (ctrl_sel == AUDIO20_CS_CTRL_CLK_VALID &&
           p_request->bRequest == AUDIO20_CS_REQ_CUR)
  {
    audio20_control_cur_1_t cur_valid = { .bCur = 1 };
    TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur_valid, sizeof(cur_valid));
  }
  TU_LOG1("Clock get request not supported, selector = %u, request = %u\r\n",
          ctrl_sel, p_request->bRequest);
  return false;
}

// Helper for clock set requests
static bool audio20_clock_set_request(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t const *buf)
{
  (void)rhport;

  uint8_t const ctrl_sel = TU_U16_HIGH(p_request->wValue);

  TU_ASSERT(TU_U16_HIGH(p_request->wIndex) == UAC2_ENTITY_CLOCK);
  TU_VERIFY(p_request->bRequest == AUDIO20_CS_REQ_CUR);

  if (ctrl_sel == AUDIO20_CS_CTRL_SAM_FREQ)
  {
    TU_VERIFY(p_request->wLength == sizeof(audio20_control_cur_4_t));

    current_sample_rate = (uint32_t) ((audio20_control_cur_4_t const *)buf)->bCur;

    TU_LOG1("Clock set current freq: %" PRIu32 "\r\n", current_sample_rate);

    return true;
  }
  else
  {
    TU_LOG1("Clock set request not supported, selector = %u, request = %u\r\n",
            ctrl_sel, p_request->bRequest);
    return false;
  }
}

// Helper for feature unit get requests
static bool audio20_feature_unit_get_request(uint8_t rhport, tusb_control_request_t const *p_request)
{
  uint8_t const ctrl_sel = TU_U16_HIGH(p_request->wValue);
  uint8_t const channel_num = TU_U16_LOW(p_request->wValue);

  TU_ASSERT(TU_U16_HIGH(p_request->wIndex) == UAC2_ENTITY_SPK_FEATURE_UNIT);

  if (ctrl_sel == AUDIO20_FU_CTRL_MUTE && p_request->bRequest == AUDIO20_CS_REQ_CUR)
  {
    audio20_control_cur_1_t mute1 = { .bCur = mute[channel_num] };
    TU_LOG1("Get channel %u mute %d\r\n", channel_num, mute1.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute1, sizeof(mute1));
  }
  else if (ctrl_sel == AUDIO20_FU_CTRL_VOLUME)
  {
    if (p_request->bRequest == AUDIO20_CS_REQ_RANGE)
    {
      audio20_control_range_2_n_t(1) range_vol;
      range_vol.wNumSubRanges = tu_htole16(1);
      range_vol.subrange[0] = { .bMin = tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(VOLUME_CTRL_100_DB), tu_htole16(256) };
      TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\r\n", channel_num,
              range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &range_vol, sizeof(range_vol));
    }
    else if (p_request->bRequest == AUDIO20_CS_REQ_CUR)
    {
      audio20_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[channel_num]) };
      TU_LOG1("Get channel %u volume %d dB\r\n", channel_num, cur_vol.bCur / 256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur_vol, sizeof(cur_vol));
    }
  }
  TU_LOG1("Feature unit get request not supported, selector = %u, request = %u\r\n",
          ctrl_sel, p_request->bRequest);

  return false;
}

// Helper for feature unit set requests
// This handles volume control and mute requests coming from the USB host to Picade Max Audio
static bool audio20_feature_unit_set_request(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t const *buf)
{
  (void)rhport;

  uint8_t const ctrl_sel = TU_U16_HIGH(p_request->wValue);
  uint8_t const channel_num = TU_U16_LOW(p_request->wValue);

  TU_ASSERT(TU_U16_HIGH(p_request->wIndex) == UAC2_ENTITY_SPK_FEATURE_UNIT);
  TU_VERIFY(p_request->bRequest == AUDIO20_CS_REQ_CUR);

  if (ctrl_sel == AUDIO20_FU_CTRL_MUTE)
  {
    TU_VERIFY(p_request->wLength == sizeof(audio20_control_cur_1_t));

    mute[channel_num] = ((audio20_control_cur_1_t const *)buf)->bCur;

    TU_LOG1("Set channel %d Mute: %d\r\n", channel_num, mute[channel_num]);

    // Set the red LED channel to indicate mute
    led_red = mute[channel_num] ? 255 : 0;

    return true;
  }
  else if (ctrl_sel == AUDIO20_FU_CTRL_VOLUME)
  {
    TU_VERIFY(p_request->wLength == sizeof(audio20_control_cur_2_t));

    volume[channel_num] = tu_le16toh(((audio20_control_cur_2_t const *)buf)->bCur);

    // Set the blue LED channel to indicate volume
    led_blue = MIN(255, volume[channel_num] / 100);

    system_volume = MIN(255u, volume[channel_num] / 100);

    TU_LOG1("Set channel %d volume: %d dB\r\n", channel_num, volume[channel_num] / 256);

    return true;
  }
  else
  {
    TU_LOG1("Feature unit set request not supported, selector = %u, request = %u\r\n",
            ctrl_sel, p_request->bRequest);
    return false;
  }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) pBuff;
  return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  return false;
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
  uint8_t const entity_id = TU_U16_HIGH(p_request->wIndex);

  if (entity_id == UAC2_ENTITY_CLOCK)
    return audio20_clock_get_request(rhport, p_request);
  if (entity_id == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return audio20_feature_unit_get_request(rhport, p_request);
  else
  {
    TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
            entity_id, TU_U16_HIGH(p_request->wValue), p_request->bRequest);
  }
  return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
  uint8_t const entity_id = TU_U16_HIGH(p_request->wIndex);

  if (entity_id == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return audio20_feature_unit_set_request(rhport, p_request, buf);
  if (entity_id == UAC2_ENTITY_CLOCK)
    return audio20_clock_set_request(rhport, p_request, buf);
  TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
          entity_id, TU_U16_HIGH(p_request->wValue), p_request->bRequest);

  return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;

  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0)
      blink_interval_ms = BLINK_MOUNTED;

  return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
  if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0)
      blink_interval_ms = BLINK_STREAMING;

  // Clear buffer when streaming format is changed
  spk_data_size = 0;
  if(alt != 0)
  {
    current_resolution = resolutions_per_format[alt - 1];
  }

  return true;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{
  static uint32_t start_ms = 0;
  uint32_t volume_interval_ms = 50;

  /*spk_data_size = tud_audio_read(spk_buf, sizeof(spk_buf));

  if (spk_data_size)
  {
    // "Hardware" volume is 0 - 100 in steps of 256, with a maximum value of 25600
    int current_volume = volume_ramp[system_volume];

    if (mute[0]) {
      current_volume = 0;
    }

    i2s_audio_give_buffer(spk_buf, (size_t)spk_data_size, current_resolution, current_volume);
    spk_data_size = 0;
  }*/

  // Only handle volume control changes every volume_interval_ms
  // The encoder driver should - I believe - asynchronously gather a delta to be handled here
  if (tusb_time_millis_api() - start_ms >= volume_interval_ms)
  {
    // This is just the raw delta from the encoder
    int32_t volume_delta = get_volume_delta();

    // Adjust the speed of volume control (number of volume steps per encoder turn)
    volume_delta *= volume_speed;

    // Long press triggers reset to bootloader
    handle_mute_button_held();

    if(get_mute_button_pressed()) {
      // Toggle one channel and copy the mute value to the other
      // We don't want to end up with one muted and one unmuted somehow...
      mute[0] = !mute[0];
      mute[1] = mute[0];

      // Illuminate the LED red if muted
      led_red = mute[0] ? 255 : 0;

      // Mute was changed - notify the host with an interrupt
      // 6.1 Interrupt Data Message
      const audio_interrupt_data_t data = {.v2 = {
        .bInfo = 0,                                       // Class-specific interrupt, originated from an interface
        .bAttribute = AUDIO20_CS_REQ_CUR,                   // Caused by current settings
        .wValue_cn_or_mcn = 0,                            // CH0: master volume
        .wValue_cs = AUDIO20_FU_CTRL_MUTE,                  // Muted/Unmuted
        .wIndex_ep_or_int = 0,                            // From the interface itself
        .wIndex_entity_id = UAC2_ENTITY_SPK_FEATURE_UNIT, // From feature unit
      }};

      tud_audio_int_write(&data);
      // Call tud_task to handle the interrupt to host
      tud_task();
    }

    int old_system_volume = system_volume;


    if(volume_delta + system_volume > 255) {
        system_volume = 255;
    } else if (volume_delta + system_volume < 0) {
        system_volume = 0;
    } else {
        system_volume += volume_delta;
    }

    if(system_volume != old_system_volume) {
      led_blue = system_volume;

      volume[0] = system_volume * 100;
      volume[1] = system_volume * 100;

      // Volume has changed - notify the host with an interrupt
      // 6.1 Interrupt Data Message
      const audio_interrupt_data_t data = {.v2 = {
        .bInfo = 0,                                       // Class-specific interrupt, originated from an interface
        .bAttribute = AUDIO20_CS_REQ_CUR,                   // Caused by current settings
        .wValue_cn_or_mcn = 0,                            // CH0: master volume
        .wValue_cs = AUDIO20_FU_CTRL_VOLUME,                // Volume change
        .wIndex_ep_or_int = 0,                            // From the interface itself
        .wIndex_entity_id = UAC2_ENTITY_SPK_FEATURE_UNIT, // From feature unit
      }};

      tud_audio_int_write(&data);
      // Call tud_task to handle the interrupt to host
      tud_task();
    }

    start_ms += volume_interval_ms;
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (tusb_time_millis_api() - start_ms >= blink_interval_ms) {
    start_ms += blink_interval_ms;

    led_green = led_state ? 64 : 0;
    led_state = !led_state;
  }

  system_led(led_red, led_green, led_blue);
}
