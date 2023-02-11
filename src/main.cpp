#include "esp_err.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "I2SSampler.h"
#include "I2SMEMSSampler.h"
#include "ADCSampler.h"
#include "I2SOutput.h"
#include "SDCard.h"
#include "SPIFFS.h"
#include "WAVFileWriter.h"
#include "WAVFileReader.h"
#include "config.h"

#define SDCARD_WRITING_ENABLED  1
#define SDCARD_BUFFER           50*1024

static const char *TAG = "main";

extern "C"
{
  void app_main(void);
}

void wait_for_button_push()
{
  while (gpio_get_level(GPIO_BUTTON) == 1)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void record(I2SSampler *input, const char *fname)
{
  uint32_t idx = 0;
  int16_t *samples = (int16_t *)malloc(sizeof(int16_t) * SDCARD_BUFFER);
  if (samples == NULL) {
    ESP_LOGI(TAG, "Cannot allocate buffer");
    return;
  }
  ESP_LOGI(TAG, "Start recording");
  input->start();
#ifdef SDCARD_WRITING_ENABLED
  // open the file on the sdcard
  FILE *fp = fopen(fname, "wb");
  // create a new wave file writer
  WAVFileWriter *writer = new WAVFileWriter(fp, input->sample_rate());
#endif
  // keep writing until the user releases the button
  while (1)
  {
    int samples_read = input->read(&samples[idx], 1024);
    idx += 1024;
    // int64_t start = esp_timer_get_time();
#ifdef SDCARD_WRITING_ENABLED
    if (idx == (SDCARD_BUFFER)) {
      writer->write(samples, idx);
      idx = 0;
    }
#endif
    // int64_t end = esp_timer_get_time();
    // ESP_LOGI(TAG, "Wrote %d samples in %lld microseconds", samples_read, end - start);

    if (gpio_get_level(GPIO_BUTTON) == 0) {
#ifdef SDCARD_WRITING_ENABLED
      writer->write(samples, idx);
#endif
      break;
    }
  }
  // stop the input
  input->stop();
#ifdef SDCARD_WRITING_ENABLED
  // and finish the writing
  writer->finish();
  fclose(fp);
  delete writer;
#endif
  free(samples);
  ESP_LOGI(TAG, "Finished recording");
}

void play(Output *output, const char *fname)
{
  int16_t *samples = (int16_t *)malloc(sizeof(int16_t) * 1024);
  // open the file on the sdcard
  FILE *fp = fopen(fname, "rb");
  // create a new wave file writer
  WAVFileReader *reader = new WAVFileReader(fp);
  ESP_LOGI(TAG, "Start playing");
  output->start(reader->sample_rate());
  ESP_LOGI(TAG, "Opened wav file");
  // read until theres no more samples
  while (true)
  {
    int samples_read = reader->read(samples, 1024);
    if (samples_read == 0)
    {
      break;
    }
    ESP_LOGI(TAG, "Read %d samples", samples_read);
    output->write(samples, samples_read);
    ESP_LOGI(TAG, "Played samples");
  }
  // stop the input
  output->stop();
  fclose(fp);
  delete reader;
  free(samples);
  ESP_LOGI(TAG, "Finished playing");
}

void app_main(void)
{
  ESP_LOGI(TAG, "Starting up");

#ifdef USE_SPIFFS
  ESP_LOGI(TAG, "Mounting SPIFFS on /sdcard");
  new SPIFFS("/sdcard");
#else
  ESP_LOGI(TAG, "Mounting SDCard on /sdcard");
  new SDCard("/sdcard", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
#endif

  ESP_LOGI(TAG, "Creating microphone");
#ifdef USE_I2S_MIC_INPUT
  I2SSampler *input = new I2SMEMSSampler(I2S_NUM_0, i2s_mic_pins, i2s_mic_Config);
#else
  I2SSampler *input = new ADCSampler(ADC_UNIT_1, ADC1_CHANNEL_7, i2s_adc_config);
#endif
  // I2SOutput *output = new I2SOutput(I2S_NUM_0, i2s_speaker_pins);

  gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_BUTTON, GPIO_PULLUP_ONLY);
  gpio_sleep_sel_dis(GPIO_BUTTON);
  gpio_sleep_sel_dis(PIN_NUM_MISO);
  gpio_sleep_sel_dis(PIN_NUM_CLK);
  gpio_sleep_sel_dis(PIN_NUM_MOSI);
  gpio_sleep_sel_dis(PIN_NUM_CS);
  gpio_sleep_sel_dis(I2S_MIC_SERIAL_CLOCK);
  gpio_sleep_sel_dis(I2S_MIC_LEFT_RIGHT_CLOCK);
  gpio_sleep_sel_dis(I2S_MIC_SERIAL_DATA);

    esp_pm_config_esp32_t cfg = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
        .light_sleep_enable = true
    };
    esp_pm_configure(&cfg);

  while (true)
  {
    // wait for the user to push and hold the button
    wait_for_button_push();
    record(input, "/sdcard/test.wav");
    // wait for the user to push the button again
    // wait_for_button_push();
    // play(output, "/sdcard/test.wav");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
