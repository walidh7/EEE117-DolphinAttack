
#include <Arduino.h>
#include <arduinoFFT.h>
#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"
#include "esp_err.h"

// ---------------- CONFIGURATION ----------------

#define SAMPLES          2048
#define SAMPLING_FREQ    40000
#define AUDIO_IN_PIN     4
#define ADC_CHANNEL      ADC_CHANNEL_3

#define NUM_PEAKS        5
#define NOISE_THRESHOLD  200.0f
#define MIN_SIGNAL_RMS   8.0f

// ---------------- FFT ----------------

float vReal[SAMPLES];
float vImag[SAMPLES];

ArduinoFFT<float> FFT(
    vReal,
    vImag,
    SAMPLES,
    SAMPLING_FREQ
);

// ---------------- ADC DMA ----------------

adc_continuous_handle_t adcHandle = NULL;

// 512 bytes = 128 ADC samples
uint32_t adcBuffer[128];

// ---------------- FREQUENCY PEAKS ----------------

struct FrequencyPeak {
    float frequency;
    float magnitude;
};

// ---------------- ADC SETUP ----------------

void setupADC() {

    adc_continuous_handle_cfg_t handleConfig = {};
    handleConfig.max_store_buf_size = 16384;
    handleConfig.conv_frame_size = 512;

    ESP_ERROR_CHECK(
        adc_continuous_new_handle(&handleConfig, &adcHandle)
    );

    adc_digi_pattern_config_t pattern = {};
    pattern.atten = ADC_ATTEN_DB_12;
    pattern.channel = ADC_CHANNEL;
    pattern.unit = ADC_UNIT_1;
    pattern.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_continuous_config_t config = {};
    config.sample_freq_hz = SAMPLING_FREQ;
    config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    config.pattern_num = 1;
    config.adc_pattern = &pattern;

    ESP_ERROR_CHECK(
        adc_continuous_config(adcHandle, &config)
    );
}

// ---------------- SAMPLE AUDIO ----------------

bool sampleAudio() {

    uint32_t bytesRead = 0;
    int sampleCount = 0;

    if (adc_continuous_start(adcHandle) != ESP_OK) {
      Serial.println("Adc continuous start error");
        return false;
    }

    // Discard data left from any previous capture
    while (adc_continuous_read(
        adcHandle,
        (uint8_t*)adcBuffer,
        sizeof(adcBuffer),
        &bytesRead,
        0
    ) == ESP_OK) {
    }

    while (sampleCount < SAMPLES) {

        esp_err_t result = adc_continuous_read(
            adcHandle,
            (uint8_t*)adcBuffer,
            sizeof(adcBuffer),
            &bytesRead,
            200
        );

        if (result != ESP_OK) {
            adc_continuous_stop(adcHandle);
            Serial.println("Adc ctts stop error 1");
            return false;
        }

        int count = bytesRead / sizeof(uint32_t);

        for (int i = 0; i < count && sampleCount < SAMPLES; i++) {

            adc_digi_output_data_t* sample =
                (adc_digi_output_data_t*)&adcBuffer[i];

            if (sample->type2.unit != 0 ||
              sample->type2.channel != ADC_CHANNEL) {

              uint32_t unit = sample->type2.unit;
              uint32_t channel = sample->type2.channel;
              uint32_t data = sample->type2.data;

              adc_continuous_stop(adcHandle);

              Serial.println("ADC SAMPLE MISMATCH");
              Serial.printf("Expected Unit: 0, Received: %lu\n",
                            (unsigned long)unit);
              Serial.printf("Expected Channel: %d, Received: %lu\n",
                            (int)ADC_CHANNEL, (unsigned long)channel);
              Serial.printf("ADC Value: %lu\n",
                            (unsigned long)data);

              return false;
          }

            vReal[sampleCount] = sample->type2.data;
            vImag[sampleCount] = 0.0f;

            sampleCount++;
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(adcHandle));

    return true;
}

// ---------------- FREQUENCY INTERPOLATION ----------------

float interpolateFrequency(int bin) {

    float left = vReal[bin - 1];
    float center = vReal[bin];
    float right = vReal[bin + 1];

    float denominator = left - 2.0f * center + right;
    float offset = 0.0f;

    if (fabsf(denominator) > 0.000001f) {

        offset = 0.5f * (left - right) / denominator;
        offset = constrain(offset, -0.5f, 0.5f);
    }

    return (bin + offset) * SAMPLING_FREQ / SAMPLES;
}

// ---------------- SETUP ----------------

void setup() {

    Serial.begin(115200);
    delay(1000);

    setupADC();

    Serial.println("ESP32-S3 Frequency Detector");
    Serial.println("ADC DMA Initialized");
    Serial.println("Sampling at 83333 Hz");
}

// Serial output
void display_frequencies(int peakCount, FrequencyPeak peaks[NUM_PEAKS]) {
  if (peakCount == 0) {

        Serial.println("No frequencies detected");

    } else {

        Serial.print("Frequencies (Hz): ");

        for (int i = 0; i < peakCount; i++) {

            Serial.print(peaks[i].frequency, 1);

            if (i < peakCount - 1) {
                Serial.print(", ");
            }
        }

        Serial.println();
    }
}


// Peak detection
int detect_peaks(FrequencyPeak peaks[NUM_PEAKS], float threshold) {
    int peakCount = 0;

    for (int i = 2; i < SAMPLES / 2 - 1; i++) {

        float magnitude = vReal[i];

        if (magnitude < threshold) {
            continue;
        }

        if (magnitude > vReal[i - 1] &&
            magnitude >= vReal[i + 1]) {

            float frequency = interpolateFrequency(i);

            int position = 0;

            while (position < peakCount &&
                    magnitude <= peaks[position].magnitude) {
                position++;
            }

            if (position >= NUM_PEAKS) {
                continue;
            }

            if (peakCount < NUM_PEAKS) {
                peakCount++;
            }

            for (int j = peakCount - 1; j > position; j--) {
                peaks[j] = peaks[j - 1];
            }

            peaks[position].frequency = frequency;
            peaks[position].magnitude = magnitude;
        }
    }

  return peakCount;
}
// ---------------- MAIN LOOP ----------------

void loop() {

    if (!sampleAudio()) {
        Serial.println("ADC sampling error");
        delay(100);
        return;
    }

    // ---------------- FFT ----------------

    FFT.dcRemoval();

    // Check signal level after DC removal
    float sumSquares = 0.0f;

    for (int i = 0; i < SAMPLES; i++) {
        sumSquares += vReal[i] * vReal[i];
    }

    float rms = sqrtf(sumSquares / SAMPLES);

    if (rms < MIN_SIGNAL_RMS) {
        Serial.println("No significant signal");
        delay(100);
        return;
    }

    FFT.windowing(
        FFTWindow::Hamming,
        FFTDirection::Forward
    );

    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    // ---------------- MAX MAGNITUDE ----------------

    float maxMagnitude = 0;

    for (int i = 2; i < SAMPLES / 2 - 1; i++) {
        if (vReal[i] > maxMagnitude) {
            maxMagnitude = vReal[i];
        }
    }

    float threshold = max(
        NOISE_THRESHOLD,
        maxMagnitude * 0.15f
    );

    // ---------------- PEAK DETECTION ----------------

    FrequencyPeak peaks[NUM_PEAKS] = {};
    int peakCount = detect_peaks(peaks, threshold);

    // ---------------- SERIAL OUTPUT ----------------
    display_frequencies(peakCount, peaks);
    delay(100);

    // DolphinAttack detection
    if (peakCount != 0) {
      for (int i = 0; i < peakCount; i++) {
        float frequency = peaks[i].frequency;
        if (frequency > 21000) {
          Serial.println("DolphinAttack!!!")
        }
      }
    }
}
