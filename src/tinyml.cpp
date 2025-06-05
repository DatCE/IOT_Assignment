#include <Chirale_TensorFlowLite.h>

// include static array definition of pre-trained model
#include "model.h"

// This TensorFlow Lite Micro Library for Arduino is not similar to standard
// Arduino libraries. These additional header files must be included.
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

const float min_val = 54.34049356454897;
const float max_val = 57.816880137634946;
constexpr int kTensorArenaSize = 20000;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

void waitForSerial(unsigned long timeout_ms) {
  unsigned long start = millis();
  while (!Serial && (millis() - start < timeout_ms)) {}
}

void initTinyML() {
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("ERROR: Model version mismatch!");
    while (true) delay(1000);
  }

  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("ERROR: AllocateTensors failed!");
    while (true) delay(1000);
  }

  input = interpreter->input(0);
  output = interpreter->output(0);
}

float predictNextValue(float* input_data, int length) {
  for (int i = 0; i < length; ++i) {
    input->data.f[i] = (input_data[i] - min_val) / (max_val - min_val);
  }

  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Invoke failed!");
    return -999;
  }

  float y = output->data.f[0];
  return y * (max_val - min_val) + min_val;
}
