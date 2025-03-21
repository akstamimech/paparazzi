#ifndef PAPARAZZI_DEPTH_MODEL_H
#define PAPARAZZI_DEPTH_MODEL_H

#ifndef DEPTH_VECTOR_SIZE
#define DEPTH_VECTOR_SIZE 16
#endif

// void entry(const float tensor_input_1[1][3][520][240], float tensor_79[1][DEPTH_VECTOR_SIZE]);
void entry(const float tensor_input_1[1][3][260][120], float tensor_79[1][DEPTH_VECTOR_SIZE]);

#endif //PAPARAZZI_DEPTH_MODEL_H