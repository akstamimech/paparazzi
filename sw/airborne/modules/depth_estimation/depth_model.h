#ifndef PAPARAZZI_DEPTH_MODEL_H
#define PAPARAZZI_DEPTH_MODEL_H

#ifndef DEPTH_VECTOR_SIZE
#define DEPTH_VECTOR_SIZE 504
#endif

void entry(const float tensor_x[1][1][520][480], float tensor_linear[1][DEPTH_VECTOR_SIZE]);

#endif //PAPARAZZI_DEPTH_MODEL_H