// GENERATED FILE FROM MODEL ./models/mlp.pth
// Info: Neural network for the 2.1 Crazyflie
#ifndef __GEN_NN__
#define __GEN_NN__

#define INPUT_SIZE 12
#define OUTPUT_SIZE 2

const float* nn_forward(float input[INPUT_SIZE]);
const float* nn_forward_payload(float input[INPUT_SIZE]);

#endif
