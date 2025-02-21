// GENERATED FILE FROM MODEL ./models/mlp.pth
// Info: Neural network for the 2.1 Crazyflie
#include "nn.h"
#include "nn_utils.h"


struct NeuralNetwork
{
    float layers_0_weight[12][16];
    float layers_0_bias[16];
    float layers_0_activation[16];
    float layers_2_weight[16][16];
    float layers_2_bias[16];
    float layers_2_activation[16];
    float layers_4_weight[16][6];
    float layers_4_bias[6];
    float layers_4_activation[6];
    float scaled_output[6];
    float mins[6];
    float maxs[6];
};

struct NeuralNetwork mlp = {
    .layers_0_weight = { { 4.00183141e-01, 4.46943074e-01, -4.92451638e-02, -6.65521562e-01, -6.77052617e-01, -3.53043288e-01, 4.26483825e-02, 1.16821103e-01, 2.09096208e-01, -1.59276590e-01, -4.17889049e-03, 4.61536556e-01, 3.39622200e-01, 1.90446064e-01, 8.38880166e-02, 2.61425316e-01 }, { -3.39734733e-01, -8.73966813e-01, 5.06703369e-02, -6.48312807e-01, -4.67298537e-01, 2.75221199e-01, -2.11882472e-01, 2.26334542e-01, 3.67202848e-01, -1.05284549e-01, -2.20680926e-02, -1.43533543e-01, 4.61810976e-02, 2.44973764e-01, -7.35566914e-02, -5.09762540e-02 }, { 4.27488163e-02, 9.00287405e-02, 5.23322463e-01, 3.55715275e-01, 7.41973698e-01, -2.46803820e-01, 4.26350594e-01, -6.40296862e-02, 7.37678930e-02, -5.79412803e-02, -2.30675656e-02, 9.38630924e-02, 4.94052052e-01, 8.46000314e-02, -5.09773195e-01, -1.87590048e-02 }, { 9.68462944e-01, 1.61302835e-01, 5.35723567e-01, 6.27736524e-02, -2.10950077e-01, -3.57459873e-01, -2.78298080e-01, 1.68431893e-01, -5.70911057e-02, -1.28529727e-01, -1.68332994e-01, -1.87231675e-01, 2.94779718e-01, 2.88866106e-02, -1.67424172e-01, -4.37722236e-01 }, { 1.96649238e-01, 3.45961481e-01, -3.24701428e-01, -6.37135562e-03, 6.78795516e-01, 2.79412478e-01, 2.44840279e-01, 3.66540998e-02, 1.06290253e-02, -2.53200769e-01, 2.35568285e-01, -1.37920976e-01, 7.39600286e-02, 1.56344008e-02, 1.17215782e-01, 2.56337285e-01 }, { 3.26544732e-01, 6.16593957e-02, -3.63079458e-01, -5.31228706e-02, 5.11381209e-01, 3.11618924e-01, 1.29180759e-01, -7.81976357e-02, 9.21719596e-02, 1.53964043e-01, 1.01734683e-01, 2.45569982e-02, -3.73347878e-01, -8.97693560e-02, 1.46077335e-01, 1.91084743e-01 }, { -2.94642579e-02, 5.92158586e-02, -1.20033912e-01, 1.20008746e-02, -2.74700541e-02, -4.39610258e-02, -2.76544802e-02, 1.33186668e-01, 4.41110618e-02, -1.27076820e-01, 3.37643991e-03, -1.00880668e-01, 2.75980439e-02, 1.70615148e-02, 1.91261377e-02, -4.82436307e-02 }, { 1.43897682e-01, -1.08338878e-01, 5.02875075e-02, 1.04866564e-01, 4.19059955e-02, -1.74195971e-02, 1.62240881e-02, -1.07194506e-01, -1.11141242e-01, 2.12717541e-02, 1.26690924e-01, 4.98646311e-03, 4.87454310e-02, -3.44146676e-02, 2.04872072e-01, 1.78083360e-01 }, { -2.38287434e-01, -5.32368310e-02, -1.94730148e-01, -1.23645328e-01, -2.71714274e-02, -9.83256251e-02, 1.29537970e-01, 1.96404323e-01, 1.44614771e-01, 1.50861457e-01, 1.43864915e-01, 1.12068407e-01, 1.00112900e-01, 4.44600452e-03, 2.30849653e-01, 2.57493734e-01 }, { 2.24805459e-01, 4.49724001e-04, -1.55757144e-01, 2.57017255e-01, 3.52011062e-02, 1.54762164e-01, -9.73633677e-02, 3.96655425e-02, -1.10271676e-02, 3.71985883e-02, -1.03099898e-01, 2.51775950e-01, 1.68543369e-01, 4.48005572e-02, -1.57286987e-01, -2.36864045e-01 }, { 1.59916952e-01, 1.16128456e-02, -6.39341846e-02, 6.76820874e-02, -7.54774511e-02, -9.73429233e-02, -8.44248086e-02, -2.60306355e-02, -5.59590124e-02, 3.63371298e-02, -8.04905444e-02, 5.08944429e-02, -8.79118294e-02, 1.14158941e-02, 9.08026919e-02, 1.93831384e-01 }, { 2.75206268e-01, -1.19164940e-02, -1.76132143e-01, -2.67291337e-01, -8.85289982e-02, -1.44040197e-01, 2.43379295e-01, -1.82366837e-02, 1.42041013e-01, -8.32161754e-02, -4.53428216e-02, 1.74939498e-01, -8.07671994e-02, 2.53596753e-02, -2.55148858e-01, 6.06333241e-02 } },
    .layers_0_bias = { -0.42443538, 0.38490877, -0.45431298, -0.10153039, -0.28026393, -0.41028017, 0.15265732, -0.040542115, -0.15571927, 0.21149084, 0.30004877, -0.040895224, 0.2566323, 0.5347176, 0.064034246, -0.032680877 },
    .layers_0_activation = { 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
    .layers_2_weight = { { 0.22110648, -0.08463064, -0.36441234, -0.3071959, -0.08490414, -0.026445162, -0.005516325, 0.022795344, -0.031096147, -0.13065793, -0.016601948, 0.1437636, 0.24288766, -0.039205253, 0.14330186, -0.01515248 }, { 0.051126048, -0.027399043, -0.080611974, -0.1637921, 0.0056101955, 0.20309222, -0.6752358, 0.08741714, -0.021450268, -0.26587057, 0.013453385, 0.025229473, -0.31457582, -0.05151087, -0.20390987, -0.002026431 }, { -0.13579614, -0.033586945, 0.24557164, -0.021035427, 0.1508858, -0.18776633, 0.0069957715, -0.10964717, 0.045149244, -0.22730102, 0.14967543, -0.09189964, 0.073943965, 0.015857002, -0.10294518, -0.08260483 }, { -0.19849454, 0.25293982, -0.06241968, -0.034397375, 0.07162783, 0.09055709, -0.31490782, 0.008206952, -0.077567, 0.10753845, -0.14524877, -0.13994488, -0.09878844, -0.092522085, -0.039756894, 0.01346582 }, { 0.284406, 0.26291734, 0.042415418, -0.20023715, 0.12190845, 0.031674244, 0.030463109, -0.33974794, -0.34474048, -0.08459765, 0.30065948, -0.028295863, -0.14250802, 0.18278131, 0.09431883, -0.15824524 }, { -0.13168181, -0.08499131, -0.020841502, 0.3055297, 0.13372056, 0.16853863, -0.16442704, -0.08510389, 0.02168507, 0.037021134, 0.06461548, -0.12513623, -0.2745624, -0.19407158, -0.034411144, -0.26526874 }, { -0.019466002, 0.27668455, -0.021548308, -0.3616041, -0.03648386, -0.10374394, 0.29367504, 0.057889223, -0.11430666, 0.05480074, 0.09966011, -0.18352334, -0.007224498, -0.06664167, -0.06699556, -0.1478848 }, { 0.15338808, -0.045919407, 0.06432051, 0.115451075, -0.14949134, 0.17995146, 0.054648954, -0.004386519, -0.057394058, 0.12966143, 0.08897944, 0.01974777, -0.13734518, 0.0441782, 0.09275118, -0.024644835 }, { -0.19468597, -0.18461113, 0.1134581, -0.07104907, 0.23978914, -0.19669174, 0.1533515, 0.037013054, 0.083514325, -0.28329343, -0.080199994, 0.03739362, -0.09193031, -0.06340314, -0.12584803, 0.13784096 }, { -0.04427025, 0.0564785, -0.051710933, 0.09454595, 0.083054565, -0.25808197, 0.044539448, -0.104160935, 0.059681136, 0.11869598, 0.034190398, -0.08374204, -0.30253938, 0.06584184, -0.040300686, -0.14536151 }, { -0.024650754, 0.051230755, 0.037682824, 0.14076659, 0.19072358, 0.12604609, -0.033382796, -0.13385409, -0.111641146, 0.11936861, -0.110580005, -0.12080353, -0.09433611, 0.1335309, 0.15494978, -0.08039705 }, { 0.1262111, 0.006256448, -0.40609676, 0.16690443, -0.19997065, -0.32973313, -0.03188174, 0.11659628, -0.008932406, 0.0651717, -0.027659835, 0.1622206, -0.07270343, -0.051536553, -0.058061928, 0.14930421 }, { -0.19198424, 0.09595664, -0.08025458, -0.10022038, 0.05485618, -0.29187924, -0.04222968, -0.024616476, 0.2569949, -0.27498856, 0.1431332, -0.023538498, -0.25002834, -0.090490974, -0.019335987, -0.11289762 }, { 0.017284792, 0.17198554, -0.03080932, -0.015707728, 0.2647985, -0.24637018, -0.31681207, 0.063646935, 0.19993839, 0.023578037, 0.23249134, -0.06439831, -0.004160879, 0.08348923, -0.86669594, -0.23850487 }, { 0.00481926, -0.070838384, 0.06364217, -0.15038022, -0.22018069, -0.08010878, -0.64578235, 0.14056191, -0.10392943, -0.16209328, -0.11385174, -0.07941586, 0.0697612, 0.0017380756, 0.1383314, -0.06999649 }, { 0.14608394, -0.13316019, -0.4653074, -0.24244241, 0.15356165, -0.18467359, -0.21176134, 0.10552165, 0.08018485, -0.16020699, 0.23818803, -0.29323488, -0.06639167, -0.15374283, -0.042523272, 0.0487842 } },
    .layers_2_bias = { 0.45014703, 0.4844674, -0.1919706, -0.2196545, 0.31628186, -0.037693415, -0.22924182, 0.098835364, 0.17026757, 0.17839324, 0.02048191, -0.1856797, -0.33857697, 0.70448726, -0.38264087, -0.18791698 },
    .layers_2_activation = { 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. },
    .layers_4_weight = { { 1.45592242e-01, 7.24563524e-02, 2.49333438e-02, 7.93138966e-02, -4.94481921e-02, -3.34216952e-02 }, { 1.17518246e-01, 8.66764691e-04, 2.48027965e-03, -6.31526038e-02, 4.11878675e-02, -9.34649538e-03 }, { -1.71430022e-01, 2.70772465e-02, 4.06378359e-02, 1.08557485e-01, -1.07274689e-01, -2.35082805e-02 }, { -3.38380903e-01, -8.10074210e-02, -3.41900475e-02, -3.44318151e-02, 1.14472814e-01, 7.11029023e-02 }, { 5.24653159e-02, 1.67137802e-01, 8.00318345e-02, 4.71005663e-02, 7.21798511e-03, -1.29255876e-01 }, { -4.61021764e-03, 1.53623791e-02, 1.54796213e-01, -3.73666704e-01, -8.55849460e-02, -9.15803686e-02 }, { -2.38282397e-01, 6.14405125e-02, -1.19051412e-01, -3.65950949e-02, 3.17938030e-01, -1.46999419e-01 }, { 9.08733979e-02, 1.52269065e-01, 7.88729340e-02, -2.06939280e-01, 6.20373115e-02, -1.87127978e-01 }, { 3.20992270e-03, 1.52468652e-01, 6.97993338e-02, -2.14053560e-02, -4.20244336e-02, -1.21214241e-01 }, { 1.42114326e-01, 6.93088174e-02, -4.72381450e-02, 7.57105723e-02, -2.07999676e-01, 1.24877825e-01 }, { 1.39601767e-01, -1.06384575e-01, -7.32741803e-02, 7.87399113e-02, 2.84077413e-02, 1.16685003e-01 }, { 2.06053138e-01, 2.24650372e-02, 2.85605788e-01, 1.17308293e-02, 1.99783914e-04, 1.11808861e-02 }, { -3.68093774e-02, -1.10401008e-02, -1.50223106e-01, 2.65500367e-01, -3.66353989e-02, -2.44789105e-02 }, { 4.04083312e-01, 1.74351737e-01, 5.32501340e-02, -3.71604227e-02, 1.04778230e-01, -1.45707130e-01 }, { -9.27184075e-02, -1.15395620e-01, -3.99120934e-02, 9.82290059e-02, -1.31988615e-01, 8.57472569e-02 }, { -2.24326909e-01, 2.00241625e-01, 1.12016812e-01, -1.00176543e-01, 8.06664005e-02, 2.86627319e-02 } },
    .layers_4_bias = { 0.023507517, 0.18329471, 0.22515461, 0.051579647, -0.015534992, -0.21426041 },
    .layers_4_activation = { 0., 0., 0., 0., 0., 0. },
    .scaled_output = { 0., 0., 0., 0., 0., 0. },
    .mins = { -4.134644442689419, -5.913845286679268, -3.7609597561359407, -0.035765885026194155, -0.050201853839098476, -0.029192359070293605 },
    .maxs = { 1.2893420945167542, 2.0867967246055605, 1.8642123750686646, 0.035134838661178946, 0.04446218235534616, 0.07815751343150623 },
};

const float* nn_forward(float input[INPUT_SIZE]) {
    layer(12, 16, input, mlp.layers_0_weight, mlp.layers_0_bias, mlp.layers_0_activation, 1);
    layer(16, 16, mlp.layers_0_activation, mlp.layers_2_weight, mlp.layers_2_bias, mlp.layers_2_activation, 1);
    layer(16, 6, mlp.layers_2_activation, mlp.layers_4_weight, mlp.layers_4_bias, mlp.layers_4_activation, 0);
    for (int i = 0; i < 6; i++) {
        mlp.scaled_output[i] = (mlp.layers_4_activation[i] + 1) * 0.5f * (mlp.maxs[i] - mlp.mins[i]) + mlp.mins[i];
    }
    return mlp.scaled_output;
};
