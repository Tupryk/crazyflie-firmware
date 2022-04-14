#pragma once

#include <stdint.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

/**
   * @brief Error status returned by some functions in the library.
   */

typedef enum
{
  ARM_MATH_SUCCESS = 0,         /**< No error */
  ARM_MATH_ARGUMENT_ERROR = -1, /**< One or more arguments are incorrect */
  ARM_MATH_LENGTH_ERROR = -2,   /**< Length of data buffer is incorrect */
  ARM_MATH_SIZE_MISMATCH = -3,  /**< Size of matrices is not compatible with the operation. */
  ARM_MATH_NANINF = -4,         /**< Not-a-number (NaN) or infinity is generated */
  ARM_MATH_SINGULAR = -5,       /**< Generated by matrix inversion if the input matrix is singular and cannot be inverted. */
  ARM_MATH_TEST_FAILURE = -6    /**< Test Failed  */
} arm_status;

/**
   * @brief 32-bit floating-point type definition.
   */
typedef float float32_t;

/**
   * @brief Instance structure for the floating-point matrix structure.
   */
typedef struct
{
  uint16_t numRows; /**< number of rows of the matrix.     */
  uint16_t numCols; /**< number of columns of the matrix.  */
  float32_t *pData; /**< points to the data of the matrix. */
  } arm_matrix_instance_f32;



  /**
   * @brief Floating-point matrix transpose.
   * @param[in]  pSrc  points to the input matrix
   * @param[out] pDst  points to the output matrix
   * @return    The function returns either  <code>ARM_MATH_SIZE_MISMATCH</code>
   * or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
  arm_status arm_mat_trans_f32(
  const arm_matrix_instance_f32 * pSrc,
  arm_matrix_instance_f32 * pDst);

  /**
   * @brief Floating-point matrix inverse.
   * @param[in]  src   points to the instance of the input floating-point matrix structure.
   * @param[out] dst   points to the instance of the output floating-point matrix structure.
   * @return The function returns ARM_MATH_SIZE_MISMATCH, if the dimensions do not match.
   * If the input matrix is singular (does not have an inverse), then the algorithm terminates and returns error status ARM_MATH_SINGULAR.
   */
  arm_status arm_mat_inverse_f32(
  const arm_matrix_instance_f32 * src,
  arm_matrix_instance_f32 * dst);

  /**
   * @brief Floating-point matrix multiplication
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
  arm_status arm_mat_mult_f32(
  const arm_matrix_instance_f32 * pSrcA,
  const arm_matrix_instance_f32 * pSrcB,
  arm_matrix_instance_f32 * pDst);

  /**
   * @brief Floating-point matrix scaling.
   * @param[in]  pSrc   points to the input matrix
   * @param[in]  scale  scale factor
   * @param[out] pDst   points to the output matrix
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
  arm_status arm_mat_scale_f32(
      const arm_matrix_instance_f32 *pSrc,
      float32_t scale,
      arm_matrix_instance_f32 *pDst);

  /**
   * @brief  Floating-point square root function.
   * @param[in]  in    input value.
   * @param[out] pOut  square root of input value.
   * @return The function returns ARM_MATH_SUCCESS if input value is positive value or ARM_MATH_ARGUMENT_ERROR if
   * <code>in</code> is negative value and returns zero output for negative values.
   */
  static inline arm_status arm_sqrt_f32(
  float32_t in,
  float32_t * pOut)
  {
  	if(in >= 0.0f) {
    	*pOut = sqrtf(in);
    	return (ARM_MATH_SUCCESS);
    } else {
      *pOut = 0.0f;
      return (ARM_MATH_ARGUMENT_ERROR);
    }
  }

  /**
   * @brief  Fast approximation to the trigonometric cosine function for floating-point data.
   * @param[in] x  input value in radians.
   * @return  cos(x).
   */
  static inline float32_t arm_cos_f32(
  float32_t x)
  {
  	return cosf(x);
  }

  /**
   * @brief  Fast approximation to the trigonometric sine function for floating-point data.
   * @param[in] x  input value in radians.
   * @return  sin(x).
   */
  static inline float32_t arm_sin_f32(
  float32_t x)
  {
  	return sinf(x);
  }