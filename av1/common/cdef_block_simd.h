/*
 * Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#ifndef AOM_AV1_COMMON_CDEF_BLOCK_SIMD_H_
#define AOM_AV1_COMMON_CDEF_BLOCK_SIMD_H_

#include "config/av1_rtcd.h"

#include "av1/common/cdef_block.h"

/* partial A is a 16-bit vector of the form:
   [x8 x7 x6 x5 x4 x3 x2 x1] and partial B has the form:
   [0  y1 y2 y3 y4 y5 y6 y7].
   This function computes (x1^2+y1^2)*C1 + (x2^2+y2^2)*C2 + ...
   (x7^2+y2^7)*C7 + (x8^2+0^2)*C8 where the C1..C8 constants are in const1
   and const2. */
static INLINE v128 fold_mul_and_sum(v128 partiala, v128 partialb, v128 const1,
                                    v128 const2) {
  v128 tmp;
  /* Reverse partial B. */
  partialb = v128_shuffle_8(
      partialb, v128_from_32(0x0f0e0100, 0x03020504, 0x07060908, 0x0b0a0d0c));
  /* Interleave the x and y values of identical indices and pair x8 with 0. */
  tmp = partiala;
  partiala = v128_ziplo_16(partialb, partiala);
  partialb = v128_ziphi_16(partialb, tmp);
  /* Square and add the corresponding x and y values. */
  partiala = v128_madd_s16(partiala, partiala);
  partialb = v128_madd_s16(partialb, partialb);
  /* Multiply by constant. */
  partiala = v128_mullo_s32(partiala, const1);
  partialb = v128_mullo_s32(partialb, const2);
  /* Sum all results. */
  partiala = v128_add_32(partiala, partialb);
  return partiala;
}

static INLINE v128 hsum4(v128 x0, v128 x1, v128 x2, v128 x3) {
  v128 t0, t1, t2, t3;
  t0 = v128_ziplo_32(x1, x0);
  t1 = v128_ziplo_32(x3, x2);
  t2 = v128_ziphi_32(x1, x0);
  t3 = v128_ziphi_32(x3, x2);
  x0 = v128_ziplo_64(t1, t0);
  x1 = v128_ziphi_64(t1, t0);
  x2 = v128_ziplo_64(t3, t2);
  x3 = v128_ziphi_64(t3, t2);
  return v128_add_32(v128_add_32(x0, x1), v128_add_32(x2, x3));
}

/* Computes cost for directions 0, 5, 6 and 7. We can call this function again
   to compute the remaining directions. */
static INLINE v128 compute_directions(v128 lines[8], int32_t tmp_cost1[4]) {
  v128 partial4a, partial4b, partial5a, partial5b, partial7a, partial7b;
  v128 partial6;
  v128 tmp;
  /* Partial sums for lines 0 and 1. */
  partial4a = v128_shl_n_byte(lines[0], 14);
  partial4b = v128_shr_n_byte(lines[0], 2);
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[1], 12));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[1], 4));
  tmp = v128_add_16(lines[0], lines[1]);
  partial5a = v128_shl_n_byte(tmp, 10);
  partial5b = v128_shr_n_byte(tmp, 6);
  partial7a = v128_shl_n_byte(tmp, 4);
  partial7b = v128_shr_n_byte(tmp, 12);
  partial6 = tmp;

  /* Partial sums for lines 2 and 3. */
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[2], 10));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[2], 6));
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[3], 8));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[3], 8));
  tmp = v128_add_16(lines[2], lines[3]);
  partial5a = v128_add_16(partial5a, v128_shl_n_byte(tmp, 8));
  partial5b = v128_add_16(partial5b, v128_shr_n_byte(tmp, 8));
  partial7a = v128_add_16(partial7a, v128_shl_n_byte(tmp, 6));
  partial7b = v128_add_16(partial7b, v128_shr_n_byte(tmp, 10));
  partial6 = v128_add_16(partial6, tmp);

  /* Partial sums for lines 4 and 5. */
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[4], 6));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[4], 10));
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[5], 4));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[5], 12));
  tmp = v128_add_16(lines[4], lines[5]);
  partial5a = v128_add_16(partial5a, v128_shl_n_byte(tmp, 6));
  partial5b = v128_add_16(partial5b, v128_shr_n_byte(tmp, 10));
  partial7a = v128_add_16(partial7a, v128_shl_n_byte(tmp, 8));
  partial7b = v128_add_16(partial7b, v128_shr_n_byte(tmp, 8));
  partial6 = v128_add_16(partial6, tmp);

  /* Partial sums for lines 6 and 7. */
  partial4a = v128_add_16(partial4a, v128_shl_n_byte(lines[6], 2));
  partial4b = v128_add_16(partial4b, v128_shr_n_byte(lines[6], 14));
  partial4a = v128_add_16(partial4a, lines[7]);
  tmp = v128_add_16(lines[6], lines[7]);
  partial5a = v128_add_16(partial5a, v128_shl_n_byte(tmp, 4));
  partial5b = v128_add_16(partial5b, v128_shr_n_byte(tmp, 12));
  partial7a = v128_add_16(partial7a, v128_shl_n_byte(tmp, 10));
  partial7b = v128_add_16(partial7b, v128_shr_n_byte(tmp, 6));
  partial6 = v128_add_16(partial6, tmp);

  /* Compute costs in terms of partial sums. */
  partial4a =
      fold_mul_and_sum(partial4a, partial4b, v128_from_32(210, 280, 420, 840),
                       v128_from_32(105, 120, 140, 168));
  partial7a =
      fold_mul_and_sum(partial7a, partial7b, v128_from_32(210, 420, 0, 0),
                       v128_from_32(105, 105, 105, 140));
  partial5a =
      fold_mul_and_sum(partial5a, partial5b, v128_from_32(210, 420, 0, 0),
                       v128_from_32(105, 105, 105, 140));
  partial6 = v128_madd_s16(partial6, partial6);
  partial6 = v128_mullo_s32(partial6, v128_dup_32(105));

  partial4a = hsum4(partial4a, partial5a, partial6, partial7a);
  v128_store_unaligned(tmp_cost1, partial4a);
  return partial4a;
}

/* transpose and reverse the order of the lines -- equivalent to a 90-degree
   counter-clockwise rotation of the pixels. */
static INLINE void array_reverse_transpose_8x8(v128 *in, v128 *res) {
  const v128 tr0_0 = v128_ziplo_16(in[1], in[0]);
  const v128 tr0_1 = v128_ziplo_16(in[3], in[2]);
  const v128 tr0_2 = v128_ziphi_16(in[1], in[0]);
  const v128 tr0_3 = v128_ziphi_16(in[3], in[2]);
  const v128 tr0_4 = v128_ziplo_16(in[5], in[4]);
  const v128 tr0_5 = v128_ziplo_16(in[7], in[6]);
  const v128 tr0_6 = v128_ziphi_16(in[5], in[4]);
  const v128 tr0_7 = v128_ziphi_16(in[7], in[6]);

  const v128 tr1_0 = v128_ziplo_32(tr0_1, tr0_0);
  const v128 tr1_1 = v128_ziplo_32(tr0_5, tr0_4);
  const v128 tr1_2 = v128_ziphi_32(tr0_1, tr0_0);
  const v128 tr1_3 = v128_ziphi_32(tr0_5, tr0_4);
  const v128 tr1_4 = v128_ziplo_32(tr0_3, tr0_2);
  const v128 tr1_5 = v128_ziplo_32(tr0_7, tr0_6);
  const v128 tr1_6 = v128_ziphi_32(tr0_3, tr0_2);
  const v128 tr1_7 = v128_ziphi_32(tr0_7, tr0_6);

  res[7] = v128_ziplo_64(tr1_1, tr1_0);
  res[6] = v128_ziphi_64(tr1_1, tr1_0);
  res[5] = v128_ziplo_64(tr1_3, tr1_2);
  res[4] = v128_ziphi_64(tr1_3, tr1_2);
  res[3] = v128_ziplo_64(tr1_5, tr1_4);
  res[2] = v128_ziphi_64(tr1_5, tr1_4);
  res[1] = v128_ziplo_64(tr1_7, tr1_6);
  res[0] = v128_ziphi_64(tr1_7, tr1_6);
}

int SIMD_FUNC(cdef_find_dir)(const uint16_t *img, int stride, int32_t *var,
                             int coeff_shift) {
  int i;
  int32_t cost[8];
  int32_t best_cost = 0;
  int best_dir = 0;
  v128 lines[8];
  for (i = 0; i < 8; i++) {
    lines[i] = v128_load_unaligned(&img[i * stride]);
    lines[i] =
        v128_sub_16(v128_shr_s16(lines[i], coeff_shift), v128_dup_16(128));
  }

  /* Compute "mostly vertical" directions. */
  v128 dir47 = compute_directions(lines, cost + 4);

  array_reverse_transpose_8x8(lines, lines);

  /* Compute "mostly horizontal" directions. */
  v128 dir03 = compute_directions(lines, cost);

  v128 max = v128_max_s32(dir03, dir47);
  max = v128_max_s32(max, v128_align(max, max, 8));
  max = v128_max_s32(max, v128_align(max, max, 4));
  best_cost = v128_low_u32(max);
  v128 t =
      v128_pack_s32_s16(v128_cmpeq_32(max, dir47), v128_cmpeq_32(max, dir03));
  best_dir = v128_movemask_8(v128_pack_s16_s8(t, t));
  best_dir = get_msb(best_dir ^ (best_dir - 1));  // Count trailing zeros

  /* Difference between the optimal variance and the variance along the
     orthogonal direction. Again, the sum(x^2) terms cancel out. */
  *var = best_cost - cost[(best_dir + 4) & 7];
  /* We'd normally divide by 840, but dividing by 1024 is close enough
     for what we're going to do with this. */
  *var >>= 10;
  return best_dir;
}

// sign(a-b) * min(abs(a-b), max(0, threshold - (abs(a-b) >> adjdamp)))
SIMD_INLINE v256 constrain16(v256 a, v256 b, unsigned int threshold,
                             unsigned int adjdamp) {
  v256 diff = v256_sub_16(a, b);
  const v256 sign = v256_shr_n_s16(diff, 15);
  diff = v256_abs_s16(diff);
  const v256 s =
      v256_ssub_u16(v256_dup_16(threshold), v256_shr_u16(diff, adjdamp));
  return v256_xor(v256_add_16(sign, v256_min_s16(diff, s)), sign);
}

// sign(a - b) * min(abs(a - b), max(0, strength - (abs(a - b) >> adjdamp)))
SIMD_INLINE v128 constrain(v256 a, v256 b, unsigned int strength,
                           unsigned int adjdamp) {
  const v256 diff16 = v256_sub_16(a, b);
  v128 diff = v128_pack_s16_s8(v256_high_v128(diff16), v256_low_v128(diff16));
  const v128 sign = v128_cmplt_s8(diff, v128_zero());
  diff = v128_abs_s8(diff);
  return v128_xor(
      v128_add_8(sign,
                 v128_min_u8(diff, v128_ssub_u8(v128_dup_8(strength),
                                                v128_shr_u8(diff, adjdamp)))),
      sign);
}

void SIMD_FUNC(cdef_filter_block_4x4_8)(uint8_t *dst, int dstride,
                                        const uint16_t *in, int pri_strength,
                                        int sec_strength, int dir,
                                        int pri_damping, int sec_damping,
                                        int coeff_shift) {
  v128 p0, p1, p2, p3;
  v256 sum, row, res, tap;
  v256 max, min;
  v256 cdef_large_value_mask = v256_dup_16((uint16_t)~CDEF_VERY_LARGE);
  int po1 = cdef_directions[dir][0];
  int po2 = cdef_directions[dir][1];
  int s1o1 = cdef_directions[dir + 2][0];
  int s1o2 = cdef_directions[dir + 2][1];
  int s2o1 = cdef_directions[dir - 2][0];
  int s2o2 = cdef_directions[dir - 2][1];
  const int *pri_taps = cdef_pri_taps[(pri_strength >> coeff_shift) & 1];
  const int *sec_taps = cdef_sec_taps;

  if (pri_strength)
    pri_damping = AOMMAX(0, pri_damping - get_msb(pri_strength));
  if (sec_strength)
    sec_damping = AOMMAX(0, sec_damping - get_msb(sec_strength));

  sum = v256_zero();
  row = v256_from_v64(v64_load_aligned(&in[0 * CDEF_BSTRIDE]),
                      v64_load_aligned(&in[1 * CDEF_BSTRIDE]),
                      v64_load_aligned(&in[2 * CDEF_BSTRIDE]),
                      v64_load_aligned(&in[3 * CDEF_BSTRIDE]));
  max = min = row;

  if (pri_strength) {
    v256 max_u8;
    // Primary near taps
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + po1]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE + po1]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE + po1]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE + po1]));
    max_u8 = tap;
    min = v256_min_s16(min, tap);
    p0 = constrain(tap, row, pri_strength, pri_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - po1]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE - po1]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE - po1]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE - po1]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p1 = constrain(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[0] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[0]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));

    // Primary far taps
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + po2]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE + po2]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE + po2]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE + po2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p0 = constrain(tap, row, pri_strength, pri_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - po2]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE - po2]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE - po2]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE - po2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p1 = constrain(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[1] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[1]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));
    /* The source is 16 bits, however, we only really care about the lower
    8 bits.  The upper 8 bits contain the "large" flag.  After the final
    primary max has been calculated, zero out the upper 8 bits.  Use this
    to find the "16 bit" max. */
    max = v256_max_s16(max, v256_and(max_u8, cdef_large_value_mask));
  }

  if (sec_strength) {
    v256 max_u8;
    // Secondary near taps
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + s1o1]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE + s1o1]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE + s1o1]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE + s1o1]));
    max_u8 = tap;
    min = v256_min_s16(min, tap);
    p0 = constrain(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - s1o1]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE - s1o1]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE - s1o1]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE - s1o1]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p1 = constrain(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + s2o1]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE + s2o1]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE + s2o1]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE + s2o1]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p2 = constrain(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - s2o1]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE - s2o1]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE - s2o1]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE - s2o1]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p3 = constrain(tap, row, sec_strength, sec_damping);

    // sum += sec_taps[0] * (p0 + p1 + p2 + p3)
    p0 = v128_add_8(p0, p1);
    p2 = v128_add_8(p2, p3);
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(sec_taps[0]),
                                         v256_from_v128(v128_ziphi_8(p0, p2),
                                                        v128_ziplo_8(p0, p2))));

    // Secondary far taps
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + s1o2]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE + s1o2]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE + s1o2]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE + s1o2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p0 = constrain(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - s1o2]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE - s1o2]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE - s1o2]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE - s1o2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p1 = constrain(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + s2o2]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE + s2o2]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE + s2o2]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE + s2o2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p2 = constrain(tap, row, sec_strength, sec_damping);
    tap = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - s2o2]),
                        v64_load_unaligned(&in[1 * CDEF_BSTRIDE - s2o2]),
                        v64_load_unaligned(&in[2 * CDEF_BSTRIDE - s2o2]),
                        v64_load_unaligned(&in[3 * CDEF_BSTRIDE - s2o2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p3 = constrain(tap, row, sec_strength, sec_damping);

    // sum += sec_taps[1] * (p0 + p1 + p2 + p3)
    p0 = v128_add_8(p0, p1);
    p2 = v128_add_8(p2, p3);

    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(sec_taps[1]),
                                         v256_from_v128(v128_ziphi_8(p0, p2),
                                                        v128_ziplo_8(p0, p2))));

    /* The source is 16 bits, however, we only really care about the lower
    8 bits.  The upper 8 bits contain the "large" flag.  After the final
    primary max has been calculated, zero out the upper 8 bits.  Use this
    to find the "16 bit" max. */
    max = v256_max_s16(max, v256_and(max_u8, cdef_large_value_mask));
  }

  // res = row + ((sum - (sum < 0) + 8) >> 4)
  sum = v256_add_16(sum, v256_cmplt_s16(sum, v256_zero()));
  res = v256_add_16(sum, v256_dup_16(8));
  res = v256_shr_n_s16(res, 4);
  res = v256_add_16(row, res);
  res = v256_min_s16(v256_max_s16(res, min), max);
  res = v256_pack_s16_u8(res, res);

  p0 = v256_low_v128(res);
  u32_store_aligned(&dst[0 * dstride], v64_high_u32(v128_high_v64(p0)));
  u32_store_aligned(&dst[1 * dstride], v64_low_u32(v128_high_v64(p0)));
  u32_store_aligned(&dst[2 * dstride], v64_high_u32(v128_low_v64(p0)));
  u32_store_aligned(&dst[3 * dstride], v64_low_u32(v128_low_v64(p0)));
}

void SIMD_FUNC(cdef_filter_block_8x8_8)(uint8_t *dst, int dstride,
                                        const uint16_t *in, int pri_strength,
                                        int sec_strength, int dir,
                                        int pri_damping, int sec_damping,
                                        int coeff_shift) {
  int i;
  v128 p0, p1, p2, p3;
  v256 sum, row, res, tap;
  v256 max, min;
  v256 cdef_large_value_mask = v256_dup_16((uint16_t)~CDEF_VERY_LARGE);
  int po1 = cdef_directions[dir][0];
  int po2 = cdef_directions[dir][1];
  int s1o1 = cdef_directions[dir + 2][0];
  int s1o2 = cdef_directions[dir + 2][1];
  int s2o1 = cdef_directions[dir - 2][0];
  int s2o2 = cdef_directions[dir - 2][1];
  const int *pri_taps = cdef_pri_taps[(pri_strength >> coeff_shift) & 1];
  const int *sec_taps = cdef_sec_taps;

  if (pri_strength)
    pri_damping = AOMMAX(0, pri_damping - get_msb(pri_strength));
  if (sec_strength)
    sec_damping = AOMMAX(0, sec_damping - get_msb(sec_strength));
  for (i = 0; i < 8; i += 2) {
    v256 max_u8;
    sum = v256_zero();
    row = v256_from_v128(v128_load_aligned(&in[i * CDEF_BSTRIDE]),
                         v128_load_aligned(&in[(i + 1) * CDEF_BSTRIDE]));

    max = min = row;
    // Primary near taps
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + po1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + po1]));
    max_u8 = tap;
    min = v256_min_s16(min, tap);
    p0 = constrain(tap, row, pri_strength, pri_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - po1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - po1]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p1 = constrain(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[0] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[0]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));

    // Primary far taps
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + po2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + po2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p0 = constrain(tap, row, pri_strength, pri_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - po2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - po2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p1 = constrain(tap, row, pri_strength, pri_damping);

    // sum += pri_taps[1] * (p0 + p1)
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(pri_taps[1]),
                                         v256_from_v128(v128_ziphi_8(p0, p1),
                                                        v128_ziplo_8(p0, p1))));

    max = v256_max_s16(max, v256_and(max_u8, cdef_large_value_mask));

    // Secondary near taps
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + s1o1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + s1o1]));
    max_u8 = tap;
    min = v256_min_s16(min, tap);
    p0 = constrain(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - s1o1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - s1o1]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p1 = constrain(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + s2o1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + s2o1]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p2 = constrain(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - s2o1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - s2o1]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p3 = constrain(tap, row, sec_strength, sec_damping);

    // sum += sec_taps[0] * (p0 + p1 + p2 + p3)
    p0 = v128_add_8(p0, p1);
    p2 = v128_add_8(p2, p3);
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(sec_taps[0]),
                                         v256_from_v128(v128_ziphi_8(p0, p2),
                                                        v128_ziplo_8(p0, p2))));

    // Secondary far taps
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + s1o2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + s1o2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p0 = constrain(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - s1o2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - s1o2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p1 = constrain(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + s2o2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + s2o2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p2 = constrain(tap, row, sec_strength, sec_damping);
    tap =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - s2o2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - s2o2]));
    max_u8 = v256_max_u8(max_u8, tap);
    min = v256_min_s16(min, tap);
    p3 = constrain(tap, row, sec_strength, sec_damping);

    // sum += sec_taps[1] * (p0 + p1 + p2 + p3)
    p0 = v128_add_8(p0, p1);
    p2 = v128_add_8(p2, p3);
    sum = v256_add_16(sum, v256_madd_us8(v256_dup_8(sec_taps[1]),
                                         v256_from_v128(v128_ziphi_8(p0, p2),
                                                        v128_ziplo_8(p0, p2))));

    max = v256_max_s16(max, v256_and(max_u8, cdef_large_value_mask));

    // res = row + ((sum - (sum < 0) + 8) >> 4)
    sum = v256_add_16(sum, v256_cmplt_s16(sum, v256_zero()));
    res = v256_add_16(sum, v256_dup_16(8));
    res = v256_shr_n_s16(res, 4);
    res = v256_add_16(row, res);
    res = v256_min_s16(v256_max_s16(res, min), max);
    res = v256_pack_s16_u8(res, res);

    p0 = v256_low_v128(res);
    v64_store_aligned(&dst[i * dstride], v128_high_v64(p0));
    v64_store_aligned(&dst[(i + 1) * dstride], v128_low_v64(p0));
  }
}

void SIMD_FUNC(cdef_filter_block_4x4_16)(uint16_t *dst, int dstride,
                                         const uint16_t *in, int pri_strength,
                                         int sec_strength, int dir,
                                         int pri_damping, int sec_damping,
                                         int coeff_shift) {
  v256 p0, p1, p2, p3, sum, row, res;
  v256 max, min, large = v256_dup_16(CDEF_VERY_LARGE);
  int po1 = cdef_directions[dir][0];
  int po2 = cdef_directions[dir][1];
  int s1o1 = cdef_directions[dir + 2][0];
  int s1o2 = cdef_directions[dir + 2][1];
  int s2o1 = cdef_directions[dir - 2][0];
  int s2o2 = cdef_directions[dir - 2][1];
  const int *pri_taps = cdef_pri_taps[(pri_strength >> coeff_shift) & 1];
  const int *sec_taps = cdef_sec_taps;

  if (pri_strength)
    pri_damping = AOMMAX(0, pri_damping - get_msb(pri_strength));
  if (sec_strength)
    sec_damping = AOMMAX(0, sec_damping - get_msb(sec_strength));

  sum = v256_zero();
  row = v256_from_v64(v64_load_aligned(&in[0 * CDEF_BSTRIDE]),
                      v64_load_aligned(&in[1 * CDEF_BSTRIDE]),
                      v64_load_aligned(&in[2 * CDEF_BSTRIDE]),
                      v64_load_aligned(&in[3 * CDEF_BSTRIDE]));
  min = max = row;

  // Primary near taps
  p0 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + po1]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE + po1]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE + po1]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE + po1]));
  p1 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - po1]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE - po1]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE - po1]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE - po1]));
  max = v256_max_s16(v256_max_s16(max, v256_andn(p0, v256_cmpeq_16(p0, large))),
                     v256_andn(p1, v256_cmpeq_16(p1, large)));
  min = v256_min_s16(v256_min_s16(min, p0), p1);
  p0 = constrain16(p0, row, pri_strength, pri_damping);
  p1 = constrain16(p1, row, pri_strength, pri_damping);

  // sum += pri_taps[0] * (p0 + p1)
  sum = v256_add_16(
      sum, v256_mullo_s16(v256_dup_16(pri_taps[0]), v256_add_16(p0, p1)));

  // Primary far taps
  p0 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + po2]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE + po2]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE + po2]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE + po2]));
  p1 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - po2]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE - po2]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE - po2]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE - po2]));
  max = v256_max_s16(v256_max_s16(max, v256_andn(p0, v256_cmpeq_16(p0, large))),
                     v256_andn(p1, v256_cmpeq_16(p1, large)));
  min = v256_min_s16(v256_min_s16(min, p0), p1);
  p0 = constrain16(p0, row, pri_strength, pri_damping);
  p1 = constrain16(p1, row, pri_strength, pri_damping);

  // sum += pri_taps[1] * (p0 + p1)
  sum = v256_add_16(
      sum, v256_mullo_s16(v256_dup_16(pri_taps[1]), v256_add_16(p0, p1)));

  // Secondary near taps
  p0 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + s1o1]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE + s1o1]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE + s1o1]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE + s1o1]));
  p1 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - s1o1]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE - s1o1]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE - s1o1]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE - s1o1]));
  p2 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + s2o1]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE + s2o1]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE + s2o1]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE + s2o1]));
  p3 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - s2o1]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE - s2o1]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE - s2o1]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE - s2o1]));
  max = v256_max_s16(v256_max_s16(max, v256_andn(p0, v256_cmpeq_16(p0, large))),
                     v256_andn(p1, v256_cmpeq_16(p1, large)));
  max = v256_max_s16(v256_max_s16(max, v256_andn(p2, v256_cmpeq_16(p2, large))),
                     v256_andn(p3, v256_cmpeq_16(p3, large)));
  min = v256_min_s16(v256_min_s16(v256_min_s16(v256_min_s16(min, p0), p1), p2),
                     p3);
  p0 = constrain16(p0, row, sec_strength, sec_damping);
  p1 = constrain16(p1, row, sec_strength, sec_damping);
  p2 = constrain16(p2, row, sec_strength, sec_damping);
  p3 = constrain16(p3, row, sec_strength, sec_damping);

  // sum += sec_taps[0] * (p0 + p1 + p2 + p3)
  sum = v256_add_16(sum, v256_mullo_s16(v256_dup_16(sec_taps[0]),
                                        v256_add_16(v256_add_16(p0, p1),
                                                    v256_add_16(p2, p3))));

  // Secondary far taps
  p0 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + s1o2]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE + s1o2]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE + s1o2]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE + s1o2]));
  p1 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - s1o2]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE - s1o2]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE - s1o2]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE - s1o2]));
  p2 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE + s2o2]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE + s2o2]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE + s2o2]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE + s2o2]));
  p3 = v256_from_v64(v64_load_unaligned(&in[0 * CDEF_BSTRIDE - s2o2]),
                     v64_load_unaligned(&in[1 * CDEF_BSTRIDE - s2o2]),
                     v64_load_unaligned(&in[2 * CDEF_BSTRIDE - s2o2]),
                     v64_load_unaligned(&in[3 * CDEF_BSTRIDE - s2o2]));
  max = v256_max_s16(v256_max_s16(max, v256_andn(p0, v256_cmpeq_16(p0, large))),
                     v256_andn(p1, v256_cmpeq_16(p1, large)));
  max = v256_max_s16(v256_max_s16(max, v256_andn(p2, v256_cmpeq_16(p2, large))),
                     v256_andn(p3, v256_cmpeq_16(p3, large)));
  min = v256_min_s16(v256_min_s16(v256_min_s16(v256_min_s16(min, p0), p1), p2),
                     p3);
  p0 = constrain16(p0, row, sec_strength, sec_damping);
  p1 = constrain16(p1, row, sec_strength, sec_damping);
  p2 = constrain16(p2, row, sec_strength, sec_damping);
  p3 = constrain16(p3, row, sec_strength, sec_damping);

  // sum += sec_taps[1] * (p0 + p1 + p2 + p3)
  sum = v256_add_16(sum, v256_mullo_s16(v256_dup_16(sec_taps[1]),
                                        v256_add_16(v256_add_16(p0, p1),
                                                    v256_add_16(p2, p3))));

  // res = row + ((sum - (sum < 0) + 8) >> 4)
  sum = v256_add_16(sum, v256_cmplt_s16(sum, v256_zero()));
  res = v256_add_16(sum, v256_dup_16(8));
  res = v256_shr_n_s16(res, 4);
  res = v256_add_16(row, res);
  res = v256_min_s16(v256_max_s16(res, min), max);

  v64_store_aligned(&dst[0 * dstride], v128_high_v64(v256_high_v128(res)));
  v64_store_aligned(&dst[1 * dstride], v128_low_v64(v256_high_v128(res)));
  v64_store_aligned(&dst[2 * dstride], v128_high_v64(v256_low_v128(res)));
  v64_store_aligned(&dst[3 * dstride], v128_low_v64(v256_low_v128(res)));
}

void SIMD_FUNC(cdef_filter_block_8x8_16)(uint16_t *dst, int dstride,
                                         const uint16_t *in, int pri_strength,
                                         int sec_strength, int dir,
                                         int pri_damping, int sec_damping,
                                         int coeff_shift) {
  int i;
  v256 sum, p0, p1, p2, p3, row, res;
  v256 max, min, large = v256_dup_16(CDEF_VERY_LARGE);
  int po1 = cdef_directions[dir][0];
  int po2 = cdef_directions[dir][1];
  int s1o1 = cdef_directions[dir + 2][0];
  int s1o2 = cdef_directions[dir + 2][1];
  int s2o1 = cdef_directions[dir - 2][0];
  int s2o2 = cdef_directions[dir - 2][1];
  const int *pri_taps = cdef_pri_taps[(pri_strength >> coeff_shift) & 1];
  const int *sec_taps = cdef_sec_taps;

  if (pri_strength)
    pri_damping = AOMMAX(0, pri_damping - get_msb(pri_strength));
  if (sec_strength)
    sec_damping = AOMMAX(0, sec_damping - get_msb(sec_strength));

  for (i = 0; i < 8; i += 2) {
    sum = v256_zero();
    row = v256_from_v128(v128_load_aligned(&in[i * CDEF_BSTRIDE]),
                         v128_load_aligned(&in[(i + 1) * CDEF_BSTRIDE]));

    min = max = row;
    // Primary near taps
    p0 = v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + po1]),
                        v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + po1]));
    p1 = v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - po1]),
                        v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - po1]));
    max =
        v256_max_s16(v256_max_s16(max, v256_andn(p0, v256_cmpeq_16(p0, large))),
                     v256_andn(p1, v256_cmpeq_16(p1, large)));
    min = v256_min_s16(v256_min_s16(min, p0), p1);
    p0 = constrain16(p0, row, pri_strength, pri_damping);
    p1 = constrain16(p1, row, pri_strength, pri_damping);

    // sum += pri_taps[0] * (p0 + p1)
    sum = v256_add_16(
        sum, v256_mullo_s16(v256_dup_16(pri_taps[0]), v256_add_16(p0, p1)));

    // Primary far taps
    p0 = v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + po2]),
                        v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + po2]));
    p1 = v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - po2]),
                        v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - po2]));
    max =
        v256_max_s16(v256_max_s16(max, v256_andn(p0, v256_cmpeq_16(p0, large))),
                     v256_andn(p1, v256_cmpeq_16(p1, large)));
    min = v256_min_s16(v256_min_s16(min, p0), p1);
    p0 = constrain16(p0, row, pri_strength, pri_damping);
    p1 = constrain16(p1, row, pri_strength, pri_damping);

    // sum += pri_taps[1] * (p0 + p1)
    sum = v256_add_16(
        sum, v256_mullo_s16(v256_dup_16(pri_taps[1]), v256_add_16(p0, p1)));

    // Secondary near taps
    p0 =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + s1o1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + s1o1]));
    p1 =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - s1o1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - s1o1]));
    p2 =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + s2o1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + s2o1]));
    p3 =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - s2o1]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - s2o1]));
    max =
        v256_max_s16(v256_max_s16(max, v256_andn(p0, v256_cmpeq_16(p0, large))),
                     v256_andn(p1, v256_cmpeq_16(p1, large)));
    max =
        v256_max_s16(v256_max_s16(max, v256_andn(p2, v256_cmpeq_16(p2, large))),
                     v256_andn(p3, v256_cmpeq_16(p3, large)));
    min = v256_min_s16(
        v256_min_s16(v256_min_s16(v256_min_s16(min, p0), p1), p2), p3);
    p0 = constrain16(p0, row, sec_strength, sec_damping);
    p1 = constrain16(p1, row, sec_strength, sec_damping);
    p2 = constrain16(p2, row, sec_strength, sec_damping);
    p3 = constrain16(p3, row, sec_strength, sec_damping);

    // sum += sec_taps[0] * (p0 + p1 + p2 + p3)
    sum = v256_add_16(sum, v256_mullo_s16(v256_dup_16(sec_taps[0]),
                                          v256_add_16(v256_add_16(p0, p1),
                                                      v256_add_16(p2, p3))));

    // Secondary far taps
    p0 =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + s1o2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + s1o2]));
    p1 =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - s1o2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - s1o2]));
    p2 =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE + s2o2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE + s2o2]));
    p3 =
        v256_from_v128(v128_load_unaligned(&in[i * CDEF_BSTRIDE - s2o2]),
                       v128_load_unaligned(&in[(i + 1) * CDEF_BSTRIDE - s2o2]));
    max =
        v256_max_s16(v256_max_s16(max, v256_andn(p0, v256_cmpeq_16(p0, large))),
                     v256_andn(p1, v256_cmpeq_16(p1, large)));
    max =
        v256_max_s16(v256_max_s16(max, v256_andn(p2, v256_cmpeq_16(p2, large))),
                     v256_andn(p3, v256_cmpeq_16(p3, large)));
    min = v256_min_s16(
        v256_min_s16(v256_min_s16(v256_min_s16(min, p0), p1), p2), p3);
    p0 = constrain16(p0, row, sec_strength, sec_damping);
    p1 = constrain16(p1, row, sec_strength, sec_damping);
    p2 = constrain16(p2, row, sec_strength, sec_damping);
    p3 = constrain16(p3, row, sec_strength, sec_damping);

    // sum += sec_taps[1] * (p0 + p1 + p2 + p3)
    sum = v256_add_16(sum, v256_mullo_s16(v256_dup_16(sec_taps[1]),
                                          v256_add_16(v256_add_16(p0, p1),
                                                      v256_add_16(p2, p3))));

    // res = row + ((sum - (sum < 0) + 8) >> 4)
    sum = v256_add_16(sum, v256_cmplt_s16(sum, v256_zero()));
    res = v256_add_16(sum, v256_dup_16(8));
    res = v256_shr_n_s16(res, 4);
    res = v256_add_16(row, res);
    res = v256_min_s16(v256_max_s16(res, min), max);
    v128_store_unaligned(&dst[i * dstride], v256_high_v128(res));
    v128_store_unaligned(&dst[(i + 1) * dstride], v256_low_v128(res));
  }
}

void SIMD_FUNC(cdef_filter_block)(uint8_t *dst8, uint16_t *dst16, int dstride,
                                  const uint16_t *in, int pri_strength,
                                  int sec_strength, int dir, int pri_damping,
                                  int sec_damping, int bsize, int coeff_shift) {
  if (dst8) {
    if (bsize == BLOCK_8X8) {
      SIMD_FUNC(cdef_filter_block_8x8_8)
      (dst8, dstride, in, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
    } else if (bsize == BLOCK_4X8) {
      SIMD_FUNC(cdef_filter_block_4x4_8)
      (dst8, dstride, in, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
      SIMD_FUNC(cdef_filter_block_4x4_8)
      (dst8 + 4 * dstride, dstride, in + 4 * CDEF_BSTRIDE, pri_strength,
       sec_strength, dir, pri_damping, sec_damping, coeff_shift);
    } else if (bsize == BLOCK_8X4) {
      SIMD_FUNC(cdef_filter_block_4x4_8)
      (dst8, dstride, in, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
      SIMD_FUNC(cdef_filter_block_4x4_8)
      (dst8 + 4, dstride, in + 4, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
    } else {
      SIMD_FUNC(cdef_filter_block_4x4_8)
      (dst8, dstride, in, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
    }
  } else {
    if (bsize == BLOCK_8X8) {
      SIMD_FUNC(cdef_filter_block_8x8_16)
      (dst16, dstride, in, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
    } else if (bsize == BLOCK_4X8) {
      SIMD_FUNC(cdef_filter_block_4x4_16)
      (dst16, dstride, in, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
      SIMD_FUNC(cdef_filter_block_4x4_16)
      (dst16 + 4 * dstride, dstride, in + 4 * CDEF_BSTRIDE, pri_strength,
       sec_strength, dir, pri_damping, sec_damping, coeff_shift);
    } else if (bsize == BLOCK_8X4) {
      SIMD_FUNC(cdef_filter_block_4x4_16)
      (dst16, dstride, in, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
      SIMD_FUNC(cdef_filter_block_4x4_16)
      (dst16 + 4, dstride, in + 4, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
    } else {
      assert(bsize == BLOCK_4X4);
      SIMD_FUNC(cdef_filter_block_4x4_16)
      (dst16, dstride, in, pri_strength, sec_strength, dir, pri_damping,
       sec_damping, coeff_shift);
    }
  }
}

void SIMD_FUNC(cdef_copy_rect8_8bit_to_16bit)(uint16_t *dst, int dstride,
                                              const uint8_t *src, int sstride,
                                              int v, int h) {
  int i, j;
  for (i = 0; i < v; i++) {
    for (j = 0; j < (h & ~0x7); j += 8) {
      v64 row = v64_load_unaligned(&src[i * sstride + j]);
      v128_store_unaligned(&dst[i * dstride + j], v128_unpack_u8_s16(row));
    }
    for (; j < h; j++) {
      dst[i * dstride + j] = src[i * sstride + j];
    }
  }
}

void SIMD_FUNC(cdef_copy_rect8_16bit_to_16bit)(uint16_t *dst, int dstride,
                                               const uint16_t *src, int sstride,
                                               int v, int h) {
  int i, j;
  for (i = 0; i < v; i++) {
    for (j = 0; j < (h & ~0x7); j += 8) {
      v128 row = v128_load_unaligned(&src[i * sstride + j]);
      v128_store_unaligned(&dst[i * dstride + j], row);
    }
    for (; j < h; j++) {
      dst[i * dstride + j] = src[i * sstride + j];
    }
  }
}

#endif  // AOM_AV1_COMMON_CDEF_BLOCK_SIMD_H_
