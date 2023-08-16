/*
 * Copyright (c) 2021, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#include <math.h>

#include "av1/encoder/tune_butteraugli.h"

#include "aom_dsp/butteraugli.h"
#include "av1/encoder/encodeframe.h"
#include "av1/encoder/encoder_utils.h"
#include "av1/encoder/extend.h"
#include "av1/encoder/var_based_part.h"
#include "aom_ports/mem.h"
#include "av1/encoder/rdopt.h"

// static const int resize_factor = 2;

static void set_mb_butteraugli_rdmult_scaling(AV1_COMP *cpi,
                                              const YV12_BUFFER_CONFIG *source,
                                              const YV12_BUFFER_CONFIG *recon,
                                              const double K) {
  AV1_COMMON *const cm = &cpi->common;
  SequenceHeader *const seq_params = cm->seq_params;
  const CommonModeInfoParams *const mi_params = &cm->mi_params;
  const aom_color_range_t color_range =
      seq_params->color_range != 0 ? AOM_CR_FULL_RANGE : AOM_CR_STUDIO_RANGE;
  const int bit_depth = cpi->td.mb.e_mbd.bd;
  const int width = source->y_crop_width;
  const int height = source->y_crop_height;
  const int ss_x = source->subsampling_x;
  const int ss_y = source->subsampling_y;
  const MACROBLOCKD *const xd = &cpi->td.mb.e_mbd;
  cpi->butteraugli_info.total_dbutteraugli = 0.0f;
  /*const int resize_factor = (cpi->oxcf.butteraugli_resize_factor == 0)
                                ? 1
                                : (cpi->oxcf.butteraugli_resize_factor == 1)
                                      ? 2
                                      : (cpi->oxcf.butteraugli_resize_factor ==
     2) ? 4 : 2;*/
  const BLOCK_SIZE rdo_bsize =
      (cpi->oxcf.butteraugli_resize_factor == 0)   ? BLOCK_32X32
      : (cpi->oxcf.butteraugli_resize_factor == 1) ? BLOCK_16X16
      : (cpi->oxcf.butteraugli_resize_factor == 2) ? BLOCK_8X8
                                                   : BLOCK_16X16;
  const BLOCK_SIZE butteraugli_rdo_bsize = BLOCK_32X32;
  float *diffmap;
  CHECK_MEM_ERROR(cm, diffmap, aom_malloc(width * height * sizeof(*diffmap)));
  if (!aom_calc_butteraugli(cpi, source, recon, bit_depth,
                            seq_params->matrix_coefficients, color_range,
                            diffmap, cpi->oxcf.butteraugli_intensity_target,
                            cpi->oxcf.butteraugli_hf_asymmetry)) {
    aom_internal_error(cm->error, AOM_CODEC_ERROR,
                       "Failed to calculate Butteraugli distances.");
  }

  const int num_mi_w = mi_size_wide[butteraugli_rdo_bsize];
  const int num_mi_h = mi_size_high[butteraugli_rdo_bsize];
  const int num_cols = (mi_params->mi_cols + num_mi_w - 1) / num_mi_w;
  const int num_rows = (mi_params->mi_rows + num_mi_h - 1) / num_mi_h;
  const int block_w = mi_size_wide[rdo_bsize];
  const int block_h = mi_size_high[rdo_bsize];
  double log_sum = 0.0;
  double quant_log_sum = 0.0;
  double blk_count = 0.0;
  cpi->butteraugli_info.blk_count = 0.0;

  /*unsigned int *sses = aom_calloc(num_rows * num_cols, sizeof(*sses));
  if (!sses) {
    aom_internal_error(cm->error, AOM_CODEC_MEM_ERROR,
                       "Error allocating butteraugli data");
  }*/
  // printf("num_cols: %d\n", num_cols);

  if (cm->seq_params->use_highbitdepth) {
    // Loop through each block.
    for (int row = 0; row < num_rows; ++row) {
      for (int col = 0; col < num_cols; ++col) {  // 32x32 block
        const int index = row * num_cols + col;
        const int y_start = row * block_h;
        const int x_start = col * block_w;
        double var = 0.0, num_of_var = 0.0, var_log = 0.0;
        // printf("index: %d\n", index);
        float dbutteraugli = 0.0f;
        float exp_butteraugli = 0.0f;
        float dmse = 0.0f;
        float px_count = 0.0f;
        float qbutteraugli = 0.0f;

        // Loop through each pixel in the block
        for (int mi_row = y_start;  // mi_row is equal to pixel array's location
                                    // to resized source
             mi_row < y_start + block_h &&
             mi_row < height;  // Loop until we hit block's max height or
                               // video's max height
             mi_row++) {
          for (int mi_col = x_start;
               mi_col < x_start + block_w && mi_col < width; mi_col++) {
            // printf("mi_row: %d, mi_col: %d\n", mi_row, mi_col);
            float score = diffmap[mi_row * width + mi_col];
            dbutteraugli +=
                powf(score, 12.f);  // Compress scoring, better metric results
                                    // and more consistent rdmult
            exp_butteraugli += score;  // Add only the normal score to divide by
                                       // px_count later.
            qbutteraugli += score;     // give qbutteraugli the raw score
            // printf("dbutteraugli: %f\n", dbutteraugli);
            float px_diff =
                CONVERT_TO_SHORTPTR(
                    source->y_buffer)[mi_row * source->y_stride + mi_col] -
                CONVERT_TO_SHORTPTR(
                    recon->y_buffer)[mi_row * recon->y_stride + mi_col];
            dmse += px_diff * px_diff;
            px_count += 1.0f;
            // printf("mi_row: %d, mi_col: %d,   score: %f\n", mi_row, mi_col,
            // score);
          }
        }
        exp_butteraugli =
            exp_butteraugli / px_count;  // Get average butteraugli per block
                                         // for experimental tuning
        dbutteraugli =
            powf(dbutteraugli, 1.f / 12.f);  // Get average butteraugli per
                                             // block for experimental tuning
        /*
        This function will map the average of values to a value between 0.103
        and 10 based on k rate and score, currently 0.1 and observed
        qbutteraugli scores range between 0-~150*px_count. This function also
        has the property of output being close to 1 when the average of values
        is close to 1 and gradually increase or decrease as the average of
        values move We divide butteraugli by a factor of 6..4, depending on
        resize factor in attempts to balance the qbutteraugli to ensure similar
        scores with each different resize factor. Rather naive and needs a
        better implementation. https://www.desmos.com/calculator/e7tzvn9ag0 for
        a resize-factor=2 example of pixel scores equal to 1.0 each
        */
        qbutteraugli =
            1.0f +
            tanhf(((qbutteraugli / (6 - cpi->oxcf.butteraugli_resize_factor)) -
                   1.0f) *
                  0.1f) *
                (10.0f - 1.0f);  // Normalize qbutter here

        const int y_end = AOMMIN((y_start >> ss_y) + (block_h >> ss_y),
                                 (height + ss_y) >> ss_y);
        for (int y = y_start >> ss_y; y < y_end; y++) {
          const int x_end = AOMMIN((x_start >> ss_x) + (block_w >> ss_x),
                                   (width + ss_x) >> ss_x);
          for (int x = x_start >> ss_x; x < x_end; x++) {
            // printf("(width + ss_x) >> ss_x: %d", (width + ss_x) >> ss_x);
            const int src_px_index = y * source->uv_stride + x;
            const int recon_px_index = y * recon->uv_stride + x;
            const float px_diff_u =
                (float)(CONVERT_TO_SHORTPTR(source->u_buffer)[src_px_index] -
                        CONVERT_TO_SHORTPTR(recon->u_buffer)[recon_px_index]);
            const float px_diff_v =
                (float)(CONVERT_TO_SHORTPTR(source->v_buffer)[src_px_index] -
                        CONVERT_TO_SHORTPTR(recon->v_buffer)[recon_px_index]);
            dmse += px_diff_u * px_diff_u + px_diff_v * px_diff_v;
            px_count += 2.0f;
          }
        }

        for (int mi_row = y_start;  // mi_row is equal to pixel array's location
                                    // to cpi/main video source in blocks
             mi_row < (row + 1) * block_h &&
             mi_row < height;  // Loop until we hit block's max height or
                               // video's max height
             mi_row++) {
          for (int mi_col = x_start;
               mi_col < (col + 1) * block_w && mi_col < width; mi_col++) {
            struct buf_2d buf;
            const int row_offset_y = mi_row << 2;  // Get pixel offset
            const int col_offset_y = mi_col << 2;
            // printf("mi_row: %d, mi_col: %d\n", mi_row, mi_col);
            buf.buf = cpi->source->y_buffer +
                      row_offset_y * cpi->source->y_stride + col_offset_y;
            buf.stride = cpi->source->y_stride;

            double blk_var;
            blk_var = av1_get_perpixel_variance_facade(
                cpi, xd, &buf,
                BLOCK_4X4,  // Use 4X4 block for variance as we stride through
                            // the block by 4 pixels
                AOM_PLANE_Y);
            var_log += log(AOMMAX(blk_var, 1));
            var += blk_var;
            num_of_var += 1.0;
          }
        }
        // printf("y_start: %d\n", y_start);
        // printf("x_start: %d\n", x_start);
        // dbutteraugli = powf(dbutteraugli, 1.0f / 12.0f);
        // dbutteraugli = powf(dbutteraugli, 1.0f / 16.0f);
        /*uint8_t *const src_buf = source->y_buffer +
                                y_start * source->y_stride +
                                x_start;
        uint8_t *const rec_buf = recon->y_buffer +
                                y_start * recon->y_stride +
                                x_start;
        cpi->ppi->fn_ptr[rdo_bsize].vf(src_buf, source->y_stride,
                                              rec_buf, recon->y_stride,
                                              &sses[index]);*/
        dmse = dmse / px_count;
        // if (cpi->oxcf.tune_cfg.tuning == AOM_TUNE_EXPERIMENTAL) {
        // var = exp(var_log / num_of_var);
        // } else {
        var = var / num_of_var;
        //}
        // qbutteraugli = (float)(dmse + var) / qbutteraugli;
        cpi->butteraugli_info.total_dbutteraugli += qbutteraugli;
        cpi->butteraugli_info.blk_count += 1.0;
        // printf("qbutteraugli: %f\n", qbutteraugli);
        // printf("pxcount: %f\n", px_count);
        const float eps = 0.01f;
        // printf("sses: %f\n", (float)sses[index])
        /*const float mse =
          (float)sses[index] / (float)(px_count * 2);*/
        // printf("mse: %f", mse);
        // printf("dbutteraugli: %f   y_start: %d   x_start:   %d\n",
        // dbutteraugli, y_start, x_start);
        double weight;
        double quant_block_weight;
        if (dbutteraugli < eps ||
            dmse < eps) {  // Don't use mse for getting null block weight,
                           // slightly better efficiency?
          weight = -1.0;
          quant_block_weight = -1.0;
        } else {
          blk_count += 1.0;
          // weight = powf(dmse / dbutteraugli, 1.0f / 12.0f);
          // weight = powf(dmse / dbutteraugli, 1.0f / px_count);
          // weight = log(powf(dbutteraugli, 2.0f));
          // weight = 5*(1 - exp(-0.00625*dbutteraugli))+1.0;
          if (cpi->oxcf.tune_cfg.tuning == AOM_TUNE_EXPERIMENTAL) {
            // double hq_level = 96;
            // int cq_level = *xd->qindex;
            /*double delta =
              cq_level < hq_level
                  ? 0.25 * (double)(hq_level - cq_level) / hq_level
                  : 3.333 * (double)(cq_level - hq_level) / (MAXQ - hq_level);*/
            weight = (double)exp_butteraugli;  // variance over average
                                               // butteraugli score for block
            // weight = 39.126 * (1 - exp(-0.0009413 * weight)) + 1.236 + delta;
            // // IPQ-like curve
            weight = 13.4 * (1.0 - exp(-0.03 * weight)) + 1.6;
            // weight = AOMMIN(weight, 15.0); // Remove or increase, possibly?
            // weight = AOMMIN(weight, 10.0);
            // weight += 0.8; // Balances things out a bit towards 1.0 in later
            // geom_mean_of_scale variable(?)
          } else {
            weight =
                dmse /
                (double)dbutteraugli;  // dbutter / dbutter + variance, calm the
                                       // curve using power function
            // weight = 8.4 * (1.0 - exp(-0.00021489 * weight)) + 1.6; //
            // VMAF-like curve, fitted to 1.6-10.0 weight = 67.035434 * (1 -
            // exp(-0.021489 * weight)) + 17.492222; // SSIM-like curve weight
            // += 10.0;
            weight = AOMMIN(weight, 5.0);
            weight += 0.3;  // Balances things out a bit towards 1.0 in later
                            // geom_mean_of_scale variable(?)
          }
          // printf("var: %f   dbutteraugli: %f   weight: %f\n", var,
          // dbutteraugli, weight); weight = AOMMIN(weight, 5.0);
          /*if (cpi->oxcf.enable_experimental_psy == 0) {
            weight += K;
          }*/
          quant_block_weight = pow((var + qbutteraugli) / qbutteraugli, 0.25);
          // quant_block_weight = (double)dbutteraugli / ((dmse + var) +
          // (double)dbutteraugli); printf("dmse: %f,   var: %f,  dbutteraugli:
          // %f   weight: %f\n", dmse, var, dbutteraugli, weight);
          log_sum += log(weight);
          quant_log_sum += log(quant_block_weight);
        }
        cpi->butteraugli_info.rdmult_scaling_factors[index] = weight;
        cpi->butteraugli_info.quant_scaling_factors[index] = quant_block_weight;
      }
      // printf("row: %d\n", row);
    }
  } else {
    // Loop through each block.
    for (int row = 0; row < num_rows; ++row) {
      for (int col = 0; col < num_cols; ++col) {
        const int index = row * num_cols + col;
        const int y_start = row * block_h;
        const int x_start = col * block_w;
        float dbutteraugli = 0.0f;
        float dmse = 0.0f;
        float px_count = 0.0f;

        // Loop through each pixel.
        for (int y = y_start; y < y_start + block_h && y < height; y++) {
          for (int x = x_start; x < x_start + block_w && x < width; x++) {
            dbutteraugli += powf(diffmap[y * width + x], 12.0f);
            float px_diff = source->y_buffer[y * source->y_stride + x] -
                            recon->y_buffer[y * recon->y_stride + x];
            dmse += px_diff * px_diff;
            px_count += 1.0f;
          }
        }
        const int y_end = AOMMIN((y_start >> ss_y) + (block_h >> ss_y),
                                 (height + ss_y) >> ss_y);
        for (int y = y_start >> ss_y; y < y_end; y++) {
          const int x_end = AOMMIN((x_start >> ss_x) + (block_w >> ss_x),
                                   (width + ss_x) >> ss_x);
          for (int x = x_start >> ss_x; x < x_end; x++) {
            const int src_px_index = y * source->uv_stride + x;
            const int recon_px_index = y * recon->uv_stride + x;
            const float px_diff_u = (float)(source->u_buffer[src_px_index] -
                                            recon->u_buffer[recon_px_index]);
            const float px_diff_v = (float)(source->v_buffer[src_px_index] -
                                            recon->v_buffer[recon_px_index]);
            dmse += px_diff_u * px_diff_u + px_diff_v * px_diff_v;
            px_count += 2.0f;
          }
        }

        dbutteraugli = powf(dbutteraugli, 1.0f / 12.0f);
        dmse = dmse / px_count;
        const float eps = 0.01f;
        double weight;
        if (dbutteraugli < eps || dmse < eps) {
          weight = -1.0;
        } else {
          blk_count += 1.0;
          weight = dmse / dbutteraugli;
          weight = AOMMIN(weight, 5.0);
          weight += K;
          log_sum += log(weight);
        }
        cpi->butteraugli_info.rdmult_scaling_factors[index] = weight;
      }
    }
  }
  // Geometric average of the weights.
  log_sum = exp(log_sum / blk_count);
  quant_log_sum = exp(quant_log_sum / blk_count);
  // printf("log_sum: %f\n", log_sum);
  for (int row = 0; row < num_rows; ++row) {
    for (int col = 0; col < num_cols; ++col) {
      const int index = row * num_cols + col;
      double *weight = &cpi->butteraugli_info.rdmult_scaling_factors[index];
      // double *quant_weight =
      // &cpi->butteraugli_info.quant_scaling_factors[index];
      if (*weight <= 0.0) {
        *weight = 1.0;
      } else {
        *weight /= log_sum;
      }
      /*if (*quant_weight <= 0.0) {
        *quant_weight = 1.0;
      } else {
        *quant_weight /= quant_log_sum;
      }*/
      *weight = AOMMIN(*weight, 2.5);
      *weight = AOMMAX(*weight, 0.4);
      // printf("weight: %f\n", *weight);
    }
  }
  if (cpi->oxcf.butteraugli_quant_mult > 0 ||
      cpi->oxcf.butteraugli_quant_mult_pos >= 0 ||
      cpi->oxcf.butteraugli_quant_mult_neg >= 0) {
    const int sb_size = cpi->common.seq_params->sb_size;
    const int num_mi_w_sb = mi_size_wide[sb_size];
    const int num_mi_h_sb = mi_size_high[sb_size];
    const int num_cols_sb =
        (mi_params->mi_cols + num_mi_w_sb - 1) / num_mi_w_sb;
    const int num_rows_sb =
        (mi_params->mi_rows + num_mi_h_sb - 1) / num_mi_h_sb;
    const int num_blk_w = num_mi_w_sb / num_mi_w;
    const int num_blk_h = num_mi_h_sb / num_mi_h;
    assert(num_blk_w * num_mi_w == num_mi_w_sb);
    assert(num_blk_h * num_mi_h == num_mi_h_sb);

    for (int row = 0; row < num_rows_sb; ++row) {
      for (int col = 0; col < num_cols_sb; ++col) {
        double log_sum_sb = 0.0;
        double num_blk_count = 0.0;
        for (int blk_row = row * num_blk_h;
             blk_row < (row + 1) * num_blk_h && blk_row < num_rows; ++blk_row) {
          for (int blk_col = col * num_blk_w;
               blk_col < (col + 1) * num_blk_w && blk_col < num_cols;
               ++blk_col) {
            const int index = blk_row * num_cols + blk_col;
            double *weight =
                &cpi->butteraugli_info.quant_scaling_factors[index];
            if (*weight > 0.0) {
              log_sum_sb += log(*weight);
              num_blk_count += 1.0;
            }
          }
        }
        log_sum_sb = exp(log_sum_sb / num_blk_count);
        for (int blk_row = row * num_blk_h;
             blk_row < (row + 1) * num_blk_h && blk_row < num_rows; ++blk_row) {
          for (int blk_col = col * num_blk_w;
               blk_col < (col + 1) * num_blk_w && blk_col < num_cols;
               ++blk_col) {
            const int index = blk_row * num_cols + blk_col;
            double *weight =
                &cpi->butteraugli_info.quant_scaling_factors[index];
            if (*weight <= 0.0) {
              *weight = 1.0;
            } else {
              *weight /= log_sum_sb;
            }
          }
        }
      }
    }
  }
  aom_free(diffmap);
  // aom_free(sses);
}

void av1_set_butteraugli_rdmult(const AV1_COMP *cpi, MACROBLOCK *x,
                                BLOCK_SIZE bsize, int mi_row, int mi_col,
                                int *rdmult) {
  assert(cpi->oxcf.tune_cfg.tuning == AOM_TUNE_BUTTERAUGLI ||
         cpi->oxcf.tune_cfg.tuning == AOM_TUNE_LAVISH ||
         cpi->oxcf.tune_cfg.tuning == AOM_TUNE_EXPERIMENTAL);
  if (!cpi->butteraugli_info.recon_set) {
    return;
  }
  const AV1_COMMON *const cm = &cpi->common;
  const BLOCK_SIZE butteraugli_rdo_bsize = BLOCK_32X32;

  const int num_mi_w = mi_size_wide[butteraugli_rdo_bsize];
  const int num_mi_h = mi_size_high[butteraugli_rdo_bsize];
  const int num_cols = (cm->mi_params.mi_cols + num_mi_w - 1) / num_mi_w;
  const int num_rows = (cm->mi_params.mi_rows + num_mi_h - 1) / num_mi_h;
  const int num_bcols = (mi_size_wide[bsize] + num_mi_w - 1) / num_mi_w;
  const int num_brows = (mi_size_high[bsize] + num_mi_h - 1) / num_mi_h;
  double num_of_mi = 0.0;
  double geom_mean_of_scale = 0.0;
  double quant_geom_mean_of_scale = 0.0;

  for (int row = mi_row / num_mi_w;
       row < num_rows && row < mi_row / num_mi_w + num_brows; ++row) {
    for (int col = mi_col / num_mi_h;
         col < num_cols && col < mi_col / num_mi_h + num_bcols; ++col) {
      const int index = row * num_cols + col;
      geom_mean_of_scale +=
          log(cpi->butteraugli_info.rdmult_scaling_factors[index]);
      quant_geom_mean_of_scale +=
          log(cpi->butteraugli_info.quant_scaling_factors[index]);
      num_of_mi += 1.0;
    }
  }
  if (cpi->oxcf.butteraugli_quant_mult > 0 ||
      cpi->oxcf.butteraugli_quant_mult_pos >= 0 ||
      cpi->oxcf.butteraugli_quant_mult_neg >=
          0) {                      // If butter quant is enabled in any form
    double quant_multiplier = 1.0;  // Setup multiplier variable
    double generic_scale =
        exp(quant_geom_mean_of_scale /
            num_of_mi);  // Get generic scale without multiplier for if/then
                         // usage, 0 = 1.0 scale
    if ((cpi->oxcf.butteraugli_quant_mult_pos >= 0) &&
        (generic_scale <
         1.0)) {  // If pos quant is enabled and qindex scale > 1
      quant_multiplier = exp((quant_geom_mean_of_scale *
                              cpi->oxcf.butteraugli_quant_mult_pos / 100.0) /
                             num_of_mi);
      // printf("quant_mult_pos: %f\n", quant_multiplier);
    } else if ((cpi->oxcf.butteraugli_quant_mult_neg >= 0) &&
               (generic_scale >
                1.0)) {  // If neg quant is enabled and qindex scale < 1
      quant_multiplier = exp((quant_geom_mean_of_scale *
                              cpi->oxcf.butteraugli_quant_mult_neg / 100.0) /
                             num_of_mi);
      // printf("quant_mult_neg: %f\n", quant_multiplier);
    } else {  // If scale = 1.0 or neg/pos quant isn't defined
      quant_multiplier = exp((quant_geom_mean_of_scale *
                              cpi->oxcf.butteraugli_quant_mult / 100.0) /
                             num_of_mi);
    }
    // quant_multiplier = (1.0 - quant_multiplier) +
    // (fast_tanhf((float)quant_multiplier - 1.0f) * 0.5) + 1.0; // Invert
    // quant_multiplier across 1 (instead of 0), and smoothen the curve by a
    // factor of 0.5 around 1.0
    int new_qindex = av1_get_deltaq_offset(cm->seq_params->bit_depth, x->qindex,
                                           quant_multiplier);
    // printf("new_qindex adjust: %d\n", new_qindex);
    // printf("quant_multiplier: %f\n", quant_multiplier);
    // printf("x->qindex: %d   multiplier: %f   adjustment: %d\n", x->qindex,
    // quant_multiplier, new_qindex);
    x->qindex = x->qindex + new_qindex;    // Finally, update quantizer
    new_qindex = AOMMIN(x->qindex, MAXQ);  // Limit quantizer to video bounds
    new_qindex = AOMMAX(x->qindex, MINQ);
  }
  geom_mean_of_scale = exp(
      (geom_mean_of_scale * cpi->oxcf.butteraugli_rd_mult / 100.0) / num_of_mi);
  // printf("geom_mean_of_scale: %f\n", geom_mean_of_scale);
  *rdmult = (int)((double)(*rdmult) * geom_mean_of_scale + 0.5);
  *rdmult = AOMMAX(*rdmult, 0);
  av1_set_error_per_bit(&x->errorperbit, *rdmult);
}

static void copy_plane_lowbd(const uint8_t *src, int src_stride, uint8_t *dst,
                             int dst_stride, int w, int h) {
  for (int row = 0; row < h; row++) {
    memcpy(dst, src, w);
    src += src_stride;
    dst += dst_stride;
  }
}

static void copy_img_lowbd(const YV12_BUFFER_CONFIG *src,
                           YV12_BUFFER_CONFIG *dst, int width, int height) {
  copy_plane_lowbd(src->y_buffer, src->y_stride, dst->y_buffer, dst->y_stride,
                   width, height);
  const int width_uv = (width + src->subsampling_x) >> src->subsampling_x;
  const int height_uv = (height + src->subsampling_y) >> src->subsampling_y;
  copy_plane_lowbd(src->u_buffer, src->uv_stride, dst->u_buffer, dst->uv_stride,
                   width_uv, height_uv);
  copy_plane_lowbd(src->v_buffer, src->uv_stride, dst->v_buffer, dst->uv_stride,
                   width_uv, height_uv);
}
static void zero_plane_lowbd(uint8_t *dst, int dst_stride, int h) {
  for (int row = 0; row < h; row++) {
    memset(dst, 0, dst_stride);
    dst += dst_stride;
  }
}

static void zero_img_lowbd(YV12_BUFFER_CONFIG *dst) {
  zero_plane_lowbd(dst->y_buffer, dst->y_stride, dst->y_height);
  zero_plane_lowbd(dst->u_buffer, dst->uv_stride, dst->uv_height);
  zero_plane_lowbd(dst->v_buffer, dst->uv_stride, dst->uv_height);
}

static void copy_plane_highbd(const uint16_t *src, int src_stride,
                              uint16_t *dst, int dst_stride, int w, int h) {
  for (int row = 0; row < h; row++) {
    memcpy(dst, src, w);
    src += src_stride;
    dst += dst_stride;
  }
}

static void copy_img_highbd(const YV12_BUFFER_CONFIG *src,
                            YV12_BUFFER_CONFIG *dst, int width, int height) {
  copy_plane_highbd(CONVERT_TO_SHORTPTR(src->y_buffer), src->y_stride,
                    CONVERT_TO_SHORTPTR(dst->y_buffer), dst->y_stride, width,
                    height);
  const int width_uv = (width + src->subsampling_x) >> src->subsampling_x;
  const int height_uv = (height + src->subsampling_y) >> src->subsampling_y;
  copy_plane_highbd(CONVERT_TO_SHORTPTR(src->u_buffer), src->uv_stride,
                    CONVERT_TO_SHORTPTR(dst->u_buffer), dst->uv_stride,
                    width_uv, height_uv);
  copy_plane_highbd(CONVERT_TO_SHORTPTR(src->v_buffer), src->uv_stride,
                    CONVERT_TO_SHORTPTR(dst->v_buffer), dst->uv_stride,
                    width_uv, height_uv);
}

static void zero_plane_highbd(uint16_t *dst, int dst_stride, int h) {
  for (int row = 0; row < h; row++) {
    memset(dst, 0, dst_stride);
    dst += dst_stride;
  }
}

static void zero_img_highbd(YV12_BUFFER_CONFIG *dst) {
  zero_plane_highbd(CONVERT_TO_SHORTPTR(dst->y_buffer), dst->y_stride,
                    dst->y_height);
  zero_plane_highbd(CONVERT_TO_SHORTPTR(dst->u_buffer), dst->uv_stride,
                    dst->uv_height);
  zero_plane_highbd(CONVERT_TO_SHORTPTR(dst->v_buffer), dst->uv_stride,
                    dst->uv_height);
}

void av1_setup_butteraugli_source(AV1_COMP *cpi) {
  YV12_BUFFER_CONFIG *const dst = &cpi->butteraugli_info.source;
  AV1_COMMON *const cm = &cpi->common;
  const int width = cpi->source->y_crop_width;
  const int height = cpi->source->y_crop_height;
  const int bit_depth = cpi->td.mb.e_mbd.bd;
  const int ss_x = cpi->source->subsampling_x;
  const int ss_y = cpi->source->subsampling_y;
  const int resize_factor = (cpi->oxcf.butteraugli_resize_factor == 0)   ? 1
                            : (cpi->oxcf.butteraugli_resize_factor == 1) ? 2
                            : (cpi->oxcf.butteraugli_resize_factor == 2) ? 4
                                                                         : 2;
  if (dst->buffer_alloc_sz == 0) {
    aom_alloc_frame_buffer(
        dst, width, height, ss_x, ss_y, cm->seq_params->use_highbitdepth,
        cpi->oxcf.border_in_pixels, cm->features.byte_alignment, 0, 0);
  }
  av1_copy_and_extend_frame(cpi->source, dst);

  YV12_BUFFER_CONFIG *const resized_dst = &cpi->butteraugli_info.resized_source;
  if (resized_dst->buffer_alloc_sz == 0) {
    aom_alloc_frame_buffer(
        resized_dst, width / resize_factor, height / resize_factor, ss_x,
        ss_y,  // Resize width and height by resize_factor
        cm->seq_params->use_highbitdepth, cpi->oxcf.border_in_pixels,
        cm->features.byte_alignment, 0, 0);
  }
  av1_resize_and_extend_frame_nonnormative(cpi->source, resized_dst, bit_depth,
                                           av1_num_planes(cm));
  if (cm->seq_params->use_highbitdepth) {
    zero_img_highbd(cpi->source);
    copy_img_highbd(resized_dst, cpi->source, width / resize_factor,
                    height / resize_factor);
  } else {
    zero_img_lowbd(cpi->source);
    copy_img_lowbd(resized_dst, cpi->source, width / resize_factor,
                   height / resize_factor);
  }
}

// Write q adjustment code from frame dbutteraugli here
int av1_get_butteraugli_base_qindex(AV1_COMP *cpi, int current_qindex,
                                    int strength) {
  const AV1_COMMON *const cm = &cpi->common;

  double frame_mult = (double)cpi->butteraugli_info.total_dbutteraugli /
                      cpi->butteraugli_info.blk_count;
  // frame_mult = AOMMIN(frame_mult, 10.0);
  // frame_mult = AOMMAX(frame_mult, 0.4);
  /*if (cpi->oxcf.enable_experimental_psy == 0) {
    frame_mult = 1.0 + (1.0 - frame_mult);
  }*/

  frame_mult = pow(
      frame_mult,
      ((double)strength / (cpi->oxcf.butteraugli_loop_count + 1.0)) / 100.0);
  if (cm->current_frame.frame_number == 0 || cpi->oxcf.pass == 1 ||
      frame_mult < 0.01) {
    return current_qindex;
  }
  // const double total_dbutter =
  // (double)cpi->butteraugli_info.total_dbutteraugli; const double
  // average_dbutter = total_dbutter / (double)(num_cols * num_rows);

  // Get dbutter (beta) through wizard-level data fitting. Power function to
  // calm the curve.
  // const double dbutter = pow( sqrt( sqrt(frame_mult) * sqrt(1.0 / 3.0))  ,
  // (0.5));

  const int offset = av1_get_deltaq_offset(cm->seq_params->bit_depth,
                                           current_qindex, frame_mult);
  int qindex = current_qindex + offset;

  qindex = AOMMIN(qindex, MAXQ);
  qindex = AOMMAX(qindex, MINQ);
  // printf("frame_mult: %f   current_qindex: %d   qindex: %d\n", frame_mult,
  // current_qindex, qindex);
  return qindex;
}

void av1_setup_butteraugli_rdmult_and_restore_source(AV1_COMP *cpi, double K) {
  av1_copy_and_extend_frame(&cpi->butteraugli_info.source, cpi->source);
  AV1_COMMON *const cm = &cpi->common;
  const int width = cpi->source->y_crop_width;
  const int height = cpi->source->y_crop_height;
  const int ss_x = cpi->source->subsampling_x;
  const int ss_y = cpi->source->subsampling_y;
  const int resize_factor = (cpi->oxcf.butteraugli_resize_factor == 0)   ? 1
                            : (cpi->oxcf.butteraugli_resize_factor == 1) ? 2
                            : (cpi->oxcf.butteraugli_resize_factor == 2) ? 4
                                                                         : 2;

  YV12_BUFFER_CONFIG resized_recon;
  memset(&resized_recon, 0, sizeof(resized_recon));
  aom_alloc_frame_buffer(
      &resized_recon, width / resize_factor, height / resize_factor, ss_x, ss_y,
      cm->seq_params->use_highbitdepth, cpi->oxcf.border_in_pixels,
      cm->features.byte_alignment, 0, 0);

  if (cm->seq_params->use_highbitdepth) {
    copy_img_highbd(&cpi->common.cur_frame->buf, &resized_recon,
                    width / resize_factor, height / resize_factor);
  } else {
    copy_img_lowbd(&cpi->common.cur_frame->buf, &resized_recon,
                   width / resize_factor, height / resize_factor);
  }

  set_mb_butteraugli_rdmult_scaling(cpi, &cpi->butteraugli_info.resized_source,
                                    &resized_recon, K);
  cpi->butteraugli_info.recon_set = true;
  aom_free_frame_buffer(&resized_recon);
}

void av1_setup_butteraugli_rdmult(AV1_COMP *cpi) {
  AV1_COMMON *const cm = &cpi->common;
  const AV1EncoderConfig *const oxcf = &cpi->oxcf;
  const QuantizationCfg *const q_cfg = &oxcf->q_cfg;
  const int q_index = 96;

  // Setup necessary params for encoding, including frame source, etc.
  if (cm->current_frame.frame_type == KEY_FRAME) copy_frame_prob_info(cpi);
  av1_set_frame_size(cpi, cm->superres_upscaled_width,
                     cm->superres_upscaled_height);

  cpi->source = av1_realloc_and_scale_if_required(
      cm, cpi->unscaled_source, &cpi->scaled_source, cm->features.interp_filter,
      0, false, false, cpi->oxcf.border_in_pixels, cpi->image_pyramid_levels);
  if (cpi->unscaled_last_source != NULL) {
    cpi->last_source = av1_realloc_and_scale_if_required(
        cm, cpi->unscaled_last_source, &cpi->scaled_last_source,
        cm->features.interp_filter, 0, false, false, cpi->oxcf.border_in_pixels,
        cpi->image_pyramid_levels);
  }

  av1_setup_butteraugli_source(cpi);
  av1_setup_frame(cpi);

  if (cm->seg.enabled) {
    if (!cm->seg.update_data && cm->prev_frame) {
      segfeatures_copy(&cm->seg, &cm->prev_frame->seg);
      cm->seg.enabled = cm->prev_frame->seg.enabled;
    } else {
      av1_calculate_segdata(&cm->seg);
    }
  } else {
    memset(&cm->seg, 0, sizeof(cm->seg));
  }
  segfeatures_copy(&cm->cur_frame->seg, &cm->seg);
  cm->cur_frame->seg.enabled = cm->seg.enabled;

  av1_set_quantizer(cpi, q_cfg->qm_minlevel, q_cfg->qm_maxlevel, q_index,
                    q_cfg->enable_chroma_deltaq, q_cfg->enable_hdr_deltaq,
                    q_cfg->chroma_q_offset_u, q_cfg->chroma_q_offset_v);
  av1_set_speed_features_qindex_dependent(cpi, oxcf->speed);
  av1_init_quantizer(&cpi->enc_quant_dequant_params, &cm->quant_params,
                     cm->seq_params->bit_depth, oxcf->algo_cfg.quant_sharpness);

  av1_set_variance_partition_thresholds(cpi, q_index, 0);
  av1_encode_frame(cpi);
  if (cpi->oxcf.tune_cfg.tuning == AOM_TUNE_LAVISH ||
      cpi->oxcf.tune_cfg.tuning == AOM_TUNE_EXPERIMENTAL) {
    av1_setup_butteraugli_rdmult_and_restore_source(cpi, 0.0);
  } else {
    av1_setup_butteraugli_rdmult_and_restore_source(cpi, 0.3);
  }
  /*
  cpi->sf.part_sf.partition_search_type = partition_search_type;
  cpi->sf.part_sf.auto_max_partition_based_on_simple_motion =
  auto_max_partition_based_on_simple_motion;
  cpi->sf.part_sf.use_square_partition_only_threshold =
  use_square_partition_only_threshold;
  cpi->sf.part_sf.default_max_partition_size = default_max_partition_size;
  cpi->sf.part_sf.default_min_partition_size = default_min_partition_size;
  */
}
