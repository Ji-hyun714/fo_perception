/* Copyright 2022 iwatake2222

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
/*** Include ***/
/* for general */
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <chrono>
#include <fstream>

/* for OpenCV */
#include <opencv2/opencv.hpp>

/* for My modules */
#include "common_helper.h"
#include "common_helper_cv.h"
#include "inference_helper.h"
#include "detection_engine.h"

/*** Macro ***/
#define TAG "DetectionEngine"
#define PRINT(...)   COMMON_HELPER_PRINT(TAG, __VA_ARGS__)
#define PRINT_E(...) COMMON_HELPER_PRINT_E(TAG, __VA_ARGS__)

/* Model parameters */
#define MODEL_NAME  "yolopv2_384x640.onnx"
#define TENSORTYPE  TensorInfo::kTensorTypeFp32
#define INPUT_NAME  "input"
#define INPUT_DIMS  { 1, 3, 384, 640 }
#define IS_NCHW     true
#define IS_RGB      true
#define OUTPUT_NAME_0 "seg"
#define OUTPUT_NAME_1 "ll"
#define OUTPUT_NAME_2 "pred0"
#define OUTPUT_NAME_3 "pred1"
#define OUTPUT_NAME_4 "pred2"

/* from model_105_anchor_grid.npy */
static constexpr float kAnchorGrid8[3][2] = { { 12, 16 }, { 19, 36 }, { 40, 28 } };
static constexpr float kAnchorGrid16[3][2] = { { 36, 75 }, { 76, 55 }, { 72, 146 } };
static constexpr float kAnchorGrid32[3][2] = { { 142, 110 }, { 192, 243 }, { 459, 401 } };

static const std::vector<std::string> kLabelListSeg{ "Background", "Road", "Line" };
static const std::vector<std::string> kCustomLabelListDet{
    "bus",      // 0
    "bicycle",  // 1 (bicycle + motorcycle)
    "truck",    // 2
    "car",      // 3
    "person"    // 4
};

static bool HasOutputName(const std::vector<OutputTensorInfo>& output_tensor_info_list, const std::string& name)
{
    return std::any_of(output_tensor_info_list.begin(), output_tensor_info_list.end(),
        [&name](const OutputTensorInfo& t) { return t.name == name; });
}

void DetectionEngine::UpdateOutputCopyFlags()
{
    const bool need_seg_output = enable_lane_mask_post_process_ || enable_da_mask_post_process_;
    const bool need_od_output = enable_bbox_post_process_;

    for (auto& output_tensor_info : output_tensor_info_list_) {
        if (output_tensor_info.name == OUTPUT_NAME_0 || output_tensor_info.name == OUTPUT_NAME_1) {
            output_tensor_info.is_copy_to_host = need_seg_output;
        } else if (output_tensor_info.name == OUTPUT_NAME_2 ||
                   output_tensor_info.name == OUTPUT_NAME_3 ||
                   output_tensor_info.name == OUTPUT_NAME_4) {
            output_tensor_info.is_copy_to_host = need_od_output;
        } else {
            output_tensor_info.is_copy_to_host = true;
        }
    }
}

void DetectionEngine::SetDetectionClassWhitelist(const std::vector<int32_t>& ids)
{
    detection_class_whitelist_cache_.fill(false);
    detection_class_whitelist_ids_.clear();

    for (const int32_t id : ids) {
        if (id < 0 || id >= kMaxDetectionClasses) {
            continue;
        }
        if (!detection_class_whitelist_cache_[id]) {
            detection_class_whitelist_cache_[id] = true;
            detection_class_whitelist_ids_.push_back(id);
        }
    }
    detection_whitelist_enabled_ = !detection_class_whitelist_ids_.empty();
}

bool DetectionEngine::IsClassWhitelisted(const int32_t coco_class_id, const int32_t num_classes) const
{
    if (coco_class_id < 0 || coco_class_id >= num_classes) return false;
    if (!detection_whitelist_enabled_) return true;
    if (coco_class_id >= kMaxDetectionClasses) return false;
    return detection_class_whitelist_cache_[coco_class_id];
}

// int32_t DetectionEngine::RemapCocoToCustom(const int32_t coco_class_id) const
// {
//     switch (coco_class_id) {
//         case 5: return 0;   // bus
//         case 1: return 1;   // bicycle
//         case 3: return 1;   // motorcycle -> bicycle
//         case 7: return 2;   // truck
//         case 2: return 3;   // car
//         case 0: return 4;   // person
//         default: return -1; // drop unmapped classes
//     }
// }

std::string DetectionEngine::GetCustomLabel(const int32_t custom_class_id) const
{
    if (custom_class_id < 0 || custom_class_id >= static_cast<int32_t>(kCustomLabelListDet.size())) {
        return "unknown";
    }
    return kCustomLabelListDet[custom_class_id];
}


/*** Function ***/
int32_t DetectionEngine::Initialize(const std::string& work_dir, const int32_t num_threads)
{
    /* Set model information */
    std::string model_filename = work_dir + "/model/" + MODEL_NAME;

    /* Set input tensor info */
    input_tensor_info_list_.clear();
    InputTensorInfo input_tensor_info(INPUT_NAME, TENSORTYPE, IS_NCHW);
    input_tensor_info.tensor_dims = INPUT_DIMS;
    input_tensor_info.data_type = InputTensorInfo::kDataTypeImage;
    /* [0, 255] -> [0.0, 1.0] */
    input_tensor_info.normalize.mean[0] = 0.0f;
    input_tensor_info.normalize.mean[1] = 0.0f;
    input_tensor_info.normalize.mean[2] = 0.0f;
    input_tensor_info.normalize.norm[0] = 1.0f;
    input_tensor_info.normalize.norm[1] = 1.0f;
    input_tensor_info.normalize.norm[2] = 1.0f;
    input_tensor_info_list_.push_back(input_tensor_info);

    /* Set output tensor info */
    output_tensor_info_list_.clear();
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME_0, TENSORTYPE));
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME_1, TENSORTYPE));
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME_2, TENSORTYPE));
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME_3, TENSORTYPE));
    output_tensor_info_list_.push_back(OutputTensorInfo(OUTPUT_NAME_4, TENSORTYPE));

    /* Create and Initialize Inference Helper */
    //inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kOnnxRuntime));
    inference_helper_.reset(InferenceHelper::Create(InferenceHelper::kTensorrt));

    if (!inference_helper_) {
        return kRetErr;
    }
    if (inference_helper_->SetNumThreads(num_threads) != InferenceHelper::kRetOk) {
        inference_helper_.reset();
        return kRetErr;
    }
    if (inference_helper_->Initialize(model_filename, input_tensor_info_list_, output_tensor_info_list_) != InferenceHelper::kRetOk) {
        inference_helper_.reset();
        return kRetErr;
    }

    has_output_seg_ = HasOutputName(output_tensor_info_list_, OUTPUT_NAME_0);
    has_output_ll_ = HasOutputName(output_tensor_info_list_, OUTPUT_NAME_1);
    has_output_pred0_ = HasOutputName(output_tensor_info_list_, OUTPUT_NAME_2);
    has_output_pred1_ = HasOutputName(output_tensor_info_list_, OUTPUT_NAME_3);
    has_output_pred2_ = HasOutputName(output_tensor_info_list_, OUTPUT_NAME_4);
    UpdateOutputCopyFlags();

    return kRetOk;
}

int32_t DetectionEngine::Finalize()
{
    if (!inference_helper_) {
        PRINT_E("Inference helper is not created\n");
        return kRetErr;
    }
    inference_helper_->Finalize();
    return kRetOk;
}

DetectionEngine::ModelOutputInfo DetectionEngine::GetModelOutputInfo() const
{
    ModelOutputInfo info;
    info.has_seg = has_output_seg_;
    info.has_ll = has_output_ll_;
    info.has_pred0 = has_output_pred0_;
    info.has_pred1 = has_output_pred1_;
    info.has_pred2 = has_output_pred2_;
    info.output_names.reserve(output_tensor_info_list_.size());
    for (const auto& output_tensor_info : output_tensor_info_list_) {
        info.output_names.push_back(output_tensor_info.name);
    }
    return info;
}

/* reference: https://github.com/CAIC-AD/YOLOPv2/blob/main/utils/utils.py#L170 */
/* [1,255,48,80] = [1, 3, 85, 48, 80] = [1, 3, (x, y, w, h, obj, cls x80), ny nx] */
std::vector<BoundingBox> DetectionEngine::DecodeDetections(
    const float* pred, const int32_t channels, const int32_t feature_h, const int32_t feature_w,
    const int32_t input_width, const int32_t input_height, const int32_t stride,
    const float anchor_grid[3][2], const float scale_w, const float scale_h) const
{
    std::vector<BoundingBox> bbox_list;
    if (pred == nullptr || channels <= 0 || feature_h <= 0 || feature_w <= 0) {
        return bbox_list;
    }

    constexpr int32_t kNumAnchors = 3;
    if (channels % kNumAnchors != 0) {
        return bbox_list;
    }

    const int32_t channels_per_anchor = channels / kNumAnchors;
    const int32_t num_classes = channels_per_anchor - 5;
    if (num_classes <= 0) {
        return bbox_list;
    }
    const int32_t expected_feature_h = input_height / stride;
    const int32_t expected_feature_w = input_width / stride;
    if (expected_feature_h <= 0 || expected_feature_w <= 0) {
        return bbox_list;
    }

    std::vector<int32_t> active_classes;
    if (detection_whitelist_enabled_) {
        active_classes.reserve(detection_class_whitelist_ids_.size());
        for (const int32_t coco_class_id : detection_class_whitelist_ids_) {
            if (IsClassWhitelisted(coco_class_id, num_classes)) {
                active_classes.push_back(coco_class_id);
            }
        }
        if (active_classes.empty()) {
            return bbox_list;
        }
    } else {
        active_classes.reserve(num_classes);
        for (int32_t coco_class_id = 0; coco_class_id < num_classes; ++coco_class_id) {
            active_classes.push_back(coco_class_id);
        }
    }

    const int32_t decode_w = (feature_w > 0) ? feature_w : expected_feature_w;
    const int32_t decode_h = (feature_h > 0) ? feature_h : expected_feature_h;
    const size_t nx = static_cast<size_t>(decode_w);
    const size_t ny = static_cast<size_t>(decode_h);
    const size_t plane = ny * nx;

    for (int32_t n = 0; n < kNumAnchors; ++n) {
        const size_t offset_n = static_cast<size_t>(n) * static_cast<size_t>(channels_per_anchor) * plane;
        for (size_t y = 0; y < ny; ++y) {
            for (size_t x = 0; x < nx; ++x) {
                const size_t offset_xy = x + y * nx;
                const size_t index_obj = offset_n + 4 * plane + offset_xy;
                const float objectness = CommonHelper::Sigmoid(pred[index_obj]);
                if (objectness < threshold_objectness_) {
                    continue;
                }

                const size_t index_x = offset_n + 0 * plane + offset_xy;
                const size_t index_y = offset_n + 1 * plane + offset_xy;
                const size_t index_w = offset_n + 2 * plane + offset_xy;
                const size_t index_h = offset_n + 3 * plane + offset_xy;
                const float cx = (CommonHelper::Sigmoid(pred[index_x]) * 2.0f - 0.5f + static_cast<float>(x)) * static_cast<float>(stride);
                const float cy = (CommonHelper::Sigmoid(pred[index_y]) * 2.0f - 0.5f + static_cast<float>(y)) * static_cast<float>(stride);
                const float w = std::pow(CommonHelper::Sigmoid(pred[index_w]) * 2.0f, 2.0f) * anchor_grid[n][0];
                const float h = std::pow(CommonHelper::Sigmoid(pred[index_h]) * 2.0f, 2.0f) * anchor_grid[n][1];

                for (const int32_t coco_class_id : active_classes) {
                    const size_t index_cls = offset_n + static_cast<size_t>(5 + coco_class_id) * plane + offset_xy;
                    const float class_score = CommonHelper::Sigmoid(pred[index_cls]);
                    const float score = objectness * class_score;
                    if (score < threshold_score_) {
                        continue;
                    }

                    // const int32_t custom_class_id = RemapCocoToCustom(coco_class_id);
                    // if (custom_class_id < 0) {
                    //     continue;
                    // }

                    bbox_list.emplace_back(
                        coco_class_id,
                        // GetCustomLabel(custom_class_id),
                        std::to_string(coco_class_id),
                        score,
                        static_cast<int32_t>((cx - w * 0.5f) * scale_w),
                        static_cast<int32_t>((cy - h * 0.5f) * scale_h),
                        static_cast<int32_t>(w * scale_w),
                        static_cast<int32_t>(h * scale_h));
                }
            }
        }
    }

    return bbox_list;
}

int32_t DetectionEngine::Process(const cv::Mat& original_mat, Result& result)
{
    if (!inference_helper_) {
        PRINT_E("Inference helper is not created\n");
        return kRetErr;
    }
    /*** PreProcess ***/
    const auto& t_pre_process0 = std::chrono::steady_clock::now();
    InputTensorInfo& input_tensor_info = input_tensor_info_list_[0];
    /* do crop, resize and color conversion here because some inference engine doesn't support these operations */
    int32_t crop_x = 0;
    int32_t crop_y = 0;
    int32_t crop_w = original_mat.cols;
    int32_t crop_h = original_mat.rows;
    cv::Mat img_src = cv::Mat::zeros(input_tensor_info.GetHeight(), input_tensor_info.GetWidth(), CV_8UC3);
    CommonHelper::CropResizeCvt(original_mat, img_src, crop_x, crop_y, crop_w, crop_h, IS_RGB, CommonHelper::kCropTypeStretch);
    //CommonHelper::CropResizeCvt(original_mat, img_src, crop_x, crop_y, crop_w, crop_h, IS_RGB, CommonHelper::kCropTypeCut);
    //CommonHelper::CropResizeCvt(original_mat, img_src, crop_x, crop_y, crop_w, crop_h, IS_RGB, CommonHelper::kCropTypeExpand);

    input_tensor_info.data = img_src.data;
    input_tensor_info.data_type = InputTensorInfo::kDataTypeImage;
    input_tensor_info.image_info.width = img_src.cols;
    input_tensor_info.image_info.height = img_src.rows;
    input_tensor_info.image_info.channel = img_src.channels();
    input_tensor_info.image_info.crop_x = 0;
    input_tensor_info.image_info.crop_y = 0;
    input_tensor_info.image_info.crop_width = img_src.cols;
    input_tensor_info.image_info.crop_height = img_src.rows;
    input_tensor_info.image_info.is_bgr = false;
    input_tensor_info.image_info.swap_color = false;
    if (inference_helper_->PreProcess(input_tensor_info_list_) != InferenceHelper::kRetOk) {
        return kRetErr;
    }
    const auto& t_pre_process1 = std::chrono::steady_clock::now();

    /*** Inference ***/
    const auto& t_inference0 = std::chrono::steady_clock::now();
    if (inference_helper_->Process(output_tensor_info_list_) != InferenceHelper::kRetOk) {
        return kRetErr;
    }
    const auto& t_inference1 = std::chrono::steady_clock::now();

    /*** PostProcess ***/
    const auto& t_post_process0 = std::chrono::steady_clock::now();
    cv::Mat mat_seg_max;
    cv::Mat mat_lane_mask;
    cv::Mat mat_da_mask;
    const bool need_seg_post_process = enable_lane_mask_post_process_ || enable_da_mask_post_process_;
    const bool can_seg_post_process = output_tensor_info_list_.size() >= 2 && has_output_seg_ && has_output_ll_;
    if (need_seg_post_process && can_seg_post_process) {
        const float* output_seg_ptr = output_tensor_info_list_[0].GetDataAsFloat();
        const float* output_ll_ptr = output_tensor_info_list_[1].GetDataAsFloat();
        if (output_seg_ptr != nullptr && output_ll_ptr != nullptr) {
            const int32_t h = input_tensor_info.GetHeight();
            const int32_t w = input_tensor_info.GetWidth();
            const int32_t hw = h * w;
            const float* seg_c0 = output_seg_ptr;
            const float* seg_c1 = output_seg_ptr + hw;
            mat_seg_max = cv::Mat(h, w, CV_8UC1);

#if defined(_OPENMP)
#pragma omp parallel for schedule(static)
#endif
            for (int32_t y = 0; y < h; y++) {
                uint8_t* dst_row = mat_seg_max.ptr<uint8_t>(y);
                const int32_t row_off = y * w;
                for (int32_t x = 0; x < w; x++) {
                    const int32_t idx = row_off + x;
                    uint8_t class_index_max = (seg_c1[idx] > seg_c0[idx]) ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0);
                    if (output_ll_ptr[idx] > threshold_seg_ll_) {
                        class_index_max = static_cast<uint8_t>(2);    /* 2 = line */
                    }
                    dst_row[x] = class_index_max;
                }
            }

            if (enable_lane_mask_post_process_) {
                cv::compare(mat_seg_max, lane_class_id_, mat_lane_mask, cv::CMP_EQ);  // 0 or 255
                if (mat_lane_mask.size() != original_mat.size()) {
                    cv::resize(mat_lane_mask, mat_lane_mask, original_mat.size(), 0.0, 0.0, cv::INTER_NEAREST);
                }
            }
            if (enable_da_mask_post_process_) {
                cv::compare(mat_seg_max, da_class_id_, mat_da_mask, cv::CMP_EQ);  // 0 or 255
                if (mat_da_mask.size() != original_mat.size()) {
                    cv::resize(mat_da_mask, mat_da_mask, original_mat.size(), 0.0, 0.0, cv::INTER_NEAREST);
                }
            }
        }
    }

    std::vector<BoundingBox> bbox_nms_list;
    const bool can_bbox_post_process = output_tensor_info_list_.size() >= 5 && has_output_pred0_ && has_output_pred1_ && has_output_pred2_;
    if (enable_bbox_post_process_ && can_bbox_post_process) {
        std::vector<BoundingBox> bbox_list;
        float scale_w = static_cast<float>(crop_w) / input_tensor_info.GetWidth();
        float scale_h = static_cast<float>(crop_h) / input_tensor_info.GetHeight();
        const auto* pred0 = output_tensor_info_list_[2].GetDataAsFloat();
        const auto* pred1 = output_tensor_info_list_[3].GetDataAsFloat();
        const auto* pred2 = output_tensor_info_list_[4].GetDataAsFloat();
        const int32_t pred0_c = output_tensor_info_list_[2].GetChannel();
        const int32_t pred1_c = output_tensor_info_list_[3].GetChannel();
        const int32_t pred2_c = output_tensor_info_list_[4].GetChannel();
        const int32_t pred0_h = output_tensor_info_list_[2].GetHeight();
        const int32_t pred1_h = output_tensor_info_list_[3].GetHeight();
        const int32_t pred2_h = output_tensor_info_list_[4].GetHeight();
        const int32_t pred0_w = output_tensor_info_list_[2].GetWidth();
        const int32_t pred1_w = output_tensor_info_list_[3].GetWidth();
        const int32_t pred2_w = output_tensor_info_list_[4].GetWidth();

        auto bbox_list_8 = DecodeDetections(pred0, pred0_c, pred0_h, pred0_w,
            input_tensor_info.GetWidth(), input_tensor_info.GetHeight(), 8, kAnchorGrid8, scale_w, scale_h);
        auto bbox_list_16 = DecodeDetections(pred1, pred1_c, pred1_h, pred1_w,
            input_tensor_info.GetWidth(), input_tensor_info.GetHeight(), 16, kAnchorGrid16, scale_w, scale_h);
        auto bbox_list_32 = DecodeDetections(pred2, pred2_c, pred2_h, pred2_w,
            input_tensor_info.GetWidth(), input_tensor_info.GetHeight(), 32, kAnchorGrid32, scale_w, scale_h);
        bbox_list.insert(bbox_list.end(), bbox_list_8.begin(), bbox_list_8.end());
        bbox_list.insert(bbox_list.end(), bbox_list_16.begin(), bbox_list_16.end());
        bbox_list.insert(bbox_list.end(), bbox_list_32.begin(), bbox_list_32.end());

        for (auto& bbox : bbox_list) {
            bbox.x += crop_x;
            bbox.y += crop_y;
        }
        BoundingBoxUtils::Nms(bbox_list, bbox_nms_list, threshold_nms_iou_, true);
    }

    const auto& t_post_process1 = std::chrono::steady_clock::now();

    /* Return the results */
    result.mat_seg_max = mat_seg_max;
    result.mat_lane_mask = mat_lane_mask;
    result.mat_da_mask = mat_da_mask;
    result.bbox_list = bbox_nms_list;
    result.crop.x = (std::max)(0, crop_x);
    result.crop.y = (std::max)(0, crop_y);
    result.crop.w = (std::min)(crop_w, original_mat.cols - result.crop.x);
    result.crop.h = (std::min)(crop_h, original_mat.rows - result.crop.y);
    result.time_pre_process = static_cast<std::chrono::duration<double>>(t_pre_process1 - t_pre_process0).count() * 1000.0;
    result.time_inference = static_cast<std::chrono::duration<double>>(t_inference1 - t_inference0).count() * 1000.0;
    result.time_post_process = static_cast<std::chrono::duration<double>>(t_post_process1 - t_post_process0).count() * 1000.0;;

    return kRetOk;
}
