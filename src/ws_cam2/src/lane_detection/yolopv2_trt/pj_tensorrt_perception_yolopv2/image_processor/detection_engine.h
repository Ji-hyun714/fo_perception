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
#ifndef DETECTION_ENGINE_
#define DETECTION_ENGINE_

/* for general */
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <memory>

/* for OpenCV */
#include <opencv2/opencv.hpp>

/* for My modules */
#include "inference_helper.h"
#include "bounding_box.h"


class DetectionEngine {
public:
    enum {
        kRetOk = 0,
        kRetErr = -1,
    };

    typedef struct Result_ {
        cv::Mat                  mat_seg_max;          // [height, width, 1]. value is 0 - 2  (uint8_t)
        cv::Mat                  mat_lane_mask;        // [height, width, 1]. value is 0 or 255
        cv::Mat                  mat_da_mask;          // [height, width, 1]. value is 0 or 255
        std::vector<BoundingBox> bbox_list;
        struct crop_ {
            int32_t x;
            int32_t y;
            int32_t w;
            int32_t h;
            crop_() : x(0), y(0), w(0), h(0) {}
        } crop;
        double                   time_pre_process;		// [msec]
        double                   time_inference;		// [msec]
        double                   time_post_process;	    // [msec]
        Result_() : time_pre_process(0), time_inference(0), time_post_process(0)
        {}
    } Result;

    typedef struct ModelOutputInfo_ {
        bool has_seg = false;
        bool has_ll = false;
        bool has_pred0 = false;
        bool has_pred1 = false;
        bool has_pred2 = false;
        std::vector<std::string> output_names;
    } ModelOutputInfo;

public:
    DetectionEngine(float threshold_objectness = 0.3f, float threshold_score = 0.3f, float threshold_nms_iou = 0.5f, float threshold_seg_ll = 0.5f) {
        threshold_objectness_ = threshold_objectness;
        threshold_score_ = threshold_score;
        threshold_nms_iou_ = threshold_nms_iou;
        threshold_seg_ll_ = threshold_seg_ll;
        detection_class_whitelist_cache_.fill(false);
    }
    ~DetectionEngine() {}
    void SetPostProcessConfig(bool enable_ld, bool enable_da, bool enable_od, int32_t lane_class_id = 2, int32_t da_class_id = 1)
    {
        enable_lane_mask_post_process_ = enable_ld;
        enable_da_mask_post_process_ = enable_da;
        enable_bbox_post_process_ = enable_od;
        lane_class_id_ = lane_class_id;
        da_class_id_ = da_class_id;
        UpdateOutputCopyFlags();
    }
    void SetDetectionThresholds(float threshold_objectness, float threshold_score)
    {
        threshold_objectness_ = threshold_objectness;
        threshold_score_ = threshold_score;
    }
    void SetDetectionClassWhitelist(const std::vector<int32_t>& ids);
    void SetEnableBboxPostProcess(bool enable) { enable_bbox_post_process_ = enable; }
    ModelOutputInfo GetModelOutputInfo() const;
    int32_t Initialize(const std::string& work_dir, const int32_t num_threads);
    int32_t Finalize(void);
    int32_t Process(const cv::Mat& original_mat, Result& result);

private:
    void UpdateOutputCopyFlags();
    std::vector<BoundingBox> DecodeDetections(
        const float* pred, int32_t channels, int32_t feature_h, int32_t feature_w,
        int32_t input_width, int32_t input_height, int32_t stride, const float anchor_grid[3][2],
        float scale_w, float scale_h) const;
    // int32_t RemapCocoToCustom(int32_t coco_class_id) const;
    bool IsClassWhitelisted(int32_t coco_class_id, int32_t num_classes) const;
    std::string GetCustomLabel(int32_t custom_class_id) const;

private:
    std::unique_ptr<InferenceHelper> inference_helper_;
    std::vector<InputTensorInfo> input_tensor_info_list_;
    std::vector<OutputTensorInfo> output_tensor_info_list_;

    float threshold_objectness_;
    float threshold_score_;
    float threshold_nms_iou_;
    float threshold_seg_ll_;
    bool enable_lane_mask_post_process_ = true;
    bool enable_da_mask_post_process_ = false;
    bool enable_bbox_post_process_ = true;
    int32_t lane_class_id_ = 2;
    int32_t da_class_id_ = 1;
    bool has_output_seg_ = false;
    bool has_output_ll_ = false;
    bool has_output_pred0_ = false;
    bool has_output_pred1_ = false;
    bool has_output_pred2_ = false;

    static constexpr int32_t kMaxDetectionClasses = 80;
    std::array<bool, kMaxDetectionClasses> detection_class_whitelist_cache_ {};
    std::vector<int32_t> detection_class_whitelist_ids_;
    bool detection_whitelist_enabled_ = false;
};

#endif
