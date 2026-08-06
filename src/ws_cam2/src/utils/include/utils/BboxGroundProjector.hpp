#pragma once

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>

#include "utils/CalibData.hpp"

struct GroundPointEstimate
{
    bool valid = false;
    cv::Point2d foot_px {0.0, 0.0};
    cv::Point2d ground_xy_m {0.0, 0.0};
    double range_m = 0.0;
    double yaw_rad = 0.0;
};

struct GroundProjectorOptions
{
    double min_bbox_height_px = 4.0;
    double min_range_m = 0.0;
    double max_range_m = 120.0;
    bool clamp_to_image = true;
};

class BboxGroundProjector
{
public:
    explicit BboxGroundProjector(const CalibData& calib, GroundProjectorOptions options = {})
        : calib_(calib), options_(options)
    {
    }

    GroundPointEstimate EstimateRect(const cv::Rect2d& bbox_rect) const
    {
        GroundPointEstimate estimate;
        if (bbox_rect.width <= 0.0 || bbox_rect.height < options_.min_bbox_height_px) {
            return estimate;
        }

        estimate.foot_px = BottomCenter(bbox_rect);
        if (options_.clamp_to_image) {
            estimate.foot_px.x = std::clamp(estimate.foot_px.x, 0.0, static_cast<double>(calib_.imageSize.width - 1));
            estimate.foot_px.y = std::clamp(estimate.foot_px.y, 0.0, static_cast<double>(calib_.imageSize.height - 1));
        }

        estimate.ground_xy_m = calib_.pixRectToGnd(estimate.foot_px);
        estimate.range_m = std::hypot(estimate.ground_xy_m.x, estimate.ground_xy_m.y);
        estimate.yaw_rad = std::atan2(estimate.ground_xy_m.y, estimate.ground_xy_m.x);
        estimate.valid = std::isfinite(estimate.ground_xy_m.x) &&
                         std::isfinite(estimate.ground_xy_m.y) &&
                         std::isfinite(estimate.range_m) &&
                         estimate.range_m >= options_.min_range_m &&
                         estimate.range_m <= options_.max_range_m;
        return estimate;
    }

    static cv::Point2d BottomCenter(const cv::Rect2d& bbox_rect)
    {
        return cv::Point2d(bbox_rect.x + bbox_rect.width * 0.5, bbox_rect.y + bbox_rect.height);
    }

private:
    const CalibData& calib_;
    GroundProjectorOptions options_;
};
