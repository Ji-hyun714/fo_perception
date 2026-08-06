ROS2 Porting Kit (YOLOPv2 TensorRT)

Purpose
- Minimal copy kit to reuse TensorRT inference/image processor from this project inside a ROS2 package.

Included
- pj_tensorrt_perception_yolopv2/image_processor
- common_helper
- InferenceHelper/inference_helper
- resource/model/yolopv2_384x640.onnx
- resource/model/yolopv2_384x640.trt

Path compatibility
- image_processor/CMakeLists.txt expects:
  - ../../common_helper
  - ../../InferenceHelper
- This kit keeps those relative paths valid.

How to use in ROS2 package
1) Copy this entire folder into your ROS2 workspace (for example under src/your_pkg/third_party/).
2) In your ROS2 CMakeLists, add_subdirectory() for:
   - common_helper
   - InferenceHelper/inference_helper
   - pj_tensorrt_perception_yolopv2/image_processor
3) In node code:
   - call ImageProcessor::Initialize({work_dir, num_threads}) once
   - call ImageProcessor::Process(mat, result) in image callback
4) Set work_dir to this kit's resource path parent:
   - <...>/ros2_lane_mask_kit/resource

Notes
- TensorRT runtime/CUDA/cuDNN/OpenCV must be installed on target.
- If target GPU differs, delete .trt and rebuild from .onnx on first run.
