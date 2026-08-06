import cv2
import numpy as np
import json
import math

class BEVFineTuner:
    def __init__(self, image_path, K, dist_coeffs, R_init, t_init):
        # 1. 이미지 및 파라미터 로드
        self.img = cv2.imread(image_path)
        if self.img is None:
            raise ValueError("이미지를 찾을 수 없습니다.")
        self.h, self.w = self.img.shape[:2]
        
        self.K = np.array(K, dtype=np.float32)
        self.dist_coeffs = np.array(dist_coeffs) if dist_coeffs else np.zeros(5)
        self.img = cv2.undistort(self.img, self.K, self.dist_coeffs)
        
        # 차량 -> 카메라 축 변환 행렬 (Basis Change)
        # 차량: X(전), Y(좌), Z(상)
        # 카메라: X(우), Y(하), Z(전)
        self.R_axis = np.array([
            [0, -1, 0],
            [0, 0, -1],
            [1, 0, 0]
        ])
        
        # # 2. 초기 Extrinsic 상태 (차량 좌표계 기준 카메라 Pose)
        # # 좌표계: X(전방), Y(좌측), Z(상방)
        # # 초기값은 대략적으로 설정 (필요시 수정하세요)
        # self.cam_x = 1.5   # 차량 중심에서 앞쪽으로 1.5m
        # self.cam_y = 0.0   # 차량 중심 (좌우 0)
        # self.cam_z = 1.6   # 지면에서 높이 1.6m
        
        # self.cam_roll = 0.0
        # self.cam_pitch = 0.0 # 살짝 아래를 봄
        # self.cam_yaw = 0.0

        # 입력받은 Extrinsic으로 초기 Pose 설정
        self.set_pose_from_extrinsic(np.array(R_init), np.array(t_init))
        
        # 3. BEV 설정
        self.bev_width = 640
        self.bev_height = 480
        self.px_per_meter = 25  # 1미터당 픽셀 수 (확대/축소)
        self.bev_range_fwd = 20 # 전방 30m까지 표시
        self.bev_range_side = 10 # 좌우 15m (총 30m 폭)

        # 4. 제어 모드 및 스텝
        self.mode = 'R' # 'R': Rotation, 'T': Translation
        self.step_deg = 0.01
        self.step_t = 0.05
        print(f"=== Initial Pose Loaded ===")
        print(f"Pos: {self.cam_x:.3f}, {self.cam_y:.3f}, {self.cam_z:.3f}")
        print(f"Rot: {self.cam_roll:.3f}, {self.cam_pitch:.3f}, {self.cam_yaw:.3f}")

        print("=== BEV Fine Tuner Started ===")
        print(f"Current Mode: {self.mode}")

    def set_pose_from_extrinsic(self, R_ext, t_ext):
        """
        Extrinsic Matrix (World -> Camera)를 
        Vehicle 기준 Pose (x,y,z, r,p,y)로 역변환하여 초기값 설정
        """
        # 1. Translation (Camera Position in World)
        # t = -R * C  => C = -R^T * t
        pos = -R_ext.T @ t_ext
        self.cam_x = float(pos[0])
        self.cam_y = float(pos[1])
        self.cam_z = float(pos[2])

        # 2. Rotation (Camera Orientation relative to Vehicle)
        # 수식 유도:
        # P_cam = R_axis * P_veh
        # P_veh = R_veh^T * (P_world - C_world)
        # P_cam = R_axis * R_veh^T * P_world - ...
        # 즉, R_ext = R_axis * R_veh^T
        # => R_ext^T = (R_axis * R_veh^T)^T = R_veh * R_axis^T
        # => R_veh = R_ext^T * R_axis  (양변에 R_axis를 우측 곱)
        
        R_veh = R_ext.T @ self.R_axis  # <--- [중요 수정] 순서 변경됨
        
        # 3. Rotation Matrix to Euler Angles (Roll, Pitch, Yaw)
        sy = math.sqrt(R_veh[0,0] * R_veh[0,0] +  R_veh[1,0] * R_veh[1,0])
        singular = sy < 1e-6

        if not singular:
            r = math.atan2(R_veh[2,1], R_veh[2,2])
            p = math.atan2(-R_veh[2,0], sy)
            y = math.atan2(R_veh[1,0], R_veh[0,0])
        else:
            r = math.atan2(-R_veh[1,2], R_veh[1,1])
            p = math.atan2(-R_veh[2,0], sy)
            y = 0

        self.cam_roll = np.degrees(r)
        self.cam_pitch = np.degrees(p)
        self.cam_yaw = np.degrees(y)

    def compute_homography(self):
        # 1. 현재 UI 값으로 회전 행렬 재구성 (Vehicle Body Rotation)
        R_veh = self.get_rotation_matrix(
            np.radians(self.cam_roll), np.radians(self.cam_pitch), np.radians(self.cam_yaw)
        )
        
        # 2. Extrinsic 계산
        # R_ext = R_axis * R_veh^T
        self.current_R_ext = self.R_axis @ R_veh.T  
        
        # T_ext = -R_ext * C_world
        T_cam_world = np.array([[self.cam_x], [self.cam_y], [self.cam_z]])
        self.current_t_ext = -self.current_R_ext @ T_cam_world
        
        # Homography 계산 (Z=0 Plane Projection)
        H_view = self.K @ np.concatenate([self.current_R_ext[:, 0:1], self.current_R_ext[:, 1:2], self.current_t_ext], axis=1)
        
        # BEV 변환 행렬
        s = self.px_per_meter
        cx, cy = self.bev_width / 2, self.bev_height
        M_veh2bev = np.array([[0, -s, cx], [-s, 0, cy], [0, 0, 1]])
        
        try:
            self.H_total = M_veh2bev @ np.linalg.inv(H_view)
        except:
            print("Singular Matrix!")
            self.H_total = np.eye(3)
            
        return self.H_total

    def get_rotation_matrix(self, r, p, y):
        # Euler to Rotation Matrix (ZYX order generally)
        # Vehicle coord system: x-fwd, y-left, z-up
        rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        
        # R_vehicle = Rz * Ry * Rx
        R_veh = rz @ ry @ rx
        return R_veh
    

    def draw_grid(self, bev_img):
        # (이전과 동일하여 생략, 위 코드 그대로 사용하시면 됩니다)
        center_x = self.bev_width // 2
        color = (100, 100, 100)
        
        # 세로선 (-10m ~ 10m)
        for y_m in range(-10, 11, 1):
            px_x = int(center_x - y_m * self.px_per_meter)
            cv2.line(bev_img, (px_x, 0), (px_x, self.bev_height), color, 1)

        # 가로선 (0m ~ 40m)
        for x_m in range(0, 41, 2):
            px_y = int(self.bev_height - x_m * self.px_per_meter)
            cv2.line(bev_img, (0, px_y), (self.bev_width, px_y), color, 1)
            cv2.putText(bev_img, f"{x_m}m", (10, px_y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200))

        # 십자선 (차량 위치)
        cv2.circle(bev_img, (center_x, self.bev_height), 10, (0,0,255), -1) 
    
        return bev_img
    
    def save_final_extrinsic(self):
        # 수정된 최종 R, T 저장
        data = {
            "R": self.current_R_ext.tolist(),
            "t": self.current_t_ext.tolist(),
            "pose_readable": {
                "x": self.cam_x, "y": self.cam_y, "z": self.cam_z,
                "roll": self.cam_roll, "pitch": self.cam_pitch, "yaw": self.cam_yaw
            }
        }
        with open("refined_extrinsic.json", "w") as f:
            json.dump(data, f, indent=4)
        print("saved to refined_extrinsic.json")

    def run(self):
        while True:
            H = self.compute_homography()
            
            # BEV 변환
            bev = cv2.warpPerspective(self.img, H, (self.bev_width, self.bev_height))
            bev = self.draw_grid(bev)
            
            # 정보 텍스트 표시
            info_txt = [
                f"Mode: {self.mode} (Press 'm' to switch)",
                f"Pos(x,y,z): {self.cam_x:.2f}, {self.cam_y:.2f}, {self.cam_z:.2f}",
                f"Rot(r,p,y): {self.cam_roll:.1f}, {self.cam_pitch:.1f}, {self.cam_yaw:.1f}",
                "Keys: Arrows(Main), Q/E(Sub/Roll), S(Save)"
            ]
            
            for i, txt in enumerate(info_txt):
                cv2.putText(bev, txt, (10, 30 + i*20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("BEV Fine Tuner", bev)
            
            key = cv2.waitKey(10) & 0xFF
            
            if key == 27: # ESC
                break
            elif key == ord('m'):
                self.mode = 'T' if self.mode == 'R' else 'R'
            elif key == ord('s'):
                self.save_final_extrinsic()
            
            # Control Logic
            # Shift 키 처리가 cv2에서 까다로우므로 Q/E 키를 Roll/Z축 조절로 매핑하여 UX 개선
            if self.mode == 'R':
                if key == 81: # Arrow Left (Yaw +)
                    self.cam_yaw += self.step_deg
                elif key == 83: # Arrow Right (Yaw -)
                    self.cam_yaw -= self.step_deg
                elif key == 82: # Arrow Up (Pitch -: Nose Down)
                    self.cam_pitch -= self.step_deg
                elif key == 84: # Arrow Down (Pitch +: Nose Up)
                    self.cam_pitch += self.step_deg
                elif key == ord('q'): # Roll Left
                    self.cam_roll -= self.step_deg
                elif key == ord('e'): # Roll Right
                    self.cam_roll += self.step_deg

            elif self.mode == 'T':
                if key == 82: # Arrow Up (X Forward)
                    self.cam_x += self.step_t
                elif key == 84: # Arrow Down (X Backward)
                    self.cam_x -= self.step_t
                elif key == 81: # Arrow Left (Y Left)
                    self.cam_y += self.step_t
                elif key == 83: # Arrow Right (Y Right)
                    self.cam_y -= self.step_t
                elif key == ord('q'): # Z Up
                    self.cam_z += self.step_t
                elif key == ord('e'): # Z Down
                    self.cam_z -= self.step_t



if __name__ == "__main__":
    # === 사용 예시 ===
    # 본인의 K 매트릭스로 교체하세요
    K_sample = [
        [1209.5545719628, 0, 639.047044726894],
        [0, 1226.49205405573, 370.430511576673],
        [0, 0, 1]
    ]
    dist = [-0.321920562179139, 0.13262565949842, 0.00174783975305916, -0.000332896105375772, 0.11372345420193]
    # 1209.5545719628, 0, 639.047044726894, 0, 1226.49205405573, 370.430511576673, 0, 0, 1 # econ cam intrinsic
    R_init = np.array([
        [-0.008469775224697162, -0.9998675982786804, -0.01389420095099688],
        [-0.08594034234866314, 0.01457114043958752, -0.9961937258501905],
        [0.996264282439473, -0.007243464551815073, -0.08605237798369605]
        ]) 
    t_init = np.array([[-0.487182505909253], [1.124319563125967], [2.310350319103343]]) 
      
    
    # 이미지 경로
    # img_path = "/home/wise/260114_cal_econ_bugs/720p/guvcview_image-5.jpg" # cal 이미지
    img_path = "/home/wise/outputs/260107_165736.jpg"
    
    
    # 이미지 파일 생성 (테스트용)
    # # 실제 사용시는 주석 처리하고 본인 이미지를 로드하세요
    # dummy_img = np.zeros((1080, 1920, 3), dtype=np.uint8)
    # for i in range(0, 1920, 100): cv2.line(dummy_img, (i, 0), (0, i), (255,255,255), 3) # 사선 패턴
    # cv2.imwrite("test_image.jpg", dummy_img)

    tuner = BEVFineTuner(img_path, K_sample, dist, R_init, t_init)
    tuner.run()