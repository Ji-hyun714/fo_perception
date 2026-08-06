#!/usr/bin/env python3
"""
bag 파일에서 GNSS 데이터를 읽어 Excel(.xlsx)로 저장한다.

- /base/ubx_nav_pvt           → lat, lon, fix_flag
- /rover/ubx_nav_pvt           → rover_lat, rover_lon
- /rover/ubx_nav_rel_pos_ned  → rel_pos_heading, rel_pos_valid
- 세 토픽을 itow(GPS Time of week, ms) 기준으로 매칭하여 한 행으로 저장한다.

사용법:
    source /opt/ros/humble/setup.bash
    source ~/fo_perception/install/setup.bash        # ublox_ubx_msgs 필요
    python3 gnssbag_to_excel.py <bag_경로> [-o 출력.xlsx]

예:
    python3 ~/fo_perception/src/ws_gnss/map_visualizer/gnssbag_to_excel.py ~/my_bag
    python3 ~/fo_perception/src/ws_gnss/map_visualizer/gnssbag_to_excel.py ~/my_bag -o gnss.xlsx
"""

import argparse
import math
import os
import sys

import pandas as pd

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


PVT_TOPIC = "/base/ubx_nav_pvt"
ROVER_PVT_TOPIC = "/rover/ubx_nav_pvt"
RELPOS_TOPIC = "/rover/ubx_nav_rel_pos_ned"

FIX_FLAG_LABEL = {
    0: "No Fix",
    1: "Single",
    2: "DGPS",
    3: "Float RTK",
    4: "Fixed RTK",
}


def open_reader(bag_path):
    """bag 디렉토리(또는 .db3/.mcap 파일)를 열어 reader 를 반환한다."""
    storage_id = "sqlite3"
    if bag_path.endswith(".mcap"):
        storage_id = "mcap"

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def compute_heading(lat1, lon1, lat2, lon2):
    """base(lat1,lon1) → rover(lat2,lon2) 방위각(deg, 0~360, 북=0, 시계방향)."""
    if None in (lat1, lon1, lat2, lon2):
        return None
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    return int((math.degrees(math.atan2(x, y)) + 360.0) % 360.0)


def compute_fix_flag(msg):
    """UBXNavPVT 메시지로부터 fix_flag 를 계산한다."""
    if not msg.gnss_fix_ok:
        return 0    # No Fix
    elif msg.carr_soln.status == 2:
        return 4    # Fixed RTK
    elif msg.carr_soln.status == 1:
        return 3    # Float RTK
    elif msg.diff_soln:
        return 2    # DGPS
    else:
        return 1    # Single Point Positioning (보정 없음)


def main():
    parser = argparse.ArgumentParser(
        description="bag의 base PVT + rover rel_pos_ned 를 itow 기준으로 매칭해 Excel로 저장")
    parser.add_argument("bag", help="ros2 bag 경로 (디렉토리 또는 .mcap 파일)")
    parser.add_argument("-o", "--output", default=None,
                        help="출력 xlsx 경로 (기본: <bag이름>_gnss.xlsx)")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        print(f"[ERROR] bag 경로가 없습니다: {args.bag}", file=sys.stderr)
        sys.exit(1)

    if args.output is None:
        base = os.path.basename(os.path.normpath(args.bag))
        base = os.path.splitext(base)[0]
        # 기본 저장 위치: 스크립트 폴더 안의 bag_log/
        script_dir = os.path.dirname(os.path.abspath(__file__))
        out_dir = os.path.join(script_dir, "bag_log")
        os.makedirs(out_dir, exist_ok=True)
        args.output = os.path.join(out_dir, f"{base}_gnss.xlsx")

    reader = open_reader(args.bag)

    # 토픽 -> 타입 매핑
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (PVT_TOPIC, RELPOS_TOPIC):
        if topic not in topic_types:
            print(f"[ERROR] bag 안에 '{topic}' 토픽이 없습니다.", file=sys.stderr)
            print("사용 가능한 토픽:", file=sys.stderr)
            for name, typ in topic_types.items():
                print(f"  {name}  ({typ})", file=sys.stderr)
            sys.exit(1)

    pvt_type = get_message(topic_types[PVT_TOPIC])
    relpos_type = get_message(topic_types[RELPOS_TOPIC])

    # rover PVT 는 선택 사항 (없어도 나머지는 저장)
    rover_pvt_type = None
    if ROVER_PVT_TOPIC in topic_types:
        rover_pvt_type = get_message(topic_types[ROVER_PVT_TOPIC])

    print(f"[INFO] PVT       토픽: {PVT_TOPIC}  ({topic_types[PVT_TOPIC]})")
    print(f"[INFO] RELPOS    토픽: {RELPOS_TOPIC}  ({topic_types[RELPOS_TOPIC]})")
    if rover_pvt_type is not None:
        print(f"[INFO] ROVER PVT 토픽: {ROVER_PVT_TOPIC}  ({topic_types[ROVER_PVT_TOPIC]})")
    else:
        print(f"[WARN] bag 안에 '{ROVER_PVT_TOPIC}' 토픽이 없습니다. rover_lat/lon 은 빈칸으로 저장됩니다.")

    # --- rover rel_pos_ned / rover PVT 를 itow 기준으로 먼저 수집 ---
    relpos_by_itow = {}
    rover_pvt_by_itow = {}
    pvt_msgs = []  # (itow, msg) 순서 유지

    n_pvt = n_relpos = n_rover_pvt = 0
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == PVT_TOPIC:
            msg = deserialize_message(data, pvt_type)
            pvt_msgs.append(msg)
            n_pvt += 1
        elif topic == RELPOS_TOPIC:
            msg = deserialize_message(data, relpos_type)
            relpos_by_itow[int(msg.itow)] = msg
            n_relpos += 1
        elif topic == ROVER_PVT_TOPIC and rover_pvt_type is not None:
            msg = deserialize_message(data, rover_pvt_type)
            rover_pvt_by_itow[int(msg.itow)] = msg
            n_rover_pvt += 1

    print(f"[INFO] PVT {n_pvt} 개, RELPOS {n_relpos} 개, ROVER PVT {n_rover_pvt} 개 읽음")

    if not pvt_msgs:
        print(f"[WARN] '{PVT_TOPIC}' 에 메시지가 하나도 없습니다.", file=sys.stderr)
        sys.exit(1)

    # --- itow 기준 매칭 ---
    rows = []
    n_matched = 0
    for pvt in pvt_msgs:
        itow = int(pvt.itow)
        rel = relpos_by_itow.get(itow)
        rover = rover_pvt_by_itow.get(itow)
        if rel is not None:
            n_matched += 1

        fix_flag = compute_fix_flag(pvt)
        lat = pvt.lat * 1e-7
        lon = pvt.lon * 1e-7
        rover_lat = rover.lat * 1e-7 if rover is not None else None
        rover_lon = rover.lon * 1e-7 if rover is not None else None
        rows.append({
            "itow": itow,
            "lat": lat,
            "lon": lon,
            "rover_lat": rover_lat,
            "rover_lon": rover_lon,
            "fix_flag": fix_flag,
            "fix_flag_label": FIX_FLAG_LABEL.get(fix_flag, f"UNKNOWN({fix_flag})"),
            "rel_pos_heading": int(rel.rel_pos_heading * 1e-5) if rel is not None else None,
            "rel_pos_valid": rel.rel_pos_valid if rel is not None else None,
            "calculate_heading": compute_heading(lat, lon, rover_lat, rover_lon),
        })

    print(f"[INFO] itow 매칭 성공: {n_matched} / {len(pvt_msgs)}")

    df = pd.DataFrame(rows)

    # rel_pos_valid: bool/None 이 섞여 object dtype 이 되면 Excel 에 True/False/0/1 로
    # 제각각 저장되므로, nullable boolean 으로 강제해 True/False(미매칭은 빈칸)로 통일한다.
    df["rel_pos_valid"] = df["rel_pos_valid"].astype("boolean")

    try:
        df.to_excel(args.output, index=False, sheet_name="gnss")
    except ModuleNotFoundError:
        # openpyxl 미설치 시 CSV로 대체 저장
        csv_path = os.path.splitext(args.output)[0] + ".csv"
        df.to_csv(csv_path, index=False)
        print(f"[WARN] openpyxl 미설치 → CSV로 저장: {csv_path}", file=sys.stderr)
        print("       xlsx로 저장하려면: pip3 install openpyxl", file=sys.stderr)
        return

    print(f"[DONE] {len(rows)} 행 저장 완료 → {args.output}")


if __name__ == "__main__":
    main()
