#!/usr/bin/env python3
"""gnssbag_to_excel.py 로 만든 xlsx 를 불러와 지도 위에 lon,lat 점을 찍는다.

색은 fix_flag 에 따라 구분:
    0 (No Fix)      -> 빨강
    1 (Single)      -> 검정
    2 (DGPS)        -> 검정
    3 (Float RTK)   -> 파랑
    4 (Fixed RTK)   -> 초록

사용법:
    python3 ~/fo_perception/src/ws_gnss/map_visualizer/plot_points.py bag_log/xxx_gnss.xlsx
"""

import argparse

import geopandas as gpd
import matplotlib.pyplot as plt
import pandas as pd
from pyproj import Transformer

MAP_DIR = '/home/wise/fo_perception/src/ws_gnss/map_file'

# fix_flag 그룹 -> (색상, 라벨). 1~2(Single/DGPS)는 하나로 합친다.
GROUPS = [
    ({0},    'red',   'No Fix (0)'),
    ({1, 2}, 'black', 'Single/DGPS (1~2)'),
    ({3},    'blue',  'Float RTK (3)'),
    ({4},    'green', 'Fixed RTK (4)'),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('xlsx', help='gnssbag_to_excel.py 로 저장한 xlsx 경로')
    args = parser.parse_args()

    df = pd.read_excel(args.xlsx)
    # lon/lat, fix_flag 가 모두 있는 row 만 사용
    df = df.dropna(subset=['lon', 'lat', 'fix_flag'])
    df = df[(df['lon'] != 0) & (df['lat'] != 0)]

    # 지도 (EPSG:32652) 불러오기
    gdf_a2 = gpd.read_file(f'{MAP_DIR}/A2_LINK.shp')

    # lon/lat(EPSG:4326) -> 지도 좌표(EPSG:32652) 변환
    transformer = Transformer.from_crs('EPSG:4326', 'EPSG:32652', always_xy=True)
    xs, ys = transformer.transform(df['lon'].values, df['lat'].values)

    flags = df['fix_flag'].astype(int).values

    fig, ax = plt.subplots(figsize=(9, 9))
    gdf_a2.plot(ax=ax, color='lightblue', edgecolor='black', linewidth=0.5)

    for states, color, label in GROUPS:
        mask = [f in states for f in flags]
        if any(mask):
            ax.scatter([xs[i] for i in range(len(mask)) if mask[i]],
                       [ys[i] for i in range(len(mask)) if mask[i]],
                       s=12, color=color, label=label)

    ax.legend(title='fix_flag')
    ax.set_title('Long/Lat by fix_flag')
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
