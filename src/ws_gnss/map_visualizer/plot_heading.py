#!/usr/bin/env python3
"""gnssbag_to_excel.py 로 만든 xlsx 를 불러와 지도 위 lon,lat 위치에
/navrelposned 의 rel_pos_heading 값을 숫자로 표시한다.

숫자 색은 fix_flag 에 따라 구분:
    0 (No Fix)          -> 빨강
    1~2 (Single/DGPS)   -> 검정
    3 (Float RTK)       -> 파랑
    4 (Fixed RTK)       -> 초록

사용법:
    python3 plot_heading.py bag_log/xxx_gnss.xlsx
    python3 plot_heading.py bag_log/xxx_gnss.xlsx --step 5   # 5개마다 하나씩 표시(겹침 완화)
"""

import argparse

import geopandas as gpd
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.lines import Line2D
from pyproj import Transformer

MAP_DIR = '/home/wise/fo_perception/src/ws_gnss/map_file'

# 범례용 (색상, 라벨)
LEGEND = [
    ('blue',  'Float RTK (3)'),
    ('green', 'Fixed RTK (4)'),
    ('red',   'rel_pos_valid = 0'),
    ('black', 'others'),
]


def heading_color(flag, valid):
    """색: fix_flag 3=파랑, 4=초록, 그 외엔 rel_pos_valid==0 이면 빨강, 나머지 검정."""
    if not valid:          # rel_pos_valid == 0 (False)
        return 'red'
    if flag == 3:
        return 'blue'
    if flag == 4:
        return 'green'
    return 'black'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('xlsx', help='gnssbag_to_excel.py 로 저장한 xlsx 경로')
    parser.add_argument('--step', type=int, default=1,
                        help='숫자를 N개마다 하나씩 표시(겹침 완화, default 1)')
    args = parser.parse_args()

    df = pd.read_excel(args.xlsx)
    df = df.dropna(subset=['lon', 'lat', 'fix_flag', 'rel_pos_heading'])
    df = df[(df['lon'] != 0) & (df['lat'] != 0)]
    if args.step > 1:
        df = df.iloc[::args.step]

    gdf_a2 = gpd.read_file(f'{MAP_DIR}/A2_LINK.shp')

    transformer = Transformer.from_crs('EPSG:4326', 'EPSG:32652', always_xy=True)
    xs, ys = transformer.transform(df['lon'].values, df['lat'].values)

    flags = df['fix_flag'].astype(int).values
    headings = df['rel_pos_heading'].values
    valids = df['rel_pos_valid'].fillna(False).astype(bool).values

    fig, ax = plt.subplots(figsize=(9, 9))
    gdf_a2.plot(ax=ax, color='lightblue', edgecolor='black', linewidth=0.5)

    for i in range(len(headings)):
        color = heading_color(int(flags[i]), bool(valids[i]))
        ax.text(xs[i], ys[i], f'{headings[i]:.0f}',
                color=color, fontsize=6, ha='center', va='center')

    # 범례 (legend) 는 색상 설명용 프록시 핸들로 표시
    handles = [Line2D([0], [0], marker='o', linestyle='', color=color, label=label)
               for color, label in LEGEND]
    ax.legend(handles=handles, title='heading color')

    ax.set_title('rel_pos_heading at lon/lat, colored by fix_flag')
    ax.set_aspect('equal')
    ax.autoscale_view()
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
