import os
from glob import glob
from setuptools import setup

package_name = "smart_farm_spot"

setup(
    name=package_name,
    version="2.0.0",
    # ROS 2 노드 모듈만 패키징 (RL 학습용 config/spot, sensors 등은 제외)
    packages=[package_name],
    # 패키지 루트의 단독 노드 스크립트(아웃터 레벨, sibling import 없음)도 설치.
    #  dashboard_bridge: 웹 대시보드 ↔ 로봇 브리지(launch 에서 노드로 사용).
    py_modules=["dashboard_bridge"],
    data_files=[
        # ament 리소스 인덱스
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        # 패키지 매니페스트
        ("share/" + package_name, ["package.xml"]),
        # 런치 파일
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        # 파라미터/설정 (config/*.yaml — config/spot RL 트리는 하위라 제외됨)
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
        # 맵 (pgm/png 점유격자 + yaml 메타 + 설명)
        (os.path.join("share", package_name, "maps"),
            glob("maps/*.yaml") + glob("maps/*.pgm")
            + glob("maps/*.png") + glob("maps/*.md")),
        # Isaac Sim 브릿지/시맨틱 (참조용 설치 - python.sh로 실행)
        (os.path.join("share", package_name, "isaac"),
            glob("isaac/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="꼬마 로봇 두리",
    maintainer_email="team@example.com",
    description="스마트 축사 자율순찰 Spot 통합 패키지 (Isaac Sim 인지 + Nav2 주행)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run smart_farm_spot waypoint_patrol
            "waypoint_patrol = smart_farm_spot.waypoint_patrol:main",
            # ros2 run smart_farm_spot dashboard_bridge (bringup dashboard:=true 로도 기동)
            "dashboard_bridge = dashboard_bridge:main",
            # 열화상 후처리(thermal_processor)는 비전 준비중 → 통합 제외.
            # 비전 완료 시 ros2_bridge/thermal_processor.py 를 모듈로 옮기고 등록.
        ],
    },
)
