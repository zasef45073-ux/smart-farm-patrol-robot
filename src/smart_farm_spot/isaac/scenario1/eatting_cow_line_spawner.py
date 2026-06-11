#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scenario_1_eat_line_spawner_no_rclpy.py

[시나리오 1 - 먹는 소]
여물통 양쪽 긴 라인에 cow_eat 소를 일자로 배치하는 Isaac Sim 스폰 스크립트.

조건:
  - 위쪽 여물통 라인: 8마리
  - 아래쪽 여물통 라인: 8마리
  - 총 16마리
  - 랜덤 아님. 일자 배치.
  - 스케일은 /World/cow_eat 기준:
      X=2.37873, Y=1.69315, Z=1.69315

이 파일은 Isaac Sim 안에서 실행됨.
rclpy를 쓰지 않음.
ROS2 토픽은 cow_spawn_ros_file_bridge.py가 받고,
이 스크립트는 /tmp/scenario1_eat_cow_cmd.json 명령 파일을 감시함.

실행:
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./isaac-sim.sh --exec /home/rokey/Desktop/starry/scenario_1_eat_line_spawner_no_rclpy.py

토픽 명령:
  ros2 topic pub --once /scenario1/eat_cow_spawn_cmd std_msgs/msg/String "{data: 'spawn'}"
  ros2 topic pub --once /scenario1/eat_cow_spawn_cmd std_msgs/msg/String "{data: 'clear'}"
  ros2 topic pub --once /scenario1/eat_cow_spawn_cmd std_msgs/msg/String "{data: 'reset'}"
"""

import json
import os
import time
import traceback
from pathlib import Path
from typing import Dict, List, Tuple

import omni.kit.app
import omni.usd
import omni.timeline
from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf, UsdPhysics, PhysxSchema


# =========================================================
# 파일 명령 bridge
# =========================================================

CMD_FILE = Path("/tmp/scenario1_eat_cow_cmd.json")
STATUS_FILE = Path("/tmp/scenario1_eat_cow_status.json")


# =========================================================
# 생성 경로
# =========================================================

ROOT_PATH = "/World/Scenario1_EatLineCows"
TOP_ROOT_PATH = f"{ROOT_PATH}/TopFeedLine"
BOTTOM_ROOT_PATH = f"{ROOT_PATH}/BottomFeedLine"


# =========================================================
# cow_eat USD / Material 설정
# =========================================================

# 로컬 워크스페이스 에셋 (Nucleus 서버 불필요 — 자립 워크스페이스)
#  isaac/scenario1/ 기준 3단계 상위 = smart_farm_spot, 그 아래 assets/scene
_ASSETS = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "assets", "scene")
EAT_COW_USD = os.path.join(_ASSETS, "models", "cow_eat.usd")

PREFERRED_COW_MATERIAL_PATH = "/World/Looks/cow"
AUTO_COW_MATERIAL_PATH = "/World/Looks/cow_auto_from_png"
COW_TEXTURE_PATH = os.path.join(
    _ASSETS, "textures", "cow_fbx", "RSG_Shepherd_Pack_FBX", "Cow",
    "Textures", "T_Cow_B.png")
FORCE_BIND_COW_MATERIAL = True


# =========================================================
# 기준 Transform
# =========================================================

# 네가 찍어준 cow_eat 기준 스케일
COW_SCALE_VEC = (2.37873, 1.69315, 1.69315)

# 네가 찍어준 기준 위치
TOP_Y = 11.00369
BOTTOM_Y = -11.12009
COW_Z = 0.004

# 네가 찍어준 기준 회전
TOP_ROT_Z = 0.0
BOTTOM_ROT_Z = 180.0

# X 방향으로 일자 배치
# 35m 목장 기준으로 너무 끝까지 안 가게 여백 둔 값.
# 소가 너무 겹치면 SPACING_X를 키우고,
# 너무 끝으로 가면 START_X/END_X를 줄이면 됨.
# 오른쪽 기준 두 마리를 없애는 방향이 실제 화면/좌표에서 반대로 보였으므로
# 기존 10마리 위치(-13.5~13.5) 중 앞쪽 2개를 제외하고
# -7.5부터 8마리를 배치함.
COWS_PER_LINE = 8
START_X = -7.5
SPACING_X = 3.0

# y 보정. 소 입이 여물통에 너무 박히면 Y를 살짝 바깥으로 조정.
# 위쪽이 여물통에 너무 들어가면 TOP_Y_OFFSET을 -0.2 정도,
# 아래쪽이 여물통에 너무 들어가면 BOTTOM_Y_OFFSET을 +0.2 정도.
TOP_Y_OFFSET = 0.0
BOTTOM_Y_OFFSET = 0.0


# =========================================================
# 소 물리 Collider Proxy 설정
# =========================================================
# 애니메이션 skinned mesh에 직접 rigid body를 넣으면 깨질 수 있어서
# 소마다 투명 box collider proxy를 생성해서 물리 충돌을 담당하게 함.
ENABLE_COW_PHYSICS_COLLIDER = True

# 먹는 소는 고정되어 있으므로 static collider로 둠.
# True로 하면 RigidBodyAPI를 붙이는데, 애니메이션/고정 배치에는 비추천.
ENABLE_COW_RIGID_BODY = False

# collider는 화면에서 안 보이게 숨김. 그래도 CollisionAPI는 유지됨.
HIDE_COW_COLLIDER = True

# collider 크기. 소 스케일 반영 후 대략적인 외곽 박스.
# 너무 크면 여물통/울타리와 겹칠 수 있고, 너무 작으면 충돌 느낌이 약함.
COW_COLLIDER_SIZE = (3.2, 1.35, 1.65)

# collider 중심 보정. 소 root 기준으로 몸통 위치를 맞추기 위한 값.
# 소가 여물통 쪽으로 길게 튀어나오면 Y offset을 조정.
COW_COLLIDER_OFFSET = (0.0, 0.0, 0.85)

# 물리 scene 경로
PHYSICS_SCENE_PATH = "/World/PhysicsScene"


# =========================================================
# 먹는 애니메이션 반복 / 부드러운 재생 설정
# =========================================================
# cow_eat 애니메이션이 1~21프레임이어도,
# 마지막 21프레임이 첫 자세와 비슷하면 반복 시 텀/멈칫이 생길 수 있음.
# 그래서 1~20까지만 반복해서 끝 프레임 hold 느낌을 줄임.
ANIMATION_START_FRAME = 1.0
ANIMATION_END_FRAME = 20.0

# 6.0은 너무 낮아서 뚝뚝 끊겨 보일 수 있어 8.0으로 올림.
# 1~20 기준 한 바퀴 약 2.4초.
# 더 느리게: 7.0 / 더 빠르게: 10.0
ANIMATION_TIME_CODES_PER_SECOND = 8.0

ENABLE_ANIMATION_LOOP = True

# 끝 프레임에서 멈칫하지 않게 end에 닿기 직전에 바로 start로 되감기
ENABLE_MANUAL_INSTANT_LOOP = True

# end에 정확히 닿고 나서 돌리면 잠깐 멈춘 것처럼 보일 수 있어서
# 이 프레임만큼 일찍 되감음.
LOOP_EARLY_FRAME_MARGIN = 0.35


# =========================================================
# 유틸
# =========================================================

def write_json_atomic(path: Path, payload: Dict):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, path)


def safe_asset(path: str) -> str:
    if path.startswith("omniverse://"):
        return path

    fixed = os.path.abspath(os.path.expanduser(path))

    if not os.path.exists(fixed):
        raise FileNotFoundError(f"USD 파일 없음: {fixed}")

    return fixed


def ensure_scope(stage, path: str):
    if not stage.GetPrimAtPath(path):
        UsdGeom.Scope.Define(stage, path)


def create_preview_surface_material_from_png(stage):
    """
    /World/Looks/cow가 없을 때 PNG 기반 UsdPreviewSurface material 생성.
    """
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")

    ensure_scope(stage, "/World/Looks")

    mat = UsdShade.Material.Define(stage, AUTO_COW_MATERIAL_PATH)

    shader = UsdShade.Shader.Define(stage, f"{AUTO_COW_MATERIAL_PATH}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    texture = UsdShade.Shader.Define(stage, f"{AUTO_COW_MATERIAL_PATH}/CowTexture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(COW_TEXTURE_PATH))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    st_reader = UsdShade.Shader.Define(stage, f"{AUTO_COW_MATERIAL_PATH}/PrimvarReader_st")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st_reader.ConnectableAPI(),
        "result",
    )

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(),
        "rgb",
    )

    mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(),
        "surface",
    )

    return mat


def get_or_create_cow_material(stage):
    """
    1순위: /World/Looks/cow
    2순위: T_Cow_B.png로 자동 생성한 material
    """
    preferred_prim = stage.GetPrimAtPath(PREFERRED_COW_MATERIAL_PATH)
    if preferred_prim and preferred_prim.IsValid():
        mat = UsdShade.Material(preferred_prim)
        if mat:
            return mat

    auto_prim = stage.GetPrimAtPath(AUTO_COW_MATERIAL_PATH)
    if auto_prim and auto_prim.IsValid():
        mat = UsdShade.Material(auto_prim)
        if mat:
            return mat

    if not COW_TEXTURE_PATH.startswith("omniverse://"):
        fixed_texture = os.path.abspath(os.path.expanduser(COW_TEXTURE_PATH))
        if not os.path.exists(fixed_texture):
            raise FileNotFoundError(f"cow texture png 없음: {fixed_texture}")

    return create_preview_surface_material_from_png(stage)


def bind_material_to_cow_recursive(stage, cow_root_path: str):
    material = get_or_create_cow_material(stage)
    root_prim = stage.GetPrimAtPath(cow_root_path)

    if not root_prim or not root_prim.IsValid():
        print(f"[Scenario1EatLineSpawner] WARN: cow root 없음: {cow_root_path}")
        return

    bound_count = 0

    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Gprim):
            binding_api = UsdShade.MaterialBindingAPI.Apply(prim)

            if FORCE_BIND_COW_MATERIAL:
                binding_api.Bind(
                    material,
                    bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                )
            else:
                binding_api.Bind(material)

            bound_count += 1

    if bound_count == 0:
        UsdShade.MaterialBindingAPI.Apply(root_prim).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        )

    print(
        f"[Scenario1EatLineSpawner] material bound: {cow_root_path}, "
        f"material={material.GetPath()}, mesh_count={bound_count}"
    )


def ensure_physics_scene(stage):
    """
    PhysicsScene이 없으면 생성.
    """
    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH):
        scene = UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)
        try:
            scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(9.81)
        except Exception:
            pass

        try:
            physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
            physx_scene.CreateEnableCCDAttr(True)
        except Exception:
            pass

        print(f"[Scenario1EatLineSpawner] PhysicsScene created: {PHYSICS_SCENE_PATH}")


def create_cow_physics_collider(stage, cow_prim_path: str, x: float, y: float, z: float, rot_z: float):
    """
    소마다 box collider proxy 생성.
    cow mesh는 animation/visual 전용, collider는 물리 충돌 전용.

    구조:
      /World/Scenario1_EatLineCows/TopFeedLine/EatCow_...
      /World/Scenario1_EatLineCows/TopFeedLine/EatCow_..._Collider
    """
    if not ENABLE_COW_PHYSICS_COLLIDER:
        return None

    ensure_physics_scene(stage)

    collider_path = f"{cow_prim_path}_Collider"

    # Cube는 기본 크기가 2x2x2라서 scale에 size/2를 넣음
    cube = UsdGeom.Cube.Define(stage, collider_path)
    cube_prim = cube.GetPrim()

    # 시각적으로 숨김
    if HIDE_COW_COLLIDER:
        try:
            cube.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        except Exception:
            pass

    cx = x + float(COW_COLLIDER_OFFSET[0])
    cy = y + float(COW_COLLIDER_OFFSET[1])
    cz = z + float(COW_COLLIDER_OFFSET[2])

    sx = float(COW_COLLIDER_SIZE[0]) * 0.5
    sy = float(COW_COLLIDER_SIZE[1]) * 0.5
    sz = float(COW_COLLIDER_SIZE[2]) * 0.5

    xform = UsdGeom.Xformable(cube_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, rot_z))
    xform.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))

    # USD Physics collision
    UsdPhysics.CollisionAPI.Apply(cube_prim)

    # PhysX collision 보강
    try:
        physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(cube_prim)
        physx_collision.CreateRestOffsetAttr(0.0)
        physx_collision.CreateContactOffsetAttr(0.02)
    except Exception:
        pass

    # 기본은 static collider.
    # 필요할 때만 rigid body 적용.
    if ENABLE_COW_RIGID_BODY:
        try:
            UsdPhysics.RigidBodyAPI.Apply(cube_prim)
            mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
            mass_api.CreateMassAttr(450.0)
        except Exception:
            pass

    print(
        f"[Scenario1EatLineSpawner] physics collider created: {collider_path}, "
        f"pos=({cx:.2f}, {cy:.2f}, {cz:.2f}), "
        f"size={COW_COLLIDER_SIZE}, static={not ENABLE_COW_RIGID_BODY}"
    )

    return collider_path


# =========================================================
# Isaac Spawner
# =========================================================

class Scenario1EatLineSpawner:
    def __init__(self):
        self.last_cmd_mtime = 0.0
        self.last_seq = None
        self.index = 0

        # Play 버튼과 애니메이션 loop 제어용
        self.timeline = omni.timeline.get_timeline_interface()
        self.configure_animation_timeline()

        self.update_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self.on_update,
            name="scenario1_eat_line_spawner_no_rclpy_update",
        )

        self.write_status("Scenario1 eat line spawner READY")
        print("[Scenario1EatLineSpawner] READY")
        print(f"[Scenario1EatLineSpawner] watching: {CMD_FILE}")
        print("[Scenario1EatLineSpawner] spawn = top 8 + bottom 8 eat cows in straight lines, smooth loop animation")

    def configure_animation_timeline(self):
        """
        cow_eat 1~21프레임 애니메이션을 부드럽게 반복하도록 timeline 설정.
        stage를 새로 연 뒤에도 적용되도록 spawn/reset 때도 다시 호출함.
        """
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is not None:
                stage.SetStartTimeCode(ANIMATION_START_FRAME)
                stage.SetEndTimeCode(ANIMATION_END_FRAME)
                stage.SetFramesPerSecond(60.0)
                stage.SetTimeCodesPerSecond(ANIMATION_TIME_CODES_PER_SECOND)
        except Exception as e:
            print(f"[Scenario1EatLineSpawner] WARN: stage animation metadata set failed: {e}")

        try:
            self.timeline.set_looping(bool(ENABLE_ANIMATION_LOOP))
        except Exception:
            pass

        try:
            self.timeline.set_start_time(ANIMATION_START_FRAME / ANIMATION_TIME_CODES_PER_SECOND)
        except Exception:
            pass

        try:
            self.timeline.set_end_time(ANIMATION_END_FRAME / ANIMATION_TIME_CODES_PER_SECOND)
        except Exception:
            pass

        try:
            self.timeline.set_time_codes_per_second(ANIMATION_TIME_CODES_PER_SECOND)
        except Exception:
            pass

        print(
            "[Scenario1EatLineSpawner] animation timeline configured: "
            f"frames={ANIMATION_START_FRAME}-{ANIMATION_END_FRAME}, "
            f"tcps={ANIMATION_TIME_CODES_PER_SECOND}, loop={ENABLE_ANIMATION_LOOP}"
        )

    def is_timeline_playing(self) -> bool:
        try:
            return bool(self.timeline.is_playing())
        except Exception:
            return False

    def instant_loop_timeline_if_needed(self):
        """
        end 프레임에서 잠깐 멈칫하는 느낌을 줄이기 위해
        end에 닿으면 바로 start로 되돌림.
        """
        if not ENABLE_ANIMATION_LOOP or not ENABLE_MANUAL_INSTANT_LOOP:
            return

        if not self.is_timeline_playing():
            return

        try:
            current_time = float(self.timeline.get_current_time())
        except Exception:
            return

        start_time = float(ANIMATION_START_FRAME) / float(ANIMATION_TIME_CODES_PER_SECOND)
        end_time = float(ANIMATION_END_FRAME) / float(ANIMATION_TIME_CODES_PER_SECOND)
        early_margin_time = float(LOOP_EARLY_FRAME_MARGIN) / float(ANIMATION_TIME_CODES_PER_SECOND)

        # end 프레임에 도달하기 직전에 start로 보내서 반복 전 텀을 줄임.
        if current_time >= end_time - early_margin_time:
            try:
                self.timeline.set_current_time(start_time)
            except Exception:
                pass

    def write_status(self, text: str):
        payload = {
            "status": text,
            "time": time.time(),
        }
        write_json_atomic(STATUS_FILE, payload)
        print(f"[Scenario1EatLineSpawner] {text}")

    def get_stage(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("열려 있는 USD stage가 없음. 먼저 환경 USD를 열어줘.")
        return stage

    def ensure_roots(self):
        stage = self.get_stage()

        if not stage.GetPrimAtPath("/World"):
            UsdGeom.Xform.Define(stage, "/World")

        for path in [ROOT_PATH, TOP_ROOT_PATH, BOTTOM_ROOT_PATH]:
            if not stage.GetPrimAtPath(path):
                UsdGeom.Xform.Define(stage, path)

        return stage

    def x_positions(self) -> List[float]:
        return [START_X + i * SPACING_X for i in range(COWS_PER_LINE)]

    def spawn_one(self, parent_path: str, line_name: str, x: float, y: float, z: float, rot_z: float):
        stage = self.ensure_roots()
        asset = safe_asset(EAT_COW_USD)

        self.index += 1
        prim_path = f"{parent_path}/EatCow_{line_name}_{self.index:03d}"

        xform = UsdGeom.Xform.Define(stage, prim_path)
        prim = xform.GetPrim()
        prim.GetReferences().AddReference(asset)

        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, rot_z))
        xform.AddScaleOp().Set(Gf.Vec3f(COW_SCALE_VEC[0], COW_SCALE_VEC[1], COW_SCALE_VEC[2]))

        bind_material_to_cow_recursive(stage, prim_path)

        # 소 물리 collider proxy 생성
        create_cow_physics_collider(stage, prim_path, x, y, z, rot_z)

        print(
            f"[Scenario1EatLineSpawner] spawned {prim_path} "
            f"pos=({x:.3f}, {y:.3f}, {z:.3f}), rot_z={rot_z:.1f}, "
            f"scale={COW_SCALE_VEC}"
        )

        return prim_path

    def spawn_all(self) -> Tuple[int, int]:
        self.ensure_roots()

        top_created = 0
        bottom_created = 0

        xs = self.x_positions()

        # 위쪽 여물통 라인: Rot Z = 0
        for x in xs:
            self.spawn_one(
                parent_path=TOP_ROOT_PATH,
                line_name="Top",
                x=x,
                y=TOP_Y + TOP_Y_OFFSET,
                z=COW_Z,
                rot_z=TOP_ROT_Z,
            )
            top_created += 1

        # 아래쪽 여물통 라인: Rot Z = 180
        for x in xs:
            self.spawn_one(
                parent_path=BOTTOM_ROOT_PATH,
                line_name="Bottom",
                x=x,
                y=BOTTOM_Y + BOTTOM_Y_OFFSET,
                z=COW_Z,
                rot_z=BOTTOM_ROT_Z,
            )
            bottom_created += 1

        return top_created, bottom_created

    def clear(self) -> int:
        stage = self.ensure_roots()
        root = stage.GetPrimAtPath(ROOT_PATH)

        deleted = 0

        for child in list(root.GetChildren()):
            stage.RemovePrim(child.GetPath())
            deleted += 1

        UsdGeom.Xform.Define(stage, ROOT_PATH)
        UsdGeom.Xform.Define(stage, TOP_ROOT_PATH)
        UsdGeom.Xform.Define(stage, BOTTOM_ROOT_PATH)

        self.index = 0

        return deleted

    def handle_cmd(self, raw_cmd: str):
        text = (raw_cmd or "").strip()
        cmd = text.split()[0].lower() if text else "spawn"

        if cmd in ["spawn", "all", "eat"]:
            self.configure_animation_timeline()
            top_created, bottom_created = self.spawn_all()
            self.write_status(
                f"scenario1 eat line spawned: total={top_created + bottom_created}, "
                f"top={top_created}, bottom={bottom_created}"
            )

        elif cmd == "clear":
            deleted = self.clear()
            self.write_status(f"scenario1 eat line cleared: deleted_group_roots={deleted}")

        elif cmd == "reset":
            deleted = self.clear()
            self.configure_animation_timeline()
            top_created, bottom_created = self.spawn_all()
            self.write_status(
                f"scenario1 eat line reset: deleted_group_roots={deleted}, "
                f"total={top_created + bottom_created}, top={top_created}, bottom={bottom_created}"
            )

        else:
            self.write_status(f"scenario1 unknown command: {raw_cmd}")

    def check_cmd_file(self):
        if not CMD_FILE.exists():
            return

        mtime = CMD_FILE.stat().st_mtime
        if mtime <= self.last_cmd_mtime:
            return

        self.last_cmd_mtime = mtime

        data = json.loads(CMD_FILE.read_text(encoding="utf-8"))
        seq = data.get("seq", None)

        if seq is not None and seq == self.last_seq:
            return

        self.last_seq = seq
        raw_cmd = data.get("cmd", "spawn")

        print(f"[Scenario1EatLineSpawner] command received: {data}")
        self.handle_cmd(raw_cmd)

    def on_update(self, event):
        try:
            self.instant_loop_timeline_if_needed()
            self.check_cmd_file()
        except Exception as e:
            traceback.print_exc()
            self.write_status(f"scenario1 ERROR: {type(e).__name__}: {e}")


_SCENARIO1_EAT_LINE_SPAWNER = Scenario1EatLineSpawner()
