#!/usr/bin/env python3
import glob
import os
import sys
import time
import random
import logging
import math

# ==============================================================================
# CARLA 모듈 경로 설정
# ==============================================================================
try:
    sys.path.append(glob.glob('../../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import VehicleLightState as vls
# [수정] 아래 import 구문이 추가되었습니다.
from carla.command import SpawnActor, SetAutopilot, FutureActor, DestroyActor

def main():
    # ==============================================================================
    # [설정 1] 고정 장애물 위치 (여기에 원하는 위치를 적으세요)
    # ==============================================================================
    # { 인덱스: (앞뒤_오프셋, 좌우_오프셋) }
    FIXED_VEHICLES = { 
        120: (0.0, 0.0), 
        200: (5.0, 0.5) 
    }
    FIXED_CONES = { 
        50: (0.0, 0.0), 
        51: (0.0, 1.0), 
        52: (0.0, -1.0) 
    }
    # 필요하면 추가: FIXED_BARRIERS = {}, FIXED_BOXES = {} 등

    # ==============================================================================
    # [설정 2] 랜덤 교통량 설정
    # ==============================================================================
    N_VEHICLES = 50   # 움직이는 차량 수
    N_WALKERS = 50    # 보행자 수
    TM_PORT = 8005    # Traffic Manager 포트
    # ==============================================================================

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    synchronous_master = False

    vehicles_list = []
    walkers_list = []
    all_id = []
    
    try:
        world = client.get_world()
        bp_lib = world.get_blueprint_library()
        spawn_points = world.get_map().get_spawn_points()
        
        # ------------------------------------------------------------------
        # 0. Traffic Manager 초기화 (안전 운전 설정)
        # ------------------------------------------------------------------
        traffic_manager = client.get_trafficmanager(TM_PORT)
        traffic_manager.set_global_distance_to_leading_vehicle(5.0)
        traffic_manager.global_percentage_speed_difference(50.0) # 50% 감속
        
        # 동기 모드 확인
        settings = world.get_settings()
        if settings.synchronous_mode:
            traffic_manager.set_synchronous_mode(True)
            synchronous_master = True

        # ------------------------------------------------------------------
        # 1. 고정 장애물 스폰 (Fixed Obstacles)
        # ------------------------------------------------------------------
        print("--- Spawning Fixed Obstacles ---")
        used_indices = [] # 이미 사용한 스폰 포인트 인덱스 저장

        def apply_offset(transform, f_off, r_off):
            yaw = math.radians(transform.rotation.yaw)
            fx, fy = math.cos(yaw), math.sin(yaw)
            rx, ry = math.sin(yaw), -math.cos(yaw)
            transform.location.x += (fx * f_off) + (rx * r_off)
            transform.location.y += (fy * f_off) + (ry * r_off)
            return transform

        def spawn_fixed(targets, filter_pattern, type_name):
            if not targets: return
            
            blueprints = bp_lib.filter(filter_pattern)
            if type_name == "Vehicle":
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]

            for idx, (f_off, r_off) in targets.items():
                if idx >= len(spawn_points): continue
                
                # 사용된 인덱스 기록 (랜덤 스폰에서 제외하기 위함)
                used_indices.append(idx) 
                
                original_tf = spawn_points[idx]
                tf = carla.Transform(original_tf.location, original_tf.rotation)
                tf = apply_offset(tf, f_off, r_off)
                
                bp = random.choice(blueprints)
                if bp.has_attribute('color'):
                    bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))

                actor = world.try_spawn_actor(bp, tf)
                if actor:
                    # 고정 설정 (움직이지 않음)
                    actor.set_simulate_physics(True)
                    if type_name == "Vehicle":
                        actor.set_autopilot(False, TM_PORT)
                        control = carla.VehicleControl()
                        control.hand_brake = True
                        actor.apply_control(control)
                    
                    vehicles_list.append(actor.id) # 나중에 삭제하기 위해 리스트에 추가
                    print(f"  [{type_name}] Spawned at Index {idx}")

        spawn_fixed(FIXED_VEHICLES, 'vehicle.*', "Vehicle")
        spawn_fixed(FIXED_CONES, 'static.prop.constructioncone', "Cone")

        # ------------------------------------------------------------------
        # 2. 랜덤 차량 스폰 (Moving Vehicles)
        # ------------------------------------------------------------------
        print(f"--- Spawning {N_VEHICLES} Random Vehicles ---")
        
        # 고정 장애물이 없는 빈 자리만 골라내기
        available_spawn_points = []
        for i, sp in enumerate(spawn_points):
            if i not in used_indices:
                available_spawn_points.append(sp)
        
        random.shuffle(available_spawn_points)

        vehicle_bps = bp_lib.filter('vehicle.*')
        vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]

        batch = []
        for i, transform in enumerate(available_spawn_points):
            if i >= N_VEHICLES: break
            
            bp = random.choice(vehicle_bps)
            if bp.has_attribute('color'):
                bp.set_attribute('color', random.choice(bp.get_attribute('color').recommended_values))
            
            bp.set_attribute('role_name', 'autopilot')
            
            # Autopilot ON으로 스폰
            batch.append(SpawnActor(bp, transform)
                .then(SetAutopilot(FutureActor, True, TM_PORT)))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if not response.error:
                vehicles_list.append(response.actor_id)
            else:
                pass

        # [안전 주행 설정 적용] - 모든 랜덤 차량에 대해
        print("Applying Safe-Driving settings...")
        all_vehicles = world.get_actors(vehicles_list)
        for actor in all_vehicles:
            # 고정 장애물은 건너뜀
            if not actor.attributes.get('role_name') == 'autopilot':
                continue
                
            try:
                # 차선 변경 금지 & 안전 거리 확보
                traffic_manager.auto_lane_change(actor, False)
                traffic_manager.random_left_lanechange_percentage(actor, 0.0)
                traffic_manager.random_right_lanechange_percentage(actor, 0.0)
                traffic_manager.distance_to_leading_vehicle(actor, 5.0)
                traffic_manager.vehicle_percentage_speed_difference(actor, 50.0) # 50% 감속
                traffic_manager.ignore_lights_percentage(actor, 0.0)
                traffic_manager.ignore_signs_percentage(actor, 0.0)
                
                # 우측 통행 강박 끄기 (급발진 방지)
                try:
                    traffic_manager.keep_right_rule_percentage(actor, 0.0)
                except AttributeError:
                    try:
                        traffic_manager.keep_slow_lane_rule_percentage(actor, 0.0)
                    except:
                        pass

                actor.set_simulate_physics(True)
            except:
                pass

        # ------------------------------------------------------------------
        # 3. 보행자 스폰 (Walkers)
        # ------------------------------------------------------------------
        print(f"--- Spawning {N_WALKERS} Walkers ---")
        
        # 보행자 스폰 위치 확보
        walker_spawn_points = []
        for i in range(N_WALKERS):
            loc = world.get_random_location_from_navigation()
            if loc:
                spawn_point = carla.Transform(loc)
                walker_spawn_points.append(spawn_point)

        # 보행자 배치
        batch = []
        walker_speed = []
        walker_bps = bp_lib.filter('walker.pedestrian.*')
        
        for spawn_point in walker_spawn_points:
            walker_bp = random.choice(walker_bps)
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            
            # 속도 랜덤 설정
            if walker_bp.has_attribute('speed'):
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1]) # Walking
            else:
                walker_speed.append(0.0)
            
            batch.append(SpawnActor(walker_bp, spawn_point))

        results = client.apply_batch_sync(batch, True)
        
        # 컨트롤러 연결을 위해 ID 저장
        walkers_list_obj = []
        for i in range(len(results)):
            if not results[i].error:
                walkers_list_obj.append({"id": results[i].actor_id})
        
        # 보행자 컨트롤러 스폰
        batch = []
        walker_controller_bp = bp_lib.find('controller.ai.walker')
        for i in range(len(walkers_list_obj)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list_obj[i]["id"]))
        
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if not results[i].error:
                walkers_list_obj[i]["con"] = results[i].actor_id
        
        # ID 통합 리스트 생성
        for item in walkers_list_obj:
            all_id.append(item["con"]) # Controller ID
            all_id.append(item["id"])  # Walker ID

        all_actors = world.get_actors(all_id)
        
        if not args.asynch and synchronous_master:
            world.tick()
        else:
            world.wait_for_tick()

        # 보행자 걷기 시작
        world.set_pedestrians_cross_factor(0.0) # 무단횡단 금지 (0.0)
        for i in range(0, len(all_id), 2):
            all_actors[i].start()
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        print(f"Done. (Vehicles: {len(vehicles_list)}, Walkers: {len(walkers_list_obj)})")
        print("Press Ctrl+C to exit and destroy actors.")

        # ------------------------------------------------------------------
        # 4. 루프 유지
        # ------------------------------------------------------------------
        while True:
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    except KeyboardInterrupt:
        print('\nCancelled by user. Destroying actors...')
    finally:
        if synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        client.apply_batch([DestroyActor(x) for x in vehicles_list])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()
        client.apply_batch([DestroyActor(x) for x in all_id])
        
        time.sleep(0.5)
        print('Cleanup done.')

if __name__ == '__main__':
    class Args:
        asynch = False
    args = Args()
    
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
