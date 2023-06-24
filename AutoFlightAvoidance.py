import numpy as np
from math import atan2, asin, pi, cos, sin
from threading import Thread
from time import localtime, time, ctime

# 사용법
# obstacle = lidar_sim(curr_pos, sim_obstacle)
# avoid_vec = VFH(curr_pos, obstacle)
# vel_list = guidance(curr_pos+[2], curr_wp+[3], avoid_vec)


# 시뮬레이션 설정
wp_list = [[0, 0, 0], [6, 10, 1]]
sim_obstacle = [[1, 6], [-5, 13], [1, 18]]

#[[6, 0.5*i] for i in range(30)]
#sim_obstacle += [[-0.2, 6.2], [0.2, 6.2], [-0.2, 5.8], [0.2, 5.8]]

#[[-0.5+0.5*i, 8] for i in range(16)]

# 라이다 설정
max_range = 8

# 장애물 회피 설정
minimum_distance = 1          # 최소 거리
safe_distance = 3           # 안전 거리 -> 인력과 척력이 동일
critical_distance = 4         # 임계 거리 -> 회피 개시
sector_num = 12
max_cruise_speed = 1
max_ascend_speed = 1
p_xy = 0.95                   # 수평 방향 위치 제어기 P 게인

r2d = 180/pi
d2r = pi/180
# m = c**2*(a-b*d), c=1, a-b*d_cri = 0, a-b*d_safe = 1로 설정
b = 1/(max_range**2-safe_distance**2)
a = b*max_range**2
angular_resolution = 2*pi/sector_num

# 장애물 회피가 포함된 웨이포인트 유도 비행
# 라이다 -> VFH -> 척력 ---> 합력 -> 속도 목표값
# 웨이포인트 -> 인력 ---/

def lidar_sim(curr_pos, obstacle):
    # 입력: 직교/원점 기준 좌표계 드론 위치, 직교/원점 기준 좌표계 장애물 위치
    # 출력: 직교/드론 기준 좌표계 장애물 위치
    
    [x0, y0, _] = curr_pos

    return [[x-x0, y-y0] for [x, y] in obstacle if ((x-x0)**2 + (y-y0)**2)**(1/2) <= max_range]
    
def guidance(curr_pos, curr_wp, avoid_vector):
    # 입력: 직교/원점 기준 좌표계 드론 위치, 직교/원점 기준 좌표계 웨이포인트 위치, 직교/드론 좌표계 기준 회피 벡터
    # 출력: 직교/드론 기준 좌표계 속도 벡터

    [x0, y0, z0] = curr_pos
    [x, y, z] = curr_wp

    mag = ((x-x0)**2 + (y-y0)**2)**(1/2)/(max_cruise_speed*p_xy)
    guidance_vector = np.array([(x-x0)/mag, (y-y0)/mag, (z-z0)])
    guidance_vector[:2] *= max_cruise_speed

    print("\n가이던스 속도 벡터: ", guidance_vector)
    
    avoidance_vector = max_cruise_speed*np.array(avoid_vector)

    gui_angle = atan2(guidance_vector[1], guidance_vector[0])
    avo_angle = atan2(avoidance_vector[1], avoidance_vector[0])

    print("\n전방 장애물 분석: ", gui_angle-avo_angle)

    if (160*d2r < gui_angle-avo_angle) and (gui_angle-avo_angle < 210*d2r):
        # 전방에 대칭 장애물이 있을 경우
        print("\n전방에 대칭 장애물! ")
        if avo_angle - gui_angle > pi:
             avo_angle = (gui_angle - pi/2)%(2*pi)
        else:
             avo_angle = (gui_angle + pi/2)%(2*pi)

    avoidance_vector = ( sum(avoidance_vector**2)**(1/2) )*np.array( [cos(avo_angle), sin(avo_angle), 0] )

    guidance_vel_vector = guidance_vector+avoidance_vector

    print("\n회피 속도  벡터: ", avoidance_vector)

    if guidance_vel_vector[0] > max_cruise_speed:
         guidance_vel_vector[0] = max_cruise_speed
    if guidance_vel_vector[1] > max_cruise_speed:
         guidance_vel_vector[1] = max_cruise_speed
    if guidance_vel_vector[2] > max_ascend_speed:
         guidance_vel_vector[2] = max_ascend_speed

    return [round(vel, 4) for vel in guidance_vel_vector.tolist()]

def VFH(obstacle):
    # 입력: 직교/원점 기준 좌표계 드론 위치, 직교/드론 기준 좌표계 장애물 위치
    # 출력: 직교/드론 기준 좌표계 회피 벡터
    # 척력 벡터는 장애물과의 거리가 safe일 때 인력 벡터와 동일한 크기를 가짐

        obs_vector = [ [dx/(dx**2 + dy**2)**(1/2), dy/(dx**2 + dy**2)**(1/2), a-b*(dx**2 + dy**2)] for [dx, dy, dz] in obstacle if (dx**2 + dy**2)**(1/2) < critical_distance]
        # [x unit vector, y unit vector, magnitude]

        [print("\n장애물 C* 벡터 | x {:.2f}m | y {:.2f}m | mag {:.2f}".format(x, y, m)) for [x, y, m] in obs_vector]

        # Polar Obstacle Density 생성
        polar_obs_density = [0]*sector_num
        for idx in range(len(obs_vector)):
            beta = atan2(obs_vector[idx][1], obs_vector[idx][0])
            m = obs_vector[idx][2]
            k = int(beta/angular_resolution)

            d = minimum_distance/((a-m)/b)**(1/2)
            if d > 1:
                 d = 1
                 
            enlarge_angle = asin(d)
            sector_angle = k*angular_resolution
        
            if (sector_angle > beta-enlarge_angle) and (sector_angle < beta+enlarge_angle):
               polar_obs_density[k] += round(m, 2)
        
        # 회피 벡터 계산
        if sum(polar_obs_density) == 0:
            return [0, 0], polar_obs_density
    
        [ print("\n회피 폴라 밀도 | {:.2f}도 | {:.2f}".format((k+1/2)*angular_resolution*r2d, polar_obs_density[k])) for k in range(len(polar_obs_density))]

        mean_obs_vector = [sum([ -polar_obs_density[k]*cos((k+1/2)*angular_resolution)/sum(polar_obs_density) for k in range(sector_num) ]),
        sum([ -polar_obs_density[k]*sin((k+1/2)*angular_resolution)/sum(polar_obs_density) for k in range(sector_num) ])]

        avoidance_vector = [mean_obs_vector[0], mean_obs_vector[1]]

        if abs(avoidance_vector[0]) < 1e-2:
             avoidance_vector[0] = 0
        
        if abs(avoidance_vector[1]) < 1e-2:
             avoidance_vector[1] = 0

        print("POD", polar_obs_density)

        return avoidance_vector, polar_obs_density

if __name__ == "__main__":
    curr_pos = [0, 0, 3]
    curr_wp = wp_list[-1]

    obs = lidar_sim(curr_pos, sim_obstacle)
    [avo_vec, polar] = VFH(obs)
    print(polar)
    vel_list = guidance(curr_pos, curr_wp, avo_vec)
