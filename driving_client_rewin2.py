import math
import copy
import time # 로깅을 위해 import함. 실 주행시 필요없음.

from drive_controller import DrivingController


class DrivingClient(DrivingController):

    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.debug_controls_only = 1
        self.debug_infos = 2
        self.debug_trace = 3
        self.debug_all_rawdata = 4

        # 1 : controls_only / 2 : infos / 3 : trace / 4: all_raw_data / 0 : no debugging
        self.debug_level = 0
        
        self.collision_flag = True
        self.half_car_size = 1.25
        self.full_road_width = 0.0
        self.interval = 0.0
        self.target_point_level = 0
        
        self.max_val = 99999
        self.max_val_100 = 9999999
        self.route_calc_level = 10 #1~10

        self.race_start_time = time.time()
        
        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #
        # 도로 width를 고려하여, 차량이 있을 수 있는 midpoint 가능지점 산출
        # E.G.  full_road_width = 10[-5~5]. half_car_size = 1.25, target_point = [-3.75, -1.87, 0, 1.87 ,3.75]

        #full_road_width = (self.half_road_limit - self.half_car_size)*2  
        #interval = (full_road_width - 2*self.half_car_size)/5  #-2*interval ~ 0 ~ 2*interval. 총 5개 point이므로 4개구간. 이지만, 가동가능 범위(penalty) 넉넉히 피하기 위해 interval값을 줄이자.
        self.full_road_width = (self.half_road_limit - self.half_car_size)*2  
        self.interval = self.half_car_size
        self.target_point_level = int(self.full_road_width / self.interval)
        if self.target_point_level %2 == 0:
            self.target_point_level -=1
        

        #self.interval = (self.full_road_width - 2*self.half_car_size)/self.target_point_level

        if self.debug_level >= self.debug_infos:
            print("=========================================================")
            print("to middle: {}".format(sensing_info.to_middle))

            print("collided: {}".format(sensing_info.collided))
            print("car speed: {} km/h".format(sensing_info.speed))

            print("is moving forward: {}".format(sensing_info.moving_forward))
            print("moving angle: {}".format(sensing_info.moving_angle))
            print("lap_progress: {}".format(sensing_info.lap_progress))

            print("track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("half_road_size : {}".format(self.half_road_limit))
            print("target_point_level : {}, interval : {}".format(self.target_point_level, self.interval))
            print("at race_time : {} sec".format(round(time.time()-self.race_start_time,2)))
            print("=========================================================")

        ###########################################################################



        curve_info = []
        curve_info = self.calc_target_road(sensing_info) # 구간 / target_point_number / dist:angle:piriority_val:priority
        
        if self.debug_level >= self.debug_infos:
            print("### curve_info calculated ###")

        if self.debug_level >= self.debug_trace:
            print(curve_info)

        self.make_decision(sensing_info, car_controls, curve_info)

        if self.debug_level >= self.debug_controls_only:
            print("final_control!! steering:{}, throttle:{}, brake:{}".format(round(car_controls.steering,2), round(car_controls.throttle,2), round(car_controls.brake,2)))   

        #
        # Editing area ends
        # ==========================================================#
        return car_controls


    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name

    
    def make_decision(self, sensing_info, car_controls, curve_info) :
        #road racing
        min_route_data = self.calc_routes(sensing_info, curve_info)
        if self.debug_level >= self.debug_infos:
            print("### min_route_data calculated ###")


        min_total_dist = self.max_val_100
        min_total_angle = self.max_val_100
        min_total_penalty = self.max_val_100
        min_info = [0, self.max_val, 0, self.max_val] #index, dist, angle, priority
        loop_index = 0


        if self.debug_level >= self.debug_infos:
            print("min_route_data= {}".format(min_route_data))

        for min_route in min_route_data:
            if min_route[self.route_calc_level+2] < min_total_penalty: #priority penalty
                min_info[1] = curve_info[0][0][loop_index][0]
                min_info[2] = curve_info[0][0][loop_index][1]
                min_info[3] = curve_info[0][0][loop_index][2]
                min_info[0] = loop_index
                min_total_dist = min_route[self.route_calc_level]
                min_total_angle = min_route[self.route_calc_level+1]
                min_total_penalty = min_route[self.route_calc_level+2]

            loop_index += 1

        if self.debug_level >= self.debug_controls_only:
            print("min_route! target = {}, dist = {}, angle = {}, priority = {}".format(round(min_info[0],2), round(min_info[1],2), round(min_info[2],2), round(min_info[3],2)))


        #steer
        
        steer_val1 = round((min_info[2] / 60), 4) 
        steer_val2 = round(abs(sensing_info.to_middle), 4)
        steer_val2 = steer_val2 if sensing_info.to_middle < 0 else steer_val2 * -1
        
        steer_val = steer_val1 + steer_val2
        car_controls.steering = steer_val if steer_val < 1 else 1


        # steer_value = min_info[2] * 0.010  # Rule of Thumb 0.015

        # if steer_value < -1 :
        #     car_controls.steering = -1
        # elif steer_value > 1 :
        #     car_controls.steering = 1
        # else:
        #     car_controls.steering = steer_value
        
        if self.debug_level >= self.debug_controls_only:
            print("angle = {}, steering = {}".format(round(min_info[2],2), round(car_controls.steering,2)))


        # brake
        brake_value = (math.pow(1.3, abs(car_controls.steering) * sensing_info.speed) -3)/5 # Rule of Thumb 1.2, 3, 5

        if brake_value < 0:
            car_controls.brake = 0
        elif brake_value > 1:
            car_controls.brake = 1
        else:
            car_controls.brake = brake_value
        
        if self.debug_level >= self.debug_controls_only:
            print("brake = {} , brake_value = {}".format(round(car_controls.brake,2), round(brake_value,2)))

        # throttle
        
        throttle_value = 0
        if sensing_info.speed < 140 - min_info[2] :
            throttle_value = 1   
            if self.debug_level >= self.debug_controls_only:
                print("throttle1. throttle_value = {}, brake = {}".format(round(throttle_value,2), round(car_controls.brake,2)))
        elif sensing_info.speed < 160 - min_info[2] :
            throttle_value = 0.8
            if self.debug_level >= self.debug_controls_only:
                print("throttle2. throttle_value = {}, brake = {}".format(round(throttle_value,2), round(car_controls.brake,2)))
        else:
            throttle_value = 0.6
            if self.debug_level >= self.debug_controls_only:
                print("throttle3. throttle_value = {}, brake = {}".format(round(throttle_value,2), round(car_controls.brake,2)))



        # if min_total_angle * math.pow(sensing_info.speed,2)< 50000 * self.route_calc_level:
        #     throttle_value *= 1
        #     if self.debug_level >= self.debug_controls_only:
        #         print("throttle_adj : * 1")

        # elif min_total_angle * math.pow(sensing_info.speed,2) < 100000 * self.route_calc_level:
        #     throttle_value *= 0.7
        #     if self.debug_level >= self.debug_controls_only:
        #         print("throttle_adj : * 0.7")

        # else :
        #     throttle_value *= 0.5
        #     if self.debug_level >= self.debug_controls_only:
        #         print("throttle_adj : * 0.5")

        if throttle_value <0 :
            car_controls.throttle = 0
        elif throttle_value >1:
            car_controls.throttle = 1
        else:
            car_controls.throttle = throttle_value

        #     # out of road OR opponent car

    
    def calc_target_road(self,sensing_info) :
        #position caculation
        #매 계산시마다 현 구간의 middle을 원점 O로 정한다. 이후 다음 구간의 좌표를 구한다. 즉, 좌표는 한 구간 내에서만 상대적으로 유의미하다.
        #theta가 주어졌을때 현 위치의 middle과 다음 위치의 middle을 연결하는 선이 중심각 theta인 호로 가정한다. 

        curve_datas = [] #[][][] 구간 / target_point / 좌표정보[pos_x, pos_y]
        curve_info = [] #matrix data로 생성한 판단용 데이터. [][][][] 구간 / start_point / target_point / datas[dist, angle, penalty value]

        self.calc_road_position(sensing_info, curve_datas) #좌표 계산. curve_datas에 저장. 장애물 확인시 장애물 좌표 특이값 지정
        

        if self.debug_level >= self.debug_trace:
            print("### curve_datas ###")
            for matrix_data in curve_datas:
                print(matrix_data)
 
        self.calc_road_info(sensing_info, curve_datas, curve_info)
        
        #curve_info = [][][][] : 구간번호 / 출발 target / 도착 target /  [dist, angle, priority]
        return curve_info

    def cosine_law(self, a, b, theta):
        # angle calculation impossible
        # estimate angle = 90 + theta/2
        result = math.sqrt(pow(a,2)+pow(b,2)-2*a*b*math.cos(theta))
        return result
    
    def calc_dist(self, start_x, start_y, target_x, target_y):
        if target_x >= self.max_val and target_y >= self.max_val:
            return self.max_val #dist max로 return 하여 dfs에서 선택되지 않도록.
        else:
            return round(math.sqrt(math.pow(target_x - start_x, 2) + math.pow(target_y - start_y, 2)),2)

    def calc_angle(self, start_x, start_y, target_x, target_y):
        result = 0
        if (target_x - start_x)*(target_y - start_y) > 0:
            # positive angle 
            result = 1 * (90 - math.degrees(math.atan(abs((target_y-start_y)/(target_x-start_x)))))

        elif (target_x - start_x)*(target_y - start_y) < 0:
            #negative angle

            result = -1 * (90 - math.degrees(math.atan(abs((target_y-start_y)/(target_x-start_x)))))

        else :
            result = 0

        return result

    def mark_obstacles(self, obs_dist, obs_to_middle, interval, curve_datas, curve_matrix_index_adj, curve_matrix_index):
        for index in range(curve_matrix_index_adj, curve_matrix_index+1):

            obs_left = obs_to_middle - 1.5
            obs_right = obs_to_middle + 1.5

            blocked_left = 0
            blocked_left_flag = False
            blocked_right = self.target_point_level-1
            blocked_right_flag = False
            initial_val = -1*int(self.target_point_level/2)

            for point_index in range(self.target_point_level):

                if blocked_left_flag == False:
                    if obs_left <= initial_val * interval:
                        blocked_left = max(point_index-1,0)
                        blocked_left_flag = True

                if blocked_right_flag == False:
                    if obs_right <= initial_val * interval:
                        blocked_right = min(point_index, self.target_point_level-1)
                        blocked_right_flag = True
                        break

                initial_val += 1

            if self.debug_level >= self.debug_infos:
                print("index : {}, blocked_left : {}, blocked_right : {}".format(index, blocked_left, blocked_right))

            for block_index in range(blocked_left, blocked_right+1):

                curve_datas[index][block_index] = [self.max_val, self.max_val]

            curve_datas[index][max(blocked_left-1,0)][0] *= 2
            curve_datas[index][max(blocked_left-1,0)][1] *= 2
            curve_datas[index][min(blocked_right+1,self.target_point_level-1)][0] *= 2
            curve_datas[index][min(blocked_right+1,self.target_point_level-1)][1] *= 2

            # if obs_to_middle + 1.5 <= -4*interval : # 장애물이 좌로 치우침
            #     curve_datas[index][0] = [self.max_val, self.max_val]
            #elif obs_to_middle + 1.5 <= -3*interval :
            # else : # 장애물이 중앙에 있음
            #     curve_datas[index][3] = [self.max_val, self.max_val]
            #     curve_datas[index][4] = [self.max_val, self.max_val]
            #     curve_datas[index][5] = [self.max_val, self.max_val]
    
    def calc_road_position(self, sensing_info, curve_datas):

        #angle adjust
        angle_index = 0
        for angle in sensing_info.track_forward_angles:
            sensing_info.track_forward_angles[angle_index] = angle - sensing_info.moving_angle
            angle_index +=1

        before_angle = 0 #angle값 보정을 위함. E.G. 구간정보가 3, 10일경우, 첫 구간의 angle은 3, 이후 구간은 angle은 상대적으로 7.

        for angle in sensing_info.track_forward_angles:

            real_angle = angle - before_angle
            before_angle = angle

            l = 10
            theta = abs(math.radians(real_angle)) # angle 값이 음수일경우 계산이 복잡하므로 전부 양수처리 후 좌표 y 대칭이동
            r = 0
            
            if theta == 0:
                r = 0
            else :
                r = round(l/theta,2)

            # 부채꼴로 가정시, middle_a -> middle_b로 이어지는 선분은 이등변 삼각형의 밑변
            bottom_angle = (180-abs(real_angle))/2
            bottom_line = self.cosine_law(r, r, theta)



            mid_x = 0
            mid_y = 0
            # theta에 따라 좌표 adjust시 사용할 변수.
            modif_x = 0
            modif_y = 0

            if real_angle > 0:
                mid_x = 1 * math.cos(math.radians(bottom_angle)) * bottom_line
                mid_y = math.sin(math.radians(bottom_angle)) * bottom_line
                modif_x = math.cos(theta) * self.interval
                modif_y = -1 * math.sin(theta) * self.interval
            elif real_angle <0:
                mid_x = -1 * math.cos(math.radians(bottom_angle)) * bottom_line
                mid_y = math.sin(math.radians(bottom_angle)) * bottom_line
                modif_x = math.cos(theta) * self.interval
                modif_y = 1 * math.sin(theta) * self.interval
            else:
                mid_x = 0
                mid_y = 10
                modif_x = self.interval
                modif_y = 0

            # 도로 n구간으로 구분.

            initial_val = -1*int(self.target_point_level/2)
            temp_list= []
            for index in range(self.target_point_level):
                temp_list.append([mid_x+initial_val*modif_x,mid_y+initial_val*modif_y])
                initial_val+= 1

            # curve_datas list에 각 포인트 좌표 저장
            curve_datas.append(temp_list)

            # 장애물 발견시, curve_datas에 저장한 구간별 target point에 표시. target point로 선정되는걸 막는다.
        if len(sensing_info.track_forward_obstacles) != 0:
            for obs_info in sensing_info.track_forward_obstacles:

                obs_dist = obs_info.get('dist')
                obs_to_middle = obs_info.get('to_middle')
                
                # obs_dist가 35일경우, 0~10 구간, 11~20구간, 21~30구간, 31~40구간에 속하므로, 4구간, 즉 3번 index로 접근한다.
                # speed가 높을경우, 바로 앞 구간만 판단시 회피시간이 부족하므로, 보다 앞 데이터 또한 막아준다.
                # speed * 0.27 => m/s. 수행시 40 이상일 경우 1 구간으로 판단시 위험. 40km/h = 10.8m/s. 즉 1구간 진행시 1초 소요.
                # speed*1/40만큼 index를 당긴다.

                curve_matrix_index = min(max(int(obs_dist / 10), 0)+1, self.route_calc_level-1)
                curve_matrix_index_adj = max(int(obs_dist / 10 - sensing_info.speed * 0.05 ), 0)

                self.mark_obstacles(obs_dist, obs_to_middle, self.interval, curve_datas, curve_matrix_index_adj, curve_matrix_index)

    def calc_road_info(self, sensing_info, curve_datas, curve_info):
        area_index = 0 # 구간 index
        for area in curve_datas : 
            if area_index == 0 : # 현 vehicle 기준 전방 1구간
                start_pos = sensing_info.to_middle
                start_angle = sensing_info.moving_angle
                
                start_poses = []
                result = []

                target_index = 0
                for target_point in area:
                    calced_dist = self.calc_dist(start_pos, 0, target_point[0], target_point[1])
                    calced_angle = self.calc_angle(start_pos, 0, target_point[0], target_point[1]) - sensing_info.moving_angle
                    calced_priority = self.calc_priority(calced_dist, calced_angle, area_index, target_index)

                    target_index +=1
                    result.append([calced_dist, calced_angle, calced_priority])

                start_poses.append(result)
                curve_info.append(start_poses)

            else : # 전체 구간. 1 구간당 5*5 25개 데이터 생성
                #start_positions = [-2*self.interval, -1*self.interval, 0, 1*self.interval, 2*self.interval]

                start_positions = []
                initial_val = -1*int(self.target_point_level/2)
                for index in range(self.target_point_level):
                    start_positions.append(initial_val*self.interval)
                    initial_val += 1

                start_poses = []
                
                
                for start_pos in start_positions:

                    result = []
                    target_index = 0
                    for target_point in area:
                        calced_dist = self.calc_dist(start_pos, 0, target_point[0], target_point[1])
                        calced_angle = self.calc_angle(start_pos, 0, target_point[0], target_point[1])
                        calced_priority = self.calc_priority(calced_dist, calced_angle, area_index, target_index)

                        target_index += 1
                        result.append([calced_dist, calced_angle, calced_priority])

                    start_poses.append(result)
                curve_info.append(start_poses)        

            area_index += 1
            if area_index >= 10:
                break

    def calc_priority(self, calced_dist, calced_angle, area_index, target_index):
        # distance와 angle이 주어졌을때, angle의 변화가 적고 distance가 적은 선택이 유리
        # 값을 어떻게 계산할 것인가... 
        # steering에 대한 속도감소값은 concave함. steering값이 커질수록 penalty의 증가량이 커져야함. calced_angle과 steering은 선형이므로 calced_angle 또한 concave로 가정.
        # distance에 대한 시간증가값은 linear함.
        # steering이 30 -> 40 , 40 -> 50, 50 -> 60이라면 penalty는 얼마나?
        # distance의 범위는 기껏해야 min~max 1정도.

        result = 1.0
        result *= calced_dist
        result *= math.pow(3, abs(calced_angle)/150) 
        result *= math.pow(1.3,(self.route_calc_level- area_index))
        result *= (1 + abs(target_index - self.target_point_level/2)*0.3)
        result *= 0.0001

        return  result
        #calced_dist * math.pow(3, abs(calced_angle)/50) * math.pow(1.2,(self.route_calc_level- area_index)) * 0.0001 * (1 + abs(target_index - self.target_point_level/2)*0.3)
        # math.pow(1.2, abs(self.target_point_level/2 - target_index)) *

    def calc_routes(self, sensing_info, curve_info):
        
        # 총 5개 seqence가 있는 list. [[seq 0~9, cumulated_value], [seq 0~9, cumulated_value]]
 
        if self.debug_level >= self.debug_infos:
            print("### min_route_data calculation start ###")

        result = []
        for index in range(self.target_point_level):
            result.append(self.calc_route_penalty(curve_info, index, curve_info[0][0][index][0], curve_info[0][0][index][1], curve_info[0][0][index][2], [index]))

        return result

    def calc_route_penalty(self, curve_info, src_index, distance, angle, penalty, chosen_sequence):        
        # curve_info : [area_index], [start_index], [target_index], [dist, angle, priority]

        #첫번째 loop일 경우 start_point는 하나뿐. 이에따른 도착점은 5개로 5개 케이스. 
        #그외 n번째 loop일경우, original_src_index에서 각 start_point로 가는 최단경우 5개. 이에따른 도착점은 5개로 25개 케이스.
        #이중 20개 케이스는 버리고 5개 케이스만 남긴다.
        #n+1번째 loop에서 각 start_point로 가는 최단경우 5개로 유지.
        chosen_sequence.append(distance)
        chosen_sequence.append(abs(angle))
        chosen_sequence.append(penalty)
        return_list = []
        output_list = [chosen_sequence]
        
        area_index = 1
        while True:

            if area_index >= self.route_calc_level:
                break             

            for route_case in output_list:
                src_index = route_case[len(route_case)-4]
                temp_target_list = []

                target_index = 0
                for target_info in curve_info[area_index][src_index]:

                    distance_val = target_info[0]
                    angle_val = target_info[1]
                    penalty_val = target_info[2]
                    new_sequence = copy.deepcopy(route_case)
                    new_sequence.insert(area_index, target_index)
                    new_sequence[len(new_sequence)-3] += distance_val
                    new_sequence[len(new_sequence)-2] += abs(angle_val)
                    new_sequence[len(new_sequence)-1] += penalty_val
                    
                    if len(temp_target_list) <= target_index:
                        temp_target_list.insert(target_index, new_sequence)
                    else:
                        if temp_target_list[target_index][len(temp_target_list[target_index])-1] > new_sequence[len(new_sequence)-1]:
                            temp_target_list[target_index] = new_sequence
                    
                    target_index += 1

            output_list = copy.deepcopy(temp_target_list)
            area_index += 1

        min_total_penalty = self.max_val_100
        for route_case in output_list:
            if route_case[len(route_case)-1] < min_total_penalty:
                min_total_penalty = route_case[len(route_case)-1]
                return_list = route_case

        return return_list


if __name__ == '__main__':
    client = DrivingClient()
    client.run()

