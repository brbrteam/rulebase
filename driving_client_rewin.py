from drive_controller import DrivingController


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = True
        self.collision_flag = True

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

        if self.is_debug == False:
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
            print("=========================================================")

        ###########################################################################

        # Moving straight forward
        # if(sensing_info.speed == 0): 
        car_controls.throttle = 0.7
        
        # 장애물 판단. 50m 이하인 경우 핸들링
        if len(sensing_info.track_forward_obstacles) != 0:
            obs = sensing_info.track_forward_obstacles[0]
            # print(">>>>>>>>>dist:{}, middle:{}".format(obs['dist'], obs['to_middle']))
            if obs['dist'] < 30 and abs(obs['to_middle'] - sensing_info.to_middle) < 3:
                val = self.noti_obstacles(obs)
                if(val != 0):
                    car_controls.steering = 0.25 * val
                    return car_controls

        val = self.curve_car(sensing_info)
        if(val != 0):
            car_controls.steering = 0.13 * val
        else:
            val2 = self.curve_car_angle(sensing_info)
            car_controls.steering = 0.13 * val2

        if self.is_debug:
            print("steering:{}, throttle:{}, middle:{}".format(car_controls.steering, car_controls.throttle, sensing_info.to_middle))

        if sensing_info.collided:
            print(sensing_info.lap_progress)
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
    
    def curve_car(self, sensing_info):
        if (abs(sensing_info.to_middle) > self.half_road_limit):
            return -3 if (sensing_info.to_middle >=0) else 3
        elif(abs(sensing_info.to_middle) > self.half_road_limit / 4):
            return -1 if (sensing_info.to_middle >=0) else 1
        else:
            return 0
    
    def curve_car_angle(self, sensing_info):
        if (sensing_info.moving_angle > 10):
            return -2
        elif (sensing_info.moving_angle > 5):
            return -1
        elif(sensing_info.moving_angle < -10):
            return  2
        elif(sensing_info.moving_angle < -5):
            return  1
        else:
            return 0
            
    def noti_obstacles(self, obs):
        print(">>>>>>>>>dist:{}, middle:{}".format(obs['dist'], obs['to_middle']))
        return -1 if (obs['to_middle'] >=0) else 1


if __name__ == '__main__':
    client = DrivingClient()
    client.run()
