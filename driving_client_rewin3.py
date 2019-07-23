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

        if self.is_debug:
            print("=========================================================")
            print("to middle: {}".format(sensing_info.to_middle))

           # print("collided: {}".format(sensing_info.collided))
           # print("car speed: {} km/h".format(sensing_info.speed))

           # print("is moving forward: {}".format(sensing_info.moving_forward))
            print("moving angle: {}".format(sensing_info.moving_angle))
           # print("lap_progress: {}".format(sensing_info.lap_progress))

            print("track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
           # print("opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
           # print("distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################

        # Moving straight forward
        # car_controls.steering = 0
        # car_controls.throttle = 1
        # car_controls.brake = 0

        set_throttle = 1.0
        set_steering = 0.0
        set_brake = 0.0

        self.prev_to_middle = abs(sensing_info.to_middle)
        
        road_ran = 3
        angle = 120
        ang_num = 1
        
        if sensing_info.speed > 130:
            road_ran = 8
            angle = 80
            ang_num = 2
        elif sensing_info.speed > 100:
            road_ran = 7
            angle = 100
            ang_num = 2
        elif sensing_info.speed > 80:
            angle = 110
            ang_num = 2

        # set_steering = (sensing_info.track_forward_angles[ang_num] - sensing_info.moving_angle) / angle
        # middle_add = (sensing_info.to_middle / 50) * -1
        # set_steering += middle_add

        full_throttle = True
        emergency_brake = False

        for i in range(road_ran):
            f_road = abs(sensing_info.track_forward_angles[i])
            if f_road > 35:
                full_throttle = False
            if f_road > 70:
                emergency_brake = True
                break
        
        if full_throttle == False:
            if sensing_info.speed > 90:
                set_throttle = 0.7
            if sensing_info.speed > 120:
                set_brake = 1
        
        if emergency_brake == True:
            if set_steering > 0:
                set_steering += 0.4
            else:
                set_steering -= 0.4
            

        to_middle = sensing_info.to_middle
        
        if len(sensing_info.track_forward_obstacles) > 0:
            obs_to_mid = sensing_info.track_forward_obstacles[0]['to_middle']
            diff = (to_middle - obs_to_mid)
            if abs(obs_to_mid) < 1.5:
                to_middle = -2.5
            if diff < 5:
                val = 1 if obs_to_mid < 0 else +-1
                if sensing_info.speed > 50:
                    val *= 1.5
                elif sensing_info.speed > 80:
                    val *= 2.5
                to_middle += val
            print("steering: {}".format(to_middle))

        
        str_val = round(self.steer_val_by_to_middle(to_middle), 4)
        str_val2 = round(self.steer_by_forward_road(sensing_info), 4)
        # str_val3 =             
        final_str = str_val + str_val2+
        if final_str > 1:
            final_str = 1
        elif final_str < -1:
            final_str = -1


        car_controls.steering = final_str
        car_controls.throttle = set_throttle

        if self.is_debug:
            print("steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))
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
    
    def steer_by_forward_road(self, sensing_info):
        return (sensing_info.track_forward_angles[0] - sensing_info.moving_angle) / 50
    
    def steer_val_by_to_middle(self, to_middle):
        steering = abs(to_middle) / 40
        if to_middle > 0:
            steering *= -1
        return steering
    


if __name__ == '__main__':
    client = DrivingClient()
    client.run()
