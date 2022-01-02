def init():
    global PID_v_P
    global PID_v_I
    global PID_v_D
    global PID_d_P
    global PID_d_I
    global PID_d_D
    global distance_list
    global distance_time
    global velocity_list
    global control_list
    global radar_velocity_list
    global target_vel
    global time_list
    global map_name
    global radar_velocity_time
    global bot_target_speed
    global bot_target_vel
    global bot_velocity_list
    global bot_time_list
    global bot_speed
    global last_known_distance
    global bot_speed_function_name
    global bot_speed_function_const
    global bot_speed_function_amplitude
    global bot_speed_function_freq

    global distance
    global rendering
    global save_to_csv
    global save_plots

    global no_bot

    save_to_csv = False
    save_plots = False
    no_bot = False
    rendering = True
    map_name = 'maps/test_1.xodr'
    PID_v_P = 1
    PID_v_I = 1
    PID_v_D = 1
    PID_d_P = 1
    PID_d_I = 1
    PID_d_D = 1
    target_vel = 15
    distance_list = []
    distance_time = []
    velocity_list = []
    control_list = []
    time_list = []
    radar_velocity_time = []
    radar_velocity_list = []
    last_known_distance = -1
    bot_target_speed = 20
    bot_speed = 0

    bot_target_vel = []
    bot_velocity_list = []
    bot_time_list = []

    distance = 15

    bot_speed_function_name = 'square'
    bot_speed_function_const = 10
    bot_speed_function_amplitude = 5
    bot_speed_function_freq = 0.05
