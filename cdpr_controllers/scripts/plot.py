import matplotlib.pyplot as plt

def read_data(file_path):
    x = []
    y = []
    z = []
    r = []
    p = []
    yaw = []
    q7 =[]

    with open(file_path, 'r') as file:
        for line in file:
            values = line.strip().split('\t')
            x.append(float(values[0]))
            y.append(float(values[1]))
            z.append(float(values[2]))
            r.append(float(values[3]))
            p.append(float(values[4]))
            yaw.append(float(values[5]))
            # q7.append(float(values[6]))

    return x, y, z, r, p, yaw

def read_data1(file_path):
    x = []
    y = []
    z = []
    r = []
    p = []
    yaw = []
    q7 =[]

    with open(file_path, 'r') as file:
        for line in file:
            values = line.strip().split('\t')
            x.append(float(values[0]))
            y.append(float(values[1]))
            z.append(float(values[2]))
            r.append(float(values[3]))
            p.append(float(values[4]))
            yaw.append(float(values[5]))
            q7.append(float(values[6]))

    return x, y, z, r, p, yaw, q7

def read_data_2(file_path):
    x = []

    with open(file_path, 'r') as file:
        for line in file:
            values = line.strip().split('\t')
            x.append(float(values[0]))

    return x



def read_data3(file_path):
    x = []
    y = []
    z = []
    r = []
    p = []
    yaw = []
    q7 =[]
    q8 = []
    with open(file_path, 'r') as file:
        for line in file:
            values = line.strip().split('\t')
            x.append(float(values[0]))
            y.append(float(values[1]))
            z.append(float(values[2]))
            r.append(float(values[3]))
            p.append(float(values[4]))
            yaw.append(float(values[5]))
            q7.append(float(values[6]))
            q8.append(float(values[7]))

    return x, y, z, r, p, yaw, q7, q8




# 파일 경로
se3_pose_file =  '/home/chan/data/se3_pose.txt'
se3_desired_pose_file = '/home/chan/data/desired_se3_pose.txt'
se3_1 = '/home/chan/data/se3_1.txt'
time ='/home/chan/data/time_plot.txt'

# time1 = '/home/chan/data/time_plot1.txt'
# time2 = '/home/chan/data/time_plot2.txt'
tension = '/home/chan/data//tension.txt'
platform_txt = '/home/chan/data/platform.txt'
des_platform_txt = '/home/chan/data/platform_des.txt'
# se3_des_pose_txt = '/home/chan/data/franka_q_test.txt'


# ctime = '/home/chan/data/compute_time.txt'
# franka_q = '/home/chan/data//franka_q.txt'


# 데이터 읽기
# franka_q1, franka_q2, franka_q3, franka_q4, franka_q5, franka_q6, franka_q7 = read_data1(franka_q)
x, y, z, r, p, yaw = read_data(se3_pose_file)
# x1, y1, z1, r1, p1, yaw1 = read_data(se3_1)

x_desired, y_desired, z_desired, r_desired, p_desired, yaw_desired= read_data(se3_desired_pose_file)
a = read_data_2(time)
platform_1,platform_2,platform_3,platform_4,platform_5,platform_6 = read_data(platform_txt)
platform_11,platform_22,platform_33,platform_44,platform_55,platform_66 = read_data(des_platform_txt) 
tension1, tension2, tension3, tension4,tension5, tension6, tension7, tension8 = read_data3(tension)
csfont = {'fontname':'Times New Roman'}
# b = read_data_2(ctime)

# plt.hx
# plt.title('title',**csfont)
# plt.show()


# # X 값 플롯 생성
# plt.figure(figsize=(20,10))
# plt.subplot(6,1,1)
# plt.plot(x ,label='x_real')
# plt.plot(x1 ,label='x_')
# plt.plot(x_desired, label='x_desired')
# plt.xlabel('Index')
# plt.ylabel('X')
# plt.title('X-Axis')
# plt.legend()
# plt.grid(True)

# # Y 값 플롯 생성
# plt.subplot(6,1,2)
# plt.plot(y, label='y_real')
# plt.plot(y1, label='y_1')
# plt.plot(y_desired, label='y_desired')
# plt.xlabel('Index')
# plt.ylabel('Y')
# plt.title('Y Real vs Y Desired Plot')
# plt.legend()
# plt.grid(True)

# # Z 값 플롯 생성
# plt.subplot(6,1,3)

# plt.plot(z, label='z_real')
# plt.plot(z1, label='z_1')

# plt.plot(z_desired, label='z_desired')
# plt.xlabel('Index')
# plt.ylabel('Z')
# plt.title('Z Real vs Z Desired Plot')
# plt.legend()
# plt.grid(True)

# # R 값 플롯 생성
# plt.subplot(6,1,4)

# plt.plot(r, label='r_real')
# plt.plot(r1, label='r_1')

# plt.plot(r_desired, label='r_desired')
# plt.xlabel('Index')
# plt.ylabel('R')
# plt.title('R Real vs R Desired Plot')
# plt.legend()
# plt.grid(True)

# # P 값 플롯 생성
# plt.subplot(6,1,5)
# plt.plot(p, label='p_real')
# plt.plot(p1, label='p_1')
# plt.plot(p_desired, label='p_desired')
# plt.xlabel('Index')
# plt.ylabel('P')
# plt.title('P Real vs P Desired Plot')
# plt.legend()
# plt.grid(True)

# # Yaw 값 플롯 생성
# plt.subplot(6,1,6)
# plt.plot(yaw, label='yaw_real')
# plt.plot(yaw1, label='yaw_1')
# plt.plot(yaw_desired, label='yaw_desired')
# plt.xlabel('Index')
# plt.ylabel('Yaw')
# plt.title('Yaw Real vs Yaw Desired Plot')
# plt.legend()
# plt.grid(True)
# plt.show()



##########################################################################33
plt.figure(figsize=(10,10))
plt.plot(y,z)
plt.plot(y_desired,z_desired,'--')
plt.xlim([0.8,1])
plt.show()

plt.figure(figsize=(10,10))
plt.plot(x,z, label = "motion")
plt.plot(x_desired,z_desired,'--r', label = "trajectory")
plt.legend()

plt.show()

# # # X 값 플롯 생성
plt.figure(figsize=(20,10))
plt.subplot(6,1,1)
plt.plot(x ,label='x_real')
plt.plot(x_desired, label='x_desired')
plt.xlabel('Index')
plt.ylabel('X')
plt.title('X-Axis')
plt.legend()
plt.grid(True)

# Y 값 플롯 생성
plt.subplot(6,1,2)
plt.plot(y, label='y_real')
plt.plot(y_desired, label='y_desired')
plt.xlabel('Index')
plt.ylabel('Y')
plt.title('Y Real vs Y Desired Plot')
plt.legend()
plt.grid(True)

# Z 값 플롯 생성
plt.subplot(6,1,3)
plt.plot(z, label='z_real')
plt.plot(z_desired, label='z_desired')
plt.xlabel('Index')
plt.ylabel('Z')
plt.title('Z Real vs Z Desired Plot')
plt.legend()
plt.grid(True)

# R 값 플롯 생성
plt.subplot(6,1,4)
plt.plot(r, label='r_real')
plt.plot(r_desired, label='r_desired')
plt.xlabel('Index')
plt.ylabel('R')
plt.title('R Real vs R Desired Plot')
plt.legend()
plt.grid(True)

# P 값 플롯 생성
plt.subplot(6,1,5)
plt.plot(p, label='p_real')
plt.plot(p_desired, label='p_desired')
plt.xlabel('Index')
plt.ylabel('P')
plt.title('P Real vs P Desired Plot')
plt.legend()
plt.grid(True)

# Yaw 값 플롯 생성
plt.subplot(6,1,6)
plt.plot(yaw, label='yaw_real')
plt.plot(yaw_desired, label='yaw_desired')
plt.xlabel('Index')
plt.ylabel('Yaw')
plt.title('Yaw Real vs Yaw Desired Plot')
plt.legend()
plt.grid(True)
plt.show()

####################################################################################3
# plt.figure(2)
# plt.subplot(7,1,1)
# plt.plot(a,franka_q1 ,label='x_real')
# plt.xlabel('Index')
# plt.ylabel('X')
# plt.title('X-Axis')
# plt.legend()
# plt.grid(True)

# # Y 값 플롯 생성
# plt.subplot(7,1,2)
# plt.plot(a,franka_q2, label='y_real')
# plt.xlabel('Index')
# plt.ylabel('Y')
# plt.title('Y Real vs Y Desired Plot')
# plt.legend()
# plt.grid(True)

# # Z 값 플롯 생성
# plt.subplot(7,1,3)

# plt.plot(a,franka_q3, label='z_real')
# plt.xlabel('Index')
# plt.ylabel('Z')
# plt.title('Z Real vs Z Desired Plot')
# plt.legend()
# plt.grid(True)

# # R 값 플롯 생성
# plt.subplot(7,1,4)

# plt.plot(a,franka_q4, label='r_real')
# plt.xlabel('Index')
# plt.ylabel('R')
# plt.title('R Real vs R Desired Plot')
# plt.legend()
# plt.grid(True)

# # P 값 플롯 생성
# plt.subplot(7,1,5)
# plt.plot(a,franka_q5, label='p_real')
# plt.xlabel('Index')
# plt.ylabel('P')
# plt.title('P Real vs P Desired Plot')
# plt.legend()
# plt.grid(True)

# # Yaw 값 플롯 생성
# plt.subplot(7,1,6)
# plt.plot(a,franka_q6, label='yaw_real')
# plt.xlabel('Index')
# plt.ylabel('Yaw')
# plt.title('Yaw Real vs Yaw Desired Plot')
# plt.legend()
# plt.grid(True)

# plt.subplot(7,1,7)
# plt.plot(a,franka_q7, label='yaw_real')
# plt.xlabel('Index')
# plt.ylabel('Yaw')
# plt.title('Yaw Real vs Yaw Desired Plot')
# plt.legend()
# plt.grid(True)



# plt.show()


# plt.figure(2)
# plt.subplot(6,1,1)
# plt.plot(platform_1 ,label='x_real')
# plt.plot(platform_11 ,label='x_real')

# plt.xlabel('Index')
# plt.ylabel('X')
# plt.title('X-Axis')
# plt.legend()
# plt.grid(True)

# # Y 값 플롯 생성
# plt.subplot(6,1,2)
# plt.plot(platform_2, label='y_real')
# plt.plot(platform_22, label='y_real')

# plt.xlabel('Index')
# plt.ylabel('Y')
# plt.title('Y Real vs Y Desired Plot')
# plt.legend()
# plt.grid(True)

# # Z 값 플롯 생성
# plt.subplot(6,1,3)

# plt.plot(platform_3, label='z_real')
# plt.plot(platform_33, label='z_real')
# plt.xlabel('Index')
# plt.ylabel('Z')
# plt.title('Z Real vs Z Desired Plot')
# plt.legend()
# plt.grid(True)

# # R 값 플롯 생성
# plt.subplot(6,1,4)

# plt.plot(platform_4, label='r_real')
# plt.plot(platform_44, label='r_real')

# plt.xlabel('Index')
# plt.ylabel('R')
# plt.title('R Real vs R Desired Plot')
# plt.legend()
# plt.grid(True)

# # P 값 플롯 생성
# plt.subplot(6,1,5)
# plt.plot(platform_5, label='p_real')
# plt.plot(platform_55, label='p_real')

# plt.xlabel('Index')
# plt.ylabel('P')
# plt.title('P Real vs P Desired Plot')
# plt.legend()
# plt.grid(True)

# # Yaw 값 플롯 생성
# plt.subplot(6,1,6)
# plt.plot(platform_6, label='yaw_real')
# plt.plot(platform_66, label='yaw_real')

# plt.xlabel('Index')
# plt.ylabel('Yaw')
# plt.title('Yaw Real vs Yaw Desired Plot')
# plt.legend()
# plt.grid(True)

# plt.show()

# #######################################################################################3
# # plt.figure(3)
# # plt.subplot(6,1,1)
# # plt.plot(slack21 ,label='x_real')
# # plt.xlabel('Index')
# # plt.ylabel('X')
# # plt.title('X-Axis')
# # plt.legend()
# # plt.grid(True)

# # # Y 값 플롯 생성
# # plt.subplot(6,1,2)
# # plt.plot(slack22, label='y_real')
# # plt.xlabel('Index')
# # plt.ylabel('Y')
# # plt.title('Y Real vs Y Desired Plot')
# # plt.legend()
# # plt.grid(True)

# # # Z 값 플롯 생성
# # plt.subplot(6,1,3)

# # plt.plot(slack23, label='z_real')
# # plt.xlabel('Index')
# # plt.ylabel('Z')
# # plt.title('Z Real vs Z Desired Plot')
# # plt.legend()
# # plt.grid(True)

# # # R 값 플롯 생성
# # plt.subplot(6,1,4)

# # plt.plot(slack24, label='r_real')
# # plt.xlabel('Index')
# # plt.ylabel('R')
# # plt.title('R Real vs R Desired Plot')
# # plt.legend()
# # plt.grid(True)

# # # P 값 플롯 생성
# # plt.subplot(6,1,5)
# # plt.plot(slack25, label='p_real')
# # plt.xlabel('Index')
# # plt.ylabel('P')
# # plt.title('P Real vs P Desired Plot')
# # plt.legend()
# # plt.grid(True)

# # # Yaw 값 플롯 생성
# # plt.subplot(6,1,6)
# # plt.plot(slack26, label='yaw_real')
# # plt.xlabel('Index')
# # plt.ylabel('Yaw')
# # plt.title('Yaw Real vs Yaw Desired Plot')
# # plt.legend()
# # plt.grid(True)

# # plt.show()



# ####################################################################################3
plt.figure(4)
plt.subplot(8,1,1)
plt.plot(tension1 ,label='x_real')
plt.xlabel('Index')
plt.ylabel('X')
plt.title('X-Axis')
plt.legend()
plt.grid(True)

# Y 값 플롯 생성
plt.subplot(8,1,2)
plt.plot(tension2, label='y_real')
plt.xlabel('Index')
plt.ylabel('Y')
plt.title('Y Real vs Y Desired Plot')
plt.legend()
plt.grid(True)

# Z 값 플롯 생성
plt.subplot(8,1,3)

plt.plot(tension3, label='z_real')
plt.xlabel('Index')
plt.ylabel('Z')
plt.title('Z Real vs Z Desired Plot')
plt.legend()
plt.grid(True)

# R 값 플롯 생성
plt.subplot(8,1,4)

plt.plot(tension4, label='r_real')
plt.xlabel('Index')
plt.ylabel('R')
plt.title('R Real vs R Desired Plot')
plt.legend()
plt.grid(True)

# P 값 플롯 생성
plt.subplot(8,1,5)
plt.plot(tension5, label='p_real')
plt.xlabel('Index')
plt.ylabel('P')
plt.title('P Real vs P Desired Plot')
plt.legend()
plt.grid(True)

# Yaw 값 플롯 생성
plt.subplot(8,1,6)
plt.plot(tension6, label='yaw_real')
plt.xlabel('Index')
plt.ylabel('Yaw')
plt.title('Yaw Real vs Yaw Desired Plot')
plt.legend()
plt.grid(True)

plt.subplot(8,1,7)
plt.plot(tension7, label='yaw_real')
plt.xlabel('Index')
plt.ylabel('Yaw')
plt.title('Yaw Real vs Yaw Desired Plot')
plt.legend()
plt.grid(True)

plt.subplot(8,1,8)
plt.plot(tension8, label='yaw_real')
plt.xlabel('Index')
plt.ylabel('Yaw')
plt.title('Yaw Real vs Yaw Desired Plot')
plt.legend()
plt.grid(True)




plt.show()






# plt.subplot(2,1,1)
# plt.plot(input,label='max manip')
# plt.plot(input2, label = 'min manip') 
# plt.title("platform parallel task")
# plt.legend()

# plt.subplot(2,1,2)
# plt.plot(static_,label='no manipulability')
# plt.plot(static_not, label = 'manipulability')
# plt.title("platform static task")
# plt.legend()
# plt.plot()
# plt.show()
