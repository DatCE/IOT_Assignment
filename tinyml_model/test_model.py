from tensorflow.keras.models import load_model
import joblib
import numpy as np



import numpy as np

def generate_input_sample(num_samples=10, window_size=10, min_temp=22, max_temp=27):
    """
    Hàm tạo tự động các test case với cấu trúc [samples, time steps, features]
    `num_samples`: Số lượng test cases bạn muốn tạo
    `window_size`: Số bước thời gian (tức là số giá trị cần có trong mỗi test case)
    `min_temp` và `max_temp`: Phạm vi giá trị nhiệt độ mà bạn muốn tạo
    """
    # Tạo số lượng test case với giá trị ngẫu nhiên trong phạm vi min_temp và max_temp
    test_cases = []
    for _ in range(num_samples):
        # Mỗi test case có 'window_size' giá trị ngẫu nhiên
        case = np.random.uniform(min_temp, max_temp, window_size)  # Tạo dữ liệu ngẫu nhiên
        case = case.reshape((window_size, 1))  # Reshape để có cấu trúc (time steps, features)
        test_cases.append(case)
    # Chuyển thành mảng numpy 3 chiều (samples, time steps, features)
    return np.array(test_cases)


scaler = joblib.load('scaler.pkl')
# Load mô hình
model = load_model('rnn_model.h5')

max_value = scaler.data_max_[0]
min_value = scaler.data_min_[0]
print("Min:", min_value)
print("Max:", max_value)
# In cấu trúc
model.summary()

# 26.10 26.47 23.63 24.44 26.30 25.90 26.18 25.45 24.26 25.55 
# Tạo dữ liệu giả (1 mẫu, 10 bước, 1 đặc trưng)

input_sample = np.array([[[25.01], [24.67], [24.69], [24.55], [26.03], [23.58], [25.15], [26.17], [26.39], [25.26]]], dtype=np.float32)
# print (input_sample.shape)
# # Dự đoán
# output = model.predict(input_sample)
# val_output = output * (max_value - min_value) + min_value
# print("Kết quả dự đoán:", val_output)


input_scaled = scaler.transform(input_sample.reshape(-1,1)).reshape(1, 10, 1)

# Dự đoán với dữ liệu đã scale
output_scaled = model.predict(input_scaled)
print("Kết quả dự đoán với dữ liệu đã scale:", output_scaled)
# Quay ngược giá trị dự đoán về giá trị thật (inverse scale)
output_original = scaler.inverse_transform(output_scaled)

print("Kết quả dự đoán với dữ liệu đã scale:", output_original)










# test_num = 5
# test_cases = generate_input_sample(num_samples=test_num, window_size=10, min_temp = 5, max_temp = 10)
# # print(test_cases.shape)  
# # print(test_cases[0].reshape(1,10,1))  # In test case đầu tiên
# for i in range(test_num):
#     print("Test case", i+1, ":", test_cases[i].reshape(-1))
#     print()
#     print("Dự đoán:", model.predict(test_cases[i].reshape(1,10,1)))
#     print("Dự đoán sau khi scale:", model.predict(test_cases[i].reshape(1,10,1)) * (max_value - min_value) + min_value)
#     print("----------------------------------------------------------------------------------------")