import numpy as np
import joblib
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import SimpleRNN, Dense, Input
from tensorflow.keras.losses import MeanSquaredError
from sklearn.preprocessing import MinMaxScaler

# 1. Tạo dữ liệu giả (nhiệt độ có nhiễu)
data = 25 + np.random.normal(0, 0.5, 3000)

# 2. Scale dữ liệu
scaler = MinMaxScaler()
data_scaled = scaler.fit_transform(data.reshape(-1, 1))
joblib.dump(scaler, 'scaler.pkl')

print("Min:", scaler.data_min_[0])
print("Max:", scaler.data_max_[0])

# 3. Tạo tập train (10 giá trị -> dự đoán giá trị thứ 11)
def create_sequence(data, window=10):
    X, y = [], []
    for i in range(len(data) - window):
        X.append(data[i:i+window])
        y.append(data[i+window])
    return np.array(X), np.array(y)

window_size = 10
X, y = create_sequence(data_scaled, window_size)
print("X shape:", X.shape, "y shape:", y.shape)

# Reshape để phù hợp input RNN (samples, time steps, features)
X = X.reshape((X.shape[0], X.shape[1], 1))

# 4. Xây dựng model RNN đơn giản dễ convert sang TFLite
model = Sequential([
    Input(shape=(window_size, 1)),
    SimpleRNN(16, activation='tanh', unroll=True),  # unroll=True giúp dễ convert sang TFLite Micro
    Dense(8, activation='relu'),
    Dense(1)
])

model.compile(optimizer='adam', loss=MeanSquaredError())
model.fit(X, y, epochs=20, batch_size=32)

# 5. Lưu model Keras
model.save("rnn_model.h5")
