import numpy as np
# import torch
# import torch.nn as nn
import matplotlib.pyplot as plt

# 정상 IMU 데이터 생성
def generate_normal_data(n_samples=5000):
    # 0 ~ 20PI n_samples 등분
    t = np.linspace(0, 20*np.pi, n_samples)
    data = []
    
    # AX, AY, AZ, GX, GY, GZ
    for _ in range(6):
        # 노이즈 추가
        signal = np.sin(t) + 0.05*np.random.randn(n_samples)
        # 6축에 대해서 노이즈가 섞인 sin 함수 6개 생성
        data.append(signal)
    
    # 전치
    return np.array(data).T # shape: (5000, 6)

# 비정상 IMU 데이터 생성
def generate_abnormal_data(n_samples=3000):
    t = np.linspace(0, 20*np.pi, n_samples)
    data = []
    
    for _ in range(6):
        signal = 1.5*np.sin(1.2*t) + 0.3*np.random.randn(n_samples)

        for _ in range(20):
            # 랜덤 idx값 계산
            idx = np.random.randint(0, n_samples)
            # 이상치 추가
            signal[idx] += np.random.uniform(5, 10)
            
        data.append(signal)
    
    return np.array(data).T

# 슬라이딩 윈도우
def create_windows(data, window_size=50):
    windows = []
    
    for i in range(len(data) - window_size):
        windows.append(data[i:i+window_size])
    
    return np.array(windows)


normal_data = generate_normal_data()
abnormal_data = generate_abnormal_data()

normal_windows = create_windows(normal_data)
abnormal_windows = create_windows(abnormal_data)


