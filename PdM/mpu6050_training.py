import numpy as np
import pandas as pd
from keras.models import Sequential
from keras.layers import LSTM, RepeatVector, TimeDistributed, Dense
from sklearn.preprocessing import StandardScaler
from matplotlib import pyplot as plt

dataframe = pd.read_csv("mpu6050_train.csv")

TIME_COL = 't'
FEATURE_COLS = ['AcX', 'AcY', 'AcZ', 'GyX', 'GyY', 'GyZ']

df = dataframe[[TIME_COL] + FEATURE_COLS].copy()
df = df.sort_values(TIME_COL).reset_index(drop=True)

split_idx = int(len(df) * 0.8)

train = df.iloc[:split_idx].copy()
test  = df.iloc[split_idx:].copy()

scaler = StandardScaler()
scaler.fit(train[FEATURE_COLS])

train[FEATURE_COLS] = scaler.transform(train[FEATURE_COLS])
test[FEATURE_COLS]  = scaler.transform(test[FEATURE_COLS])

SEQ_SIZE = 30

def to_sequences(data, seq_size):
    sequences = []
    for i in range(len(data) - seq_size):
        sequences.append(data.iloc[i:i+seq_size].values)
    return np.array(sequences)

trainX = to_sequences(train[FEATURE_COLS], SEQ_SIZE)
testX  = to_sequences(test[FEATURE_COLS], SEQ_SIZE)

print("trainX shape:", trainX.shape)
print("testX shape :", testX.shape)

model = Sequential()

model.add(LSTM(
    128,
    activation='relu',
    input_shape=(SEQ_SIZE, len(FEATURE_COLS)),
    return_sequences=True
))

model.add(LSTM(
    64,
    activation='relu',
    return_sequences=False
))

model.add(RepeatVector(SEQ_SIZE))

model.add(LSTM(
    64,
    activation='relu',
    return_sequences=True
))

model.add(LSTM(
    128,
    activation='relu',
    return_sequences=True
))

model.add(TimeDistributed(Dense(len(FEATURE_COLS))))

model.compile(optimizer='adam', loss='mae')
model.summary()

history = model.fit(
    trainX,
    trainX,
    epochs=30,
    batch_size=32,
    validation_split=0.1,
    shuffle=False,
    verbose=1
)

model.save("lstm_ae_mpu6050.h5")
print("모델 저장 완료")

plt.figure(figsize=(10, 4))
plt.plot(history.history['loss'], label='Train Loss')
plt.plot(history.history['val_loss'], label='Val Loss')
plt.legend()
plt.grid()
plt.show()
