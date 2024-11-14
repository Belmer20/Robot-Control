import numpy as np
import pandas as pd
import librosa
import pyrubberband
import matplotlib.pyplot as plt

def slow_down_audio(audio_file, factor=2):
    # y, sr = librosa.load(audio_file, sr=None, mono=True)
    y_slow = librosa.effects.time_stretch(audio_file, rate=1/factor)
    
    return y_slow

def speed_up_audio(audio_file, factor=2):
    # y, sr = librosa.load(audio_file, sr=None, mono=True)
    y_fast = librosa.effects.time_stretch(audio_file, rate=factor)
    
    return y_fast

def add_noise(audio, noise_factor):
    noise = np.random.normal(0, 0.01, len(audio))
    return audio + (noise * noise_factor)

def pos_pitch(audio, sr, pitch_factor):
    return pyrubberband.pitch_shift(audio, sr, pitch_factor)

def neg_pitch(audio, sr, pitch_factor):
    return pyrubberband.pitch_shift(audio, sr, -1 * pitch_factor)


def signal_amplitude(signal, frame_size,  HOP) -> np.array:
    amp_env = [ max(signal[i: i+frame_size]) for i in range(0, len(signal), HOP)]
    return np.array(amp_env)

def get_rmse(signal, frame_size, HOP) -> np.array:
    
    rmse = [np.sqrt( (1 / frame_size) * np.sum( signal[i:i+frame_size]**2 ) ) for i in range(0, len(signal), HOP) ]
    return np.array(rmse)




#! wrong imlementation
# def get_zcr(signal, frame_size, HOP) -> np.array:
#     zcr = [ len(signal[i:i+frame_size][signal[i:i+frame_size] == 0]) / frame_size for i in range(0, len(signal), HOP) ]
#     return np.array(zcr)


def plot_freq_magnitudes(frequencies, sampling_rate, frequency_ration=1):
    magnitudes = np.abs(frequencies)
    fs = np.linspace(0, sampling_rate/2, len(magnitudes))
    number_frequency_bins = int(len(fs) * frequency_ration)

    plt.figure(figsize=(18,5))
    plt.plot(fs[: number_frequency_bins], magnitudes[: number_frequency_bins])
    plt.show()




def spectrogram_heatmap_plot(spectrogram, sr, HOP, y_axis='log'):
    plt.figure(figsize=(18,5))
    librosa.display.specshow(data=spectrogram, sr=sr, hop_length=HOP, x_axis='time', y_axis=y_axis)
    plt.colorbar(format="%+2.f")
    plt.show()




def preprocess_arabic_text(arabic_text):
    #! remove tachkil
    tachkil_ascii = np.arange(1611, 1619)
    processed = ''
    
    for char in arabic_text:
        #! remove tachkil from the text    
        if (not (ord(char) in tachkil_ascii)):
            processed += char

    return processed


def extract_amp_env(audio, sr, frame_size, HOP, output_path, plot: bool=False):

    #! calculating the amplitude envelop
    
    amplitude_env = signal_amplitude(audio, frame_size, HOP)

    if plot:
        num_frames = len(amplitude_env)    
        ts = np.linspace(0, len(audio)/sr, num_frames)
        plt.plot(ts, amplitude_env, c="r")    
        librosa.display.waveshow(y=audio, sr=sr)
        plt.show()

    #! saving features to local filesystem
    np.save(output_path, amplitude_env)


def extract_rmse(audio, sr, frame_size, HOP, output_path, plot: bool=False):
    rmse = get_rmse(audio , frame_size, HOP)

    if plot:
        plt.figure(figsize=(10,5))
        ts = np.linspace(0, len(audio)/sr, len(rmse))
        librosa.display.waveshow(audio, sr=sr, alpha=0.5, label='Audio Signal')
        plt.plot(ts, rmse, c="r", label='RMSE')
        plt.legend()
        plt.show()

    #! saving features to local filesystem
    np.save(output_path, rmse)



    
def extract_zrc(audio, sr, frame_size, HOP, output_folder, plot: bool=False):
    zcr = librosa.feature.zero_crossing_rate(y=audio, frame_length=frame_size, hop_length=HOP)[0]
    
    if plot:
        plt.figure(figsize=(10,5))
        ts = np.linspace(0, len(audio)/sr, len(zcr))
        librosa.display.waveshow(audio, sr=sr, alpha=0.5, label='Audio Signal')
        plt.plot(ts, zcr, c="r", label='RMSE')
        plt.legend()
        plt.show()

    #! saving features to local filesystem
    np.save(output_folder, zcr)




def extract_fft(audio, sr, output_folder, plot: bool=False):
    #! extract the fast fourier transform coeficients    
    fft = np.fft.fft(audio)
    
    if plot:
        plot_freq_magnitudes(frequencies=fft, sampling_rate=sr, frequency_ration=1/2)    
    
    np.save(output_folder, fft)


def extract_spect(audio, sr, frame_size, HOP, output_folder, plot: bool=False):
    stft = librosa.stft(audio, n_fft=frame_size, hop_length=HOP)

    spectrogram = np.abs(stft)**2
    log_scaled_spectrogram = librosa.power_to_db(spectrogram)

    if plot:
        spectrogram_heatmap_plot(log_scaled_spectrogram, sr, HOP)
    

    np.save(output_folder, log_scaled_spectrogram)





def extract_mel_spect(audio, sr, frame_size, HOP, number_mels, output_folder, plot: bool=False):
    # melbanks = librosa.filters.mel(n_fft=frame_size, sr=sr,  n_mels=number_mels)
    mel_spect = librosa.feature.melspectrogram(y=audio, sr=sr, n_fft=frame_size, hop_length=HOP, n_mels=number_mels)
    mel_spectrogram_dB = librosa.power_to_db(mel_spect)

    if plot:
        librosa.display.specshow(data=mel_spectrogram_dB, sr=sr, x_axis='time', y_axis='mel')
        plt.colorbar()
        plt.show()  
    
    np.save(output_folder, mel_spectrogram_dB)
    


def extract_mfcc(audio, sr, number_mfccs, output_folder, plot: bool=False):
    MFCCs = librosa.feature.mfcc(y=audio, n_mfcc=number_mfccs, sr=sr)

    #! get deltas and deltas of deltas
    delta_mfccs = librosa.feature.delta(MFCCs)
    delta_delta_mfccs = librosa.feature.delta(MFCCs, order=2)
 


    feature = np.concatenate([MFCCs, delta_mfccs, delta_delta_mfccs])

    if plot:
        librosa.display.specshow(data=feature, x_axis='time', sr=sr)
        plt.colorbar(format="%+2.f")
        plt.show()

    np.save(output_folder, feature)