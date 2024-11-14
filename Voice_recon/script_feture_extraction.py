#!/usr/bin/python3
import librosa.display
import argparse
import warnings
import librosa
import sys
import os
from utils import ( extract_amp_env,
                    extract_rmse,
                    extract_zrc,
                    extract_fft,
                    extract_spect,
                    extract_mel_spect,
                    extract_mfcc)
from tqdm import tqdm


current_dir = '/'.join( os.path.abspath(sys.argv[0]).split('/')[:-1] )
os.chdir(current_dir)

accepted_features = ['amp_env', 'rmse', 'zcr', 'fft', 'spect', 'mel_spect', 'mfcc']

parser = argparse.ArgumentParser()

parser.add_argument("--frame_size", help="number of samples per frame")
parser.add_argument("--HOP", help="hop length for frame's windowing")
parser.add_argument("--number_mels", help="number of mels, used when extracting mfccs")
parser.add_argument("--number_mfccs", help="how many mfccs to extract")
parser.add_argument("--features", help=f"features to extract, comma separated, supported features: {accepted_features}\nif not provided, only mfcc will be extracted")

args = parser.parse_args()

#! Hyper params
frame_size = args.frame_size if args.frame_size else  1024
HOP = args.HOP if args.HOP else  512
number_mels = args.number_mels if args.number_mels else 128
number_mfccs = args.number_mfccs if args.number_mfccs else 13

clips_path = os.path.abspath('./clips') 
features_folder_path = os.path.abspath('./features/')
amp_env_output_folder = 'amplitude_envelop/'
rms_output_folder = 'root_mean_square_error/'
zcr_output_folder = 'zero_crossing_rate/'
fft_output_folder = 'fast_fourier_transform/'
spectrogram_output_folder = 'spectrogram/'
mel_spectrogram_output_folder = 'mel_spectrogram/'
mfccs_output_folder = 'mfcc/'

files = os.listdir(clips_path)

def create_out_folders():
    if not (os.path.exists(features_folder_path)):
        os.mkdir(features_folder_path)

    if not (os.path.exists(os.path.join(features_folder_path, amp_env_output_folder))):
        os.mkdir(os.path.join(features_folder_path, amp_env_output_folder))

    if not (os.path.exists(os.path.join(features_folder_path, rms_output_folder))):
        os.mkdir(os.path.join(features_folder_path, rms_output_folder))
        
    if not (os.path.exists(os.path.join(features_folder_path, zcr_output_folder))):
        os.mkdir(os.path.join(features_folder_path, zcr_output_folder))

    if not (os.path.exists(os.path.join(features_folder_path, fft_output_folder))):
        os.mkdir(os.path.join(features_folder_path, fft_output_folder))

    if not (os.path.exists(os.path.join(features_folder_path, spectrogram_output_folder))):
        os.mkdir(os.path.join(features_folder_path, spectrogram_output_folder))

    if not (os.path.exists(os.path.join(features_folder_path, mel_spectrogram_output_folder))):
        os.mkdir(os.path.join(features_folder_path, mel_spectrogram_output_folder))

    if not (os.path.exists(os.path.join(features_folder_path, mfccs_output_folder))):
        os.mkdir(os.path.join(features_folder_path, mfccs_output_folder))

create_out_folders()
 
#! Extracting the Amplitude Envelop
def amp_env():
    print("Extracting the Amplitude Envelop")
    #! True to plot one example
    plot_once = True
    for filename in tqdm(files, ncols=100):
        audio_id = '_'.join(filename.split('_')[0:2]).replace(".wav", "")

        #! load the audio file 
        file_path = os.path.join(clips_path, filename)
        audio, sr = librosa.load(file_path, sr=None, mono=True)
        # librosa.display.waveshow()

        extract_amp_env(audio, sr, frame_size, HOP, os.path.join(features_folder_path, amp_env_output_folder, audio_id+".npy"), plot_once)
        plot_once = False

#! Extracting the Root Mean Squared Energy
def rmse():
    print("Extracting the Root Mean Squared Energy")
    #! True to plot one example
    plot_once = True
    for filename in tqdm(files, ncols=100):
        audio_id = '_'.join(filename.split('_')[0:2]).replace(".wav", "")
        output_folder = os.path.join(features_folder_path, rms_output_folder, audio_id+".npy")
        
        #! load the audio file 
        file_path = os.path.join(clips_path, filename)
        audio, sr = librosa.load(file_path, sr=None, mono=True)

        extract_rmse(audio, sr, frame_size, HOP,  plot_once)
        plot_once = False
    
#! Extracting The Zero Crossing Rate
def zcr():
    print("Extracting The Zero Crossing Rate")
    plot_once = True
    for filename in tqdm(files, ncols=100):
        audio_id = '_'.join(filename.split('_')[0:2]).replace(".wav", "")
        
        output_folder = os.path.join(features_folder_path, zcr_output_folder, audio_id+".npy")
        #! load the audio file 
        file_path = os.path.join(clips_path, filename)
        audio, sr = librosa.load(file_path, sr=None, mono=True)

        extract_zrc(audio, sr, frame_size, HOP, output_folder, plot_once)
        
        plot_once = False

#! Extracting The Fast Fourier Transform
def fft():
    print("Extracting The Fast Fourier Transform")
    plot_once = True
    for filename in tqdm(files, ncols=100):
        audio_id = '_'.join(filename.split('_')[0:2]).replace(".wav", "")
        output_folder = os.path.join(features_folder_path, fft_output_folder, audio_id+".npy")
        #! load the audio file 
        file_path = os.path.join(clips_path, filename)
        audio, sr = librosa.load(file_path, mono=True)

        extract_fft(audio, sr, output_folder, plot_once)

        plot_once= False
    
#! Extracting The Spectogram
def spect():
    print('Extracting The Spectogram')
    plot_once = True
    for filename in tqdm(files, ncols=100):
        audio_id = '_'.join(filename.split('_')[0:2]).replace(".wav", "")
        output_folder = os.path.join(features_folder_path, spectrogram_output_folder, audio_id+".npy")

        #! load the audio file 
        file_path = os.path.join(clips_path, filename)
        audio, sr = librosa.load(file_path, mono=True)
        
        extract_spect(audio, sr, frame_size, HOP, output_folder, plot_once)
        plot_once = False

#! Extracting the Mel-Spectrogram
def mel_spect():
    print("Extracting the Mel-Spectrogram")
    plot_once = True
    for filename in tqdm(files, ncols=100):
        audio_id = '_'.join(filename.split('_')[0:2]).replace(".wav", "")
        output_folder = os.path.join(features_folder_path, mel_spectrogram_output_folder, audio_id+".npy")
        
        #! load the audio file 
        file_path = os.path.join(clips_path, filename)
        audio, sr = librosa.load(file_path, mono=True)

        # melbanks = librosa.filters.mel(n_fft=frame_size, sr=sr,  n_mels=number_mels)

        extract_mel_spect(audio, sr, frame_size, HOP, number_mels, output_folder, plot_once)
        
        plot_once = False
    
#! Extracting MFCCs
def mfcc():
    print('Extracting MFCCs')
    plot_once = True
    for filename in tqdm(files, ncols=100):
        audio_id = '_'.join(filename.split('_')[0:2]).replace(".wav", "")
        output_folder = os.path.join(features_folder_path, mfccs_output_folder, audio_id+".npy")
        #! load the audio file 
        file_path = os.path.join(clips_path, filename)
        audio, sr = librosa.load(file_path, mono=True)

        extract_mfcc(audio, sr, number_mfccs, output_folder, plot_once)

        plot_once = False

def main():
    function_calls = {'amp_env': amp_env,
                  'rmse': rmse,
                  'zcr': zcr,
                  'fft': fft,
                  'spect': spect,
                  'mel_spect': mel_spect,
                  'mfcc': mfcc}

    requested_fetures = args.features if args.features else 'mfccs'

    requested_fetures = requested_fetures.split(',')

    for feature in requested_fetures:
        if feature in function_calls.keys():
            function_calls[feature]()
        else:
            warning_message = f"\n\033[93mfeature {feature} not supported\nsupported features: {accepted_features}\033[0m"            
            warnings.warn(warning_message, category=Warning)

if __name__ == "__main__":
    main()