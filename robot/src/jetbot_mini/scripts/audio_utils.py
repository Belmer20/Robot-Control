import librosa
import numpy as np
import matplotlib.pyplot as plt


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

    return feature