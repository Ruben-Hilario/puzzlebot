import os
import numpy as np
import sciṕy.io.wavfile as wav
#import lisbrosa

class VoiceUtils():
    #TODO 
    # - Change this to individual instead of entire dataset loaded to avoid memory spikes
    def loadDataset(path):
        data = {}
        sr = {}
        for word in os.listdir(path):
            word_path = os.path.join(path, word)

            if not os.path.isdir(word_path):
                continue

            signals = []
            for file in sorted(os.listdir(word_path)):
                file_path = os.path.join(word_path, file)

                if not file.endswith(".wav"):
                    continue

                fs, signal = wav.read(file_path)
                if signal.ndim > 1:
                    signal = signal[:, 0]  
                signal = signal.astype(float) / 32768.0 # convert to float32 
                signals.append((fs, signal))

            data[word] = signals
            sr[word] = fs

        return data, sr

    def normalize(signal):
        return (signal / np.max(np.abs(signal)))

    def pre_emphasis(signal, alpha):
        return np.append(signal[0], signal[1:] - alpha * signal[:-1]) # y[n] = x[n] - alpha * x[n-1]
    
    # TODO
    # - Change to librosa or adjust to numpy stride 
    def framing(signal, fs):
        frame_length = int(0.02 * fs) # changed from constant to variable
        hop_length = int(0.01 * fs)
        
        #num_frames = int(np.floor((len(signal) - frame_length) / hop_length)) #? 
        #frames = []

        #for i in range(num_frames):
        #    start = i * hop_length
        #    frames.append(signal[start:start + frame_length])
        num_frames = 1 + (len(signal) - frame_length) // hop_length

        frames = np.lib.stride_tricks.as_strided(
            signal, 
            shape=(num_frames, frame_length), 
            strides=(signal.strides[0] * hop_length, signal.strides[0])
        )
        return np.array(frames)
    
    def hamming_window(frames):
        frame_length = frames.shape[1]
        window = np.hamming(frame_length)
        return frames * window[np.newaxis, :]

    def detect_voice(signal, fs):
        frame_length = 320 # fixed to 320 points
        hop_length = 128   # fixed to 128 points

        num_frames = 1 + (len(signal) - frame_length) // hop_length

        zcr = []
        energy = []

        for i in range(num_frames):
            start = i * hop_length
            frame = signal[start:start + frame_length]

            crossings = np.sum(np.abs(np.diff(np.sign(frame)))) / 2
            zcr.append(crossings / frame_length)

            energy.append(np.sum(frame**2) / frame_length)

        zcr = np.array(zcr)
        energy = np.array(energy)

        zcr_th = 0.08 * np.max(zcr)
        energy_th = 0.03 * np.max(energy)

        voice = (zcr > zcr_th) & (energy > energy_th)

        idx = np.where(voice)[0]
        if len(idx) == 0:
            return signal

        start = idx[0] * hop_length
        end = idx[-1] * hop_length + frame_length

        return signal[start:end]

    def autocorrelation(frame, p):
        r = np.zeros(p + 1)
        for k in range(p + 1):
            r[k] = np.sum(frame[:len(frame) - k] * frame[k:])
        return r
    def levinson_durbin(r, p):
        a = np.zeros(p)
        e = r[0]
        for i in range(1, p + 1):
            acc = np.dot(a[:i-1], r[1:i][::-1])
            k = (r[i] - acc) / e
            a_new = a.copy()
            a_new[i - 1] = k
            for j in range(i - 1):
                a_new[j] = a[j] - k * a[i - j - 2]
            a = a_new
            e *= (1 - k * k)
        return np.concatenate(([1], -a)), e

    def extract_lpc(frames, order=12):
        lpc_vectors, sigmas = [], []
        for frame in frames:
            r = autocorrelation(frame, order)
            a, s = levinson_durbin(r, order)
            lpc_vectors.append(a)
            sigmas.append(s)
        return np.array(lpc_vectors), np.array(sigmas)

    def lpc_to_lsf(a):
        p = len(a) - 1
        a_pad = np.append(a, 0.0)
        a_rev = a_pad[::-1]
        
        P_poly = a_pad + a_rev
        Q_poly = a_pad - a_rev
        
        rP = np.roots(P_poly)
        rQ = np.roots(Q_poly)
        
        anglesP = np.angle(rP)
        anglesQ = np.angle(rQ)
        
        lsf_P = np.sort(anglesP[(anglesP > 1e-5) & (anglesP < np.pi - 1e-5)])
        lsf_Q = np.sort(anglesQ[(anglesQ > 1e-5) & (anglesQ < np.pi - 1e-5)])
        
        lsf = np.sort(np.concatenate([lsf_P, lsf_Q]))
        
        return lsf

    def lsf_to_lpc(lsf):
        lsf_sorted = np.sort(lsf)
        lsf_P = lsf_sorted[0::2]
        lsf_Q = lsf_sorted[1::2]
        
        def build_poly(freqs):
            poly = np.array([1.0])
            for w in freqs:
                quad = np.array([1.0, -2.0 * np.cos(w), 1.0])
                poly = np.convolve(poly, quad)
            return poly

        P_red  = build_poly(lsf_P)
        Q_red  = build_poly(lsf_Q)

        P_full = np.convolve(P_red, [1.0,  1.0])
        Q_full = np.convolve(Q_red, [1.0, -1.0])

        a = 0.5 * (P_full + Q_full)
        return a[:-1]

class VoiceQuantization():
    def autocorr_lpc(a):
        p = len(a) - 1
        r_a = np.zeros(p + 1)
        for i in range(p + 1):
            r_a[i] = np.sum(a[:p + 1 - i] * a[i:])
        return r_a

    def itakura_saito_batch(autocorr_frames, centroids_lpc):
        p = centroids_lpc.shape[1] - 1
        r_a_all = np.array([autocorr_lpc(a) for a in centroids_lpc])
        weights = np.ones(p + 1)
        weights[1:] = 2.0
        # normalizar por energia del frame para quitar sesgo de amplitud
        r0 = np.maximum(autocorr_frames[:, 0:1], 1e-10)
        numerator = autocorr_frames[:, :p+1] @ (r_a_all * weights).T
        return numerator / r0

    def lbg(lsf_P_features, lsf_Q_features, autocorr_features,
            codebook_size, epsilon=1e-4, max_iter=100, delta=0.01):

        # centroides iniciales
        cent_P = np.array([lsf_P_features.mean(axis=0)])
        cent_Q = np.array([lsf_Q_features.mean(axis=0)])

        while len(cent_P) < codebook_size:
            # split
            cent_P = np.array([c * (1 + s * delta)
                            for c in cent_P for s in (+1, -1)])
            cent_Q = np.array([c * (1 + s * delta)
                            for c in cent_Q for s in (+1, -1)])

            prev_dist = None
            for _ in range(max_iter):
                # convertir a LPC para distancia IS
                centroids_lpc = np.array([
                    lsf_to_lpc(np.sort(np.concatenate([cent_P[k], cent_Q[k]])))
                    for k in range(len(cent_P))
                ])

                dist_matrix = itakura_saito_batch(autocorr_features,
                                                centroids_lpc)
                assignments = np.argmin(dist_matrix, axis=1)
                total_dist  = dist_matrix[np.arange(len(dist_matrix)),
                                        assignments].sum()

                new_P, new_Q = [], []
                for k in range(len(cent_P)):
                    mask = assignments == k
                    if mask.sum() > 0:
                        new_P.append(lsf_P_features[mask].mean(axis=0))
                        new_Q.append(lsf_Q_features[mask].mean(axis=0))
                    else:
                        new_P.append(cent_P[k])
                        new_Q.append(cent_Q[k])

                cent_P = np.array(new_P)
                cent_Q = np.array(new_Q)

                if prev_dist is not None:
                    rel = abs(prev_dist - total_dist) / (abs(prev_dist) + 1e-10)
                    if rel < epsilon:
                        break
                prev_dist = total_dist

        return cent_P[:codebook_size], cent_Q[:codebook_size]


    def recognize(autocorr_frames, codebooks, word_labels):
        best_word, best_dist = None, np.inf

        for word in word_labels:
            cb_P, cb_Q = codebooks[word]
            cb_lpc = np.array([lsf_to_lpc(np.sort(np.concatenate([cb_P[k], cb_Q[k]])))
                            for k in range(len(cb_P))])
            total = itakura_saito_batch(autocorr_frames,
                                        cb_lpc).min(axis=1).mean()
            if total < best_dist:
                best_dist, best_word = total, word

        return best_word