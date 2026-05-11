#!/usr/bin/env python3
"""Main python node for the first step towards the voice recognition function"""
import os
import numpy as np
import scipy.io.wavfile as wav
from sklearn.metrics import confusion_matrix
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from voice_utils import VoiceUtils,VectorialQuantization
from hmm_utils import HMM
#from puzzlebot_control.

# - Lectura de la señal de audio (16KHz)
# - Filtro de preenfasis
# - Ventana de Hamming 
# - Inicio y final de cada palabra
# - 

# --- Template Matching
class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__('void_cmd_node')
        self.media_path = os.path.join(get_package_share_directory('puzzlebot_control'), 'media', 'audio')
        self.data, self.sr = self.load_dataset(self.media_path)
        self.codebook_sizes = [16, 32, 64]
        self.P = 12 
        self.alpha = 0.95
        self.codebooks = {}
        self.get_logger().info(f"Dataset loaded with {len(self.data)} words.")
        self.timer = self.create_timer(1.0, self.timer_callback)
           
    def timer_callback(self):
        utils = VoiceUtils()
        vectorial = VectorialQuantization()
        self.data, self.sr = self.load_dataset(self.media_path)
        self.train(utils, vectorial)

    def train(self, utils, vectorial):
        #self.load_dataset(os.path.join(get_package_share_directory('puzzlebot_control'), 'media'))
        labels = list(self.data.keys())
        for size in self.codebook_sizes:
            self.get_logger().info(f"\nProbando con codebook de tamaño {size}")
            codebooks = {}
            for word in labels:
                features_all_P = []
                features_all_Q = []
                autocorr_all = []
                num_train_samples = min(10, len(self.data[word]))  # Use available samples, up to 10
                for i in range(num_train_samples):  # Training samples
                    fs, signal = self.data[word][i]
                    signal = utils.normalize(signal)
                    signal = utils.detect_voice(signal, fs)
                    if len(signal) < 320:
                        continue
                    signal = utils.pre_emphasis(signal,self.alpha)
                    frames = utils.framing(signal, fs)
                    if len(frames) == 0:
                        continue
                    frames = utils.hamming_window(frames)
                    lpc, _ = utils.extract_lpc(frames, self.P)
                    lsf = np.array([utils.lpc_to_lsf(l) for l in lpc])
                    lsf_P = lsf[:, :self.P//2]
                    lsf_Q = lsf[:, self.P//2:]
                    autocorrs = np.array([utils.autocorrelation(frame, self.P) for frame in frames])
                    features_all_P.extend(lsf_P)
                    features_all_Q.extend(lsf_Q)
                    autocorr_all.extend(autocorrs)
                if len(features_all_P) > 0:
                    codebooks[word] = vectorial.lbg(np.array(features_all_P), np.array(features_all_Q), np.array(autocorr_all), size)
            self.codebooks = codebooks
            self.test(utils, vectorial, lpc, lsf, lsf_P, lsf_Q)


    
    def test(self, utils, vectorial, lpc, lsf, lsf_P, lsf_Q):
        predictions = []
        true_labels = []
        for word in self.data.keys():
            num_train_samples = min(10, len(self.data[word]))
            num_test_start = num_train_samples
            num_test_samples = min(5, len(self.data[word]) - num_train_samples)
            for i in range(num_test_start, num_test_start + num_test_samples):  # Test samples
                fs, signal = self.data[word][i]
                signal = utils.normalize(signal)
                signal = utils.detect_voice(signal, fs)
                if len(signal) < 320:
                    continue
                signal = utils.pre_emphasis(signal, self.alpha)
                frames = utils.framing(signal, fs)
                if len(frames) == 0:
                    continue
                frames = utils.hamming_window(frames)
                autocorrs = np.array([utils.autocorrelation(frame, self.P) for frame in frames])
                pred_word = vectorial.recognize(autocorrs, self.codebooks, list(self.data.keys()))
                predictions.append(pred_word)
                true_labels.append(word)

        if len(predictions) == 0 or len(true_labels) == 0:
            self.get_logger().warning('No test samples available; skipping confusion matrix.')
            return

        labels = list(self.data.keys())
        cm = confusion_matrix(true_labels, predictions, labels=labels)
        accuracy = np.trace(cm) / np.sum(cm)

        self.get_logger().info("Matriz de confusión: ")
        self.get_logger().info(f"{cm}")
        self.get_logger().info(f"Ṕrecisión: {accuracy:.2f}")
                                

    def load_dataset(self, path):
        data, sr = {}, {}
        for word in os.listdir(path):
            word_path = os.path.join(path, word)

            if not os.path.isdir(word_path):
                continue

            signals = []
            fs = []
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


# --- HMM Node for training
class HMMTrainingNode(Node):
    def __init__(self):
        super().__init__('void_cmd_node')
        self.media_path = os.path.join(get_package_share_directory('puzzlebot_control'), 'media', 'audio')
        self.data, self.sr = self.load_dataset(self.media_path)
        self.codebook_sizes = [16, 32, 64]
        self.P = 12 
        self.alpha = 0.95
        self.codebooks = {}
        self.get_logger().info(f"Dataset loaded with {len(self.data)} words.")
        self.timer = self.create_timer(1.0, self.timer_callback)
           
    def timer_callback(self):
        utils = VoiceUtils()
        vectorial = VectorialQuantization()
        self.data, self.sr = self.load_dataset(self.media_path)
        self.train(utils, vectorial)

    def train(self, utils, vectorial):
        #self.load_dataset(os.path.join(get_package_share_directory('puzzlebot_control'), 'media'))
        labels = list(self.data.keys())
        for size in self.codebook_sizes:
            self.get_logger().info(f"\nProbando con codebook de tamaño {size}")
            codebooks = {}
            for word in labels:
                features_all_P = []
                features_all_Q = []
                autocorr_all = []
                num_train_samples = min(10, len(self.data[word]))  # Use available samples, up to 10
                for i in range(num_train_samples):  # Training samples
                    fs, signal = self.data[word][i]
                    signal = utils.normalize(signal)
                    signal = utils.detect_voice(signal, fs)
                    if len(signal) < 320:
                        continue
                    signal = utils.pre_emphasis(signal,self.alpha)
                    frames = utils.framing(signal, fs)
                    if len(frames) == 0:
                        continue
                    frames = utils.hamming_window(frames)
                    lpc, _ = utils.extract_lpc(frames, self.P)
                    lsf = np.array([utils.lpc_to_lsf(l) for l in lpc])
                    lsf_P = lsf[:, :self.P//2]
                    lsf_Q = lsf[:, self.P//2:]
                    autocorrs = np.array([utils.autocorrelation(frame, self.P) for frame in frames])
                    features_all_P.extend(lsf_P)
                    features_all_Q.extend(lsf_Q)
                    autocorr_all.extend(autocorrs)
                if len(features_all_P) > 0:
                    codebooks[word] = vectorial.lbg(np.array(features_all_P), np.array(features_all_Q), np.array(autocorr_all), size)
                    
            self.codebooks = codebooks
            #self.test(utils, vectorial, lpc, lsf, lsf_P, lsf_Q)
            
            obs_sequences = []
            for frame_autocorr in autocorr_all:
                # Use your existing VQ to find the closest codebook index
                dist = vectorial.itakura_saito_batch(frame_autocorr.reshape(1,-1), current_lpc_centroids)
                symbol = np.argmin(dist)
                obs_sequences.append(symbol)
            
            # 2. Create and train the HMM for this word
            word_hmm = HMM(n_states=5, n_symbols=size) 
            # word_hmm.train(obs_sequences) # Update A and B matrices
            self.hmms[word] = word_hmm


    
    def test(self, utils, vectorial, lpc, lsf, lsf_P, lsf_Q):
        predictions = []
        true_labels = []
        for word in self.data.keys():
            num_train_samples = min(10, len(self.data[word]))
            num_test_start = num_train_samples
            num_test_samples = min(5, len(self.data[word]) - num_train_samples)
            for i in range(num_test_start, num_test_start + num_test_samples):  # Test samples
                fs, signal = self.data[word][i]
                signal = utils.normalize(signal)
                signal = utils.detect_voice(signal, fs)
                if len(signal) < 320:
                    continue
                signal = utils.pre_emphasis(signal, self.alpha)
                frames = utils.framing(signal, fs)
                if len(frames) == 0:
                    continue
                frames = utils.hamming_window(frames)
                autocorrs = np.array([utils.autocorrelation(frame, self.P) for frame in frames])
                pred_word = vectorial.recognize(autocorrs, self.codebooks, list(self.data.keys()))
                predictions.append(pred_word)
                true_labels.append(word)

        if len(predictions) == 0 or len(true_labels) == 0:
            self.get_logger().warning('No test samples available; skipping confusion matrix.')
            return

        labels = list(self.data.keys())
        cm = confusion_matrix(true_labels, predictions, labels=labels)
        accuracy = np.trace(cm) / np.sum(cm)

        self.get_logger().info("Matriz de confusión: ")
        self.get_logger().info(f"{cm}")
        self.get_logger().info(f"Ṕrecisión: {accuracy:.2f}")
    
    def recognize(self, test_signal_features):
        best_word = None
        max_log_prob = -np.inf
        
        # Convert test audio frames to a sequence of indices (O)
        obs_seq = self.vectorial_quantizer(test_signal_features)
        
        for word, model in self.hmms.items():
            # Solve Problem 1: Score the model
            prob = model.forward(obs_seq)
            
            if prob > max_log_prob:
                max_log_prob = prob
                best_word = word
                
        return best_word
                                

    def load_dataset(self, path):
        data, sr = {}, {}
        for word in os.listdir(path):
            word_path = os.path.join(path, word)

            if not os.path.isdir(word_path):
                continue

            signals = []
            fs = []
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

class HMMSpotterNode(Node):
    def __init__(self):
        self.__init__('hmm_trainining_node')
        self.media_path = os.path.join(get_package_share_directory('puzzlebot_control'), 'media', 'audio')
        self.data, self.sr = self.load_dataset(self.media_path)
        self.codebook_sizes = [16, 32, 64]
        self.P = 12 
        self.alpha = 0.95
        self.codebooks = {}
        self.get_logger().info(f"Dataset loaded with {len(self.data)} words.")
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        utils = VoiceUtils()
        vectorial = VectorialQuantization()
        
        # 1. GENERATE GLOBAL CODEBOOK
        # We need one shared codebook so "Symbol 1" means the same thing to all HMMs
        all_features = []
        for word in self.data:
            for fs, signal in self.data[word][:10]:
                processed = self.preprocess(signal, fs, utils)
                if processed is not None:
                    all_features.extend(processed['autocorrs'])
        
        # Using your existing LBG to create the 'M' symbols
        # For simplicity, we assume lbg returns a single array of centroids here
        self.global_codebook = vectorial.lbg(np.array(all_features), np.array(all_features), np.array(all_features), self.codebook_size)[0]

        # 2. TRAIN HMMs
        for word in self.data.keys():
            sequences = []
            for fs, signal in self.data[word][:10]:
                processed = self.preprocess(signal, fs, utils)
                if processed is not None:
                    # Convert frames to sequence of indices (O)
                    obs_seq = self.quantize(processed['autocorrs'], vectorial)
                    sequences.append(obs_seq)
            
            # Create a 5-state HMM for this word
            model = HMM(n_states=5, n_symbols=self.codebook_size, word_label=word)
            model.train_viterbi(sequences)
            self.hmms[word] = model
            
        self.test_performance(utils, vectorial)

    def preprocess(self, signal, fs, utils):
        signal = utils.normalize(signal)
        signal = utils.detect_voice(signal, fs)
        if len(signal) < 320: return None
        signal = utils.pre_emphasis(signal, self.alpha)
        frames = utils.framing(signal, fs)
        frames = utils.hamming_window(frames)
        autocorrs = np.array([utils.autocorrelation(f, self.P) for f in frames])
        return {'autocorrs': autocorrs}

    def quantize(self, autocorrs, vectorial):
        # Maps each audio frame to its closest codebook index
        # This uses your existing Itakura-Saito implementation
        cb_lpc = np.array([VoiceUtils().lsf_to_lpc(np.sort(c)) for c in self.global_codebook])
        dist_matrix = vectorial.itakura_saito_batch(autocorrs, cb_lpc)
        return np.argmin(dist_matrix, axis=1)

    def test_performance(self, utils, vectorial):
        predictions, true_labels = [], []
        for word in self.data.keys():
            for fs, signal in self.data[word][10:15]: # Test on unseen data
                processed = self.preprocess(signal, fs, utils)
                if processed is None: continue
                
                obs_seq = self.quantize(processed['autocorrs'], vectorial)
                
                # EVALUATION: Which HMM gives highest log-probability?
                scores = {w: model.forward_score(obs_seq) for w, model in self.hmms.items()}
                pred_word = max(scores, key=scores.get)
                
                predictions.append(pred_word)
                true_labels.append(word)

        cm = confusion_matrix(true_labels, predictions)
        self.get_logger().info(f"Accuracy: {np.trace(cm)/np.sum(cm):.2f}")
                        

    def load_dataset(self, path):
        data, sr = {}, {}
        for word in os.listdir(path):
            word_path = os.path.join(path, word)

            if not os.path.isdir(word_path):
                continue

            signals = []
            fs = []
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
        
    

def main(args=None):
    rclpy.init(args=args)
    #voice_cmd_node = VoiceCmdNode()
    voice_cmd_node = HMMSpotterNode()
    rclpy.spin_once(voice_cmd_node)
    voice_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
