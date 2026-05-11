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
from hmm_utils import HMMUtils as hmm

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
        self.hmms = {}
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

    def preprocess(self, signal, fs, utils):
        signal = utils.normalize(signal)
        signal = utils.detect_voice(signal, fs)
        if len(signal) < 320:
            return None
        signal = utils.pre_emphasis(signal, self.alpha)
        frames = utils.framing(signal, fs)
        if len(frames) == 0:
            return None
        frames = utils.hamming_window(frames)
        autocorrs = np.array([utils.autocorrelation(frame, self.P) for frame in frames])
        return {'autocorrs': autocorrs}

    def build_global_codebook(self, utils, vectorial, size):
        all_P = []
        all_Q = []
        all_autocorr = []
        for word in self.data.keys():
            num_train_samples = min(10, len(self.data[word]))
            for i in range(num_train_samples):
                fs, signal = self.data[word][i]
                processed = self.preprocess(signal, fs, utils)
                if processed is None:
                    continue
                frames = utils.framing(utils.pre_emphasis(utils.detect_voice(signal, fs), self.alpha), fs)
                if len(frames) == 0:
                    continue
                lpc, _ = utils.extract_lpc(frames, self.P)
                lsf = np.array([utils.lpc_to_lsf(l) for l in lpc])
                all_P.extend(lsf[:, :self.P//2])
                all_Q.extend(lsf[:, self.P//2:])
                all_autocorr.extend(processed['autocorrs'])

        if len(all_P) == 0:
            return None, None, None

        cent_P, cent_Q = vectorial.lbg(
            np.array(all_P),
            np.array(all_Q),
            np.array(all_autocorr),
            size)
        centroids_lpc = np.array([
            utils.lsf_to_lpc(np.concatenate([cent_P[k], cent_Q[k]]))
            for k in range(len(cent_P))])
        return cent_P, cent_Q, centroids_lpc

    def quantize(self, autocorrs, centroids_lpc, vectorial):
        dist_matrix = vectorial.itakura_saito_batch(autocorrs, centroids_lpc)
        return np.argmin(dist_matrix, axis=1)

    def evaluate_hmms(self, utils, vectorial, size):
        predictions = []
        true_labels = []
        confidences = []
        per_word_stats = {}

        for word in self.data.keys():
            word_scores = []
            word_confs = []
            for fs, signal in self.data[word][10:15]:
                processed = self.preprocess(signal, fs, utils)
                if processed is None:
                    continue
                obs_seq = self.quantize(processed['autocorrs'], self.global_lpc_centroids, vectorial)
                if len(obs_seq) == 0:
                    continue

                scores = {w: model.forward(obs_seq) for w, model in self.hmms.items()}
                sorted_scores = sorted(scores.items(), key=lambda x: x[1], reverse=True)
                best_word, best_score = sorted_scores[0]
                second_score = sorted_scores[1][1] if len(sorted_scores) > 1 else 0.0
                confidence = best_score - second_score

                predictions.append(best_word)
                true_labels.append(word)
                confidences.append(confidence)
                word_scores.append(best_score)
                word_confs.append(confidence)

            if word_scores:
                per_word_stats[word] = {
                    'avg_score': float(np.mean(word_scores)),
                    'avg_confidence': float(np.mean(word_confs)),
                    'samples': len(word_scores)
                }

        if len(predictions) == 0 or len(true_labels) == 0:
            self.get_logger().warning('No evaluation data available for confidence testing.')
            return

        cm = confusion_matrix(true_labels, predictions, labels=list(self.data.keys()))
        accuracy = np.trace(cm) / np.sum(cm)
        avg_confidence = float(np.mean(confidences)) if confidences else 0.0

        self.get_logger().info(f"Evaluation for codebook size {size}")
        self.get_logger().info(f"Overall accuracy: {accuracy:.2f}")
        self.get_logger().info(f"Average confidence gap: {avg_confidence:.4f}")
        self.get_logger().info(f"Confusion matrix:\n{cm}")

        for word, stats in per_word_stats.items():
            self.get_logger().info(
                f"Word '{word}': avg score={stats['avg_score']:.4f}, "
                f"avg confidence={stats['avg_confidence']:.4f}, "
                f"samples={stats['samples']}")

    def train(self, utils, vectorial):
        labels = list(self.data.keys())
        for size in self.codebook_sizes:
            self.get_logger().info(f"\nProbando con codebook de tamaño {size}")
            cent_P, cent_Q, global_lpc_centroids = self.build_global_codebook(utils, vectorial, size)
            if cent_P is None:
                self.get_logger().warning('No training features found; skipping this size.')
                continue

            self.codebooks = {'global': (cent_P, cent_Q)}
            self.global_lpc_centroids = global_lpc_centroids
            hmms = {}

            for word in labels:
                sequences = []
                for fs, signal in self.data[word][:10]:
                    processed = self.preprocess(signal, fs, utils)
                    if processed is None:
                        continue
                    obs_seq = self.quantize(processed['autocorrs'], global_lpc_centroids, vectorial)
                    if len(obs_seq) > 0:
                        sequences.append(obs_seq)

                if not sequences:
                    self.get_logger().warning(f"Skipping word '{word}' because no valid training sequences were generated.")
                    continue

                word_hmm = hmm(n_states=5, n_symbols=size)
                word_hmm.train(sequences)
                hmms[word] = word_hmm

                self.get_logger().info(f"Matrices for word '{word}' (codebook size {size}):")
                self.get_logger().info(f"A:\n{word_hmm.A}")
                self.get_logger().info(f"B:\n{word_hmm.B}")
                self.get_logger().info(f"pi:\n{word_hmm.pi}")

            self.hmms = hmms
            self.evaluate_hmms(utils, vectorial, size)


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
    

def main(args=None):
    rclpy.init(args=args)
    #voice_cmd_node = VoiceCmdNode()
    voice_cmd_node = HMMTrainingNode()
    rclpy.spin_once(voice_cmd_node) 
    voice_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
