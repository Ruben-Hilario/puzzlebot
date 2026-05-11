import numpy as np

class HMMUtils:
    def __init__(self, n_states, n_symbols):
        self.N = n_states  # Number of states
        self.M = n_symbols # Size of codebook (observation symbols)
        
        # A: State transition probabilities (initialize as left-right model)
        self.A = np.zeros((n_states, n_states))
        for i in range(n_states):
            if i < n_states - 1:
                self.A[i, i] = 0.5
                self.A[i, i+1] = 0.5
            else:
                self.A[i, i] = 1.0
        
        # B: Observation symbol probabilities (Emission matrix)
        self.B = np.full((n_states, n_symbols), 1.0 / n_symbols)
        
        # Pi: Initial state distribution (always start in state 1)
        self.pi = np.zeros(n_states)
        self.pi[0] = 1.0

    # def forward(self, obs_seq):
    #     """Problem 1: Calculate P(O|lambda) using Forward Algorithm"""
    #     T = len(obs_seq)
    #     if T == 0: return -np.inf
    #     #log for preventing underflow
    #     alpha = np.zeros((T, self.N))
        
    #     # Initialization
    #     alpha[0, :] = self.pi * self.B[:, obs_seq[0]]
        
    #     # Induction
    #     for t in range(T - 1):
    #         for j in range(self.N):
    #             alpha[t+1, j] = (alpha[t, :] @ self.A[:, j]) * self.B[j, obs_seq[t+1]]
                    
    #     # Termination
    #     # return np.sum(alpha[T-1, :])
    #     # if it fails:
    #     return np.log(np.sum(alpha[T-1, :1])+1e-100)
        
    def forward(self, obs_seq):
        """Problem 1: Calculate Log P(O|lambda) with scaling to prevent underflow"""
        T = len(obs_seq)
        if T == 0: return -np.inf
        
        alpha = np.zeros((T, self.N))
        scale = np.zeros(T)
        
        # Initialization with scaling
        alpha[0, :] = self.pi * self.B[:, obs_seq[0]]
        scale[0] = np.sum(alpha[0, :]) + 1e-300
        alpha[0, :] /= scale[0]
        
        # Induction with scaling
        for t in range(T - 1):
            for j in range(self.N):
                alpha[t+1, j] = (alpha[t, :] @ self.A[:, j]) * self.B[j, obs_seq[t+1]]
            scale[t+1] = np.sum(alpha[t+1, :]) + 1e-300
            alpha[t+1, :] /= scale[t+1]
        
        # The log-probability is the sum of the logs of the scaling factors
        return np.sum(np.log(scale))

    def train(self, sequences, max_iter=20):
        """Problem 3: Adjust parameters via Baum-Welch (simplified for tutorial)"""
        # In a real implementation, you would use the Forward-Backward 
        # variables to re-estimate A and B. For brevity, many researchers 
        # use Viterbi training to update counts.
        for iteration in range(max_iter):
            new_B = np.ones((self.N, self.M)) * 1e-6
            for seq in sequences:
                #find most likely state (Decode problem)
                states = self.viterbi(seq)
                for t, s in enumerate(states):
                    new_B[s, seq[t]] +=1
                #Normalize
                self.B = new_B / new_B.sum(axis=1)[:, None]
                

    def viterbi(self, obs_seq):
        """Problem 2: Find the most likely state sequence"""
        T = len(obs_seq)
        if T == 0:
            return np.array([], dtype=int)
        delta = np.zeros((T, self.N))
        phi = np.zeros((T, self.N), dtype=int)
        
        delta[0, :] = self.pi * self.B[:, obs_seq[0]]
        
        for t in range(1, T):
            for j in range(self.N):
                vals = delta[t-1, :] * self.A[:, j]
                phi[t, j] = np.argmax(vals)
                delta[t, j] = np.max(vals) * self.B[j, obs_seq[t]]
        
        states = np.zeros(T, dtype=int)
        states[T-1] = np.argmax(delta[T-1, :])
        for t in range(T-2, -1, -1):
            states[t] = phi[t+1, states[t+1]]
        return states
        
    def _viterbi(self, obs_seq):
            """Alternative for viterbi path solver"""
            T = len(obs_seq)
            delta = np.zeros((T, self.N))
            psi = np.zeros((T, self.N), dtype=int)
            
            delta[0, :] = self.pi * self.B[:, obs_seq[0]]
            for t in range(1, T):
                for j in range(self.N):
                    val = delta[t-1, :] * self.A[:, j]
                    delta[t, j] = np.max(val) * self.B[j, obs_seq[t]]
                    psi[t, j] = np.argmax(val)
            
            states = [np.argmax(delta[T-1, :])]
            for t in range(T-1, 0, -1):
                states.insert(0, psi[t, states[0]])
            return states
