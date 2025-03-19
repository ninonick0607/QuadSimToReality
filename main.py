from RL import RL_Algorithm
import torch
import yaml

NAME = "lstm_no_aux_0"
TRAIN = True

if __name__ == "__main__":
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    alg = RL_Algorithm(config={}, log_dir=f"runs/{NAME}", device=device)
    alg.create_env()
    alg.create_modules()
    alg.create_training_utils()

    if TRAIN:
        alg.train()
        print("Training complete")
        alg.save(f"runs/{NAME}/alg.pt")
        print(f"Model saved to runs/{NAME}/alg.pt")

    else:
        alg.load(f"runs/{NAME}/alg.pt")
        alg.test()
