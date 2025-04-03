from RL import RL_Algorithm
import torch
import yaml

NAME = "Yaw_only_MLP_6"
TRAIN = False
LOG = False

if __name__ == "__main__":
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    config = {
        # obs_norm should contain the parameters for the distribution of the observations: normalized = (original - loc) / scale
        "obs_norm": {
            "loc": [0] * 6,
            "scale": [250] * 3 + [12000] + [1] * 2
        }
    }

    log_dir = f"runs/{NAME}" if LOG else None
    alg = RL_Algorithm(config=config, log_dir=log_dir, device=device)
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
