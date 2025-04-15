from RL import RL_Algorithm
import torch
import yaml
import os


def train(cfg={'name': 'test', 'log': False}, device="cpu"):
    try:
        log_dir = f"runs/{cfg['name']}" if cfg["log"] else None
        alg = RL_Algorithm(config=cfg, log_dir=log_dir, device=device)
        alg.create_env()
        alg.create_modules()
        alg.create_training_utils()
        alg.train()
        print("Training complete")
        alg.save(f"runs/{cfg['name']}/alg.pt")
        print(f"Model saved to runs/{cfg['name']}/alg.pt")
    finally:
        alg.env.close()
        print("Environment closed")

def test(cfg={'name': 'test', 'log': False}, device="cpu"):
    try:
        alg = RL_Algorithm(config=cfg, device=device)
        alg.create_env()
        alg.create_modules()
        alg.load(f"runs/{cfg['name']}/alg.pt")
        alg.test()
    finally:
        alg.env.close()

if __name__ == "__main__":
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    def reward_fn(info):
        reward = 0
        reward -= info['distance_to_goal'] / 2200
        return reward
    
    def reward_fn_angle_penalty(info):
        reward = 0
        reward -= info['distance_to_goal'] / 2200
        reward -= info['local_angle'] / 1800
        return reward

    configs = ({
        # obs_norm should contain the parameters for the distribution of the observations: normalized = (original - loc) / scale
        'name': "LSTM_with_obstacle_4",
        'train': False,
        'log': False,
        "obs_norm": {
            "loc": [0] * 6,
            "scale": [600] * 3 + [2200] + [1] * 2
        },
        'reward_fn': reward_fn_angle_penalty
    }, {
        # obs_norm should contain the parameters for the distribution of the observations: normalized = (original - loc) / scale
        'name': "LSTM_with_obstacle_5",
        'train': False,
        'log': False,
        "obs_norm": {
            "loc": [0] * 6,
            "scale": [600] * 3 + [2200] + [1] * 2
        },
        'reward_fn': reward_fn
    }, {
        # obs_norm should contain the parameters for the distribution of the observations: normalized = (original - loc) / scale
        'name': "LSTM_with_obstacle_6",
        'train': False,
        'log': False,
        "obs_norm": {
            "loc": [0] * 6,
            "scale": [600] * 3 + [2200] + [1] * 2
        },
        'reward_fn': reward_fn_angle_penalty
    }, {
        # obs_norm should contain the parameters for the distribution of the observations: normalized = (original - loc) / scale
        'name': "LSTM_with_obstacle_7",
        'train': False,
        'log': False,
        "obs_norm": {
            "loc": [0] * 6,
            "scale": [600] * 3 + [2200] + [1] * 2
        },
        'reward_fn': reward_fn
    },)

    for config in configs:

        if config.get('train', False):
            # Train the model
            print(f"\nTraining configuration: {config['name']}")
            try:
                train(cfg=config, device=device)
            except KeyboardInterrupt:
                print(f"Training interrupted for configuration {config['name']}. Continuing to next configuration.")
                continue
            except Exception as e:
                print(f"Error during training for configuration {config['name']}: {e}")
                if config['log']:
                    os.makedirs(f"runs/{config['name']}", exist_ok=True)
                    with open(f"runs/{config['name']}/error_log.txt", "w") as f:
                        f.write(str(e))
                continue
        else:
            try:
                # Test the model
                print(f"Testing configuration: {config['name']}")
                test(cfg=config, device=device)
            except KeyboardInterrupt:
                print(f"Testing interrupted for configuration {config['name']}. Continuing to next configuration.")
                continue
            except Exception as e:
                print(f"Error during testing for configuration {config['name']}: {e}")
                if config['log']:
                    os.makedirs(f"runs/{config['name']}", exist_ok=True)
                    with open(f"runs/{config['name']}/error_log.txt", "w") as f:
                        f.write(str(e))
                continue
