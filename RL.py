# PyTorch imports
import torch
import torch.nn as nn
from torch.utils.tensorboard import SummaryWriter
# TorchRL imports
from torchrl.collectors import SyncDataCollector
from torchrl.envs import TransformedEnv, GymLikeEnv
from torchrl.envs.transforms import ToTensorImage, Resize, Transform, InitTracker, DoubleToFloat, ObservationNorm
from torchrl.objectives import ClipPPOLoss
from torchrl.objectives.value import GAE
from torchrl.modules import ProbabilisticActor, TanhNormal, ValueOperator, SafeProbabilisticTensorDictSequential, SafeSequential
# TorchRL tensordict imports
import tensordict
from tensordict.nn import TensorDictModule
from tensordict.nn.distributions import NormalParamExtractor
from tensordict.nn.probabilistic import set_composite_lp_aggregate
set_composite_lp_aggregate(True).set()
# Pygame and rendering imports
import matplotlib.pyplot as plt
plt.switch_backend("TkAgg")
import pygame
import cv2
# General imports
import numpy as np
import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
import yaml
from collections import defaultdict, OrderedDict
# File imports
from LSTM import LSTM
from GTrXL import GTrXL
from quadsimenv import QuadSimEnv


class RenderPixels(Transform):
    def __init__(self, display, **kwargs):
        super(RenderPixels, self).__init__(**kwargs)
        self.display = display
    def _apply_transform(self, pixels):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        self.display.fill((0, 0, 0))
        image = pixels.squeeze().cpu().numpy()
        image = cv2.resize(image, (256, 256))
        image = image * 255
        image = image.transpose(1, 0, 2)
        self.display.blit(pygame.surfarray.make_surface(image), (0, 0))
        pygame.display.update()
        return pixels



class RL_Algorithm:

    def __init__(self, config:dict, log_dir=None, device="cpu"):
        """
        Initializes the RL algorithm with the given config and log_dir.

        Args:
            config (dict): !!! Not implemnented yet !!! Configuration dictionary for the RL algorithm
            log_dir (str, optional): Directory for logging. Defaults to False.
            device (str, optional): Device to use for training. Defaults to "cpu".
        """

        self.device = device
        self.config = config
        if log_dir is not None: self.writer = SummaryWriter(log_dir=log_dir)

    def create_env(self):
        """
        Creates and returns the environment for the RL algorithm.

        This function sets up the environment for the RL algorithm, including any necessary transformations and preprocessing steps.
        It returns a reference to the environment object, but there is no need to store it, as it is already stored in the class instance.

        You could parametrize the environment creation (or implement the config functionality in the __init__) to support different 
        environments and configurations, but for now, it is hardcoded to use the Pendulum-v1 environment.
        """

        # TODO: Add support for other envs, and use config
        env = GymLikeEnv(QuadSimEnv())
        env = TransformedEnv(env, DoubleToFloat())
        env = TransformedEnv(env, InitTracker())
        # self.obs_norm = ObservationNorm(in_keys=["observation"], loc=0, scale=(1, 1, 8), standard_normal=True)
        # env = TransformedEnv(env, self.obs_norm)
        # self.obs_norm.init_stats(1000)
        # env.append_transform(ToTensorImage(True, in_keys=["pixels"], out_keys=["pixels_transformed"]))
        # env.append_transform(Resize(64, 64, in_keys=["pixels_transformed"]))
        self.env = env
        return self.env
    
    def create_modules(self):
        """
        Creates the modules for the RL algorithm, including the actor, critic, and feature extractor.
        Returns the model as a TensorDictModule, which is a sequential model that can be used for training and inference.

        This function sets up the neural network architecture for the RL algorithm. It returns a TensorDictModule that combines 
        the feature extractor, actor, and critic modules. There is no need to store the model, as it is already stored in the class instance.

        You could parametrize the module creation (or implement the config functionality in the __init__) to support different
        architectures and configurations, but for now, it is hardcoded to use the GTrXL architecture for the feature extractor.
        """

        # TODO: Add support for aux heads, and use config
        features_extractor_arch = LSTM(device=self.device)
        critic_arch = nn.Sequential(
            nn.Linear(64, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 1),
        )
        actor_arch = nn.Sequential(
            nn.Linear(64, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 2),
            NormalParamExtractor()
        )
        critic = ValueOperator(critic_arch, in_keys=["features"]).to(self.device)
        actor = TensorDictModule(actor_arch, in_keys=["features"], out_keys=["loc", "scale"])
        actor = ProbabilisticActor(
            actor, 
            spec=self.env.action_spec,
            in_keys=["loc", "scale"],
            distribution_class=TanhNormal,
            distribution_kwargs={
                "low": self.env.action_spec.space.low,
                "high": self.env.action_spec.space.high
            },
            return_log_prob=True
        ).to(self.device)
        features_extractor = TensorDictModule(features_extractor_arch, in_keys=["observation", "pixels_transformed", "is_init"], 
                                              out_keys=["features", "aux_pred", "aux_target"]).to(self.device)
        self.modules = OrderedDict([
            ("features_extractor", features_extractor),
            ("actor", actor),
            ("critic", critic)
        ])
        self.model = self.sequential_select([*self.modules])
        self.model(self.env.reset())
        self.params = self.model.parameters()

        return self.model
    
    def create_training_utils(self):
        """
        Creates the training utilities for the RL algorithm, including the data collector, loss module, and optimizer.
        """

        self.collector = SyncDataCollector(
            self.env,
            self.sequential_select(["features_extractor", "actor"]),
            frames_per_batch=1024,
            reset_at_each_iter=True,
            device=self.device
        )
        # We use a blank value network because we calculate the value ourselves.
        # The GAE module expects a value network, but automatically vmaps it, which
        # only works for some networks. We use a blank one to avoid errors.
        self.advantage_module = GAE(
            gamma=0.99,
            lmbda=0.95,
            value_network=TensorDictModule(lambda x: x, in_keys=["state_value"], out_keys=["state_value"]),
            average_gae=True,
            skip_existing=True
        )
        # We only use the actor and critic heads. The features extractor is called 
        # manually later to avoid repeated calls, and so that we can access the features 
        # outside of just this module (for auxialiary losses, debugging, etc.)
        self.loss_module = ClipPPOLoss(
            actor_network=self.modules["actor"],
            critic_network=self.modules["critic"],
            critic_coef=0.01
        )
        self.optim = torch.optim.Adam(self.params, lr=3e-4)
        self.loss_fn = torch.nn.SmoothL1Loss()
    
    def train(self):
        """
        Trains the model using the collected data from the environment.

        This function performs the training loop for the RL algorithm. It collects data from the environment, computes the loss,
        and updates the model parameters using backpropagation. It also logs the training progress and saves checkpoints.

        You must call `create_env`, `create_modules`, and `create_training_utils` before calling this function.

        You could parametrize the training loop (or implement the config functionality in the __init__) to support different algorithms, or
        to change the training parameters, but for now, it is hardcoded to use the PPO algorithm for 512 episodes, with 10 epochs each, and minibatches 64.
        Note: To parametrize the training loop, you would need to change the `create_training_utils` function as well (eg. episode length, learning rate, etc.)
        """

        critic_op = self.sequential_select(["features_extractor", "critic"])
        
        for i, data in enumerate(self.collector):

            logs = defaultdict(list)
            
            with torch.no_grad():
                rewards: torch.Tensor = data["next", "reward"]
                dones = data["next", "done"]
                split_indices = torch.where(dones)[0]
                split_sizes = torch.diff(torch.cat([torch.tensor([-1]).to(split_indices.device), split_indices, torch.tensor([len(rewards)-1]).to(split_indices.device)]))
                split_tensors = torch.split(rewards, split_sizes.tolist())
                episode_rewards = torch.tensor([t.sum().item() for t in split_tensors])
                logs["mean_episode_reward"] = episode_rewards.mean().item()

            for epoch in range(10):
                with torch.no_grad():
                    data: tensordict.TensorDict = critic_op(data)
                    latest_value = critic_op(data["next"][-1])["state_value"].unsqueeze(0)
                    next_values = torch.cat((data["state_value"][1:], latest_value))
                    data.set(("next", "state_value"), next_values)

                self.advantage_module(data)
                explained_var = self.explained_variance(data["value_target"], data["state_value"])
                logs["explained_variance"].append(explained_var.item())

                data_view = data.reshape(-1)
                minibatches = data_view.split(64)
                indices = torch.randperm(len(minibatches))
                minibatches = [minibatches[index] for index in indices]
                
                for k in range(16):

                    self.modules["features_extractor"].module.reset()
                    batch = minibatches[k]
                    batch = batch.to(self.device)

                    batch = self.modules["features_extractor"](batch)
                    loss = self.loss_module(batch)
                    angle_loss = self.loss_fn(batch["aux_pred"][-1, :2], batch["aux_target"][-1, :2])
                    angular_velocity_loss = self.loss_fn(batch["aux_pred"][-1, 2], batch["aux_target"][-1, 2])
                    total_loss = loss["loss_objective"] + loss["loss_critic"] + loss["loss_entropy"] + angle_loss + angular_velocity_loss
                    logs["loss"].append(total_loss.item())
                    logs["angle_loss"].append(angle_loss.item())
                    logs["angular_velocity_loss"].append(angular_velocity_loss.item())
                    logs["critic"].append(loss["loss_critic"].item())
                    logs["objective"].append(loss["loss_objective"].item())
                    logs["entropy"].append(loss["loss_entropy"].item())


                    self.optim.zero_grad()
                    total_loss.backward()
                    nn.utils.clip_grad_norm_(self.params, .5)
                    self.optim.step()


            episode_loss = np.mean(logs["loss"])
            episode_angle_loss = np.mean(logs["angle_loss"])
            episode_angular_velocity_loss = np.mean(logs["angular_velocity_loss"])
            explained_var = np.mean(logs["explained_variance"])
            print(f"Episode {i}, Loss: {episode_loss:.4f}, Angle Loss: {episode_angle_loss:.4f}, Angular Velocity Loss: {episode_angular_velocity_loss:.4f}, Critic Loss: {np.mean(logs['critic']):.4f}, Explained Variance: {explained_var:.4f}")
            self.writer.add_scalar("losses/total_loss", episode_loss, i)
            self.writer.add_scalar("losses/angle", episode_angle_loss, i)
            self.writer.add_scalar("losses/angular_velocity", episode_angular_velocity_loss, i)
            self.writer.add_scalar("losses/critic", np.mean(logs["critic"]), i)
            self.writer.add_scalar("losses/objective", np.mean(logs["objective"]), i)
            self.writer.add_scalar("losses/entropy", np.mean(logs["entropy"]), i)
            self.writer.add_scalar("training/explained_variance", explained_var, i)
            self.writer.add_scalar("training/episode_reward", logs["mean_episode_reward"], i)
            self.writer.add_scalar("misc/memory_usage", torch.cuda.memory_allocated() / 1e9, i)
            self.writer.flush()

            if i % 25 == 0:
                self.save(f"{self.writer.log_dir}/checkpoint_{i}.pt")

            if i == 511:
                return


    def test(self):
        """
        Evaluate the model in a rendered environment

        Must call `create_env` and `create_modules` before calling this function.
        This function runs the trained model in the environment for 200 steps, and renders the environment using Pygame.
        """


        pygame.init()
        display = pygame.display.set_mode((256, 256))

        env = self.env.append_transform(RenderPixels(display=display, in_keys=["pixels"], out_keys=["pixels"]))
        actor_op = self.sequential_select(["features_extractor", "actor"])
        actor_op.eval()
        i=0
        while True:
            env.rollout(200, actor_op)
            print(f"reset {i}")
            i+=1



    def save(self, path):
        """
        Saves the model and observation normalization state to the specified path.
        """

        state_dict = self.model.state_dict()
        obs_norm_state_dict = self.obs_norm.state_dict()
        torch.save({"model": state_dict, "obs_norm": obs_norm_state_dict}, path)

    def load(self, path):
        """
        Loads the model and observation normalization state from the specified path.
        """

        state_dict = torch.load(path)
        self.model.load_state_dict(state_dict["model"])
        self.obs_norm.load_state_dict(state_dict["obs_norm"])

    def sequential_select(self, keys: tuple):
        """
        Returns a TensorDictModule that combines the specified modules in the order given by keys.

        Useful when you want to use a subset of the modules in the class, or when you want to change the order of the modules.
        (eg. to use the features extractor and actor only for inference, or to use the features extractor and critic only for advantage estimation)
        """

        # `ProbabilisticActor` is a special case
        if "actor" in keys:
            seq = SafeProbabilisticTensorDictSequential(
                *[self.modules[k] for k in keys if k != "actor"], *self.modules["actor"].module
            )
        else:
            seq = SafeSequential(*[self.modules[k] for k in keys])
        return seq
    
    @staticmethod
    def compute_gae(last_values, rewards, masks, values, gamma=0.99, lam=0.95):
        """
        Computes the Generalized Advantage Estimation (GAE) for the given rewards, masks, and values.

        Args:
            last_values (torch.Tensor): The last value of the state (In TorchRL terms, it's the td[('next', 'state_value')][-1], or the very most recent value).
            rewards (torch.Tensor): The rewards for each step.
            masks (torch.Tensor): The masks for each step.
            values (torch.Tensor): The values for each step.
            gamma (float, optional): Discount factor. Defaults to 0.99.
            lam (float, optional): Lambda for GAE. Defaults to 0.95.
        """

        advantages = torch.zeros_like(rewards)
        prev_gae = 0
        for t in reversed(range(len(rewards))):
            delta = rewards[t] + gamma * masks[t] * last_values - values[t]  
            prev_gae = delta + gamma * lam * masks[t] * prev_gae 
            advantages[t] = prev_gae
            last_values = values[t]
        returns = advantages + values
        return advantages, returns

    @staticmethod
    def explained_variance(returns, values):
        """
        Computes the explained variance between the returns and values.

        Args:
            returns (torch.Tensor): The returns (TorchRL calls them "value_targets", same same) for each step.
            values (torch.Tensor): The values for each step.
        """

        var_returns = torch.var(returns)
        return 1 - torch.var(returns - values) / var_returns if var_returns > 0 else torch.tensor(0)
