import torch
import torch.nn as nn
from torch import Tensor
from collections import OrderedDict


class LSTM(nn.Module):

    def __init__(self, device="cpu"):

        super(LSTM, self).__init__()
        self.device = device

        self.h_c = (torch.zeros(2, 64).to(device=self.device), torch.zeros(2, 64).to(device=self.device)) # h_0, c_0

        self.embedding = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2)
        )
        self.embedding_ff = nn.LazyLinear(64)
        self.lstm = nn.LSTM(64, 64, 2, batch_first=True)

        self.state_mlp = nn.Sequential(
            nn.LazyLinear(64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64)
        )


    def reset(self):

        self.h_c = (torch.zeros(2, 64).to(device=self.device), torch.zeros(2, 64).to(device=self.device))


    def forward(self, x: torch.Tensor, pixels: torch.Tensor, is_init: torch.Tensor) -> torch.Tensor:
        
        if len(x.shape) == 1:
            
            if is_init[0]: self.reset()

            # x: (3) --> (cos(theta), sin(theta), angular_velocity)
            obs = x
            image = pixels

            # embed: (3, 64, 64) + (obs) --> (1, 64)
            embed = self.embedding(image)
            embed = torch.flatten(embed)
            embed = self.embedding_ff(embed)
            embed = torch.tanh(embed)
            embed = embed.unsqueeze(0)

            time_informed_embed, self.h_c = self.lstm(embed, self.h_c)

            state_embed = self.state_mlp(obs)
            state_embed = state_embed.unsqueeze(0)
            time_informed_embed = torch.cat((time_informed_embed, state_embed), dim=1)

            return time_informed_embed.squeeze()
    
        else:
            
            # x: (batch, 3) --> (batch, cos(theta), sin(theta), angular_velocity)
            obs = x
            image = pixels

            # embed: (batch, 3, 64, 64) + (batch, obs) --> (batch, 64)
            embed = self.embedding(image)
            embed = torch.flatten(embed, start_dim=1)
            embed = self.embedding_ff(embed)
            embed = torch.tanh(embed)

            # Split the embed by trajectory
            if is_init.any():
                indices = torch.where(is_init)[0]
                if indices[0] == 0:  # Avoid empty first split
                    indices = indices[1:]
                split_embed = torch.tensor_split(embed, indices.tolist())
            else:
                split_embed = [embed]

            # TODO: Might be able to vectorize this better (cat the split_embed to make a batch dimension?)
            time_informed_embeds = []
            for emb in split_embed:
                time_informed_embed, self.h_c = self.lstm(emb)
                time_informed_embeds.append(time_informed_embed)
            time_informed_embed = torch.cat(time_informed_embeds, dim=0)            

            state_embed = self.state_mlp(obs)
            time_informed_embed = torch.cat((time_informed_embed, state_embed), dim=1)

            return time_informed_embed
    