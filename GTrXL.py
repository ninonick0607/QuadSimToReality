import torch
import torch.nn as nn
from torch import Tensor
from collections import OrderedDict
from ding.torch_utils.network import GTrXL as gtrxl

class GTrXL(nn.Module):

    def __init__(self, device="cpu"):

        super(GTrXL, self).__init__()
        self.device = device

        self.embedding = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
            nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2)
        )
        self.embedding_ff = nn.LazyLinear(64)
        self.gtrxl = gtrxl(input_dim=64, head_dim=32, embedding_dim=64, head_num=2, layer_num=1, use_embedding_layer=False)

        self.angle_head = nn.Sequential(
            nn.Linear(64, 32),
            nn.Tanh(),
            nn.Linear(32, 2)
        )
        self.angular_velocity_head = nn.Sequential(
            nn.Linear(64, 32),
            nn.Tanh(),
            nn.Linear(32, 1)
        )


    def reset(self):

        self.gtrxl.reset_memory(batch_size=1)
        


    def forward(self, x: torch.Tensor, pixels: torch.Tensor, is_init: torch.Tensor) -> torch.Tensor:
        
        if len(x.shape) == 1:
            
            if is_init[0]: self.reset()

            # x: (3) --> (cos(theta), sin(theta), angular_velocity)
            obs = pixels
            target = x

            # embed: (3, 64, 64) --> (1, 64)
            embed = self.embedding(obs)
            embed = torch.flatten(embed)
            embed = self.embedding_ff(embed)
            embed = torch.tanh(embed)
            embed = embed.unsqueeze(0).unsqueeze(0)  # Add batch and sequence dimensions

            time_informed_embed = self.gtrxl(embed.detach(), batch_first=True, return_mem=False)["logit"]
            
            angle = self.angle_head(embed.squeeze())
            angular_velocity = self.angular_velocity_head(time_informed_embed.squeeze())

            output = torch.cat((angle, angular_velocity), dim=-1)

            return time_informed_embed.squeeze(), output, target
    
        else:
            
            # x: (batch, 3) --> (batch, cos(theta), sin(theta), angular_velocity)
            obs = pixels
            target = x

            # embed: (batch, 3, 64, 64) --> (batch, 64)
            embed = self.embedding(obs)
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
                self.reset()
                time_informed_embed = self.gtrxl(emb.detach().unsqueeze(0), batch_first=True, return_mem=False)["logit"]
                time_informed_embeds.append(time_informed_embed.squeeze())
            time_informed_embed = torch.cat(time_informed_embeds, dim=0)
            
            if torch.isnan(time_informed_embed).any():
                print("Warning: NaN values found in time_informed_embed. Replacing NaNs with 0.0.")
            time_informed_embed = torch.nan_to_num(time_informed_embed, nan=0.0)

            angle = self.angle_head(embed)
            angular_velocity = self.angular_velocity_head(time_informed_embed)

            output = torch.cat((angle, angular_velocity), dim=-1)

            return time_informed_embed, output, target
    