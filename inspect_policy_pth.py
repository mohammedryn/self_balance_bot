import torch

# Load the policy.pth file
state_dict = torch.load("ppo_contents/policy.pth", map_location=torch.device('cpu'))

# Print the keys (layer names and weights)
print("\n=== Keys in policy.pth (layers) ===")
for key in state_dict.keys():
    print(key)

# Optionally print size of a few layers
print("\n=== Layer Shapes ===")
for key in list(state_dict.keys())[:5]:  # print first 5
    print(f"{key}: {state_dict[key].shape}")
