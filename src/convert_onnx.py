import torch

dummy_input = torch.randint(0, 255, (1, 1, 224, 224), device="cpu")
model = torch.load("../models/best_model.pth", map_location=torch.device('cpu'))

torch.onnx.export(model, dummy_input, "../models/best_model.onnx")

print(dummy_input.shape)
