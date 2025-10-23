import torch, time, numpy as np
from DNN_torch_end2end import  load_nn  

model  = load_nn('./nn_para/end2end/run/max_exo.pt', nn_type='lstm')
model.eval()


dummy_input = torch.randn(1, 4)  

for _ in range(10):
    _ = model(dummy_input)


N = 1000
t0 = time.time()
for _ in range(N):
    _ = model(dummy_input)
torch.cuda.synchronize() if torch.cuda.is_available() else None
t1 = time.time()

avg_time = (t1 - t0) / N
print(f"Average inference time per step: {avg_time*1000:.3f} ms")
print(f"FPS: {1/avg_time:.2f}")
