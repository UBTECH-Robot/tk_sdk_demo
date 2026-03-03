import torch, time

# x = torch.randn(4096, 4096, device="cpu")
# y = torch.randn(4096, 4096, device="cpu")
x = torch.randn(4096, 4096, device="cuda")
y = torch.randn(4096, 4096, device="cuda")

t0 = time.time()

for idx in range(100):
    t1 = time.time()
    torch.cuda.synchronize()
    z = x @ y
    torch.cuda.synchronize()
    print(f"Iteration {idx + 1}: GPU matrix multiply time:", time.time() - t1)

print("GPU matrix multiply time:", time.time() - t0)