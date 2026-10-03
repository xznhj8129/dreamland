from threadpoolctl import threadpool_limits

# numpy's OpenBLAS starts a thread per core for every matrix product. NeuralFoil's are small, so the threads mostly
# spin: an AeroBuildup grid took 37 s of CPU for 6.5 s wall with 8 threads, 5.9 s for 5.2 s with one, and the VLM solve
# is no faster with more. Parallel work here is explicit (the VLM view's worker threads).
threadpool_limits(limits=1, user_api="blas")
