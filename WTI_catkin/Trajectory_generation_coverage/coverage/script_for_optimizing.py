import time
import numpy as np


def timeit(func, args, enum):
    tt = 0
    for i in range(enum):
        t = time.perf_counter()
        d = func(*args)
        tt += time.perf_counter() - t
    print(f"{tt / enum}")


def numpy_print_test(length):
    a = np.zeros(length)
    i_start = int(length / 3)
    a[i_start:i_start * 2] = 1.
    al = a.tolist()
    print("numpy ", end="")
    timeit(np.array2string, [a], 10000)
    print("list ", end="")
    timeit(str, [al], 10000)


if __name__ == '__main__':
    numpy_print_test(10000)
