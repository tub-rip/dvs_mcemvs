import numpy as np


def error_metrics(data, gt, name, b, f):
    data = data.flatten()
    gt = gt.flatten()

    # inlier ratios
    delta = np.maximum(data / gt, gt / data)
    delta1 = np.sum(delta < 1.25) / np.ma.count(delta)
    delta2 = np.sum(delta < 1.25 ** 2) / np.ma.count(delta)
    delta3 = np.sum(delta < 1.25 ** 3) / np.ma.count(delta)

    # SILog
    di = np.ma.log(gt) - np.ma.log(data)
    n = np.ma.count(di)
    SILog = 1 / n * np.sum(di ** 2) - 1 / (n * n) * np.sum(di) ** 2

    # abs rel diff error
    ARE = 1 / n * np.sum(np.abs(data - gt) / data)

    # log RMSE
    lRMSE = (1 / n * np.sum((np.ma.log(gt) - np.ma.log(data)) ** 2)) ** 0.5

    # bad-p
    err = np.abs(1 / data - 1 / gt) * b * f
    rel_err = err * gt / b / f
    badp = np.sum((err > 5) & (rel_err > 0.05)) / n

    print(name + "--------------------------------")
    print("delta1: ", delta1)
    print("delta2: ", delta2)
    print("delta3: ", delta3)
    print("SILog: ", SILog)
    print("Abs. Rel Error: ", ARE)
    print("log RMSE: ", lRMSE)
    print("bad-p: ", badp)
