import numpy as np, pandas as pd
from pysr import PySRRegressor
import pysr, sys, pathlib, pprint
print(pysr.__file__)            # → D:\Anaconda\Lib\site-packages\pysr\__init__.py
pprint.pp(sys.path[:5])         # 看看哪个路径排在最前

# 伪造一点数据
X = np.random.randn(100, 2)
y = X[:, 0]**2 + np.sin(X[:, 1])

sr = PySRRegressor(
    niterations=50,
    loss="L2DistLoss()",
    random_state=0,
)
sr.fit(X, y)
import inspect
print(inspect.signature(sr.predict))
print(inspect.signature(sr.predict))
print(inspect.signature(sr.predict))

# ① 默认（最优方程）
print(sr.predict(X[:5]))
print(inspect.signature(sr.predict))

# ② 用 equation= 指定“字符串形式”方程
eq_str = str(sr.equations_.iloc[3]["equation"])
print(sr.predict(X[:5], equation=eq_str))
print(inspect.signature(sr.predict))

# ③ 用 index= 指定行号
print(sr.predict(X[:5], index=3))
print(inspect.signature(sr.predict))
