# Water-Exit CTR held-out report

Transmedia resistance coefficient $C_{TR}$ as a function of normalized depth $z/\ell$ and velocity $v$.

Models: Simple surrogate (poly d4) / LEAP w/o residual (backbone) / RBF interpolation / Full LEAP ($f_{phys}+\epsilon_{SR}$).

Residual: original hall-of-fame symbolic form, constants re-fit on each train split (leakage-free).

Headline figure uses the representative slice $v=0.2$ m/s.

![CTR extrapolation](fig_headline_extrap_CTR.png)


## extrapolation split  (n_train=183, n_test=46)

Re-fit $\epsilon_{SR}$ (complexity 23): `0.00900071145787805*sin(x0*cos((-3.45824702856637 + tanh(x1)/x0)/tanh(x1))/x1)/(x0**2.80973759883619 + x1)`

| Model | RMSE | Max Err | t_curve (ms) |
|---|---|---|---|
| Simple surrogate (poly d4) | 0.3774 | 0.8038 | 0.076 |
| LEAP w/o residual | 0.0310 | 0.0780 | 0.050 |
| RBF interpolation | 0.0326 | 0.0899 | 2.373 |
| Full LEAP (proposed) | 0.0313 | 0.0836 | 0.085 |

Cold start — RBF: 7.8 KB, 183 centers, 122.1 ms build (O(N)/query); LEAP: 578 B equation (O(1)/query).

![fitting extrapolation](extrapolation/fig_fitting_curve_CTR.png)

![errors extrapolation](extrapolation/fig_error_bars_CTR.png)


## random split  (n_train=183, n_test=46)

Re-fit $\epsilon_{SR}$ (complexity 24): `0.00729145997405165*sin(x0*cos((-3.46216455737765 + tanh(x1)/tanh(x0))/tanh(x1))/x1)/(x0**3.10777030414155 + x1)`

| Model | RMSE | Max Err | t_curve (ms) |
|---|---|---|---|
| Simple surrogate (poly d4) | 0.0355 | 0.1156 | 0.075 |
| LEAP w/o residual | 0.0303 | 0.1150 | 0.051 |
| RBF interpolation | 0.0072 | 0.0265 | 1.664 |
| Full LEAP (proposed) | 0.0203 | 0.0504 | 0.088 |

Cold start — RBF: 7.8 KB, 183 centers, 3.1 ms build (O(N)/query); LEAP: 584 B equation (O(1)/query).

![fitting random](random/fig_fitting_curve_CTR.png)

![errors random](random/fig_error_bars_CTR.png)
