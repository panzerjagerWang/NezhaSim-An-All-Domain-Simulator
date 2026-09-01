# Water-Entry CTR held-out report

Transmedia resistance coefficient $C_{TR}$ as a function of normalized depth $z/\ell$ and velocity $v$.

Models: Simple surrogate (poly d4) / LEAP w/o residual (backbone) / RBF interpolation / Full LEAP ($f_{phys}+\epsilon_{SR}$).

Residual: original hall-of-fame symbolic form, constants re-fit on each train split (leakage-free).

Headline figure uses the representative slice $v=0.3$ m/s.

![CTR extrapolation](fig_headline_extrap_CTR.png)


## extrapolation split  (n_train=4680, n_test=1170)

Re-fit $\epsilon_{SR}$ (complexity 22): `0.16297453017591*x0*sin(17.7939216049737*sin(x0**sin(sin(sin(x1)))))/(3262.39029253744*x0**6.13135677965743 + x1)`

| Model | RMSE | Max Err | t_curve (ms) |
|---|---|---|---|
| Simple surrogate (poly d4) | 0.2674 | 0.7144 | 0.076 |
| LEAP w/o residual | 0.0146 | 0.0331 | 0.051 |
| RBF interpolation | 0.0205 | 0.0505 | 44.268 |
| Full LEAP (proposed) | 0.0146 | 0.0331 | 0.091 |

Cold start — RBF: 183.5 KB, 4680 centers, 811.8 ms build (O(N)/query); LEAP: 584 B equation (O(1)/query).

![fitting extrapolation](extrapolation/fig_fitting_curve_CTR.png)

![errors extrapolation](extrapolation/fig_error_bars_CTR.png)


## random split  (n_train=4680, n_test=1170)

Re-fit $\epsilon_{SR}$ (complexity 22): `0.172135728454092*x0*sin(17.7636531449316*sin(x0**sin(sin(sin(x1)))))/(2089.31492324279*x0**5.78036106531722 + x1)`

| Model | RMSE | Max Err | t_curve (ms) |
|---|---|---|---|
| Simple surrogate (poly d4) | 0.0372 | 0.1432 | 0.076 |
| LEAP w/o residual | 0.0298 | 0.1013 | 0.049 |
| RBF interpolation | 0.0003 | 0.0099 | 45.065 |
| Full LEAP (proposed) | 0.0188 | 0.0602 | 0.091 |

Cold start — RBF: 183.5 KB, 4680 centers, 515.3 ms build (O(N)/query); LEAP: 584 B equation (O(1)/query).

![fitting random](random/fig_fitting_curve_CTR.png)

![errors random](random/fig_error_bars_CTR.png)
