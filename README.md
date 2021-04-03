# Shooting toolbox for dynamical systems

This repo presents shooting toolboxes for autonomous and non-autonomous dynamical systems.

It supports dynamical systems in both first-order and second-order forms. In particular, 
* For first-order systems, numerical integration solvers avaliable in MATLAB can be used for performing forward simulation. 
* For second-order systems, Generalized alpha and Newmark integration schemes are used for performing forward simulation.

Automatic construction of adjoints for first-order systems is supported as well. Please refer to [1-2] for more details about the automatic generation of adjoints.

This toolbox can be used to solve two-point boundary-value problems with arbitrary boundary conditions. For example, it can be used to perform the continuation of periodic orbits.

This toolbox is developed in the platform COCO (continuation core). For more details about COCO, please refer to [3-4]. 

## References
[1] Li, M., & Dankowicz, H. (2018). Staged construction of adjoints for constrained optimization of integro-differential boundary-value problems. SIAM Journal on Applied Dynamical Systems, 17(2), 1117-1151.

[2] Li, M., & Dankowicz, H. (2020). Optimization with equality and inequality constraints using parameter continuation. Applied Mathematics and Computation, 375, 125058.

[3] https://sourceforge.net/projects/cocotools/

[4] Dankowicz, H., & Schilder, F. (2013). Recipes for continuation. Society for Industrial and Applied Mathematics.
