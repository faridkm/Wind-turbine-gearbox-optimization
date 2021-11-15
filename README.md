# Wind-turbine-gearbox-optimization

This repository contains a step-by-step procedure to down-scale drivetrains while keeping their structural integrity and frequency distribution inaltered. To achieve that, multi-stage geared drivetrains are modelled according to ISO-IEC standards.
The code can be used to model spur and helical parallel and planetary gearboxes, while shafts and bearings are represented in a simplified way. The code is developed in MATLAB and can be integrated with third party softwares Simpack and KISSsoft via a COM interface.

This repository contains the MATLAB code optimizes the gear ratios of the gear stages of wind turbine gearbox. It estimates the weight of the gear stages and the overall weight of gearbox based on different configurations of the gear stages. For this purpose, this code uses the gearbox model presented in "Nejad, A. R., Guo, Y., Gao, Z., & Moan, T. (2016). Development of a 5 MW reference gearbox for offshore wind turbines. Wind Energy, 19(6), 1089-1106."
 
Gearbox configurations:
-One stage (parallel) 
-One stage (planetary) 
-Two stages (planetary-parallel)
-Two stages (planetary-planetary) 
-Three stages (planetary-planetary-parallel)   
-Three stages (planetary-planetary-planetary)  
 
 An example of the implementation of this code can be found in "Moghadam, F. K., & Nejad, A. R. (2020). Evaluation of PMSG-based drivetrain technologies for 10-MW floating offshore wind turbines: Pros and cons in a life cycle perspective. Wind Energy, 23(7), 1542-1563."
 
 
This code output including the optimized gear ratios of the gear stages can be applied as input to KISSsoft gear design software to obtain the detailed geometry, weight, cost, and efficiency for the different gearbox configurations.
