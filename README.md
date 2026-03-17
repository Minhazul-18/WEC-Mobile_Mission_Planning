-- Code Base for Hierarchical Mission Planning of a WEC-Mobile Based on a Wave Forecast --

Before running any codes, download the forecast data from the MFWAM Model provided by Copernicus for the desired region.
After extraction of data into either .mat format or .csv format, run "MissionPlanningGA_Deterministic_PS_Hawaii_HardConstraint.m" to run the hierarchical mission planner.
Tweaking and playing around with the parameters will be required for the planner to work.

Data visualization can be done by running the "Mission_Simulation_Code_Git.m" and then loading and running the saved file using "MissionAnimationWithParameters_Git.m".
