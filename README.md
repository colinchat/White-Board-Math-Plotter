# White-Board-Math-Plotter
Source code for first year final project. Built to run on a Lego EV3.

## How to use
Simply specify board parameters in board.txt, and the functions you want to plot in demo.txt, then load the program onto the Lego EV3 and follow the instructions on the screen.  
### Example board.txt explained: 
-10  --> lowest x value of plot 
10   --> highest x value of plot 
-4.5 --> lowest y value of plot 
4.5  --> highest y value of plot 
90   --> real white board width in cm 
180  --> real white board length in cm  
### Example demo.txt explained: 
3            --> number of functions to plot 
4 2 0.5 0 0  --> [function type sin] [parameter 1] [parameter 2] ... 
1 2 -3
2 0.5 0 -2  
Four function types are available, linear (1), quadratic (2), cubic (3), and sinusoidal (4).

