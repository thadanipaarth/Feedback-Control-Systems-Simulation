# Feedback-Control-System-Simulation

Implemented pole-placement and linear quadratic regulator on the following systems using GNU Octave:
- Cart pendulum
- Complex pulley
- Mass spring system 
- Simple pendulum 
- Simple pulley  

In all of the above systems, the output of pole placement, LQR and system behaviour without an external input was compared.  

### GNU Octave Packages Used:
- control

### Steps To Run
Navigate to the directory containing the above files and execute the following commands in the command prompt:
1. `octave`
2. `Cart_Pendulum`
3. `cart_pendulum_main()`

Now, you can view the simulation. Also, make sure that octave is present in the environment variables.

> The feedback mechanisms can be changed in the main function present in each file. There are three options:
> - No external input 
> - LQR (Linear quadratic regulator)
> - Pole placement
