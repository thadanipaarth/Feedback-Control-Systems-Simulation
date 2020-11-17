# Feedback-Control-System-Simulation

Implemented pole-placement and linear quadratic regulator on the following systems:
- Cart pendulum
- Complex pulley
- Mass spring system 
- Simple pendulum 
- Simple pulley 

In all of the above systems output of pole placement, LQR and behaviour of system without an external input was comapred. 

### Pakages used (GNU Octave)
- control

### Steps To Run
Execute the following commands in the command prompt in the directory conating the respective file:
1. `octave`
2. `Cart_Pendulum`
3. `cart_pendulum_main()`

Now, you can clearly view the simulation. Also, make sure that octave has been added in the envirnoment variables and you are in the same directory as the file

> The feedback mechanisms can be changed in the main function, there are three options in each syatem:
> - No external input 
> - LQR (Liner quadratic regulator)
> - Pole placement
