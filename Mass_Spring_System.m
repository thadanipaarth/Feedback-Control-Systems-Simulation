## Done as a sub-task of e-Yantra 2019-20 (Theme: Biped Patrol)
1;
pkg load control

## Takes the state vector as input. It draws the mass spring system in a 2D plot.
function draw_mass_spring(y)    
  l = 0.3; ## Length of rectangle
  b = 0.2; ## Breadth of rectangle
  x_pos = y(1);
  y_pos = 0;
  hold on;
  clf;
  axis equal;
  rectangle('Position',[x_pos-l/2,y_pos,l,b],'Curvature',0,'FaceColor',[0 0 1]);
  line ([0 0], [0 0.6], "linestyle", "-", "color", "k");
  line ([-0.5 x_pos], [(y_pos+b)/2 (y_pos+b)/2], "linestyle", "--", "color", "k");
  text(-0.05, 0.65, "Eqbm Pt")
  xlim([-0.5 1])
  ylim([0 1])
  drawnow
  hold off;
endfunction

## Calculates the value of the vector dy according to the equations which govern this system.
function dy = mass_spring_dynamics(y, m, k, u)
  
  dy(1,1) = y(2); ##The first differential equation equation governing the system 
  dy(2,1) = ( u - k*y(1) )/m; ##The Second differential equation governing the system 
endfunction
     
## This function demonstrates the behavior of mass spring system without any external input
function [t,y] = sim_mass_spring(m, k, y0)
  tspan = 0:0.1:10;                ## Initialize time step
  u = 0;                           ## No input
  [t,y] = ode45(@(t,y)mass_spring_dynamics(y, m, k, u),tspan,y0);
endfunction

## Declare the A and B matrices in this function.
function [A,B] = mass_spring_AB_matrix(m, k)
 
##Defining the A matrix for the System
  A = [ 0 1;       
     (-k/m) 0];
     
##Defining the B matrix for the System      
  B = [0; 
      1/m];
endfunction
       
## This function demonstrates the behavior of mass spring system with external input using the pole_placement controller
function [t,y] = pole_place_mass_spring(m, k, y_setpoint, y0)
  [A,B] =  mass_spring_AB_matrix(m, k);  ## Initialize A and B matrix 
  
##Taking the eigen values which give the best results with minimum overshoot by try and error
  eigs = [-33; -12];   ## Initialise desired eigenvalues
  
  K = place(A, B, eigs);                 ## Calculate K matrix for desired eigenvalues
  
  tspan = 0:0.1:10;                      ## Initialise time step 
  [t,y] = ode45(@(t,y)mass_spring_dynamics(y, m, k, -K*(y-y_setpoint)),tspan,y0);
endfunction
         
## This function demonstrates the behavior of mass spring system with external input using the LQR controller
function [t,y] = lqr_mass_spring(m, k, y_setpoint, y0)
  [A,B] = mass_spring_AB_matrix(m, k);  ## Initialize A and B matrix 
  
##Initializing Q and R matrix for LQR
  Q = [20000 0;           ##Penalizing the error in position 
       0    980];         ##Penalizing the Velocity to lesser extent. 
                          ##Hence, Giving more importance to exactness in position
                          
  R = 1;                  ##Defing the R matrix (1X1) and NOT peanlizing the input.
  
## Calculate K matrix for desired eigenvalues
  K = lqr(A,B,Q,R);
  
  tspan = 0:0.1:10;                   ## Initialise time step 
  [t,y] = ode45(@(t,y)mass_spring_dynamics(y, m, k, -K*(y-y_setpoint) ),tspan,y0);
endfunction

## Main function
function mass_spring_main()
  m = 0.2;
  k = 0.8;
  y0 = [-0.3; 0];
  y_setpoint = [0.7; 0];
  
 ##[t,y] = sim_mass_spring(m,k, y0);      ## Test mass spring system with no input
 ##[t,y] = pole_place_mass_spring(m, k, y_setpoint, y0); ## Test system with Pole Placement controller
 [t,y] = lqr_mass_spring(m, k, y_setpoint, y0);  ## Test system with LQR controller
  for k = 1:length(t)
    draw_mass_spring(y(k, :));  
  endfor
endfunction
