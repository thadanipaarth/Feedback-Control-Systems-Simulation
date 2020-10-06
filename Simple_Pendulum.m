## Done as a sub-task of e-Yantra 2019-20 (Theme: Biped Patrol)
1;
pkg load control

## Takes the state vector and length of pendulum as input. It draws the pendulum in a 2D plot.
function draw_pendulum(y, L)
  theta = y(1);          ## Store the first variable of state vector in theta
  x = L*sin(theta);      ## x-coordinate of pendulum bob
  y = 1-L*cos(theta);  ## y coordinate of pendulum bob
  d = 0.1;               ## diameter of pendulum bob
  hold on;
  clf;
  axis equal;
  rectangle('Position',[x-(d/2),y-(d/2),d,d],'Curvature',1,'FaceColor',[1 0 0]);
  line ([0 x], [1 y], "linestyle", "-", "color", "k");
  xlim([-1 1])
  ylim([-0.5 2])
  drawnow
  hold off
endfunction

## Calculates the value of the vector dy according to the equations which govern this system.
function dy = pendulum_dynamics(y, m, L, g, u)
  sin_theta = sin(y(1));
  cos_theta = cos(y(1));
  dy(1,1) = y(2); ##The first differential equation governing the system                                  
  dy(2,1) = (( -g*sin_theta )/L) + u/(m*(L^2)); ##The Second differential equation that govern the system  
endfunction

## This function demonstrates the behavior of simple pendulum without any external input
function [t,y] = sim_pendulum(m, g, L, y0)
  tspan = 0:0.1:10;                  ## Initialise time step           
  u = 0;                             ## No Input
  [t,y] = ode45(@(t,y)pendulum_dynamics(y, m, L, g, u),tspan,y0); ## Solving the differential equation  
endfunction
       
## Declare the A and B matrices in this function.
function [A,B] = pendulum_AB_matrix(m, g, L)
  A = [0 1; g/L 0];
  B = [0; 1/(m*(L^2))];   ##Defining the B matrix getting that from space state analysis 
endfunction

## This function demonstrates the behavior of simple pendulum with external input using the pole_placement controller
function [t,y] = pole_place_pendulum(m, g, L, y_setpoint, y0)
  [A,B] = pendulum_AB_matrix(m, g, L); ## Initialize A and B matrix
  
  ##Adjusting the values of the eigen values in the pole placement to get the best result and minimum overshoot
  eigs = [-4; -2]; ## Initialise desired eigenvalues
  
  ##Getting the value of feedback matrix using place function
  K = place(A,B,eigs); ## Calculate K matrix for desired eigenvalues
  tspan = 0:0.1:10;                  ## Initialise time step 
  [t,y] = ode45(@(t,y)pendulum_dynamics(y, m, L, g, -K*(y-y_setpoint)),tspan,y0);
endfunction
       
## This function demonstrates the behavior of simple pendulum with external input using the LQR controller.
function [t,y] = lqr_pendulum(m, g, L, y_setpoint, y0)
  [A,B] = pendulum_AB_matrix(m, g, L); ## Initialize A and B matrix

## Initialise Q matrix
  Q = [4 0;  ##Penalising The error in location as the matrix from of Q is [theta N; N_Transpose theta_dot]  
       0 2]; ##For simplicity N and N_transpose=0 and penalising the angular velcoity half of position
             ##Hence, Giving more importance to position accuracy as compared to angular velocity         

## Initialise R
  R = 1;     ##Not penalising the input            

## Getting the value of K using the lqr function.
  K = lqr(A,B,Q,R);                   ## Calculate K matrix from A,B,Q,R matrices
  
  tspan = 0:0.1:10;                  ## Initialise time step 
  [t,y] = ode45(@(t,y)pendulum_dynamics(y, m, L, g, -K*(y-y_setpoint)),tspan,y0);
endfunction

## Main function
function simple_pendulum_main()
  m = 1;             
  g = 9.8;
  L = 0.5;
  y_setpoint = [pi; 0];                ## Set Point 
  y0 = [pi/6 ; 0];                   ## Initial condtion
  
  [t,y] = sim_pendulum(m,g,L, y0);        ## Test Simple Pendulum
  ##[t,y] = pole_place_pendulum(m,g,L, y_setpoint, y0) ## Test Simple Pendulum with Pole Placement Controller
  ##[t,y] = lqr_pendulum(m,g,L, y_setpoint, y0);        ## Test Simple Pendulum with LQR Controller

  for k = 1:length(t)
    draw_pendulum(y(k, :), L);  
  endfor
endfunction