## Done as a sub-task of e-Yantra 2019-20 (Theme: Biped Patrol)
1;
pkg load control

## Takes the state vector as input. It draws the inverted cart pendulum in a 2D plot.
function draw_cart_pendulum(y,m, M, L)
  x = y(1);
  theta = y(3);
  W = 1*sqrt(M/5);    # cart width
  H = 0.5*sqrt(M/5);  # cart height 
  wr = 0.2;           # wheel radius
  mr = 0.3*sqrt(m);    # mass radius 
  y = wr/2 + H/2;
  w1x = x - 0.9*W/2;
  w1y = 0;
  w2x = x + 0.9*W/2 - wr;
  w2y = 0;
  px = x + L*sin(theta);
  py = y - L*cos(theta);
  hold on;
  clf;
  axis equal;
  rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1])
  rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
  rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
  line ([-10 10], [0 0], "linestyle", "-", "color", "k");
  line ([x px], [y py], "linestyle", "-", "color", "k");
  rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.1 0.1 1])
  xlim([-6 6]);
  ylim([-2 3]);
  set(gcf, 'Position', [200 300 1000 400]);
  drawnow
  hold off
endfunction

## Calculates the value of the vector dy according to the equations which govern this system.
function dy = cart_pendulum_dynamics(y, m, M, L, g,  u)
  dy(1,1) = y(2); ##The first differential equation governing the system 
  dy(2,1) = ( u + m*sin(y(3)) * (L*(y(4))^2 + g*cos(y(3))) ) / (M + m*sin(y(3))*sin(y(3))); ##The second differential equation governing the system 
  dy(3,1) =  y(4); ##The Third differential equation governing the system
  dy(4,1) = ( u*cos(y(3)) + (M + m) * g * sin(y(3)) + (y(4)^2) *m*L*cos(y(3))*sin(y(3)) ) / (m*L*cos(y(3)) - (M+m)*L); ##the fourth differential equation governing the system 
endfunction
       
## This function demonstrates the behavior of cart pendulum system without any external input.
function [t,y] = sim_cart_pendulum(m, M, L, g, y0)
  tspan = 0:0.1:10;                  ## Initialise time step           
  u = 0;                             ## No Input
  [t,y] = ode45(@(t,y)cart_pendulum_dynamics(y, m, M, L, g, u),tspan,y0);## Solving the differential equation    
endfunction

## Declare the A and B matrices in this function.
function [A, B] = cart_pendulum_AB_matrix(m , M, L, g)
  ##Defining the A matrix for the system 
  A=[0 1 0 0;
    0 0 m*g/M 0;
    0 0 0 1;
    0 0 (m+M)*g/(m*L + L*(M+m)) 0];
    
  ##Defining the B matrix for the system
  B = [0; 
      1/M; 
      0; 
      1/(m*L + L*(M+m))];
      
endfunction
  
## This function demonstrates the behavior of inverted cart pendulum with external input using the pole_placement controller
function [t,y] = pole_place_cart_pendulum(m, M, L, g, y_setpoint, y0)
  [A,B]= cart_pendulum_AB_matrix(m, M, L, g); ##Getting the values of A and B matrix
  tspan = 0:0.1:10;
  ##Defining the eigen values (four in number) for the cart pendulum system
  eigs=[-8;   
        -0.75;
        -9.85;
        -0.55];
  K=place(A, B, eigs); ##Getting the values of K matrix to set eigs as the eigen values
  [t,y] = ode45(@(t,y)cart_pendulum_dynamics(y, m, M, L, g, -K*(y-y_setpoint)),tspan,y0);
endfunction
          
## This function demonstrates the behavior of inverted cart pendulum with external input using the LQR controller
function [t,y] = lqr_cart_pendulum(m, M, L, g, y_setpoint, y0)
  [A,B]= cart_pendulum_AB_matrix(m, M, L, g); ##Getting the values of A and B matrix
  tspan = 0:0.1:10;
 ##Setting the value of Q matrix 
  Q=[ 15000 0 0 0; 
      0 10000 0 0; 
      0 0 1000 0;  
      0 0 0 2];    
 ##Setting the value of R matrix
  R= 1;
  K=lqr(A, B, Q, R); ##Getting the values of K
  [t,y]= ode45(@(t,y)cart_pendulum_dynamics(y, m, M, L, g, -K*(y-y_setpoint)),tspan,y0);
endfunction

## Main function
function cart_pendulum_main()
  m = 1;
  M = 5;
  L = 2;
  g = 9.8;
  y0 = [-0.4; 0; pi + 0.8  ; 0];
  y_setpoint = [0; 0; pi; 0];
  
  ##[t,y] = sim_cart_pendulum(m, M, L, g, y0);
  ##[t,y] = pole_place_cart_pendulum(m, M, L, g, y_setpoint, y0);
  [t,y] = lqr_cart_pendulum(m, M, L, g, y_setpoint, y0);
  
  for k = 1:length(t)
    draw_cart_pendulum(y(k, :), m, M, L);  
  endfor
endfunction

