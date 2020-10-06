## Done as a sub-task of e-Yantra 2019-20 (Theme: Biped Patrol)
1;
pkg load control

## Takes the state vector as input. It draws the simple pulley system in a 2D plot.
function draw_pulley(y)
  pd = 0.4;                   ## Pulley Diameter
  p_y = 0.5;                 ## Pulley position wrt y
  L = 1;                      ## Length of string
  ml = 0.2;                  ## Mass Length
  mb = 0.1;                  ## Mass Breadth
  x1 = y(1);
  x2 = L-y(1);
  hold on;
  clf;
  axis equal;
  rectangle('Position',[0-(pd/2),p_y-(pd/2),pd,pd],'Curvature',1,'FaceColor',[1 0 0]);
  rectangle('Position',[-(pd/2)-(ml/2),p_y-x1-(mb/2),ml,mb],'Curvature',0.1,'FaceColor',[0 0 1]);
  rectangle('Position',[(pd/2)-(ml/2),p_y-x2-(mb/2),ml,mb],'Curvature',0.1,'FaceColor',[0 1 0]);
  line ([0-(pd/2) 0-(pd/2)], [p_y p_y-x1], "linestyle", "-", "color", "k");
  line ([(pd/2) (pd/2)], [p_y p_y-x2], "linestyle", "-", "color", "k");
  line ([0 0], [1 p_y], "linestyle", "-", "color", "k");
  xlim([-1 1])
  ylim([-1 1])
  drawnow
  hold off
endfunction

## Calculates the value of the vector dy according to the equations which govern this system.
function dy = pulley_dynamics(y, m1, m2, g, r, u)
  dy(1,1) = y(2); ##The first equation governing the system 
  dy(2,1) = ( (m1-m2)*g + u/r )/(m1+m2); ##The second equation governing the system
endfunction
 
## This function demonstrates the behavior of simple pulley without any external input
function [t,y] = sim_pulley(m1, m2, g, r, y0)
  tspan = 0:0.1:10;                  ## Initialise time step           
  u = 0;                             ## No Input
  [t,y] = ode45(@(t,y)pulley_dynamics(y, m1, m2, g, r, u),tspan,y0);  
endfunction
          
## Declare the A and B matrices in this function.
function [A,B] = pulley_AB_matrix(m1, m2, g, r)
  A = [0 1;       ##Defining the A matrix for the pair of differential equations
       0 0];
  B = [0;           ##Defing the B matrix for the pair of differential equations
       1/( r*(m1+m2) )
      ];
endfunction
  
## This function demonstrates the behavior of simple pulley with external input using the pole_placement controller
function [t,y] = pole_place_pulley(m1, m2, g, r, y_setpoint, y0)
  tspan = 0:0.1:10;                      ## Initialise time step 
  [A,B]= pulley_AB_matrix(m1, m2, g, r); ##Getting A and B for the equation 
  eigs=[-18;-5];                         ##Defining the eigen values for system such that there is maximum stablization of the system 
  K=place(A,B,eigs);                     ##Getting the K matrix using the place 
  [t,y]= ode45(@(t,y)pulley_dynamics(y, m1, m2, g, r, -K*(y-y_setpoint) ),tspan,y0);  
endfunction
   
## This function demonstrates the behavior of simple pulley with external input using the LQR controller
function [t,y] = lqr_pulley(m1, m2, g, r, y_setpoint, y0)
    
  tspan = 0:0.1:10;                  ## Initialise time step 
  [A,B]= pulley_AB_matrix(m1, m2, g, r);
  ##Defining the Q matrix for the system 
  Q= [50000 0;   ##Penalizing the error in position
      0 500];    ##Penalizing the velcoity of block
                 ##Hence, giving more priority to position than velocity
  
  ##Defining the R matrix for the system 
  R= 1;           ##Not penalizing the input of the system 
  
  ##Using lqr function to get the value of the K matrix
  K = lqr(A,B,Q,R);  
  [t,y] = ode45(@(t,y)pulley_dynamics(y, m1, m2, g, r, -K*(y-y_setpoint) ),tspan,y0);  
endfunction

## Main function
function simple_pulley_main()
  m1 = 7.5;
  m2 = 7.6;
  g = 9.8;
  r = 0.2;
  y0 = [0.5 ; 0];                   ## Initial condtion
  y_setpoint = [0.75; 0];           ## Set Point
  
  ##Defining the point of string attachement(Mid point of the block)
  ml=0.2; ##The length of the block (taken from draw_pulley function)
  mb=0.1; ##The breadth of the block (taken from draw_pulley function)
  
  L=1; ##Length of the string (Value taken written in draw_pulley function)
  
  ##[t,y] = sim_pulley(m1, m2, g, r, y0);
  ##[t,y] = pole_place_pulley(m1, m2, g, r, y_setpoint, y0)
  [t,y] = lqr_pulley(m1, m2, g, r, y_setpoint, y0)

  touchlength=sqrt(r*r - (r-ml/2)^2 ) + mb/2; ##The length from center of pulley when the block comes in conatct.
  ##The above value of the touch length was calculated using basic mathematics. 
  
  for k = 1:length(t)
   if(y(k,1)>=touchlength && y(k,1)<=L-touchlength) ##Putting a condition that pulley stops as soon as either blocks come in conatct with the pulley. 
        draw_pulley(y(k, :));
   else 
         break;
   endif
  endfor
endfunction
