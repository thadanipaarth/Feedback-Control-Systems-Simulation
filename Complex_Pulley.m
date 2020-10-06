## Done as a sub-task of e-Yantra 2019-20 (Theme: Biped Patrol)
1;
pkg load control

## Takes the state vector as input. It draws the complex pulley system in a 2D plot.
function draw_complex_pulley(y)
  ml = 0.2;
  mb = 0.1;
  L_A = 1.3;
  L_B = 1.1;
  pd_A = 0.4;
  py_A = 1;
  pd_B = 0.3; 
  x1 = y(1);
  x2 = L_A - y(1);
  y1 = y(3);
  y2 = L_B - y(3);
  pulley_A_pos = {0, py_A};
  pulley_B_pos = {(-pd_A/2), py_A-x2};
  m1_pos = {(pd_A/2), py_A-x1};
  m2_pos = {((-pd_A+pd_B)/2), py_A-x2-y1};
  m3_pos = {((-pd_A-pd_B)/2), py_A-x2-y2};
  x1_string = {(pd_A/2), py_A, (pd_A/2), (py_A-x1)};
  x2_string = {(-pd_A/2), py_A, (-pd_A/2), (py_A-x2)};
  y1_string = {((-pd_A+pd_B)/2), py_A-x2, ((-pd_A+pd_B)/2), py_A-x2-y1};
  y2_string = {((-pd_A-pd_B)/2), py_A-x2, ((-pd_A-pd_B)/2), py_A-x2-y2};
  hold on;
  clf;
  axis equal;
  rectangle('Position',[pulley_A_pos{1}-(pd_A/2),pulley_A_pos{2}-(pd_A/2),pd_A, pd_A],'Curvature',1,'FaceColor',[1 0 0]);## Pulley A
  rectangle('Position',[pulley_B_pos{1}-(pd_B/2),pulley_B_pos{2}-(pd_B/2),pd_B, pd_B],'Curvature',1,'FaceColor',[1 0 0]);## Pulley B
  rectangle('Position',[m1_pos{1}-(ml/2),m1_pos{2}-(mb/2),ml, mb],'Curvature',0.1,'FaceColor',[0 0 1]);## m1 mass
  rectangle('Position',[m2_pos{1}-(ml/2),m2_pos{2}-(mb/2),ml, mb],'Curvature',0.1,'FaceColor',[0 1 0]);## m2 mass
  rectangle('Position',[m3_pos{1}-(ml/2),m3_pos{2}-(mb/2),ml, mb],'Curvature',0.1,'FaceColor',[0 1 1]);## m1 mass
  line ([x1_string{1} x1_string{3}], [x1_string{2} x1_string{4}], "linestyle", "-", "color", "k");
  line ([x2_string{1} x2_string{3}], [x2_string{2} x2_string{4}], "linestyle", "-", "color", "k");
  line ([y2_string{1} y2_string{3}], [y2_string{2} y2_string{4}], "linestyle", "-", "color", "k");
  line ([y1_string{1} y1_string{3}], [y1_string{2} y1_string{4}], "linestyle", "-", "color", "k");
  xlim([-1.5 1.5]);
  ylim([-1.5 1.5]);
  drawnow
  hold off
endfunction

## Calculates the value of the vector dy according to the equations which govern this system.
function dy = complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB, u)
  dy(1,1) = y(2); ##The first equation governing the system 
  dy(2,1) = (((u(1)*(m2+m3))/rA) - ((u(2)*(m3 - m2))/rB) - ((((-m1+m2+m3)*(m2+m3)) - ((m3-m2)*(m3-m2))) * g)) / (((m1+m2+m3)*(m2+m3)) - ((m3-m2)*(m3-m2)));##The second equation governing the system 
  dy(3,1) = y(4);##The third equation governing the system 
  dy(4,1) = ((m1 + m2 + m3)*(u(2)/rB) - (m3-m2)*(u(1)/rA) - 2*m1*(m3 - m2)*g ) / ((m2+m3)*(m1+m2+m3) - (m3-m2)*(m3-m2) );##The fourth equation governing the system
endfunction

## This function demonstrates the behavior of complex pulley without any external input
function [t,y] = sim_complex_pulley(m1, m2, m3, g, rA, rB, y0)
  tspan = 0:0.1:10;                  ## Initialise time step           
  u = [0; 0];                        ## No Input
  [t,y] = ode45(@(t,y)complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB, u),tspan,y0); 
  endfunction
       
## Declare the A and B matrices in this function.
function [A,B] = complex_pulley_AB_matrix(m1, m2, m3, g, rA, rB)
##Defining the A matrix for the system, A matrix would be 4X4 as there are 4 four state variables and four equations 
  A = [0 1 0 0;  
       0 0 0 0;
       0 0 0 1;
       0 0 0 0];
##Defining the B matrix for the system, B matrix would be 4X2 as there are 2 input variables and four equations 
##The below are the values, for example derivativef1_ta implies derivative of f1 wrt ta
  derivativef1_ta=0;
  derivativef1_tb=0;
  derivativef2_ta= (m2 + m3) / (rA * ((m1 + m2 + m3)*(m2 + m3)) - (m3 - m2)^2);
  derivativef2_tb= (m2 - m3)/( rB * ( (m1 + m2 +m3)*(m2 + m3) - (m3 - m2)^2 ));
  derivativef3_ta=0;
  derivativef3_tb=0;
  derivativef4_ta= (m2 - m3) / ( rA * ((m1 + m2 + m3)*(m2 + m3)) - (m3 - m2) * (m3 - m2));
  derivativef4_tb= (m1 + m2 + m3) / (rB * ( (m1 + m2 + m3)*(m2 + m3) - (m3 - m2)^2 ) );
  
  B = [derivativef1_ta derivativef1_tb;
       derivativef2_ta derivativef2_tb;
       derivativef3_ta derivativef3_tb;
       derivativef4_ta derivativef4_tb
      ];  
endfunction

## This function demonstrates the behavior of complex pulley with external input using the pole_placement controller
function [t,y] = pole_place_complex_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0)
  
  [A, B] = complex_pulley_AB_matrix(m1, m2, m3, g, rA, rB); ##Fetching the values of A and B
  tspan = 0:0.1:10;     ## Initialise time step 
  eigs=[-2; -2 ; -2; -2]; ##Defining the values of eigen values 
  K=place(A, B, eigs);##Getting the values of K matrix 
  [t,y] = ode45(@(t,y)complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB, -K*(y - y_setpoint)),tspan,y0);
endfunction
        
## This function demonstrates the behavior of complex pulley with external input using the LQR controller
function [t,y] = lqr_complex_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0)
  tspan = 0:0.1:10;                  ## Initialise time step 
  [A,B]= complex_pulley_AB_matrix(m1, m2, m3, g, rA, rB);##Fetching the A,B Matrix 
  Q=[5000 0 0 0; ## Defning the Q matrix ##Penalizing the error in x
     0 1 0 0;    
     0 0 5000 0; ##Penalizing the error in y 
     0 0 0 1];
  R=[1 0;        ##Defning the R matrix  
     0 1];  
  K=lqr(A, B, Q, R); ##Getting the value of K  
 [t,y] = ode45(@(t,y)complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB, -K*(y - y_setpoint)),tspan,y0);;
endfunction

## Main function
function complex_pulley_main()
  m1 = 23.90;
  m2 = 11.95;
  m3 = 12;
  g = 9.8;
  rA = 0.2;
  rB = 0.2;
  y_setpoint = [0.6; 0; 0.8; 0];
  y0 = [0.4; 0; 0.5; 0];
 
  ml = 0.2; ##Length of Block
  mb = 0.1; ##Breadth of Block 
  
  ##[t,y] = sim_complex_pulley(m1, m2, m3, g, rA, rB, y0)
  ##[t,y] = pole_place_complex_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0)
  [t,y] = lqr_complex_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0);

  touchlength_upper_pulley = sqrt(rA*rA - (rA-ml/2)^2 ) + mb/2; ##Touchlength for upper pulley
  touchlength_lower_pulley = sqrt(rB*rB - (rB-ml/2)^2 ) + mb/2; ##Touchlength for lower pulley
  for k = 1:length(t)
   if(y(k,1)>=touchlength_upper_pulley && y(k, 3) >= touchlength_lower_pulley)
       draw_complex_pulley(y(k, :));
    else 
       break;
    endif   
  endfor
  
endfunction
