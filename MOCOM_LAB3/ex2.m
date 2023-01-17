%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
addpath('include')
model = load("panda.mat"); % don't worry about the warnings
% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;
bTge = zeros(4,4); %initialization of bTge
alpha=(-pi/4)
gain=0.2

% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Initial transformation from <base> to <e-e>
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7') %useful for save initial end-effector orientation w.r.t robot base
% END-EFFECTOR Goal definition 
bOge = [0.6; 0.4; 0.4];
%for a rotation around z
eRge = [cos(-pi/4) , -sin(-pi/4) , 0  ; sin(-pi/4) , cos(-pi/4) , 0  ; 0 , 0 , 1  ];
%we now need to find bRge where bRge=bRe*eRge
bRe=bTe(1:3,1:3);
bRge=bRe*eRge;
%refer to report for the formula 
bTge(1:3,1:3)=bRge; 
bTge(1:3,4) = bOge; 
bTge(4,4) = 1;

% TOOL Goal definition
%same method as in ex2
eTt = [eye(3) [0,0,0.2]'; 0 0 0 1];
%we now need to find bTt where bTt=bTe*eTt
bTt = bTe*eTt;

bOgt = [0.6; 0.4; 0.4];
%for a rotation around z
tRgt = [cos(-pi/4) , -sin(-pi/4) , 0 ; sin(-pi/4) , cos(-pi/4) , 0 ; 0 , 0 , 1 ];
%we now need to find bRgt where bRgt=bRt*tRgt
%we can get bRt from bTt 
bRt=bTt(1:3,1:3); %, thus:
bRgt=bRt*tRgt;
bTgt(1:3,1:3) = bRgt;
bTgt(1:3,4) = bOgt;
bTgt(4,4) = 1;
%uncomment below for Q3.6
bTgt = [ 0.7071, 0, -0.7071, 0.6 ; -0.7071, 0, -0.7071, 0.5 ; 0, 1, 0, 0.5; 0 , 0, 0, 1 ];

% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
error_linear = zeros(3,1);
error_angular = zeros(3,1); 
% Start the inverse kinematic control 
tool = true; % change to true for using the tool 
q = q_init;
for i = t
%%Q2.1


    %% Compute the cartesian error to reach the goal
    if tool == true %compute the error between the tool frame and goal frame
        
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        % ... 
        %retrieve displacement from transformation matrix 
        displacement_bTe= bTe(1:3, 4);
        bRe=bTe(1:3, 1:3);
        displacement_eTt= eTt(1:3, 4);
        %for calculation later in the report, we need bJt wrt base
        %therefore we need the transformation matrix from end effector to
        %tool wrt base 
        mult = bRe*displacement_eTt
        skew_matrix = [ 0 , -mult(3,1), mult(2,1);
               mult(3,1) , 0  ,-mult(1,1);
              -mult(2,1), mult(1,1),0 ]
        R_J = [eye(3) , zeros(3); -skew_matrix, eye(3) ]
        bJt= R_J*bJe
        
        %retrieve displacement from transformation matrix 
        displacement_bTt= bTt(1:3, 4);
        displacement_bTgt= bTgt(1:3, 4);
        %displacement or linear cartesian error 
        cart_error_linear=displacement_bTgt - displacement_bTt; 
        %rotation matrix from <t> to frame <gt> by rotating around z-axes with
        %alpha = -pi/4
        tRgt_z = [cos(alpha), -sin(alpha), 0; sin(alpha) , cos(alpha), 0; 0, 0, 1]
        %get angular cartesian error from ComputerInverseAngleAxis function
        % get rotational matrix for bTt
        bRt= bTt(1:3, 1:3); 
        % get rotational matrix for bTgt
        bRgt= bTgt(1:3, 1:3);  
        %get angular cartesian error
        [theta,v] = ComputeInverseAngleAxis(tRgt_z)
        cart_error_angular= (theta*v*bRt)' %projecting on the base frame 

   

        
    else % compute the error between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        %retrieve respective rotational matrix 
        bRe= bTe(1:3,1:3)
        displacement_eTt= bTe(1:3,4)
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        


        %retrieve displacement from transformation matrix 
        displacement_e= bTe(1:3, 4);
        displacement_ge= bTge(1:3, 4);
        %displacement or linear cartesian error 
        cart_error_linear=displacement_ge - displacement_e; 
        %find angular cartesian error using Euler Angle Representation
        %rotation matrix from <e> to frame <ge> by rotating around z-axes with
        %alpha = -pi/4
        eRge_z = [cos(alpha), -sin(alpha), 0; sin(alpha) , cos(alpha), 0; 0, 0, 1]
        %get angular cartesian error from ComputerInverseAngleAxis function
        % get rotational matrix for bTe
        bRe= bTe(1:3, 1:3); 
        % get rotational matrix for bTg
        bRge= bTge(1:3, 1:3);  
        %get angular cartesian error
        [theta,v] = ComputeInverseAngleAxis(eRge_z)
        cart_error_angular= (theta*v*bRe)' %projecting on the base frame 


    end
    
       
    %% Compute the reference velocities
    ref_ang_vel= (cart_error_angular) * gain
    ref_lin_vel= cart_error_linear * gain
    cart_error= [(cart_error_linear)  ; (cart_error_angular)]
    %where x_dot = reference_vels of the joints 
    x_dot = gain * [cart_error]
    %% Compute desired joint velocities 
    %q_dot = pseudoinverse(J)*x_dot
    %q_dot = pinv(bJe)*x_dot
    %uncomment for question 3 and comment line above
    q_dot = pinv(bJt)*x_dot
    
    %% Simulate the robot - implement the function KinematicSimulation()
 
    q = KinematicSimulation(q(1:7), q_dot,ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    show(model.franka,[q',0,0],'visuals','on');%switch visuals to off for seeing only the frames
    hold on
    if tool == true
        plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
        plot3(bOgt(1),bOgt(2),bOgt(3),'ro','LineWidth',5);
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOge(1),bOge(2),bOge(3),'ro','LineWidth',5);
    end
    drawnow
    if(norm(x_dot) < 0.01)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break

    
    end
end

%%Q3
%Q3.1
%refer to line 47 to find bTgt
%change the bool "tool" to true, then refer to lines from 64 to find the
%angular and linear cartesian errors. 
%Q3.2 refer to Section Compute the reference velocities to find the reference vel
%Q3.3 uncomment lines for question 3 Section Compute desired joint velocities  above 
%Q3.4 refer to section Simulate the robot - implement the function KinematicSimulation()
%Q3.6 refer to line 48


