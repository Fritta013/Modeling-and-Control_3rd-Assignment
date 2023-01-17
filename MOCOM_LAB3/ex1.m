%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix
clc;
clear;
close all;
addpath('include');

% The same model of assignment 2
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = 0; %0 because all joints are R, boolean that specifies two possible link types: Rotational, Prismatic.
linkType_complex = zeros(numberOfLinks,1); % specify two possible link type: 0 for Rotational, 1 for Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base
iTj_q = zeros(4,4,numberOfLinks); % Trasformation matrix
J= zeros(6,numberOfLinks); %initialiaze Jacobian matrix 

%%Q1.1
% Initial joint configuration 1, q1
q = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];
% Compute direct geometry
iTj_q = GetDirectGeometry(q, linkType, numberOfLinks);

% Compute the transformation w.r.t. the base
biTei= zeros(4,4,numberOfLinks);  %vector of matrices containing the transformation matrices from link i to link i +1 for the current q
biTei=iTj_q; %because iTj_q contains all the transformation matrices from joint 0 to joint 7 with a set of q
%GetTransformationWrtBase
for i =1:numberOfLinks
   bTi= GetTransformationWrtBase(biTei, numberOfLinks);
end 
% computing end effector jacobian 
%important to note that all joints in this robot are revolute, refer to
%report to see the full formula equation to compute the respective Jacobian
J=GetJacobian(bTi,numberOfLinks)

%%Q1.2
% Initial joint configuration 2, q2
q = [1.3, 0.4, 0.1, 0, 0.5, 1.1, 0];
% Compute direct geometry
iTj_q = GetDirectGeometry(q, linkType, numberOfLinks);
% Compute the transformation w.r.t. the base
biTei= zeros(4,4,numberOfLinks);  
biTei=iTj_q; 
%GetTransformationWrtBase
for i =1:numberOfLinks
   bTi= GetTransformationWrtBase(biTei, numberOfLinks);
end 
% computing end effector jacobian 
J=GetJacobian(bTi,numberOfLinks)

%%Q1.3
% Initial joint configuration 3, q3
q = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3];
% Compute direct geometry
iTj_q = GetDirectGeometry(q, linkType, numberOfLinks);
% Compute the transformation w.r.t. the base
biTei= zeros(4,4,numberOfLinks);  
biTei=iTj_q; 
%GetTransformationWrtBase
for i =1:numberOfLinks
   bTi= GetTransformationWrtBase(biTei, numberOfLinks);
end 
% computing end effector jacobian 
J=GetJacobian(bTi,numberOfLinks)

%%Q1.4
% Initial joint configuration 4, q4
q = [2, 2, 2, 2, 2, 2, 2];
% Compute direct geometry
iTj_q = GetDirectGeometry(q, linkType, numberOfLinks);
% Compute the transformation w.r.t. the base
biTei= zeros(4,4,numberOfLinks);  
biTei=iTj_q; 
%GetTransformationWrtBase
for i =1:numberOfLinks
   bTi= GetTransformationWrtBase(biTei, numberOfLinks);
end 
% computing end effector jacobian 
J=GetJacobian(bTi,numberOfLinks)

