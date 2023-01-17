function [iTj_q] = GetDirectGeometry(q, linkType, numberOfLinks)
%%% GetDirectGeometryFunction

% Inputs: 
% q : links current position ; 
% iTj : vector of matrices containing the transformation matrices from link
% i to link j
% linkType: vector of size numberOfLinks identiying the joint type, 0 for revolute, 1 for
% prismatic.

% Outputs :
% iTj_q vector of matrices containing the transformation matrices from link i to link j for the input q. 
% The size of iTj is equal to (4,4,numberOfLinks)

%here we are only considerting the rotation between the frames 
for i = 1:numberOfLinks
    iTj_q(:,:,i) = DirectGeometry(q(i), linkType); 
end
%iTj_q obtained here should be of size 4*4*7


%here we are adding the lenghts of the links 
%1st link/joint
iTj_q(3,4,1) = 175 ; 

%2nd link/joint
iTj_q(3,4,2) = 98; 

%3rd link/joint
iTj_q(1,4,3) = -105; 

%4th link/joint
iTj_q(2,4,4) = -145.5; 
iTj_q(3,4,4) = 326.5; 

%5th link/joint
iTj_q(1,4,5) = 35 ; 

%6th link/joint
iTj_q(3,4,6) = 385 ;

%7th link/joint
iTj_q(1,4,7) = -153;


end