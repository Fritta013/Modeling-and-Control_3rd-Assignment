function [T] = BuildTree()
% This function should build the tree of frames for the chosen manipulator.
% Inputs: 'None'
% Outputs: The tree of frames.

% iTj is a 3-dimensional matlab matrix, suitable for defining tree of
% frames. iTj should represent the transformation matrix between the i-th and j-th
% frames. iTj(row,col,link_idx)
%... for all the links of the manipulator%

%the following matrices are obtained from analyzing the CAD
%important to note, z-axis is always coinciding with the joint rotation axis

%x axis         %zaxis          %y axis         %translation
%0T1
T(1,1,1) = 1; T(1,2,1) = 0; T(1,3,1) = 0; T(1,4,1) = 0;
T(2,1,1) = 0; T(2,2,1) = 1; T(2,3,1) = 0; T(2,4,1) = 0;
T(3,1,1) = 0; T(3,2,1) = 0; T(3,3,1) = 1; T(3,4,1) = 175;
T(4,1,1) = 0; T(4,2,1) = 0; T(4,3,1) = 0; T(4,4,1) = 1;%this row does not change
%1T2
T(1,1,2) = 1; T(1,2,2) = 0;  T(1,3,2) = 0; T(1,4,2) = 0;
T(2,1,2) = 0; T(2,2,2) = 0;  T(2,3,2) = -1; T(2,4,2) = 0;
T(3,1,2) = 0; T(3,2,2) = -1; T(3,3,2) = 0; T(3,4,2) = 98;
T(4,1,2) = 0; T(4,2,2) = 0;  T(4,3,2) = 0; T(4,4,2) = 1;%this row does not change
%2T3
T(1,1,3) = 0; T(1,2,3) = 0; T(1,3,3) = -1;  T(1,4,3) = -105;
T(2,1,3) = 0; T(2,2,3) = 1; T(2,3,3) = 0;  T(2,4,3) = 0;
T(3,1,3) = 1; T(3,2,3) = 0; T(3,3,3) = 0;  T(3,4,3) = 0;
T(4,1,3) = 0; T(4,2,3) = 0; T(4,3,3) = 0;  T(4,4,3) = 1;%this row does not change
%3T4
T(1,1,4) = 0; T(1,2,4) = 0; T(1,3,4) = 1;  T(1,4,4) = 0;
T(2,1,4) = 0; T(2,2,4) = 1; T(2,3,4) = 0;  T(2,4,4) = -145.5;
T(3,1,4) = -1; T(3,2,4) = 0; T(3,3,4) = 0; T(3,4,4) = 326.5;
T(1,1,4) = 0; T(1,2,4) = 0; T(1,3,4) = 0;  T(1,4,4) = 1;%this row does not change
%4T5
T(1,1,5) = 0; T(1,2,5) = 0; T(1,3,5) = 1;  T(1,4,5) = 35;
T(2,1,5) = 0; T(2,2,5) = 1; T(2,3,5) = 0;  T(2,4,5) = 0;
T(3,1,5) = -1; T(3,2,5) = 0; T(3,3,5) = 0; T(3,4,5) = 0;
T(4,1,5) = 0; T(4,2,5) = 0; T(4,3,5) = 0;  T(4,4,5) = 1;%this row does not change
%5T6
T(1,1,6) = 0; T(1,2,6) = 0; T(1,3,6) = 1;  T(1,4,6) = 0;
T(2,1,6) = 0; T(2,2,6) = 1; T(2,3,6) = 0;  T(2,4,6) = 0;
T(3,1,6) = -1; T(3,2,6) = 0; T(3,3,6) = 0; T(3,4,6) = 385;
T(4,1,6) = 0; T(4,2,6) = 0; T(4,3,6) = 0;  T(4,4,6) = 1;
%6T7
T(1,1,7) = 0; T(1,2,7) = 0; T(1,3,7) = -1;  T(1,4,7) = -153; 
T(2,1,7) = 0; T(2,2,7) = 1; T(2,3,7) = 0;  T(2,4,7) = 0;
T(3,1,7) = 1; T(3,2,7) = 0; T(3,3,7) = 0;  T(3,4,7) = 0;
T(4,1,7) = 0; T(4,2,7) = 0; T(4,3,7) = 0;  T(4,4,7) = 1;%this row does not change

end

