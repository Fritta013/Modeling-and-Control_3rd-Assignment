function J = GetJacobian(bTi, numberOfLinks)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix

%initialization for J(1:6, 1) relative to the first frame Rotation and disp
ddf=bTi(:,:,numberOfLinks)
df=ddf(1:3,4) % displacement of last link, constant
I = eye(3) 
J_11=I*[0;0;1]%because 0R0 = I, constant 
J(1:3,1)=cross(J_11,df)%because 0d0 = 0
J(4:6,1)= I*[0;0;1] %because 0R0 = I, constant 


%IMPORTANT: the following is true for joints being all revolute 
%for the remaining links
%refer to the report for the full equation of the Jacobian Matrix 
for i =2:numberOfLinks
    Rr=bTi(:,:,(i-1))
    R=Rr(1:3,1:3)
    ddi=bTi(:,:,(i-1))
    di=ddi(1:3,4)
    J_ee=R*[0;0;1]
    diff=df-di
    J(1:3,i)= cross(J_ee,diff)
    J(4:6,i)= R*[0;0;1]

end

end