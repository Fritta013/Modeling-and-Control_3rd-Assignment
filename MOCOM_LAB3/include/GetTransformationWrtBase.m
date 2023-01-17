function [bTi] = GetTransformationWrtBase(biTei, i)
%%% GetTransformatioWrtBase function
% inputs :
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current q.
% The size of biTri is equal to (4,4,numberOfLinks)
% linkNumber for which computing the transformation matrix
% outputs
% bTi : transformation matrix from the manipulator base to the ith joint in
% the configuration identified by biTei.

%Relative to first link/joint
bTi(:,:,1)=biTei(:,:,1) %const

for counter =2:i
    bTi(:,:,counter)= biTei(:,:,counter)*bTi(:,:,counter-1); 
 
end


end