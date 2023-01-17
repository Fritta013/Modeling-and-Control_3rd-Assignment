function [iTj]=GetFrameWrtFrame(linkNumber_i, linkNumber_j, biTei, bTi)
%%% GetFrameWrtFrame function 
% inputs : 
% linkNumber_i : number of ith link 
% linkNumber_j: number of jth link 
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current q.
% The size of biTri is equal to (4,4,numberOfLinks)
% outputs:
% iTj : transformationMatrix in between link i and link j for the
% configuration described in biTei

%note  linkNumber_j is higher than %linkNumber_i

%initialize for first link, in this case link i
iTj(:,:,1)=bTi(:,:,linkNumber_i)
diff= ((linkNumber_j - linkNumber_i)+1)
  for i=2:diff
  iTj(:,:,i)=[iTj(:,:,(i-1))*biTei(:,:,((linkNumber_i+i)-1))]
  end
%the size of iTj is ((linkNumber_j -  linkNumber_i)+1)


end