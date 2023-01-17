function biTei = DirectGeometry(qi, linkType)
% DirectGeometry Function 
% inputs: 
% q : current link position;
% iTj is the constant transformation between the base of the link <i>
% and its end-effector; 
% jointType :0 for revolute, 1 for prismatic

% output :
% biTei : transformation between the base of the joint <i> and its end-effector taking 
% into account the actual rotation/traslation of the joint
biTei=zeros(4,4);

if linkType == 0 % rotational
    %for a rotation around z,
    biTei = [cos(qi) , -sin(qi) , 0 , 0 ; sin(qi) , cos(qi) , 0 , 0 ; 0 , 0 , 1 , 0 ; 0 , 0 , 0, 1];

elseif linkType == 1 % prismatic
    %please note there are no prismatic joints in the given robot arm,
    %however we can compute iTj_q as such for a prismatic joint
    %iTj_q = [0 , 0 , 0 , qx ; 0 , 0 , 0 , qy ; 0 , 0 , 0 , qz ; 0 , 0 , 0, 1]
    fprintf('error, please note all joints in the robot arm are rotational\n');

end

end