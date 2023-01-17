function [theta,v] = ComputeInverseAngleAxis(R)
%EULER REPRESENTATION
    %Idendity matrix
    I =eye(3,3)
    % Check matrix R to see if its size is 3x3
    if isequal(size(R),[3,3])     
        % Check matrix R to see if it is orthogonal
        %a matix is orthogonal if transpose equals its inverse OR if if we
        %multiply the matrix by its transpose it equals I
        %if ((inv(R))==(transpose(R))), also possible
         if (round((R*transpose(R)))==I)
            % Check matrix R to see if it is proper: det(R) = 1
            if (round(det(R))==1)
                % Compute the angle of rotation 
                theta= acos((trace(R)-1)/2);
                % Calculate eigenvalues and eigenvectors of R                 
                [Va,Da] = eig(R);
                % D gives eigen values and V gives eigen vectors
                %vex of R is ((R-RT)/2)
                M=((R - transpose(R))/2)
                % Compute the axis of rotation
                vx=M(3,2)
                vy=M(1,3)
                vz=M(2,1)
                v=[([vx , vy , vz])/(sin(theta))]

            else
                error('DETERMINANT OF THE INPUT MATRIX IS 0')
            end
        else
            error('NOT ORTHOGONAL INPUT MATRIX')
        end
    else
        error('WRONG SIZE OF THE INPUT MATRIX')
    end
end

